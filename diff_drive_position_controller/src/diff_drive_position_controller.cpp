// Copyright 2026 Bilal Gill
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diff_drive_position_controller/diff_drive_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diff_drive_position_controller
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

double wrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

Pose2d computeBodyFrameError(const Pose2d & current, const Pose2d & reference)
{
  const double dx = reference.x - current.x;
  const double dy = reference.y - current.y;
  const double cos_yaw = std::cos(current.yaw);
  const double sin_yaw = std::sin(current.yaw);
  return Pose2d{
    cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy,
    wrapAngle(reference.yaw - current.yaw)};
}

UnicycleCommand trackingControl(
  const Pose2d & error_body, const UnicycleCommand & feedforward, const TrackingGains & gains)
{
  const double linear = feedforward.linear * std::cos(error_body.yaw) + gains.kx * error_body.x;
  const double angular = feedforward.angular + gains.ky * feedforward.linear * error_body.y +
                         gains.ktheta * std::sin(error_body.yaw);
  return UnicycleCommand{linear, angular};
}

WheelVelocities inverseKinematics(
  const UnicycleCommand & command, double wheel_separation, double wheel_radius)
{
  const double half_separation = 0.5 * wheel_separation;
  return WheelVelocities{
    (command.linear - command.angular * half_separation) / wheel_radius,
    (command.linear + command.angular * half_separation) / wheel_radius};
}

controller_interface::CallbackReturn DiffDrivePositionController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffDrivePositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  odometry_.setWheelParams(params_.wheel_separation, params_.wheel_radius, params_.wheel_radius);
  odometry_.setVelocityRollingWindowSize(
    static_cast<size_t>(params_.velocity_rolling_window_size));

  publish_period_ = rclcpp::Duration::from_seconds(1.0 / params_.publish_rate);

  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  if (params_.enable_odom_tf)
  {
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DiffDrivePositionController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.left_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY,
     params_.right_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY}};
}

controller_interface::InterfaceConfiguration
DiffDrivePositionController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.left_wheel_name + "/" + hardware_interface::HW_IF_POSITION,
     params_.right_wheel_name + "/" + hardware_interface::HW_IF_POSITION}};
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
DiffDrivePositionController::on_export_reference_interfaces_list()
{
  exported_references_.clear();
  for (const std::string & joint_name : params_.reference_joint_names)
  {
    // Exported interface prefixes must begin with the controller's node name.
    exported_references_.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        std::string(get_node()->get_name()) + "/" + joint_name,
        hardware_interface::HW_IF_POSITION));
  }
  return exported_references_;
}

std::vector<hardware_interface::StateInterface::SharedPtr>
DiffDrivePositionController::on_export_state_interfaces_list()
{
  exported_states_.clear();
  for (const std::string & joint_name : params_.reference_joint_names)
  {
    const std::string prefix = std::string(get_node()->get_name()) + "/" + joint_name;
    exported_states_.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        prefix, hardware_interface::HW_IF_POSITION, "double", "0.0"));
    exported_states_.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        prefix, hardware_interface::HW_IF_VELOCITY, "double", "0.0"));
  }
  return exported_states_;
}

controller_interface::CallbackReturn DiffDrivePositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Seed the odometry's previous wheel positions with the current ones so the first
  // update does not integrate the full absolute wheel angle as motion.
  const std::optional<double> left_pos = state_interfaces_[0].get_optional();
  const std::optional<double> right_pos = state_interfaces_[1].get_optional();
  if (!left_pos.has_value() || !right_pos.has_value())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to read wheel positions on activation.");
    return controller_interface::CallbackReturn::ERROR;
  }
  odometry_.init(get_node()->now());
  std::ignore = odometry_.update(*left_pos, *right_pos, get_node()->now());
  odometry_.resetOdometry();

  for (const auto & reference : exported_references_)
  {
    std::ignore = reference->set_value(kNaN);
  }
  prev_reference_valid_ = false;
  filtered_feedforward_ = UnicycleCommand{0.0, 0.0};
  previous_publish_timestamp_ = get_node()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiffDrivePositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  haltWheels();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DiffDrivePositionController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No external command path: unchained operation halts the wheels in
  // update_and_write_commands because the references stay NaN.
  return controller_interface::return_type::OK;
}

controller_interface::return_type DiffDrivePositionController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }

  const std::optional<double> left_pos = state_interfaces_[0].get_optional();
  const std::optional<double> right_pos = state_interfaces_[1].get_optional();
  if (!left_pos.has_value() || !right_pos.has_value())
  {
    // Transient lock contention: hold the previous command for one cycle.
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Unable to read wheel position state interfaces.");
    return controller_interface::return_type::OK;
  }

  std::ignore = odometry_.update(*left_pos, *right_pos, time);
  updateExportedStates();
  publishOdometry(time);

  const std::optional<double> ref_x = exported_references_[0]->get_optional();
  const std::optional<double> ref_y = exported_references_[1]->get_optional();
  const std::optional<double> ref_yaw = exported_references_[2]->get_optional();
  const bool references_valid = ref_x.has_value() && !std::isnan(*ref_x) &&
                                ref_y.has_value() && !std::isnan(*ref_y) &&
                                ref_yaw.has_value() && !std::isnan(*ref_yaw);
  if (!references_valid)
  {
    // No preceding controller wrote this cycle (JTC inactive or between goals).
    haltWheels();
    prev_reference_valid_ = false;
    filtered_feedforward_ = UnicycleCommand{0.0, 0.0};
    return controller_interface::return_type::OK;
  }

  // TODO(world-odom): apply the world->odom offset to the reference here once the
  // sim no longer guarantees that the robot spawns at the world origin.
  const Pose2d reference{*ref_x, *ref_y, *ref_yaw};

  if (params_.enable_feedforward && prev_reference_valid_ && period.seconds() > 0.0)
  {
    const double dt = period.seconds();
    const double x_dot = (reference.x - prev_reference_.x) / dt;
    const double y_dot = (reference.y - prev_reference_.y) / dt;
    const double yaw_dot = wrapAngle(reference.yaw - prev_reference_.yaw) / dt;
    // Project the world-frame reference velocity onto the reference heading, then
    // low-pass to soften the steps that finite differencing makes at trajectory knots.
    const double linear_raw = std::clamp(
      std::cos(reference.yaw) * x_dot + std::sin(reference.yaw) * y_dot,
      -params_.max_linear_velocity, params_.max_linear_velocity);
    const double angular_raw =
      std::clamp(yaw_dot, -params_.max_angular_velocity, params_.max_angular_velocity);
    filtered_feedforward_.linear +=
      params_.ff_lowpass_alpha * (linear_raw - filtered_feedforward_.linear);
    filtered_feedforward_.angular +=
      params_.ff_lowpass_alpha * (angular_raw - filtered_feedforward_.angular);
  }
  else
  {
    filtered_feedforward_ = UnicycleCommand{0.0, 0.0};
  }
  prev_reference_ = reference;
  prev_reference_valid_ = true;

  const Pose2d current{odometry_.getX(), odometry_.getY(), odometry_.getHeading()};
  const Pose2d error_body = computeBodyFrameError(current, reference);
  const TrackingGains gains{params_.gains.kx, params_.gains.ky, params_.gains.ktheta};
  const UnicycleCommand raw_command = trackingControl(error_body, filtered_feedforward_, gains);
  const UnicycleCommand command{
    std::clamp(raw_command.linear, -params_.max_linear_velocity, params_.max_linear_velocity),
    std::clamp(raw_command.angular, -params_.max_angular_velocity, params_.max_angular_velocity)};

  const WheelVelocities wheel_velocities =
    inverseKinematics(command, params_.wheel_separation, params_.wheel_radius);
  if (
    !command_interfaces_[0].set_value(wheel_velocities.left) ||
    !command_interfaces_[1].set_value(wheel_velocities.right))
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Unable to write wheel velocity commands.");
  }

  // Consume the references: the preceding controller must write every cycle, so a
  // stopped JTC halts the base within one update.
  for (const auto & reference : exported_references_)
  {
    std::ignore = reference->set_value(kNaN);
  }

  return controller_interface::return_type::OK;
}

void DiffDrivePositionController::haltWheels()
{
  if (!command_interfaces_[0].set_value(0.0) || !command_interfaces_[1].set_value(0.0))
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Unable to write zero wheel velocity commands.");
  }
}

void DiffDrivePositionController::updateExportedStates()
{
  const double yaw = odometry_.getHeading();
  const double linear = odometry_.getLinear();
  // Exported base state is world-frame: position from the odometry pose, velocity as
  // the body twist rotated by the heading.
  const std::array<double, 6> values{
    odometry_.getX(),          linear * std::cos(yaw), odometry_.getY(),
    linear * std::sin(yaw),    yaw,                    odometry_.getAngular()};
  for (size_t i = 0; i < exported_states_.size(); ++i)
  {
    std::ignore = exported_states_[i]->set_value(values[i]);
  }
}

void DiffDrivePositionController::publishOdometry(const rclcpp::Time & time)
{
  if (
    previous_publish_timestamp_.get_clock_type() == time.get_clock_type() &&
    time - previous_publish_timestamp_ < publish_period_)
  {
    return;
  }
  previous_publish_timestamp_ = time;

  const double yaw = odometry_.getHeading();
  const double qz = std::sin(0.5 * yaw);
  const double qw = std::cos(0.5 * yaw);

  nav_msgs::msg::Odometry odometry_message;
  odometry_message.header.stamp = time;
  odometry_message.header.frame_id = params_.odom_frame_id;
  odometry_message.child_frame_id = params_.base_frame_id;
  odometry_message.pose.pose.position.x = odometry_.getX();
  odometry_message.pose.pose.position.y = odometry_.getY();
  odometry_message.pose.pose.orientation.z = qz;
  odometry_message.pose.pose.orientation.w = qw;
  odometry_message.twist.twist.linear.x = odometry_.getLinear();
  odometry_message.twist.twist.angular.z = odometry_.getAngular();
  std::ignore = realtime_odometry_publisher_->try_publish(odometry_message);

  if (realtime_odometry_transform_publisher_)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = time;
    transform.header.frame_id = params_.odom_frame_id;
    transform.child_frame_id = params_.base_frame_id;
    transform.transform.translation.x = odometry_.getX();
    transform.transform.translation.y = odometry_.getY();
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;
    tf2_msgs::msg::TFMessage transform_message;
    transform_message.transforms.push_back(transform);
    std::ignore = realtime_odometry_transform_publisher_->try_publish(transform_message);
  }
}

}  // namespace diff_drive_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diff_drive_position_controller::DiffDrivePositionController,
  controller_interface::ChainableControllerInterface)

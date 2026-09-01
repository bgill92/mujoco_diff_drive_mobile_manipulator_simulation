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

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "diff_drive_position_controller/diff_drive_position_controller.hpp"
#include "diff_drive_position_controller/odometry.hpp"

namespace ddpc = diff_drive_position_controller;

constexpr double kWheelSeparation = 0.634;
constexpr double kWheelRadius = 0.075;
constexpr ddpc::TrackingGains kGains{2.0, 10.0, 3.0};

TEST(InverseKinematics, StraightLine)
{
  const ddpc::WheelVelocities wheels =
    ddpc::inverseKinematics({1.0, 0.0}, kWheelSeparation, kWheelRadius);
  EXPECT_DOUBLE_EQ(wheels.left, 1.0 / kWheelRadius);
  EXPECT_DOUBLE_EQ(wheels.right, 1.0 / kWheelRadius);
}

TEST(InverseKinematics, PureRotation)
{
  const ddpc::WheelVelocities wheels =
    ddpc::inverseKinematics({0.0, 1.0}, kWheelSeparation, kWheelRadius);
  EXPECT_DOUBLE_EQ(wheels.left, -0.5 * kWheelSeparation / kWheelRadius);
  EXPECT_DOUBLE_EQ(wheels.right, 0.5 * kWheelSeparation / kWheelRadius);
  EXPECT_DOUBLE_EQ(wheels.left, -wheels.right);
}

TEST(TrackingControl, ZeroErrorIsPureFeedforward)
{
  const ddpc::UnicycleCommand command =
    ddpc::trackingControl({0.0, 0.0, 0.0}, {0.7, 0.3}, kGains);
  EXPECT_DOUBLE_EQ(command.linear, 0.7);
  EXPECT_DOUBLE_EQ(command.angular, 0.3);
}

TEST(TrackingControl, LateralOffsetSteersTowardReference)
{
  // Reference is to the robot's left (positive body y) while moving forward:
  // the robot must turn left (positive angular velocity).
  const ddpc::UnicycleCommand left_offset =
    ddpc::trackingControl({0.0, 0.2, 0.0}, {0.5, 0.0}, kGains);
  EXPECT_GT(left_offset.angular, 0.0);

  const ddpc::UnicycleCommand right_offset =
    ddpc::trackingControl({0.0, -0.2, 0.0}, {0.5, 0.0}, kGains);
  EXPECT_LT(right_offset.angular, 0.0);
}

TEST(TrackingControl, LongitudinalErrorDrivesForward)
{
  const ddpc::UnicycleCommand command =
    ddpc::trackingControl({0.5, 0.0, 0.0}, {0.0, 0.0}, kGains);
  EXPECT_DOUBLE_EQ(command.linear, kGains.kx * 0.5);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
}

TEST(BodyFrameError, WrapsHeadingAcrossPi)
{
  const ddpc::Pose2d error = ddpc::computeBodyFrameError(
    {0.0, 0.0, 0.9 * std::numbers::pi}, {0.0, 0.0, -0.9 * std::numbers::pi});
  // Shortest way from +0.9*pi to -0.9*pi is +0.2*pi, not -1.8*pi.
  EXPECT_NEAR(error.yaw, 0.2 * std::numbers::pi, 1e-12);
}

TEST(BodyFrameError, RotatesIntoBodyFrame)
{
  // Robot at origin facing +y; reference 1 m ahead along world +y is a pure
  // longitudinal error in the body frame.
  const ddpc::Pose2d error = ddpc::computeBodyFrameError(
    {0.0, 0.0, 0.5 * std::numbers::pi}, {0.0, 1.0, 0.5 * std::numbers::pi});
  EXPECT_NEAR(error.x, 1.0, 1e-12);
  EXPECT_NEAR(error.y, 0.0, 1e-12);
  EXPECT_NEAR(error.yaw, 0.0, 1e-12);
}

TEST(Odometry, StraightLineIntegration)
{
  ddpc::Odometry odometry;
  odometry.setWheelParams(kWheelSeparation, kWheelRadius, kWheelRadius);
  odometry.init(rclcpp::Time(0));

  // Both wheels advance 1 rad per 10 ms step for 100 steps: 100 * kWheelRadius meters.
  for (int i = 1; i <= 100; ++i)
  {
    ASSERT_TRUE(
      odometry.update(static_cast<double>(i), static_cast<double>(i), rclcpp::Time(i * 10000000)));
  }
  EXPECT_NEAR(odometry.getX(), 100.0 * kWheelRadius, 1e-9);
  EXPECT_NEAR(odometry.getY(), 0.0, 1e-9);
  EXPECT_NEAR(odometry.getHeading(), 0.0, 1e-9);
}

TEST(Odometry, ConstantArcMatchesClosedForm)
{
  ddpc::Odometry odometry;
  odometry.setWheelParams(kWheelSeparation, kWheelRadius, kWheelRadius);
  odometry.init(rclcpp::Time(0));

  // Right wheel spins twice as fast as the left: constant-curvature arc.
  constexpr double left_rate = 1.0;   // rad per step
  constexpr double right_rate = 2.0;  // rad per step
  constexpr int steps = 200;
  for (int i = 1; i <= steps; ++i)
  {
    ASSERT_TRUE(
      odometry.update(left_rate * i, right_rate * i, rclcpp::Time(i * 10000000)));
  }

  const double total_linear = 0.5 * (left_rate + right_rate) * kWheelRadius * steps;
  const double total_angular =
    (right_rate - left_rate) * kWheelRadius * steps / kWheelSeparation;
  const double arc_radius = total_linear / total_angular;
  EXPECT_NEAR(odometry.getHeading(), total_angular, 1e-9);
  EXPECT_NEAR(odometry.getX(), arc_radius * std::sin(total_angular), 1e-9);
  EXPECT_NEAR(odometry.getY(), arc_radius * (1.0 - std::cos(total_angular)), 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

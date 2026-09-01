# MuJoCo Diff-Drive Mobile Manipulator Simulation

ROS 2 Jazzy + MuJoCo simulation of a Neobotix ROX base with a UR5e arm and a
Robotiq 2F-85 gripper, controlled through
[`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control).

- **Arm**: `joint_trajectory_controller/JointTrajectoryController` (position)
- **Gripper**: `position_controllers/GripperActionController`
- **Base**: kinematic velocity override via `BaseVelocityPlugin` on `/cmd_vel`
  (a true `diff_drive_controller` with wheel physics is a planned follow-up)

MuJoCo runs *inside* the patched `ros2_control_node` executable from the
`mujoco_ros2_control` package. It publishes `/clock`; every node runs with
`use_sim_time: true`.

## Prerequisites

Everything runs through [pixi](https://pixi.sh) (RoboStack, no `/opt/ros`):

```bash
pixi install
```

## Build

```bash
pixi run bash -c "colcon build --packages-up-to mobile_manipulator_simulation"
```

This builds the vendored `mujoco_ros2_control` packages
(`external_packages/mujoco_ros2_control`: msgs → plugins → core) and the
first-party `mobile_manipulator_simulation` package, and skips the `rox`
packages (they have unresolved `neo_*` dependencies).

> **Note**: if the repo directory is ever moved or renamed, the pixi env's
> Python entry points (colcon, ros2, ...) break with "required file not found"
> (stale absolute shebangs). Fix: `pixi clean && pixi install`, then
> `rm -rf build install log` and rebuild.

## Run

```bash
pixi run ros2 launch mobile_manipulator_simulation mujoco_sim.launch.py
```

Opens the MuJoCo Simulate window and RViz. Launch arguments:

| arg | default | meaning |
|-----|---------|---------|
| `headless` | `false` | no MuJoCo window |
| `rviz` | `true` | launch RViz (fixed frame `odom`) |
| `mujoco_model` | source-tree `description/scene.xml` | MJCF scene path |

The MJCF and its ~75 MB of mesh assets deliberately stay in the source tree
(not installed to `share/`), so the default `mujoco_model` is an absolute
source-tree path — override it if you relocate the repo.

Pausing the MuJoCo window freezes `/clock`, which stalls the controllers too;
leave the simulation running while testing.

## Test

In a second terminal:

```bash
# Controllers: expect joint_state_broadcaster, joint_trajectory_controller,
# gripper_controller -- all active
pixi run ros2 control list_controllers

# Move the arm (action interface)
pixi run ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: \
  [ur5eshoulder_pan_joint, ur5eshoulder_lift_joint, ur5eelbow_joint, \
  ur5ewrist_1_joint, ur5ewrist_2_joint, ur5ewrist_3_joint], points: \
  [{positions: [0.5, -1.2, 1.0, -0.8, 0.6, 0.3], time_from_start: {sec: 4}}]}}"

# Gripper
pixi run ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 10.0}}"

# Drive the base (Ctrl+C stops; base halts within the 0.5 s cmd_timeout)
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.2}}"

# ... or keyboard teleop
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Odometry and TF
pixi run ros2 topic echo /simulator/floating_base_state
pixi run ros2 run tf2_ros tf2_echo odom ur5etool0
```

Expected behavior:

- The arm tracks the trajectory; swinging it does **not** move the base
  (the kinematic override is immune to reaction forces).
- The base slides without wheel spin — wheels are visual-only in this phase.
- The base stops within 0.5 s of releasing `/cmd_vel`.

### Physics-only checks (no ROS)

```bash
cd mobile_manipulator_simulation/description

# Interactive viewer
pixi run python -m mujoco.viewer --mjcf=scene.xml

# Settle regression check (run after any converter re-run or MJCF edit)
pixi run python check_stability.py
```

## Layout

```
mobile_manipulator_simulation/   first-party package
  description/                   URDFs, MJCF (scene.xml + generated model),
                                 conversion docs, check_stability.py
  config/                        controllers.yaml, mujoco_plugins.yaml
  launch/                        mujoco_sim.launch.py, view_robot.launch.py
  scripts/odom_to_tf.py          odom -> base_link TF bridge
external_packages/
  mujoco_ros2_control/           vendored ros-controls hardware interface
  rox/                           Neobotix ROX description (not built)
```

See `mobile_manipulator_simulation/description/URDF_TO_MJCF_WALKTHROUGH.md`
for the URDF→MJCF conversion record, including the required post-conversion
floating-base edit (and why frictionless geoms must use `condim="1"`, never
`friction="0 0 0"`).

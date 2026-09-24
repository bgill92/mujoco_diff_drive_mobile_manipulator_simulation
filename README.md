# MuJoCo Diff-Drive Mobile Manipulator Simulation

ROS 2 Jazzy + MuJoCo simulation of a Neobotix ROX base with a UR5e arm and a
Robotiq 2F-85 gripper, controlled through
[`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control).

- **Arm + base**: one `joint_trajectory_controller/JointTrajectoryController`
  (position) owning the 6 arm joints plus 3 virtual base joints chained into
  `diff_drive_position_controller` (this repo's chainable controller — see its
  [README](diff_drive_position_controller/README.md)), which closes the base
  pose loop on wheel-encoder odometry and publishes the `odom -> base_link` TF
- **Gripper**: `position_controllers/GripperActionController`
- **Teleop fallback**: the stock `diff_drive_controller` config is kept for
  manually spawning on `/cmd_vel` (conflicts with the position controller over
  the wheel interfaces, so one at a time)

MuJoCo runs *inside* the patched `ros2_control_node` executable from the
`mujoco_ros2_control` package. It publishes `/clock`; every node runs with
`use_sim_time: true`.

![MuJoCo sim executing the L-shaped whole-body trajectory — desired
end-effector path in green, actual in red](docs/demo.gif)

*The sim executing the L-shaped whole-body trajectory at 2× speed: desired
end-effector path in green, actual (traced from the `grasp_link` site) in red.
Regenerate with [`scripts/render_replay.py`](scripts/render_replay.py).*

## TL;DR

Run the whole-body trajectory generator against the sim — the robot spawns at
the L-shaped path's start configuration and executes it base + arm together:

```bash
# 1. Install pixi (https://pixi.sh) and Rust (https://rustup.rs)

# 2. Clone with submodules
git clone --recurse-submodules git@github.com:bgill92/mujoco_diff_drive_mobile_manipulator_simulation.git
cd mujoco_diff_drive_mobile_manipulator_simulation

# 3. Set up the ROS environment and build
pixi install
pixi run bash -c "colcon build --packages-up-to mobile_manipulator_simulation"

# 4. Terminal 1: launch the sim (MuJoCo window + RViz)
pixi run ros2 launch mobile_manipulator_simulation mujoco_sim.launch.py

# 5. Terminal 2: solve the L path from the robot's measured state and execute it
cd external_packages/whole_body_differential_drive_trajectory_generation
cargo run --release -- assets/sim_config_l_path.yaml
```

The base drives the L while the arm tracks the end-effector waypoints. RViz
overlays the desired end-effector path (green, published by the generator) on
the actual one (red, sampled from TF); a Rerun window shows the solved
trajectory and solver diagnostics. Re-running step 5 replans from wherever
the robot stopped. The generator inherits `ROS_DOMAIN_ID` from your shell —
it must match the sim's.

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
# gripper_controller, diff_drive_controller -- all active
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

# Drive the base (Ctrl+C stops; base halts within the 0.5 s cmd_vel_timeout).
# Zero-stamp TwistStamped messages are auto-stamped by the controller.
pixi run ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"

# ... or keyboard teleop (stamped mode required)
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# Odometry and TF: encoder odom from the controller, ground truth from MuJoCo
pixi run ros2 topic echo /diff_drive_controller/odom
pixi run ros2 topic echo /simulator/floating_base_state
pixi run ros2 run tf2_ros tf2_echo odom ur5etool0
```

Expected behavior:

- The base drives on its wheels: wheel spin is visible in MuJoCo and RViz, and
  swinging the arm rocks the base a few mm (reaction forces are real now).
- Encoder odometry (`/diff_drive_controller/odom`) tracks the MuJoCo ground
  truth (`/simulator/floating_base_state`) within a few percent; the residual
  is wheel slip.
- The base stops within 0.5 s of releasing `/cmd_vel`.

### Physics-only checks (no ROS)

```bash
cd mobile_manipulator_simulation/description

# Interactive viewer
pixi run python -m mujoco.viewer --mjcf=scene.xml

# Settle + wheel-traction regression check (run after any converter re-run or MJCF edit)
pixi run python check_stability.py
```

## Whole-body trajectory generation

`external_packages/whole_body_differential_drive_trajectory_generation` (Rust,
cargo-only — no `package.xml`, so colcon never sees it and no COLCON_IGNORE is
needed) plans base + arm trajectories and sends them to the sim's
`joint_trajectory_controller` over DDS. It reads the robot's current state
first (`/joint_states` for the arm, `/diff_drive_position_controller/odom` for
the base) and plans from it, so re-running it from wherever the robot stopped
works.

```bash
# Terminal 1: the sim
pixi run ros2 launch mobile_manipulator_simulation mujoco_sim.launch.py

# Terminal 2: the generator (honors ROS_DOMAIN_ID from the environment;
# it must match the sim's domain)
cd external_packages/whole_body_differential_drive_trajectory_generation
cargo run --release -- assets/sim_config_l_path.yaml
```

The `ros2:` section of the config enables state reading + sending; its
`base_joint_name_prefix: "diff_drive_position_controller/"` maps the
generator's bare base joint names onto the chainable controller's prefixed
reference interfaces.

Frames: the generator plans in its URDF `world` frame; odometry starts at
(0, 0, 0) wherever the robot spawns, so the generator's `ros2.world_from_odom`
config (the spawn pose `[x, y, yaw]` in world) reconciles the two — it lifts
measured odometry into the world frame and maps outgoing base references back
into odom. The robot currently spawns at the whole-body L-path start pose
(base pose in the MJCF `base_link` body, arm in the xacro `initial_value`s,
both from the generator's `initial_pose` cargo example); the generator's
`assets/sim_config_l_path.yaml` carries the matching `world_from_odom`. A
robot spawning at the origin uses the default identity offset.

## Layout

```
mobile_manipulator_simulation/   first-party package
  description/                   URDFs, MJCF (scene.xml + generated model),
                                 conversion docs, check_stability.py
  config/                        controllers.yaml
  launch/                        mujoco_sim.launch.py, view_robot.launch.py
diff_drive_position_controller/  chainable position-reference base controller
external_packages/
  mujoco_ros2_control/           vendored ros-controls hardware interface
  rox/                           Neobotix ROX description (not built)
  whole_body_differential_drive_trajectory_generation/
                                 Rust whole-body trajectory generator (cargo)
```

See `mobile_manipulator_simulation/description/URDF_TO_MJCF_WALKTHROUGH.md`
for the URDF→MJCF conversion record, including the required post-conversion
floating-base edit (and why frictionless geoms must use `condim="1"`, never
`friction="0 0 0"`).

## License

Apache-2.0 (see [LICENSE](LICENSE)). Copied meshes and the odometry code carry
their upstream BSD / Apache-2.0 notices; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

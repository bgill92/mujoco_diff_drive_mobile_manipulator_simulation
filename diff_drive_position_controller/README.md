# diff_drive_position_controller

Chainable ros2_control controller (Jazzy) that lets one
`JointTrajectoryController` command a differential-drive base and an arm from
a single position-only `FollowJointTrajectory` goal.

## Why it exists

A JTC applies the same command interface type to every joint it owns, and the
arm hardware only accepts `position` commands — so a whole-body trajectory
(base + arm in one goal) needs the base to accept **position** references too.
Velocity-reference base controllers (e.g. the mecanum drive controller this
one is structurally modeled on) push the position loop into the JTC, which
would force `velocity` command interfaces on the arm as well. This controller
instead exports position reference interfaces and closes the pose loop
itself, against its own wheel-encoder odometry.

## Interfaces

Claimed from hardware:

| Interface | Type |
|---|---|
| `<left/right wheel>/velocity` | command |
| `<left/right wheel>/position` | state (odometry feedback) |

Exported (chainable; `ros2_control` requires the controller-name prefix, so a
JTC's `joints` list — and therefore incoming goal joint names — must use the
prefixed form):

| Interface | Meaning |
|---|---|
| `<controller>/<x,y,yaw joint>/position` | reference: desired base pose, **odom frame** |
| `<controller>/<x,y,yaw joint>/position` + `/velocity` | state: odometry pose / world-frame twist |

The three virtual joint names come from `reference_joint_names`
(default `world_base_link_planar_prismatic_x`, `..._prismatic_y`,
`..._yaw`).

## Control law

Per update (100 Hz here): wheel positions → odometry (exact-arc integration,
copied from upstream `diff_drive_controller`) → body-frame pose error vs the
references → Kanayama tracking:

```
v = v_r·cos(e_θ) + kx·e_x
ω = ω_r + ky·v_r·e_y + kθ·sin(e_θ)
```

with feedforward `(v_r, ω_r)` finite-differenced from the references and
low-passed (`ff_lowpass_alpha`), clamped to `max_linear_velocity` /
`max_angular_velocity`, then diff-drive IK to wheel velocity commands.

References are consumed every cycle (reset to NaN after use): if the
preceding controller stops writing — JTC inactive, between goals, unchained —
the wheels are commanded to zero within one update. There is **no** cmd_vel
subscriber; teleop means deactivating this controller and spawning the stock
`diff_drive_controller` instead (they conflict over the wheel command
interfaces by design).

Known behavior: a reference with a pure body-lateral offset at `v_r = 0` is
geometrically uncorrectable for a diff drive (the `ky` term switches off at
standstill), so the robot parks with that offset. Feasible (nonholonomy-
respecting) trajectories don't produce this.

## Parameters (see `src/diff_drive_position_controller_parameters.yaml`)

Wheel geometry (`left_wheel_name`, `right_wheel_name`, `wheel_separation`,
`wheel_radius` — required, read-only), gains (`gains.kx/ky/ktheta`,
runtime-tunable), feedforward (`enable_feedforward`, `ff_lowpass_alpha`),
velocity clamps, and odometry publishing (`odom_frame_id`, `base_frame_id`,
`enable_odom_tf`, `publish_rate`, `velocity_rolling_window_size`) with the
same names as upstream `diff_drive_controller`. Publishes `~/odom` and the
`odom → base` TF.

## Example wiring (from this repo's `controllers.yaml`)

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - diff_drive_position_controller/world_base_link_planar_prismatic_x
      - diff_drive_position_controller/world_base_link_planar_prismatic_y
      - diff_drive_position_controller/world_base_link_planar_yaw
      - ur5eshoulder_pan_joint
      # ... arm joints
    command_interfaces: [position]
    state_interfaces: [position, velocity]

diff_drive_position_controller:
  ros__parameters:
    left_wheel_name: wheel_left_joint
    right_wheel_name: wheel_right_joint
    wheel_separation: 0.634
    wheel_radius: 0.075
```

Spawn this controller **before** the JTC (one spawner, argument order — see
`mujoco_sim.launch.py`): the JTC's activation claims the exported reference
interfaces, which only exist once this controller is configured.

## Tests

`colcon test --packages-select diff_drive_position_controller` — gtests for
the IK, the tracking law (zero error → pure feedforward, steering signs,
±π wrap), and odometry integration against closed-form straight-line and
constant-arc motion.

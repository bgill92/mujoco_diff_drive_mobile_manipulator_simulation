# MJCF Conversion Issue Log — rox_diff_ur5e

- **Date**: 2026-08-24
- **Source URDF**: `mobile_manipulator_simulation/description/rox_diff_ur5e.urdf` (xacro-generated, rox base + UR5e + Robotiq 85)
- **Converter**: `mujoco_ros2_control` @ tag 0.1.0 (`make_mjcf_from_robot_description.py`)
- **Goal**: MJCF that opens cleanly in `mujoco-simulate`
- **Working copy**: `rox_diff_ur5e_mujoco.urdf` (original left untouched)

## Issues

## Issue 1: Bare relative mesh paths resolve against CWD
- **Stage**: Pre-conversion analysis
- **Symptom**: URDF references meshes as `meshes/ur_description/...` (no `package://`); converter's `extract_mesh_info`/`trimesh.load` use the path verbatim, so it resolves relative to the process CWD, not the URDF location.
- **Root cause**: Meshes were vendored next to the URDF with relative paths; converter only rewrites `package://` and `file://` URIs.
- **Fix**: Keep working URDF copy in the same directory and run the converter with CWD = `mobile_manipulator_simulation/description/`.
- **Status**: worked around

## Issue 2: Fake planar base chain — zero-inertia moving bodies + prismatic joints without limits
- **Stage**: Pre-conversion analysis
- **Symptom**: `world → intermediate_1 → intermediate_2 → base_link` chain uses prismatic x/y + revolute yaw joints; `intermediate_1`, `intermediate_2`, `base_link` have no `<inertial>`, and the prismatic joints have no `<limit>` tag. MuJoCo rejects moving bodies with mass/inertia below mjMINVAL, and the URDF parser rejects limit-less prismatic joints.
- **Root cause**: Chain emulates a planar floating base for ROS tooling; never meant for physics.
- **Fix**: Removed the 3 planar joints and the `world`/`intermediate_1`/`intermediate_2` links from the working copy. `base_link` is now the root and MuJoCo welds it to the world (per user decision — visualization only).
- **Status**: fixed

## Issue 3: MuJoCo URDF compiler ignores `<mimic>` — gripper four-bar would dangle
- **Stage**: Pre-conversion analysis
- **Symptom**: 5 Robotiq 85 joints are `<mimic>`-slaved to `robotiq_85_left_knuckle_joint`; MuJoCo's URDF importer drops mimic tags, leaving the fingers as free dangling joints.
- **Root cause**: Mimic is a URDF-only concept; MJCF expresses it as `<equality><joint>` constraints.
- **Fix**: Authored `mujoco_inputs.xml` with 5 `<equality><joint>` entries (polycoef multipliers matching the URDF mimic multipliers: right_knuckle −1, left_inner_knuckle +1, right_inner_knuckle −1, left_finger_tip −1, right_finger_tip +1), injected via the converter's `raw_inputs` mechanism.
- **Status**: fixed

## Issue 4: Compile failed — `unknown default class name 'visual'`
- **Stage**: Headless compile (`mujoco-compile scene.xml`)
- **Symptom**: `XML Error: unknown default class name 'visual'  Element 'geom', line 0`
- **Root cause**: The converter's obj2mjcf pass emits geoms with `class="visual"`, but the defining `<default>` block is expected to come from `raw_inputs` (the demo `test_inputs.xml` carries it); our first `mujoco_inputs.xml` omitted it.
- **Fix**: Added the `<default>` block with `visual`/`collision` classes to `raw_inputs` in `mujoco_inputs.xml` and re-ran the converter.
- **Status**: fixed

## Issue 5: Compile failed — flat meshes: `mesh 'short_frame_0' has coplanar vertices, cannot compute convex hull`
- **Stage**: Headless compile (`mujoco-compile scene.xml`)
- **Symptom**: `Error: mesh 'short_frame_0' has coplanar vertices, cannot compute convex hull. Consider using a primitive geom type (plane or thin box) instead`
- **Root cause**: Some source visual meshes contain perfectly flat sub-meshes (zero extent along one axis): `short_frame_0`, `short_frame_9` (rox frame, 0.1-scaled) and `nanoscan_3_5` (SICK lidar). MuJoCo computes a convex hull for every mesh asset and rejects degenerate (2D) geometry.
- **Fix**: Post-processed the generated OBJs in `assets/full/`: for every OBJ with a bounding-box extent < 1e-6, concatenated a copy translated 0.1 mm along the degenerate axis (script inline with trimesh). Visually identical, hull becomes 3D. Note: re-running the converter regenerates the OBJs and re-flattens them — re-apply the thickening script after any re-run.
- **Status**: worked around (script saved as `thicken_flat_meshes.py`)

## Issue 6: Actuated shoulder_pan frozen — hidden collidable geoms jam the arm
- **Stage**: Actuator addition (position actuators in `raw_inputs`)
- **Symptom**: With `ctrl` commanded, all joints tracked except `ur5eshoulder_pan_joint` (stayed at 0). `d.ncon` = 16 despite "all collisions stripped"; `qfrc_constraint` on the pan DOF (−126 Nm) cancelled the clamped actuator torque.
- **Root cause**: obj2mjcf leaves some generated sub-mesh geoms without a `class` attribute. Those fall back to MuJoCo's global geom default (`contype=1 conaffinity=1`) and are collidable — the shoulder meshes jammed against the base meshes (fused into the world body). The earlier "no contact geoms" assumption held only for geoms carrying `class="visual"`.
- **Fix**: Added a global geom default `<geom contype="0" conaffinity="0"/>` to the `<default>` block in `raw_inputs` — visual-only model, nothing collides. `ncon` = 0 afterwards.
- **Status**: fixed

## Issue 7: Wrist limit-cycle oscillation — actuator damping saturates with the P term
- **Stage**: Actuator addition
- **Symptom**: After fixing Issue 6, sim never settled: `qvel` norm ~30, `ur5ewrist_3_joint` spinning at ~30 rad/s with its actuator force pegged at ±28 Nm.
- **Root cause**: URDF `<limit effort>` becomes `jnt_actfrcrange` (28 Nm on wrists). Position-actuator demand kp·err exceeded the clamp, and the actuator's internal damping term (dampratio) is clamped *together* with the P term — so at saturation there is zero effective damping and the joint bang-bangs. Joints themselves had `damping=0`.
- **Fix**: Added real joint damping (outside the clamp) + armature via the converter's `<modify_element>` processed inputs: arm joints damping 5–10 / armature 0.1, gripper joints damping 0.1. Settles to `qvel` = 0, max tracking error 0.009 rad.
- **Status**: fixed

## Result

- **Final MJCF**: `mobile_manipulator_simulation/description/mujoco_description_formatted.xml`, wrapped by `scene.xml` (floor + light + skybox).
- **Verification**: `mujoco-compile scene.xml` passes; model loads in python (12 joints: 6 UR5e + 6 Robotiq, 5 active equality constraints, 67 meshes, 177 geoms); 500-step sim runs clean; gripper equality coupling tracks the master joint with correct ±1 ratios; offscreen render shows base + arm + gripper with materials.
- **Re-run recipe** (from `description/`):
  ```bash
  env PATH="../../.pixi/envs/default/bin:$PATH" \
      PYTHONPATH="../../mujoco_ros2_control/mujoco_ros2_control:$PYTHONPATH" \
    ../../.pixi/envs/default/bin/python \
    ../../mujoco_ros2_control/mujoco_ros2_control/scripts/make_mjcf_from_robot_description.py \
    -u rox_diff_ur5e_mujoco.urdf -m mujoco_inputs.xml --scene scene.xml -o mjcf_out -s
  rm -rf assets && mv mjcf_out/assets mjcf_out/mujoco_description_formatted.xml . && rm -rf mjcf_out
  ../../.pixi/envs/default/bin/python thicken_flat_meshes.py
  ```
- **Actuators** (added after initial conversion): 7 position actuators in `raw_inputs` — 6 UR5e joints (kp 2000–4500) + gripper master `robotiq_85_left_knuckle_joint` (kp 100, ctrlrange 0–0.8). Actuator names = joint names for a direct 1:1 match by `mujoco_ros2_control`'s `get_actuator_id`. Joint damping/armature via `modify_element` (see Issues 6–7). Verified: commanded pose `[0.5, -1.2, 1.0, -0.8, 0.6, 0.3, 0.4]` reached under gravity, max error 0.009 rad, `qvel` → 0, gripper slaves track master.
- **Accepted quirks**: base welded to world (planar chain removed per decision); wheels/casters fixed; no collision geometry at all — converter strips URDF `<collision>` and we force `contype=0 conaffinity=0` on every geom (visual/actuation model only; re-enable per-geom if contact physics ever needed).

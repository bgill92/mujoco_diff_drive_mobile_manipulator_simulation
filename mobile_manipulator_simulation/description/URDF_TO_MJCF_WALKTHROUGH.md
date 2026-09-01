# URDF → MJCF Conversion Walkthrough: rox_diff_ur5e

Complete record of converting `rox_diff_ur5e.urdf` (Neobotix ROX diff-drive base + UR5e arm + Robotiq 85 gripper, 38 links / 37 joints) into an MJCF with working position actuators, using the converter shipped in `mujoco_ros2_control`. End state: the model loads in `mujoco-simulate` / `mujoco.viewer` with no errors, holds a commanded pose under gravity, and the gripper four-bar closes as one unit.

- **Dates**: 2026-08-24 → 2026-08-25
- **Workspace**: `~/Projects/robotics/pixi_ros2_jazzy` (pixi-managed ROS 2 Jazzy env; never colcon-built)
- **Converter**: `mujoco_ros2_control` @ tag 0.1.0 — `scripts/make_mjcf_from_robot_description.py`
- **MuJoCo**: 3.8.1 (pixi env: C lib, python bindings, `mujoco-simulate`, `mujoco-compile`, `obj2mjcf` 0.0.25, trimesh)

## How the converter works

MuJoCo does not ingest URDFs at runtime in `mujoco_ros2_control`; the repo ships a Python pipeline that does the conversion offline:

1. Injects a `<mujoco><compiler assetdir="assets" balanceinertia="true" discardvisual="false"/></mujoco>` tag into the URDF.
2. **Strips every `<collision>` element** (assumes visual meshes are the better source of geometry).
3. Rewrites `package://` / `file://` URIs to absolute paths (bare relative paths are left untouched — see Issue 1).
4. Converts STL/DAE meshes to OBJ via trimesh into `assets/full/`.
5. Compiles the preprocessed URDF with `mujoco.MjModel.from_xml_path` and saves via `mj_saveLastXML` → `mujoco_description.xml`.
6. Runs `obj2mjcf` over the OBJs, then re-inserts assets plus your hand-authored extras → `mujoco_description_formatted.xml`.

Hand-authored extras live in a `<mujoco_inputs>` file (passed with `-m`):
- `raw_inputs` — MJCF fragments copied verbatim (defaults, actuators, equality constraints, options…)
- `processed_inputs` — converter-transformed tags (`modify_element`, cameras, lidars…)
- a separate `--scene` file is copied verbatim into the output dir and `<include>`s the robot MJCF.

Two things the converter does **not** do, which shaped this conversion:
- It does not translate URDF `<mimic>` tags — MuJoCo's URDF importer silently drops them.
- It does not read `<ros2_control>` blocks. Those are runtime-only: at runtime `MujocoSystemInterface` looks up **already-existing** MJCF actuators by joint name (`mujoco_system_interface.cpp:152`). Actuators must be authored in `raw_inputs`; a ros2_control block in the URDF would not have produced them.

## What was done, in order

### 1. Working copy of the URDF
`rox_diff_ur5e.urdf` → `rox_diff_ur5e_mujoco.urdf`, same directory (required — see Issue 1). Original untouched. One structural edit: removed the fake planar floating-base chain (see Issue 2), making `base_link` the root so MuJoCo welds it to the world.

### 2. Authored `mujoco_inputs.xml`
Grew across the conversion to its final contents:
- `<default>` block defining `visual`/`collision` geom classes (Issue 4) plus a global `<geom contype="0" conaffinity="0"/>` (Issue 6).
- `<equality>` block with 5 joint couplings replacing the URDF mimics (Issue 3).
- `<actuator>` block: 7 position actuators — 6 UR5e joints (kp 2000–4500, dampratio 1.0) + the gripper master `robotiq_85_left_knuckle_joint` (kp 100, ctrlrange 0–0.8). Actuator names equal joint names so `mujoco_ros2_control` gets a direct 1:1 match later.
- `<processed_inputs>`: 12 `modify_element` entries adding joint damping/armature (Issue 7).

### 3. Authored `scene.xml`
Standalone wrapper: `<include file="mujoco_description_formatted.xml"/>` + checker floor plane, directional light, skybox, camera statistics. Copied verbatim by the converter into the output dir.

### 4. Ran the converter (from the `description/` directory — mandatory, Issue 1)
```bash
cd mobile_manipulator_simulation/description
env PATH="../../.pixi/envs/default/bin:$PATH" \
    PYTHONPATH="../../mujoco_ros2_control/mujoco_ros2_control:$PYTHONPATH" \
  ../../.pixi/envs/default/bin/python \
  ../../mujoco_ros2_control/mujoco_ros2_control/scripts/make_mjcf_from_robot_description.py \
  -u rox_diff_ur5e_mujoco.urdf -m mujoco_inputs.xml --scene scene.xml -o mjcf_out -s
rm -rf assets && mv mjcf_out/assets mjcf_out/mujoco_description_formatted.xml . && rm -rf mjcf_out
../../.pixi/envs/default/bin/python thicken_flat_meshes.py   # Issue 5 — after EVERY re-run
```
No colcon build needed: `PYTHONPATH` points at the package source so the script's `mujoco_ros2_control.urdf_to_mujoco_utils` import resolves; `PATH` prepend lets it find `obj2mjcf`. (`coacd` is not installed — never use `--decompose`.)

### 5. Verified
- `mujoco-compile scene.xml <out>` — headless compile gate.
- Python: model loads (12 joints, 7 actuators, 5 active equalities, 67 meshes); commanded pose `[0.5, -1.2, 1.0, -0.8, 0.6, 0.3]` + gripper `0.4` reached under gravity with max error 0.009 rad; `qvel` settles to 0; gripper slaves track the master at exact ±1 ratios; zero contacts.
- `mujoco-simulate scene.xml` opens cleanly with 7 control sliders.

## Issues encountered and resolutions

### Issue 1: Bare relative mesh paths resolve against CWD
- **Stage**: Pre-conversion analysis
- **Symptom**: URDF references meshes as `meshes/ur_description/...` (no `package://`); the converter's `extract_mesh_info`/`trimesh.load` use the path verbatim, so it resolves relative to the process CWD, not the URDF location.
- **Root cause**: Meshes were vendored next to the URDF with relative paths; the converter only rewrites `package://` and `file://` URIs.
- **Fix (superseded)**: originally worked around by running the converter with CWD = `mobile_manipulator_simulation/description/`. The working copy now uses `package://mobile_manipulator_simulation/description/meshes/...` URIs (same form as the archived export) — the converter rewrites those to absolute paths itself, and RViz's RobotModel can resolve them (bare relative paths silently fail in RViz, which resolves against its own CWD).
- **Status**: fixed

### Issue 2: Fake planar base chain — zero-inertia moving bodies + prismatic joints without limits
- **Stage**: Pre-conversion analysis
- **Symptom**: `world → intermediate_1 → intermediate_2 → base_link` chain uses prismatic x/y + revolute yaw joints; `intermediate_1`, `intermediate_2`, `base_link` have no `<inertial>`, and the prismatic joints have no `<limit>` tag. MuJoCo rejects moving bodies with mass/inertia below mjMINVAL, and the URDF parser rejects limit-less prismatic joints.
- **Root cause**: The chain emulates a planar floating base for ROS tooling; it was never meant for physics.
- **Fix**: Removed the 3 planar joints and the `world`/`intermediate_1`/`intermediate_2` links from the working copy. `base_link` is now the root and MuJoCo welds it to the world (decision: visualization-focused model; wheels/casters stay fixed as generated).
- **Status**: fixed

### Issue 3: MuJoCo URDF compiler ignores `<mimic>` — gripper four-bar would dangle
- **Stage**: Pre-conversion analysis
- **Symptom**: 5 Robotiq 85 joints are `<mimic>`-slaved to `robotiq_85_left_knuckle_joint`; MuJoCo's URDF importer drops mimic tags, leaving the fingers as free dangling joints.
- **Root cause**: Mimic is a URDF-only concept; MJCF expresses it as `<equality><joint>` constraints.
- **Fix**: 5 `<equality><joint>` entries in `raw_inputs`, polycoef second term = the URDF mimic multiplier (right_knuckle −1, left_inner_knuckle +1, right_inner_knuckle −1, left_finger_tip −1, right_finger_tip +1), `solimp="0.95 0.99 0.001" solref="0.005 1"`.
- **Status**: fixed

### Issue 4: Compile failed — `unknown default class name 'visual'`
- **Stage**: Headless compile (`mujoco-compile scene.xml`)
- **Symptom**: `XML Error: unknown default class name 'visual'  Element 'geom', line 0`
- **Root cause**: The converter's obj2mjcf pass emits geoms with `class="visual"`, but the defining `<default>` block is expected to come from `raw_inputs` (the demo `test_inputs.xml` carries it); the first `mujoco_inputs.xml` omitted it.
- **Fix**: Added the `<default>` block with `visual`/`collision` classes to `raw_inputs` and re-ran the converter.
- **Status**: fixed

### Issue 5: Compile failed — flat meshes: `mesh 'short_frame_0' has coplanar vertices, cannot compute convex hull`
- **Stage**: Headless compile
- **Symptom**: `Error: mesh 'short_frame_0' has coplanar vertices, cannot compute convex hull. Consider using a primitive geom type (plane or thin box) instead`
- **Root cause**: Some source visual meshes contain perfectly flat sub-meshes (zero extent along one axis): `short_frame_0`, `short_frame_9` (ROX frame) and `nanoscan_3_5` (SICK lidar). MuJoCo computes a convex hull for every mesh asset and rejects degenerate (2D) geometry.
- **Fix**: `thicken_flat_meshes.py` — for every generated OBJ in `assets/full/` with a bounding-box extent < 1e-6, concatenate a copy translated 0.1 mm along the degenerate axis. Visually identical; hull becomes 3D. **Re-running the converter regenerates (re-flattens) the OBJs — re-run the script after every conversion.**
- **Status**: worked around (script kept in `description/`)

### Issue 6: Actuated shoulder_pan frozen — hidden collidable geoms jam the arm
- **Stage**: Actuator addition
- **Symptom**: With `ctrl` commanded, all joints tracked except `ur5eshoulder_pan_joint` (stayed at 0). `d.ncon` = 16 despite "all collisions stripped"; `qfrc_constraint` on the pan DOF (−126 Nm) cancelled the clamped actuator torque.
- **Root cause**: obj2mjcf leaves some generated sub-mesh geoms without a `class` attribute. Those fall back to MuJoCo's global geom default (`contype=1 conaffinity=1`) and are collidable — the shoulder meshes jammed against the base meshes (fused into the world body). The "no contact geoms" assumption held only for geoms carrying `class="visual"`.
- **Fix**: Added a global geom default `<geom contype="0" conaffinity="0"/>` to the `<default>` block in `raw_inputs` — visual-only model, nothing collides. `ncon` = 0 afterwards.
- **Status**: fixed

### Issue 7: Wrist limit-cycle oscillation — actuator damping saturates with the P term
- **Stage**: Actuator addition
- **Symptom**: After fixing Issue 6, the sim never settled: `qvel` norm ~30, `ur5ewrist_3_joint` spinning at ~30 rad/s with its actuator force pegged at ±28 Nm.
- **Root cause**: URDF `<limit effort>` becomes `jnt_actfrcrange` (28 Nm on the wrists). The position actuator's demand kp·err exceeded the clamp, and its internal damping term (dampratio) is clamped *together* with the P term — so at saturation there is zero effective damping and the joint bang-bangs. The joints themselves had `damping=0`.
- **Fix**: Real joint damping (outside the clamp) + armature via `<modify_element>` processed inputs: arm joints damping 5–10 / armature 0.1, gripper joints damping 0.1. Settles to `qvel` = 0, max tracking error 0.009 rad.
- **Status**: fixed

## Final state

- **Files** (all in `mobile_manipulator_simulation/description/`):
  - `rox_diff_ur5e_mujoco.urdf` — working copy (planar chain removed); original untouched
  - `mujoco_inputs.xml` — defaults, equalities, actuators, damping overrides
  - `scene.xml` — scene wrapper (floor, light, skybox)
  - `thicken_flat_meshes.py` — post-conversion mesh fix (Issue 5)
  - `MJCF_CONVERSION_ISSUES.md` — the live issue log this document consolidates
  - `mujoco_description_formatted.xml` — generated robot model (entry point is `scene.xml`, which includes it)
  - `assets/` — generated meshes/materials

- **How to view** (from `description/`; either works):
  ```bash
  ../../.pixi/envs/default/bin/mujoco-simulate scene.xml
  pixi run python -m mujoco.viewer --mjcf=scene.xml   # WARN about install/setup.bash is harmless
  ```

- **Headless regression check** (from `description/`):
  ```bash
  ../../.pixi/envs/default/bin/python - <<'EOF'
  import mujoco, numpy as np
  m = mujoco.MjModel.from_xml_path('scene.xml')
  d = mujoco.MjData(m)
  target = np.array([0.5, -1.2, 1.0, -0.8, 0.6, 0.3, 0.4])
  d.ctrl[:] = target
  for _ in range(10000): mujoco.mj_step(m, d)
  qpos = np.array([d.qpos[m.jnt_qposadr[m.actuator_trnid[i, 0]]] for i in range(m.nu)])
  assert np.abs(qpos - target).max() < 0.05, qpos
  assert np.linalg.norm(d.qvel) < 0.1
  print("PASS")
  EOF
  ```

- **Accepted limitations**: casters fixed and visual-only (an anti-tip skid box stands in for them); collision geometry limited to the two wheel spheres + skid box (re-enable per-geom `contype`/`conaffinity` if more contact physics is ever needed). For a future full ros2_control sim: regenerate the URDF from xacro with `include_arm_ros2_control:=true include_gripper_ros2_control:=true` (or hand-add the block per the `mobile_base.urdf` demo) — that block binds controllers to these MJCF actuators at runtime; the actuators themselves stay defined here.

## Post-conversion step: floating base for mujoco_ros2_control

**Required after every converter re-run** (like `thicken_flat_meshes.py`) — the converter
welds the URDF root, so this is a hand-edit of `mujoco_description_formatted.xml`:

1. Wrap the entire `<worldbody>` content in a `<body name="base_link" pos="0 0 0.002">`
   containing, before the existing content:
   - `<freejoint name="floating_base_joint"/>` — name matches mujoco_ros2_control's
     `odom_free_joint_name` default, so odometry publishes on
     `/simulator/floating_base_state` automatically.
   - `<inertial pos="0 0 0.2" mass="90.0" diaginertia="3.0 3.0 5.0"/>` — the fused base
     geoms lost their URDF mass at `mj_saveLastXML` time.
   - `<geom name="base_support" type="box" size="0.30 0.24 0.025" pos="0 0 0.027"
     contype="0" conaffinity="1" condim="1" rgba="1 0 0 0" group="3"/>` — frictionless
     anti-tip skid standing in for the (visual-only) casters. Its bottom sits 2 mm above
     the wheel-contact plane, so the driven wheels carry the weight; under pitch (arm
     reaction forces, braking) a box edge touches and slides frictionlessly.
2. `scene.xml`'s floor geom carries `condim="1"` too.

### Driven wheels for diff_drive_controller

Also required after every converter re-run **until** the URDF is re-exported with
`joint_type:=continuous` (the rox xacro supports it — then the converter emits the wheel
bodies/joints itself and only the collision spheres need re-adding):

3. Carve the fused wheel visual geoms (at `0 ±0.317 0.075`) out of `base_link` into
   `wheel_left_link` / `wheel_right_link` child bodies: hinge `axis="0 1 0"`
   `damping="1" armature="0.05"`, `mass="5.0"`, plus an invisible collision sphere
   `size="0.075" contype="0" conaffinity="1" condim="3" friction="1 0.005 0.0001"`
   (mirrors the URDF collision sphere; floor-only contact like base_support). A
   `condim="3"` wheel against the `condim="1"` floor is fine — contact condim is the
   max, friction the elementwise max; the instability above only strikes with *zero*
   friction coefficients at `condim="3"`.
4. The `<velocity>` wheel actuators (`kv="50" ctrlrange="-30 30"`) come from
   `mujoco_inputs.xml` and map 1:1 to the ros2_control velocity command interfaces.

Regression check for all of the above: `pixi run python check_stability.py`
(settle + drive/traction assertions).

   **Use `condim="1"` for frictionless, never `friction="0 0 0"` with the default
   `condim="3"`:** zero friction coefficients degenerate the pyramidal friction cone and
   the default Newton solver explodes within seconds (verified: the whole robot launched
   itself off the floor). `condim="1"` removes the friction rows entirely.

#!/usr/bin/env python3
"""Regenerate docs/demo.gif: replay a recorded run offscreen in MuJoCo.

Recipe (from the repo root):

    # 1. Sim, headless
    pixi run ros2 launch mobile_manipulator_simulation mujoco_sim.launch.py \
        headless:=true rviz:=false
    # 2. Recorder (separate terminal), Ctrl+C once the motion finishes
    pixi run python scripts/record_states.py /tmp/states.jsonl
    # 3. Trajectory (separate terminal; WBDD_RRD_PATH suppresses the Rerun viewer)
    cd external_packages/whole_body_differential_drive_trajectory_generation
    WBDD_RRD_PATH=/tmp/wbdd.rrd cargo run --release -- assets/sim_config_l_path.yaml
    # 4. Render + assemble
    MUJOCO_GL=egl pixi run python scripts/render_replay.py /tmp/states.jsonl /tmp/frames
    ffmpeg -y -framerate 20 -i /tmp/frames/frame_%04d.png \
        -vf "split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4" \
        -loop 0 docs/demo.gif

The replay is kinematic: joints come from /joint_states, the base free joint
from the odom->base_link TF composed with the spawn pose (odometry starts at
zero, so world = spawn * odom). Desired EE path (green) is the L from
sim_config_l_path.yaml; the actual EE trace (red) is the grasp_link site.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np
import PIL.Image

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(REPO, "mobile_manipulator_simulation", "description", "scene.xml")
# Robot spawn pose in the world frame (MJCF base_link body pos/quat; matches
# world_from_odom in the generator's sim_config_l_path.yaml).
SPAWN_XY = (0.650723, 1.084676)
SPAWN_YAW = -math.pi / 2
FPS = 20
SPEEDUP = 2.0  # sim seconds per gif second
W, H = 960, 540

states_path, out_dir = sys.argv[1], sys.argv[2]
os.makedirs(out_dir, exist_ok=True)

js, tf = [], []
with open(states_path) as f:
    for line in f:
        rec = json.loads(line)
        (js if rec["type"] == "js" else tf).append(rec)

js.sort(key=lambda r: r["t"])
tf.sort(key=lambda r: r["t"])
tf_t = np.array([r["t"] for r in tf])
tf_x = np.array([r["xyz"][0] for r in tf])
tf_y = np.array([r["xyz"][1] for r in tf])
tf_yaw = np.unwrap(np.array(
    [2.0 * math.atan2(r["quat"][3], r["quat"][0]) for r in tf]))

# Trim to the motion window (where the odom pose changes), padded 1 s.
moving = np.hypot(np.diff(tf_x), np.diff(tf_y)) + np.abs(np.diff(tf_yaw)) > 1e-5
idx = np.where(moving)[0]
t0 = tf_t[idx[0]] - 1.0
t1 = tf_t[idx[-1]] + 1.0
print(f"motion window: {t0:.1f} .. {t1:.1f} s")

js_t = np.array([r["t"] for r in js])

spec = mujoco.MjSpec.from_file(SCENE)
spec.visual.global_.offwidth = W
spec.visual.global_.offheight = H
model = spec.compile()
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

base_qadr = model.jnt_qposadr[
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")]
z0 = data.qpos[base_qadr + 2]

# Hinge joint name -> qpos address, for every recorded joint present in the model.
joint_adr = {}
for name in js[0]["names"]:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid >= 0:
        joint_adr[name] = model.jnt_qposadr[jid]

cy, sy = math.cos(SPAWN_YAW), math.sin(SPAWN_YAW)


def base_world(t):
    x = np.interp(t, tf_t, tf_x)
    y = np.interp(t, tf_t, tf_y)
    yaw = np.interp(t, tf_t, tf_yaw)
    wx = SPAWN_XY[0] + cy * x - sy * y
    wy = SPAWN_XY[1] + sy * x + cy * y
    return wx, wy, SPAWN_YAW + yaw


cam = mujoco.MjvCamera()
cam.lookat[:] = [0.1, 0.0, 0.4]
cam.distance = 5.2
cam.azimuth = 150
cam.elevation = -30

renderer = mujoco.Renderer(model, height=H, width=W)
ee_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_link")

# Desired EE path: the L from sim_config_l_path.yaml, at z = 1.
desired = [np.array(p) for p in
           [(1.0, 1.0, 1.0), (1.0, -1.0, 1.0), (-1.0, -1.0, 1.0)]]

GREEN = np.array([0.1, 0.8, 0.2, 1.0], dtype=np.float32)
RED = np.array([0.9, 0.15, 0.15, 1.0], dtype=np.float32)


def add_line(scene, p0, p1, rgba, width):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        np.zeros(3), np.zeros(3), np.zeros(9), rgba)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width,
                         np.asarray(p0, dtype=np.float64),
                         np.asarray(p1, dtype=np.float64))
    scene.ngeom += 1


times = np.arange(t0, t1, SPEEDUP / FPS)
js_pos = {n: np.array([r["pos"][r["names"].index(n)] for r in js]) for n in joint_adr}

actual = []
for i, t in enumerate(times):
    for name, adr in joint_adr.items():
        data.qpos[adr] = np.interp(t, js_t, js_pos[name])
    wx, wy, wyaw = base_world(t)
    data.qpos[base_qadr:base_qadr + 3] = [wx, wy, z0]
    data.qpos[base_qadr + 3:base_qadr + 7] = [
        math.cos(wyaw / 2), 0.0, 0.0, math.sin(wyaw / 2)]
    mujoco.mj_forward(model, data)
    actual.append(data.site_xpos[ee_site].copy())
    renderer.update_scene(data, camera=cam)
    for p0, p1 in zip(desired, desired[1:]):
        add_line(renderer.scene, p0, p1, GREEN, width=0.008)
    # Slightly wider than the desired line so the executed portion reads red.
    for p0, p1 in zip(actual, actual[1:]):
        add_line(renderer.scene, p0, p1, RED, width=0.011)
    PIL.Image.fromarray(renderer.render()).save(f"{out_dir}/frame_{i:04d}.png")

print(f"wrote {len(times)} frames to {out_dir}")

#!/usr/bin/env python3
"""Self-check: scene compiles, the base rests stably on its wheels, and the
velocity-actuated wheels have traction (commanded spin drives the base forward).

Run after any converter re-run or MJCF edit:  pixi run python check_stability.py
"""
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("scene.xml")
d = mujoco.MjData(m)
while d.time < 5.0:
    mujoco.mj_step(m, d)
    assert np.abs(d.qvel).max() < 50, f"instability at t={d.time:.2f}"
z = d.xpos[m.body("base_link").id][2]
assert abs(z) < 0.01, f"base did not settle on floor: z={z}"
for j in range(m.njnt):
    name = m.joint(j).name
    if "ur5e" in name:
        q = float(d.qpos[m.joint(j).qposadr[0]])
        assert abs(q) < 0.15, f"arm joint drifted: {name}={q}"
assert np.abs(d.qvel).max() < 0.01, "robot still moving after 5 s"

# Drive check: 5 rad/s on both wheels for 2 s of sim => ~0.075*5*2 = 0.75 m forward.
x0 = d.xpos[m.body("base_link").id][0].copy()
d.ctrl[m.actuator("wheel_left_joint").id] = 5.0
d.ctrl[m.actuator("wheel_right_joint").id] = 5.0
t0 = d.time
while d.time < t0 + 2.0:
    mujoco.mj_step(m, d)
    assert np.abs(d.qvel).max() < 50, f"instability while driving at t={d.time:.2f}"
dx = d.xpos[m.body("base_link").id][0] - x0
quat = d.xquat[m.body("base_link").id]
yaw = np.arctan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                 1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
assert 0.5 < dx < 1.0, f"wheels have no traction: dx={dx:.3f} (expected ~0.75)"
assert abs(yaw) < 0.15, f"base veered while driving straight: yaw={yaw:.3f}"
print(f"OK: base z={z:.5f}, all joints settled, drive dx={dx:.3f} m yaw={yaw:.3f} rad")

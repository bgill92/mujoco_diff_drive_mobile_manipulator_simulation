#!/usr/bin/env python3
"""Self-check: scene compiles and the floating base rests stably on the floor.

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
print(f"OK: base z={z:.5f}, all joints settled")

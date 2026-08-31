#!/usr/bin/env python3
"""Thicken degenerate (flat) OBJ meshes so MuJoCo can compute convex hulls.

Run after every converter run, from the description/ dir:
    .pixi python: <ws>/.pixi/envs/default/bin/python thicken_flat_meshes.py
See MJCF_CONVERSION_ISSUES.md, Issue 5.
"""
import glob

import numpy as np
import trimesh

for path in sorted(glob.glob("assets/full/**/*.obj", recursive=True)):
    mesh = trimesh.load(path, force="mesh", process=False)
    extents = mesh.extents
    if extents is None or min(extents) >= 1e-6:
        continue
    # ponytail: 0.1 mm offset copy along the flat axis; visually identical, hull becomes 3D.
    offset = np.zeros(3)
    offset[int(np.argmin(extents))] = 1e-4
    shifted = mesh.copy()
    shifted.apply_translation(offset)
    trimesh.util.concatenate([mesh, shifted]).export(path)
    print(f"thickened {path} (extents {extents})")

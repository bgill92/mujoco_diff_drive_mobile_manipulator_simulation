# AGENTS.md

## ROS 2 commands MUST run in the Pixi environment

This workspace (`pixi_ros2_jazzy`) uses [Pixi](https://pixi.sh) with RoboStack
for all ROS 2 Jazzy dependencies. There is no system ROS installation to rely
on. Any ROS 2 command (`ros2 ...`, `colcon ...`, launch files, nodes, tools)
MUST be executed through the pixi environment:

```bash
# Prefix any command
pixi run ros2 run my_pkg my_node
pixi run ros2 launch my_pkg my_launch.py
pixi run ros2 topic list
pixi run ros2 topic echo /cmd_vel

# Or open an activated shell
pixi shell
ros2 topic list
```

- Do NOT run `ros2`, `colcon`, or source `install/setup.bash` directly in a
  bare shell — they will not be found or will miss the workspace overlay.
- Do NOT source `/opt/ros/...` or any external ROS workspace. Pixi manages the
  entire ROS environment.
- After the first build, `[activation] scripts = ["install/setup.bash"]` in
  `pixi.toml` automatically sources the built workspace on activation, so no
  manual sourcing is ever needed.

## Common commands

Use the pixi tasks defined in `pixi.toml`:

```bash
pixi install            # install/refresh dependencies
pixi run build          # colcon build
pixi run test           # colcon test
pixi run clean          # remove build/ install/ log/
```

## Adding dependencies

- No `rosdep` — it is not supported in a Pixi environment (it calls
  `conda install`). Add missing packages explicitly:
  ```bash
  pixi add ros-jazzy-<package>       # conda/RoboStack package
  pixi add --pypi <package>          # PyPI package
  ```
- After changing `package.xml` files, regenerate the manifest:
  ```bash
  pixi ros init --distro jazzy
  ```

## Notes

- Channels: `https://prefix.dev/robostack-jazzy`, `https://prefix.dev/conda-forge`.
  RoboStack package names are prefixed `ros-jazzy-` (e.g. `ros-jazzy-ros2cli`).
- Keep `pixi.lock` committed; run `pixi install` after manifest changes.
- Long-running commands (launch files, rviz, simulations) are fine via
  `pixi run` or inside `pixi shell`; the environment is identical either way.

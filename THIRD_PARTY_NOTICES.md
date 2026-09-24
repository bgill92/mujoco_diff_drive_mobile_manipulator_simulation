# Third-party notices

This repository is licensed under the Apache License 2.0 (see `LICENSE`).
The files listed below are copied or derived from other projects and remain
under their original licenses and copyright holders.

Git submodules under `external_packages/` carry their own licenses:

- `external_packages/mujoco_ros2_control` — Apache-2.0 (see its `LICENSE`).
- `external_packages/rox` — BSD, Neobotix GmbH (fork of
  https://github.com/neobotix/rox).
- `external_packages/whole_body_differential_drive_trajectory_generation` —
  see that repository.

## ros2_controllers `diff_drive_controller` (PAL Robotics)

Files:

- `diff_drive_position_controller/include/diff_drive_position_controller/odometry.hpp`
- `diff_drive_position_controller/src/odometry.cpp`

Copied from https://github.com/ros-controls/ros2_controllers
(`diff_drive_controller`, Jazzy) with the namespace renamed and the Humble
compatibility shims removed. Copyright 2020 PAL Robotics S.L. Licensed under
the Apache License, Version 2.0; the full text is in `LICENSE`.

## Neobotix ROX description meshes

Files: `mobile_manipulator_simulation/description/meshes/rox_description/*.dae`
and the derived MuJoCo assets under
`mobile_manipulator_simulation/description/assets/full/{base,cabinet_ac,diff_caster,diff_wheel,nanoscan_3,short_frame}*` (directories, `.obj` and `mtl_*` siblings).

Source: https://github.com/neobotix/rox (`rox_description`). Copyright
Neobotix GmbH. Distributed under the BSD license as declared in the upstream
`package.xml`; the upstream repository ships no separate license file.

## Universal Robots UR5e description meshes

Files: `mobile_manipulator_simulation/description/meshes/ur_description/ur5e/**`
and the derived MuJoCo assets under
`mobile_manipulator_simulation/description/assets/full/{shoulder,upperarm,forearm,wrist1,wrist2,wrist3}*` (directories, `.obj` and `mtl_*` siblings).

Source: https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
(`ur_description`). Copyright Universal Robots A/S and contributors.
Licensed under BSD-3-Clause. (Only the UR8LONG, UR15, UR18, UR20 and UR30
meshes upstream fall under Universal Robots' "Terms and Conditions for Use of
Graphical Documentation"; none of those are used here.)

```
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

   * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

## Robotiq 2F-85 description meshes (PickNik Robotics)

Files: `mobile_manipulator_simulation/description/meshes/robotiq_description/**`
and the derived MuJoCo assets under
`mobile_manipulator_simulation/description/assets/full/{robotiq_base,left_*,right_*,ur_to_robotiq_adapter}*` (directories, `.obj` and `mtl_*` siblings).

Source: https://github.com/PickNikRobotics/ros2_robotiq_gripper
(`robotiq_description`).

```
BSD 3-Clause License

Copyright (c) 2022, PickNik Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

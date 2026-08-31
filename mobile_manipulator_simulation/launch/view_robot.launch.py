"""View the rox + UR5e + Robotiq mobile manipulator in RViz.

    ros2 launch mobile_manipulator_simulation view_robot.launch.py
    ros2 launch mobile_manipulator_simulation view_robot.launch.py use_joint_state_publisher_gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "mobile_manipulator_simulation"


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    urdf_path = os.path.join(share, "description", "rox_diff_ur5e.urdf")
    rviz_config = os.path.join(share, "rviz", "view_robot.rviz")

    # Plain URDF, no xacro args to expand, so read it once at launch time
    # instead of shelling out to a Command substitution.
    with open(urdf_path) as urdf_file:
        robot_description = urdf_file.read()

    use_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="false",
            choices=["true", "false"],
            description="Run joint_state_publisher_gui (sliders) instead of the headless joint_state_publisher",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Launch RViz",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])

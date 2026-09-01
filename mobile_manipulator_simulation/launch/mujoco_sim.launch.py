"""MuJoCo + ros2_control simulation of the ROX + UR5e + Robotiq mobile manipulator.

Usage:
    ros2 launch mobile_manipulator_simulation mujoco_sim.launch.py
    ros2 launch mobile_manipulator_simulation mujoco_sim.launch.py headless:=true rviz:=false

Move the arm:
    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: \
        [ur5eshoulder_pan_joint, ur5eshoulder_lift_joint, ur5eelbow_joint, \
        ur5ewrist_1_joint, ur5ewrist_2_joint, ur5ewrist_3_joint], points: \
        [{positions: [0.5, -1.2, 1.0, -0.8, 0.6, 0.3], time_from_start: {sec: 4}}]}}"

Move the gripper:
    ros2 action send_goal /gripper_controller/gripper_cmd \
        control_msgs/action/GripperCommand "{command: {position: 0.4, max_effort: 10.0}}"

Drive the base (diff_drive_controller; zero stamp is auto-stamped):
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
        "{twist: {linear: {x: 0.3}, angular: {z: 0.2}}}"
Keyboard teleop: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

# The MJCF and its 75 MB of mesh assets deliberately stay in the source tree
# (not installed to share/ -- see mobile_manipulator_simulation/CMakeLists.txt), so the
# default is the source-tree path; override mujoco_model:=... if the tree moves.
DEFAULT_MUJOCO_MODEL = (
    "/home/bilal/Projects/robotics/mujoco_diff_drive_mobile_manipulator_simulation/"
    "mobile_manipulator_simulation/description/scene.xml"
)


def generate_launch_description():
    pkg_share = FindPackageShare("mobile_manipulator_simulation")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "description", "rox_diff_ur5e_control.urdf.xacro"]),
            " headless:=",
            LaunchConfiguration("headless"),
            " mujoco_model:=",
            LaunchConfiguration("mujoco_model"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        # MuJoCo runs inside this patched controller_manager node
        # (mujoco_ros2_control's ros2_control_node, not controller_manager's).
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file),
            ],
            on_exit=Shutdown(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "mujoco_sim.rviz"])],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]

    for controller in [
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "gripper_controller",
        "diff_drive_controller",
    ]:
        args = [controller, "--param-file", controllers_file]
        if controller == "diff_drive_controller":
            # Keep the public topic name: the controller subscribes ~/cmd_vel (TwistStamped).
            args += ["--controller-ros-args", "-r /diff_drive_controller/cmd_vel:=/cmd_vel"]
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=args,
                output="both",
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false",
                                  description="Run simulation without the MuJoCo window"),
            DeclareLaunchArgument("rviz", default_value="true",
                                  description="Launch RViz"),
            DeclareLaunchArgument("mujoco_model", default_value=DEFAULT_MUJOCO_MODEL,
                                  description="Path to the MJCF scene"),
        ]
        + nodes
    )

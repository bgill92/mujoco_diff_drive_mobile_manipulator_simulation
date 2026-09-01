#!/usr/bin/env python3
"""Broadcast odom -> base_link TF from the simulator's floating-base odometry.

mujoco_ros2_control publishes nav_msgs/Odometry on /simulator/floating_base_state
for the freejoint, but no TF. robot_state_publisher covers everything below
base_link; this node supplies the missing root transform.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.declare_parameter("child_frame", "base_link")
        self.child_frame = self.get_parameter("child_frame").value
        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, "/simulator/floating_base_state", self.on_odom, 10)

    def on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header
        # ponytail: ignore msg.child_frame_id (MJCF body name); URDF root is base_link.
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

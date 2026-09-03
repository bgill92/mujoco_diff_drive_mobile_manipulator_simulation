#!/usr/bin/env python3
"""Publish the end-effector's actual path for RViz.

Samples the odom -> ee_frame transform from TF and accumulates it into a
nav_msgs/Path on /actual_ee_path, for visual comparison against the desired
path the trajectory generator publishes on /wbdd/desired_ee_path.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener


class EePathPublisher(Node):
    def __init__(self):
        super().__init__("ee_path_publisher")
        self.declare_parameter("ee_frame", "grasp_link")
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("min_translation", 0.005)  # m between samples
        self.declare_parameter("max_poses", 5000)

        self.ee_frame = self.get_parameter("ee_frame").value
        self.path_frame = self.get_parameter("path_frame").value
        self.min_translation = self.get_parameter("min_translation").value
        self.max_poses = self.get_parameter("max_poses").value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # Transient local so an RViz started mid-run still gets the path.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, "/actual_ee_path", qos)

        self.path = Path()
        self.path.header.frame_id = self.path_frame
        rate = self.get_parameter("rate_hz").value
        self.timer = self.create_timer(1.0 / rate, self.sample)

    def sample(self):
        try:
            transform = self.buffer.lookup_transform(
                self.path_frame, self.ee_frame, rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001 -- TF raises several lookup errors; all mean "not yet".
            return

        translation = transform.transform.translation
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            moved = math.dist(
                (translation.x, translation.y, translation.z),
                (last.x, last.y, last.z),
            )
            if moved < self.min_translation:
                return

        pose = PoseStamped()
        pose.header.frame_id = self.path_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = transform.transform.rotation
        self.path.poses.append(pose)
        # ponytail: drop-oldest cap; ring buffer if anyone runs this for hours.
        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)

        self.path.header.stamp = transform.header.stamp
        self.publisher.publish(self.path)


def main():
    rclpy.init()
    node = EePathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Record /joint_states and the odom->base_link TF to a JSONL file.

One half of the docs/demo.gif pipeline (see scripts/render_replay.py for the
other half and the full recipe). Run this while the sim executes a trajectory,
then Ctrl+C:

    pixi run python scripts/record_states.py /tmp/states.jsonl
"""
import json
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


class Recorder(Node):
    def __init__(self, out_path):
        super().__init__("state_recorder")
        self.f = open(out_path, "w")
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)

    def stamp(self, s):
        return s.sec + s.nanosec * 1e-9

    def on_js(self, msg):
        self.f.write(json.dumps({
            "type": "js",
            "t": self.stamp(msg.header.stamp),
            "names": list(msg.name),
            "pos": list(msg.position),
        }) + "\n")

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                q = tr.transform.rotation
                p = tr.transform.translation
                self.f.write(json.dumps({
                    "type": "tf",
                    "t": self.stamp(tr.header.stamp),
                    "xyz": [p.x, p.y, p.z],
                    "quat": [q.w, q.x, q.y, q.z],
                }) + "\n")


def main():
    rclpy.init()
    node = Recorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()


if __name__ == "__main__":
    main()

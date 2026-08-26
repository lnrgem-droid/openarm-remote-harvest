#!/usr/bin/env python3
"""Publish left+right EE poses on one topic (default 5 Hz)."""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

_CHILDREN = ("openarm_left_hand", "openarm_right_hand")


class EePosePublisher(Node):
    def __init__(self):
        super().__init__("ee_pose_publisher")
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("topic", "ee_poses")
        rate = max(float(self.get_parameter("rate").value), 0.1)
        self._parent = str(self.get_parameter("parent_frame").value)
        topic = str(self.get_parameter("topic").value)
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._pub = self.create_publisher(TFMessage, topic, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ee_pose_publisher {rate:g} Hz → /{topic.lstrip('/')} "
            f"({self._parent} → left/right_hand)"
        )

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        out = TFMessage()
        for child in _CHILDREN:
            try:
                t = self._buf.lookup_transform(
                    self._parent,
                    child,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:
                continue
            ts = TransformStamped()
            ts.header.stamp = stamp
            ts.header.frame_id = self._parent
            ts.child_frame_id = child
            ts.transform = t.transform
            out.transforms.append(ts)
        if out.transforms:
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EePosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

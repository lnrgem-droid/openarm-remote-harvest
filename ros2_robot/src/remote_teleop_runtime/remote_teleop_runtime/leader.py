from __future__ import annotations

import argparse
import secrets
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from remote_teleop_protocol import ActionCommand, FollowerState, PacketError, decode_message, encode_action
from .common import ACTION_PORT, GRIPPER_MAX_RAD, GRIPPER_OPEN_M, STATE_PORT


class LeaderGateway(Node):
    def __init__(self, peer: str, rate: float):
        super().__init__("remote_teleop_leader")
        self.peer = peer
        self.period = 1.0 / rate
        self.session = secrets.randbits(64) or 1
        self.sequence = 0
        self.axes = [0.0] * 16
        self.have_state = False
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", STATE_PORT))
        self.sock.setblocking(False)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 1)
        self.create_timer(self.period, self.tick)
        self.sent = self.received = self.invalid = 0
        self.last_log = time.monotonic()
        self.get_logger().info(
            f"RIGHT ONLY leader session={self.session} -> {peer}:{ACTION_PORT} at {rate:.0f} Hz")

    def on_joint_state(self, msg: JointState):
        values = dict(zip(msg.name, msg.position))
        names = [f"openarm_right_joint{i}" for i in range(1, 8)]
        if not all(name in values for name in names):
            return
        right = [float(values[name]) for name in names]
        finger = float(values.get("openarm_right_finger_joint1", 0.0))
        gripper_rad = max(0.0, min(1.0, finger / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
        with self.lock:
            self.axes[8:15] = right
            self.axes[15] = gripper_rad
            self.have_state = True

    def tick(self):
        with self.lock:
            if not self.have_state:
                return
            axes = tuple(self.axes)
        self.sequence += 1
        now_ns = time.monotonic_ns()
        msg = ActionCommand(self.session, self.sequence, now_ns, axes, 100_000_000)
        self.sock.sendto(encode_action(msg), (self.peer, ACTION_PORT))
        self.sent += 1
        try:
            while True:
                data, _ = self.sock.recvfrom(2048)
                if isinstance(decode_message(data), FollowerState):
                    self.received += 1
        except BlockingIOError:
            pass
        except PacketError:
            self.invalid += 1
        if time.monotonic() - self.last_log >= 2.0:
            self.get_logger().info(
                f"session={self.session} sent={self.sent} state_rx={self.received} invalid={self.invalid}")
            self.last_log = time.monotonic()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--peer", default="192.168.50.2")
    parser.add_argument("--rate", type=float, default=100.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LeaderGateway(args.peer, args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

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
from .common import (ACTION_PORT, GRIPPER_MAX_RAD, GRIPPER_OPEN_M,
                     LEADER_JOINT_STATES_TOPIC, LEADER_LEFT_COMMAND_TOPIC,
                     LEADER_LEFT_FORCE_FEEDBACK_TOPIC, LEADER_RIGHT_COMMAND_TOPIC,
                     LEADER_RIGHT_FORCE_FEEDBACK_TOPIC, STATE_PORT)


class LeaderGateway(Node):
    def __init__(self, peer: str, rate: float, enable_left: bool):
        super().__init__("remote_teleop_leader")
        self.peer = peer
        self.period = 1.0 / rate
        self.session = secrets.randbits(64) or 1
        self.sequence = 0
        self.axes = [0.0] * 16
        self.enable_left = enable_left
        self.have_right = False
        self.have_left = False
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", STATE_PORT))
        self.sock.setblocking(False)
        self.create_subscription(JointState, LEADER_JOINT_STATES_TOPIC, self.on_joint_state, 1)
        self.right_force_pub = self.create_publisher(JointState, LEADER_RIGHT_FORCE_FEEDBACK_TOPIC, 1)
        self.left_force_pub = (self.create_publisher(JointState, LEADER_LEFT_FORCE_FEEDBACK_TOPIC, 1)
                               if enable_left else None)
        self.right_position_feedback_pub = self.create_publisher(
            JointState, LEADER_RIGHT_COMMAND_TOPIC, 1)
        self.left_position_feedback_pub = (self.create_publisher(
            JointState, LEADER_LEFT_COMMAND_TOPIC, 1) if enable_left else None)
        # Reference pair and short action history let us distinguish genuine
        # follower contact error from ordinary network transport delay.
        self.bilateral_reference = None
        self.action_history = {}
        self.create_timer(self.period, self.tick)
        self.sent = self.received = self.invalid = 0
        self.last_log = time.monotonic()
        arms = "right + left" if enable_left else "right only"
        self.get_logger().info(
            f"{arms} leader session={self.session} -> {peer}:{ACTION_PORT} at {rate:.0f} Hz; "
            "bounded haptic stream available")

    def publish_force_feedback(self, state: FollowerState):
        # A follower effort opposing leader motion must feel resistive at the
        # leader, hence the negative sign.  The leader controller applies its
        # own low-pass, clamp, scale, and 50 ms stale-data decay.
        if state.control_state.name != "RUNNING" or state.fault_bits:
            efforts = [0.0] * 8
            left_efforts = [0.0] * 8
        else:
            efforts = [-float(value) for value in state.efforts[8:16]]
            left_efforts = [-float(value) for value in state.efforts[0:8]]
        right = JointState(); right.effort = efforts
        self.right_force_pub.publish(right)
        if self.enable_left:
            left = JointState(); left.effort = left_efforts
            self.left_force_pub.publish(left)

    def publish_position_feedback(self, state: FollowerState):
        """Reflect only follower tracking/contact error, never transport lag."""
        if state.control_state.name != "RUNNING" or state.fault_bits:
            self.bilateral_reference = None
            return
        with self.lock:
            leader_now = tuple(self.axes)
        applied = self.action_history.get(state.applied_action_sequence)
        if applied is None:
            return
        if self.bilateral_reference is None:
            self.bilateral_reference = (applied, tuple(state.positions))
        leader_zero, follower_zero = self.bilateral_reference
        follower_desired = [follower_ref + (leader_command - leader_ref) for
                            follower_ref, leader_command, leader_ref in zip(
                                follower_zero, applied, leader_zero)]
        # In free space actual≈desired and target≈leader_now, so the master
        # remains light.  When contact prevents follower motion, the signed
        # error shifts the master target against the operator's motion.
        correction_scale = 0.50
        target = [leader + correction_scale * (actual - desired) for
                  leader, actual, desired in zip(leader_now, state.positions, follower_desired)]
        right = JointState()
        right.name = [f"openarm_right_joint{i}" for i in range(1, 8)] + ["openarm_right_gripper"]
        right.position = target[8:15] + [max(0.0, min(1.0, target[15] / GRIPPER_MAX_RAD))]
        self.right_position_feedback_pub.publish(right)
        if self.enable_left:
            left = JointState()
            left.name = [f"openarm_left_joint{i}" for i in range(1, 8)] + ["openarm_left_gripper"]
            left.position = target[0:7] + [max(0.0, min(1.0, target[7] / GRIPPER_MAX_RAD))]
            self.left_position_feedback_pub.publish(left)

    def on_joint_state(self, msg: JointState):
        values = dict(zip(msg.name, msg.position))
        with self.lock:
            right_names = [f"openarm_right_joint{i}" for i in range(1, 8)]
            if all(name in values for name in right_names):
                self.axes[8:15] = [float(values[name]) for name in right_names]
                finger = float(values.get("openarm_right_finger_joint1", 0.0))
                self.axes[15] = max(0.0, min(1.0, finger / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
                self.have_right = True
            if self.enable_left:
                left_names = [f"openarm_left_joint{i}" for i in range(1, 8)]
                if all(name in values for name in left_names):
                    self.axes[0:7] = [float(values[name]) for name in left_names]
                    finger = float(values.get("openarm_left_finger_joint1", 0.0))
                    self.axes[7] = max(0.0, min(1.0, finger / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
                    self.have_left = True

    def tick(self):
        with self.lock:
            if not self.have_right or (self.enable_left and not self.have_left):
                return
            axes = tuple(self.axes)
        self.sequence += 1
        now_ns = time.monotonic_ns()
        msg = ActionCommand(self.session, self.sequence, now_ns, axes, 100_000_000)
        self.sock.sendto(encode_action(msg), (self.peer, ACTION_PORT))
        self.action_history[self.sequence] = axes
        if len(self.action_history) > 256:
            del self.action_history[min(self.action_history)]
        self.sent += 1
        try:
            while True:
                data, _ = self.sock.recvfrom(2048)
                state = decode_message(data)
                if isinstance(state, FollowerState):
                    self.received += 1
                    self.publish_force_feedback(state)
                    self.publish_position_feedback(state)
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
    parser.add_argument("--enable-left", action="store_true",
                        help="send left-arm axes as well as the default right arm")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LeaderGateway(args.peer, args.rate, args.enable_left)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

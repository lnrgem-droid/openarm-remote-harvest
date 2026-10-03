from __future__ import annotations

import argparse
import math
import secrets
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from remote_teleop_protocol import ActionCommand, FollowerState, PacketError, decode_message, encode_action
from remote_teleop_protocol.protocol import (
    COLLECTION_ACK_LEFT_CAPABLE, COLLECTION_ACK_LEFT_SHIFT, COLLECTION_STATE_FLAGS,
)
from .common import (ACTION_PORT, GRIPPER_MAX_RAD, GRIPPER_OPEN_M,
                     haptic_desired_axes, gripper_contact_reference,
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
        self.right_rx = self.left_rx = 0.0
        self.collection_ack = 0
        # None means the physical controller did not explicitly advertise left
        # collection support. Never turn an old controller's missing mode into
        # an acknowledgement that the left master is free and can be driven.
        self.left_collection_ack = None
        self.left_collection_mode_invalid = False
        self.state_session = None
        self.state_sequence = 0
        self.state_rx = 0.0
        self.return_requested = False
        self.left_return_requested = False
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", STATE_PORT))
        self.sock.setblocking(False)
        self.create_subscription(JointState, LEADER_JOINT_STATES_TOPIC, self.on_joint_state, 1)
        self.right_force_pub = self.create_publisher(JointState, LEADER_RIGHT_FORCE_FEEDBACK_TOPIC, 1)
        self.return_pub = self.create_publisher(JointState, "/leader/right_arm/collection_return", 1)
        self.left_return_pub = (self.create_publisher(
            JointState, "/leader/left_arm/collection_return", 1) if enable_left else None)
        self.left_force_pub = (self.create_publisher(JointState, LEADER_LEFT_FORCE_FEEDBACK_TOPIC, 1)
                               if enable_left else None)
        self.right_position_feedback_pub = self.create_publisher(
            JointState, LEADER_RIGHT_COMMAND_TOPIC, 1)
        self.left_position_feedback_pub = (self.create_publisher(
            JointState, LEADER_LEFT_COMMAND_TOPIC, 1) if enable_left else None)
        # Reference pair and short action history let us distinguish genuine
        # follower contact error from ordinary network transport delay.
        self.haptic_reference = None
        self.collection_flags = 0
        self.action_history = {}
        self.create_timer(self.period, self.tick)
        self.sent = self.received = self.invalid = 0
        self.last_log = time.monotonic()
        arms = "right + left" if enable_left else "right only"
        self.get_logger().info(
            f"{arms} leader session={self.session} -> {peer}:{ACTION_PORT} at {rate:.0f} Hz; "
            "bounded haptic stream available")

    def publish_force_feedback(self, state: FollowerState):
        """Publish a passive virtual spring from follower tracking error.

        The original single-computer bilateral loop exchanges fresh state at
        500 Hz.  Across Ethernet, raw follower motor torque and delayed pose
        both create false "gravity" resistance.  Instead we compare follower
        actual pose with the pose commanded by the *applied* action sequence.
        Free motion produces zero torque; contact produces a bounded opposing
        torque on the corresponding leader joint.
        """
        if state.collection_flags != self.collection_flags:
            # Detaching one arm must not reset the other arm's contact cue.
            applied = self.action_history.get(state.applied_action_sequence)
            if self.haptic_reference is not None and applied is not None:
                leader_ref, follower_ref = map(list, self.haptic_reference)
                for bit, offset in ((1, 0), (2, 8)):
                    if (state.collection_flags ^ self.collection_flags) & bit:
                        leader_ref[offset:offset+8] = applied[offset:offset+8]
                        follower_ref[offset:offset+8] = state.positions[offset:offset+8]
                self.haptic_reference = (tuple(leader_ref), tuple(follower_ref))
            self.collection_flags = state.collection_flags
        if state.control_state.name != "RUNNING" or state.fault_bits:
            efforts = [0.0] * 8
            left_efforts = [0.0] * 8
            self.haptic_reference = None
        else:
            applied = self.action_history.get(state.applied_action_sequence)
            if applied is None:
                return
            if self.haptic_reference is None:
                self.haptic_reference = (applied, tuple(state.positions))
            leader_zero, follower_zero = self.haptic_reference
            desired = haptic_desired_axes(leader_zero, follower_zero, applied)
            error = [actual - target for actual, target in zip(state.positions, desired)]
            # Nm/rad.  These gains are intentionally much lower than the
            # position-controller gains: this is an impedance cue, not a
            # delayed pose-servo.  Gripper is separately capped by the leader
            # controller to protect the printed fingers.
            gains = [2.5, 2.5, 2.0, 2.0, 0.8, 0.8, 0.6, 0.8,
                     2.5, 2.5, 2.0, 2.0, 0.8, 0.8, 0.6, 0.8]
            virtual_torque = [gain * delta for gain, delta in zip(gains, error)]
            efforts = virtual_torque[8:16]
            left_efforts = virtual_torque[0:8]
        if state.collection_flags & 1:
            left_efforts = [0.0] * 8
        if state.collection_flags & 2:
            efforts = [0.0] * 8
        right = JointState(); right.effort = efforts
        # Private haptic topic: one position value is the follower gripper's
        # actual motor angle. It is NOT an eight-axis joint_command message.
        right.position = gripper_contact_reference(state, 'right')
        self.right_force_pub.publish(right)
        if self.enable_left:
            left = JointState(); left.effort = left_efforts
            left.position = gripper_contact_reference(state, 'left')
            self.left_force_pub.publish(left)

    def on_joint_state(self, msg: JointState):
        values = dict(zip(msg.name, msg.position))
        with self.lock:
            for side, offset in (("right", 8), ("left", 0)):
                if side == "left" and not self.enable_left:
                    continue
                names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
                names.append(f"openarm_{side}_finger_joint1")
                # A partial message must not refresh old axes, including the
                # gripper, or claim a new physical collection acknowledgement.
                try:
                    axes = [float(values[name]) for name in names]
                except (KeyError, TypeError, ValueError, OverflowError):
                    continue
                if not all(math.isfinite(value) for value in axes):
                    continue
                axes[7] = max(0.0, min(1.0, axes[7] / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
                self.axes[offset:offset + 8] = axes
                setattr(self, f"have_{side}", True)
                setattr(self, f"{side}_rx", time.monotonic())
                mode = values.get(f"openarm_{side}_collection_mode")
                # JointState encodes modes as doubles. Fractional/unknown
                # values are not valid modes and must not be truncated to 0.
                mode = int(mode) if mode in (0, 1, 2) else None
                if side == "right":
                    self.collection_ack = mode if mode is not None else 2
                else:
                    self.left_collection_ack = mode
                    self.left_collection_mode_invalid = (
                        f"openarm_{side}_collection_mode" in values and mode is None)

    def collection_acknowledgement(self):
        ack = self.collection_ack
        if self.enable_left and self.left_collection_ack is not None:
            ack |= COLLECTION_ACK_LEFT_CAPABLE | (self.left_collection_ack << COLLECTION_ACK_LEFT_SHIFT)
        return ack

    def publish_return(self, state):
        # An explicit peer packet is required to release a latched servo. A
        # missing packet instead leaves the local 100 ms controller watchdog
        # holding the measured pose; it never resumes a stored trajectory.
        flags = state.collection_flags
        if flags not in COLLECTION_STATE_FLAGS:
            return  # includes simultaneous left/right servo requests
        if state.control_state.name != "RUNNING" or state.fault_bits:
            return  # local timeout stops any trajectory and latches hold
        obs_age = state.sender_monotonic_ns - state.obs_timestamp_ns
        if not 0 <= obs_age <= 100_000_000:
            return
        with self.lock:
            now = time.monotonic()
            if (not self.have_right or not 0 <= now - self.right_rx <= .1 or
                (self.enable_left and
                 (not self.have_left or not 0 <= now - self.left_rx <= .1))):
                return
            right_ack, left_ack = self.collection_ack, self.left_collection_ack
            left_mode_invalid = self.left_collection_mode_invalid
        requested_right, requested_left = bool(flags & 4), bool(flags & 8)
        if requested_right or requested_left:
            try:
                target = [float(value) for value in state.leader_return_target]
            except (TypeError, ValueError, OverflowError):
                return
            if len(target) != 8 or not all(math.isfinite(value) for value in target):
                return
            if requested_left:
                if (not self.enable_left or self.left_return_pub is None or
                    left_ack not in (0, 1) or right_ack != 0 or self.return_requested):
                    return
                publisher = self.left_return_pub
                self.left_return_requested = True
            else:
                if (right_ack not in (0, 1) or left_ack not in (None, 0) or
                    left_mode_invalid or self.left_return_requested):
                    return
                publisher = self.return_pub
                self.return_requested = True
            message = JointState()
            message.position = target
            publisher.publish(message)
            return
        # No servo bit is an explicit release, independently acknowledged by
        # each side. An opposite target never implicitly releases a busy side.
        if self.return_requested or right_ack:
            self.return_pub.publish(JointState())
            self.return_requested = False
        if (self.enable_left and self.left_return_pub is not None and
            left_ack is not None and (self.left_return_requested or left_ack)):
            self.left_return_pub.publish(JointState())
            self.left_return_requested = False

    def tick(self):
        with self.lock:
            if not self.have_right or (self.enable_left and not self.have_left):
                return
            axes = tuple(self.axes)
            if (time.monotonic()-self.right_rx > 0.10 or
                (self.enable_left and time.monotonic()-self.left_rx > 0.10)):
                return  # don't turn stale physical feedback into fresh actions
            collection_ack = self.collection_acknowledgement()
        self.sequence += 1
        now_ns = time.monotonic_ns()
        msg = ActionCommand(self.session, self.sequence, now_ns, axes, 100_000_000,
                            collection_ack)
        self.sock.sendto(encode_action(msg), (self.peer, ACTION_PORT))
        self.action_history[self.sequence] = axes
        if len(self.action_history) > 256:
            del self.action_history[min(self.action_history)]
        self.sent += 1
        try:
            while True:
                data, peer = self.sock.recvfrom(2048)
                if peer[0] != self.peer:
                    continue
                state = decode_message(data)
                if isinstance(state, FollowerState):
                    if state.applied_action_session_id != self.session:
                        continue
                    if self.state_session is None:
                        self.state_session = state.session_id
                    if state.session_id != self.state_session or state.sequence <= self.state_sequence:
                        continue
                    # Echo must refer to an action sent in the last 100 ms,
                    # not just a valid but delayed packet from this session.
                    if self.sequence-state.applied_action_sequence > max(1, int(.1/self.period)):
                        continue
                    self.state_sequence = state.sequence
                    self.state_rx = time.monotonic()
                    self.received += 1
                    self.publish_return(state)
                    self.publish_force_feedback(state)
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

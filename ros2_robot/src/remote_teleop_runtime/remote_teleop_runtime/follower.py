from __future__ import annotations

import argparse
import json
import os
import secrets
import socket
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from remote_teleop_follower_safety.local_protocol import encode_heartbeat
from remote_teleop_follower_safety.watchdog import ControllerHeartbeat
from remote_teleop_protocol import (ActionCommand, ControlState, FollowerState, FaultBits,
                                    PacketError, SequenceTracker, decode_message, encode_state)
from .common import (ACTION_PORT, FOLLOWER_DISABLE_SERVICE, FOLLOWER_JOINT_STATES_TOPIC,
                     FOLLOWER_LEFT_COMMAND_TOPIC, FOLLOWER_RIGHT_COMMAND_TOPIC,
                     GRIPPER_MAX_RAD, GRIPPER_OPEN_M,
                     RUNTIME_SOCKET, STATE_PORT, UnixDatagramClient, WATCHDOG_SOCKET,
                     safety_command)


class FollowerGateway(Node):
    def __init__(self, enable_left: bool, rate: float):
        super().__init__("remote_teleop_follower")
        self.session = secrets.randbits(64) or 1
        self.sequence = self.hb_sequence = 0
        self.tracker = SequenceTracker()
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("0.0.0.0", ACTION_PORT)); self.udp.setblocking(False)
        self.command = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        if os.path.exists(RUNTIME_SOCKET): os.unlink(RUNTIME_SOCKET)
        self.command.bind(RUNTIME_SOCKET); os.chmod(RUNTIME_SOCKET, 0o600); self.command.setblocking(False)
        self.watchdog = UnixDatagramClient(WATCHDOG_SOCKET)
        self.enable_left = enable_left
        self.right_publisher = self.create_publisher(JointState, FOLLOWER_RIGHT_COMMAND_TOPIC, 1)
        self.left_publisher = (self.create_publisher(JointState, FOLLOWER_LEFT_COMMAND_TOPIC, 1)
                               if enable_left else None)
        self.create_subscription(JointState, FOLLOWER_JOINT_STATES_TOPIC, self.on_joint_state, 1)
        self.disable_client = self.create_client(Trigger, FOLLOWER_DISABLE_SERVICE)
        self.lock = threading.Lock()
        self.positions = [0.0] * 16; self.velocities = [0.0] * 16; self.efforts = [0.0] * 16
        self.have_right_feedback = False; self.have_left_feedback = not enable_left
        self.last_feedback_ns = 0
        self.latest_action = None; self.last_action_rx_ns = 0; self.peer_ip = None
        # RUN uses a captured leader/follower pair as its reference. This avoids
        # an alignment tolerance becoming a sudden absolute-position correction
        # when remote control is enabled: follower_target = follower_zero +
        # (leader_now - leader_zero).
        self.run_leader_right = self.run_follower_right = None
        self.run_leader_left = self.run_follower_left = None
        self.run_leader_gripper = self.run_follower_gripper = None
        self.run_leader_left_gripper = self.run_follower_left_gripper = None
        self.last_target_right = self.last_target_left = None
        self.safety = {"state": "ALIGNING", "fault_bits": 0, "reason": "waiting for watchdog"}
        self.safety_rx_ns = 0; self.align_since_ns = 0
        self.applied_session = self.applied_sequence = self.action_timestamp_ns = 0
        self.create_timer(1.0 / rate, self.tick)
        arms = "right + left" if enable_left else "right only"
        self.get_logger().info(
            f"{arms} follower listening UDP :{ACTION_PORT} at {rate:.0f} Hz")

    @property
    def have_feedback(self):
        return self.have_right_feedback and self.have_left_feedback

    def on_joint_state(self, msg: JointState):
        values = dict(zip(msg.name, msg.position)); velocities = dict(zip(msg.name, msg.velocity))
        efforts = dict(zip(msg.name, msg.effort))
        now = time.monotonic_ns()
        with self.lock:
            right_names = [f"openarm_right_joint{i}" for i in range(1, 8)]
            if all(n in values for n in right_names):
                self.positions[8:15] = [float(values[n]) for n in right_names]
                self.velocities[8:15] = [float(velocities.get(n, 0.0)) for n in right_names]
                self.efforts[8:15] = [float(efforts.get(n, 0.0)) for n in right_names]
                finger = float(values.get("openarm_right_finger_joint1", 0.0))
                self.positions[15] = max(0.0, min(1.0, finger / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
                self.efforts[15] = float(efforts.get("openarm_right_finger_joint1", 0.0))
                self.have_right_feedback = True
            if self.enable_left:
                left_names = [f"openarm_left_joint{i}" for i in range(1, 8)]
                if all(n in values for n in left_names):
                    self.positions[0:7] = [float(values[n]) for n in left_names]
                    self.velocities[0:7] = [float(velocities.get(n, 0.0)) for n in left_names]
                    self.efforts[0:7] = [float(efforts.get(n, 0.0)) for n in left_names]
                    finger = float(values.get("openarm_left_finger_joint1", 0.0))
                    self.positions[7] = max(0.0, min(1.0, finger / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
                    self.efforts[7] = float(efforts.get("openarm_left_finger_joint1", 0.0))
                    self.have_left_feedback = True
            if self.have_feedback:
                self.last_feedback_ns = now

    def receive_action(self, now_ns):
        try:
            while True:
                data, peer = self.udp.recvfrom(2048)
                msg = decode_message(data)
                if isinstance(msg, ActionCommand) and self.tracker.accept(msg.session_id, msg.sequence):
                    self.latest_action = msg; self.last_action_rx_ns = now_ns; self.peer_ip = peer[0]
        except BlockingIOError: pass
        except PacketError as exc: self.get_logger().warning(f"invalid UDP action: {exc}")

    def heartbeat(self, now_ns):
        # Let the watchdog's startup grace handle controller/CAN initialization;
        # reporting can_ok=false during the first feedback cycle would latch a
        # false CAN fault that cannot recover automatically.
        if not self.have_feedback:
            return
        self.hb_sequence += 1
        # `last_control_cycle_ns` describes this gateway's 100 Hz safety cycle,
        # not the asynchronous 100 Hz ROS feedback callback.  Using the latter
        # made ordinary DDS scheduling jitter look like a stalled controller.
        # Feedback freshness is instead checked explicitly as CAN health with a
        # 250 ms bound, still well below the time needed to use stale data.
        feedback_fresh = now_ns - self.last_feedback_ns < 250_000_000
        hb = ControllerHeartbeat(self.session, self.hb_sequence, now_ns,
            now_ns, self.last_action_rx_ns,
            self.latest_action.session_id if self.latest_action else 0,
            self.have_feedback and feedback_fresh, False, 0)
        reply = self.watchdog.exchange(encode_heartbeat(hb), 0.004)
        if reply is not None:
            self.safety = reply; self.safety_rx_ns = now_ns

    def publish_target(self, now_ns):
        if not self.have_feedback: return
        running = self.safety.get("state") == "RUNNING" and now_ns - self.safety_rx_ns < 100_000_000
        fresh = self.latest_action is not None and now_ns - self.last_action_rx_ns <= 100_000_000
        right_desired = list(self.positions[8:15]); right_gripper_rad = self.positions[15]
        left_desired = list(self.positions[0:7]); left_gripper_rad = self.positions[7]
        if running and fresh and self.run_leader_right is not None:
            remote = self.latest_action
            requested = [follower_zero + (leader_now - leader_zero)
                         for follower_zero, leader_now, leader_zero in zip(
                             self.run_follower_right, remote.right_arm, self.run_leader_right)]
            right_desired = [max(q - 0.20, min(q + 0.20, target))
                             for q, target in zip(right_desired, requested)]
            # A gripper command is an opening fraction, not a shared arm pose.
            # Use the leader's absolute opening so a closed leader always
            # closes the follower even if their initial finger openings differ.
            right_gripper_rad = max(GRIPPER_MAX_RAD, min(0.0, remote.right_gripper))
            if self.enable_left and self.run_leader_left is not None:
                requested = [follower_zero + (leader_now - leader_zero)
                             for follower_zero, leader_now, leader_zero in zip(
                                 self.run_follower_left, remote.left_arm, self.run_leader_left)]
                left_desired = [max(q - 0.20, min(q + 0.20, target))
                                for q, target in zip(left_desired, requested)]
                left_gripper_rad = max(GRIPPER_MAX_RAD, min(0.0, remote.left_gripper))
            self.applied_session = remote.session_id; self.applied_sequence = remote.sequence
            self.action_timestamp_ns = now_ns
        self.last_target_right = tuple(right_desired)
        right_msg = JointState(); right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.name = [f"openarm_right_joint{i}" for i in range(1, 8)] + ["openarm_right_gripper"]
        right_msg.position = right_desired + [max(0.0, min(1.0, right_gripper_rad / GRIPPER_MAX_RAD))]
        self.right_publisher.publish(right_msg)
        if self.enable_left:
            self.last_target_left = tuple(left_desired)
            left_msg = JointState(); left_msg.header.stamp = self.get_clock().now().to_msg()
            left_msg.name = [f"openarm_left_joint{i}" for i in range(1, 8)] + ["openarm_left_gripper"]
            left_msg.position = left_desired + [max(0.0, min(1.0, left_gripper_rad / GRIPPER_MAX_RAD))]
            self.left_publisher.publish(left_msg)

    def handle_commands(self, now_ns):
        try:
            while True:
                raw, peer = self.command.recvfrom(4096); request = json.loads(raw.decode())
                cmd = request.get("command"); response = self.runtime_status()
                if cmd == "status": pass
                elif cmd == "align":
                    if not self.latest_action or not self.have_feedback: raise RuntimeError("missing action or feedback")
                    errors = [abs(a-b) for a,b in zip(self.latest_action.right_arm, self.positions[8:15])]
                    if self.enable_left:
                        errors.extend(abs(a-b) for a,b in zip(self.latest_action.left_arm, self.positions[0:7]))
                    error = max(errors)
                    if error > 0.15: raise RuntimeError(f"alignment error {error:.3f} rad > 0.15")
                    if now_ns - self.align_since_ns < 1_000_000_000: raise RuntimeError("alignment must remain <=0.15 rad for 1 second")
                    response = safety_command(self.watchdog, "alignment_complete", leader_session_id=self.latest_action.session_id) or {}
                elif cmd == "run":
                    if not self.latest_action: raise RuntimeError("no leader session")
                    response = safety_command(self.watchdog, "request_run", leader_session_id=self.latest_action.session_id) or {}
                    if response.get("state") == "RUNNING":
                        self.run_leader_right = tuple(self.latest_action.right_arm)
                        self.run_follower_right = tuple(self.positions[8:15])
                        self.run_leader_gripper = self.latest_action.right_gripper
                        self.run_follower_gripper = self.positions[15]
                        if self.enable_left:
                            self.run_leader_left = tuple(self.latest_action.left_arm)
                            self.run_follower_left = tuple(self.positions[0:7])
                            self.run_leader_left_gripper = self.latest_action.left_gripper
                            self.run_follower_left_gripper = self.positions[7]
                elif cmd == "hold":
                    response = safety_command(self.watchdog, "hold") or {}
                    self.clear_run_reference()
                elif cmd == "reset":
                    self.tracker.reset(); self.latest_action = None; self.last_action_rx_ns = 0
                    self.clear_run_reference()
                    response = safety_command(self.watchdog, "reset", estop_released=True) or {}
                elif cmd == "disable":
                    safety_command(self.watchdog, "estop")
                    if not self.disable_client.wait_for_service(timeout_sec=1.0): raise RuntimeError("disable service unavailable")
                    self.disable_client.call_async(Trigger.Request()); response = {"state":"E_STOP", "reason":"disable requested"}
                else: raise RuntimeError("command must be status/align/run/hold/reset/disable")
                self.command.sendto(json.dumps(response).encode(), peer)
        except BlockingIOError: pass
        except Exception as exc:
            if 'peer' in locals(): self.command.sendto(json.dumps({"error": str(exc)}).encode(), peer)

    def clear_run_reference(self):
        self.run_leader_right = self.run_follower_right = None
        self.run_leader_left = self.run_follower_left = None
        self.run_leader_gripper = self.run_follower_gripper = None
        self.run_leader_left_gripper = self.run_follower_left_gripper = None
        self.last_target_right = self.last_target_left = None

    def runtime_status(self):
        response = dict(self.safety)
        if self.last_target_right is not None:
            errors = [target - actual for target, actual in zip(
                self.last_target_right, self.positions[8:15])]
            response["right_target_rad"] = list(self.last_target_right)
            response["right_actual_rad"] = list(self.positions[8:15])
            response["right_tracking_error_rad"] = errors
            response["max_tracking_error_rad"] = max(abs(error) for error in errors)
        if self.latest_action is not None:
            response["leader_right_rad"] = list(self.latest_action.right_arm)
            if self.enable_left:
                response["leader_left_rad"] = list(self.latest_action.left_arm)
            response["action_age_ms"] = (time.monotonic_ns() - self.last_action_rx_ns) / 1_000_000
        if self.last_target_left is not None:
            errors = [target - actual for target, actual in zip(
                self.last_target_left, self.positions[0:7])]
            response["left_target_rad"] = list(self.last_target_left)
            response["left_actual_rad"] = list(self.positions[0:7])
            response["left_tracking_error_rad"] = errors
            response["left_max_tracking_error_rad"] = max(abs(error) for error in errors)
        response["relative_follow_reference_captured"] = self.run_leader_right is not None and (not self.enable_left or self.run_leader_left is not None)
        response["enabled_arms"] = ["right", *( ["left"] if self.enable_left else [])]
        return response

    def send_state(self, now_ns):
        if not self.peer_ip or not self.have_feedback: return
        self.sequence += 1
        state_name = self.safety.get("state", "ALIGNING")
        state = FollowerState(self.session, self.sequence, now_ns, self.last_feedback_ns,
            self.action_timestamp_ns, self.applied_session, self.applied_sequence,
            ControlState[state_name], FaultBits(int(self.safety.get("fault_bits", 0))),
            tuple(self.positions), tuple(self.velocities), tuple(self.efforts))
        self.udp.sendto(encode_state(state), (self.peer_ip, STATE_PORT))

    def tick(self):
        now_ns = time.monotonic_ns(); self.receive_action(now_ns)
        if self.latest_action and self.have_feedback:
            errors = [abs(a-b) for a,b in zip(self.latest_action.right_arm, self.positions[8:15])]
            if self.enable_left:
                errors.extend(abs(a-b) for a,b in zip(self.latest_action.left_arm, self.positions[0:7]))
            aligned = max(errors) <= 0.15
            if aligned and not self.align_since_ns: self.align_since_ns = now_ns
            elif not aligned: self.align_since_ns = 0
        self.heartbeat(now_ns)
        self.handle_commands(now_ns); self.publish_target(now_ns); self.send_state(now_ns)

    def close(self):
        self.udp.close(); self.command.close(); self.watchdog.close()
        if os.path.exists(RUNTIME_SOCKET): os.unlink(RUNTIME_SOCKET)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-left", action="store_true",
                        help="control the left follower arm as well as the default right arm")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="UDP receive / command publish rate in Hz")
    args, ros_args = parser.parse_known_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")
    rclpy.init(args=ros_args); node = FollowerGateway(args.enable_left, args.rate)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

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
from .common import (ACTION_PORT, GRIPPER_MAX_RAD, GRIPPER_OPEN_M, RUNTIME_SOCKET,
                     STATE_PORT, UnixDatagramClient, WATCHDOG_SOCKET, safety_command)


class FollowerGateway(Node):
    def __init__(self):
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
        self.publisher = self.create_publisher(JointState, "/right_arm/joint_command", 1)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 1)
        self.disable_client = self.create_client(Trigger, "/openarm_gravity_pd/disable")
        self.lock = threading.Lock()
        self.positions = [0.0] * 16; self.velocities = [0.0] * 16
        self.have_feedback = False; self.last_feedback_ns = 0
        self.latest_action = None; self.last_action_rx_ns = 0; self.peer_ip = None
        # RUN uses a captured leader/follower pair as its reference. This avoids
        # an alignment tolerance becoming a sudden absolute-position correction
        # when remote control is enabled: follower_target = follower_zero +
        # (leader_now - leader_zero).
        self.run_leader_right = None; self.run_follower_right = None
        self.run_leader_gripper = None; self.run_follower_gripper = None
        self.last_target_right = None
        self.safety = {"state": "ALIGNING", "fault_bits": 0, "reason": "waiting for watchdog"}
        self.safety_rx_ns = 0; self.align_since_ns = 0
        self.applied_session = self.applied_sequence = self.action_timestamp_ns = 0
        self.create_timer(0.01, self.tick)
        self.get_logger().info(f"RIGHT ONLY follower listening UDP :{ACTION_PORT}; left arm disabled")

    def on_joint_state(self, msg: JointState):
        values = dict(zip(msg.name, msg.position)); velocities = dict(zip(msg.name, msg.velocity))
        names = [f"openarm_right_joint{i}" for i in range(1, 8)]
        if not all(n in values for n in names): return
        now = time.monotonic_ns()
        with self.lock:
            self.positions[8:15] = [float(values[n]) for n in names]
            self.velocities[8:15] = [float(velocities.get(n, 0.0)) for n in names]
            finger = float(values.get("openarm_right_finger_joint1", 0.0))
            self.positions[15] = max(0.0, min(1.0, finger / GRIPPER_OPEN_M)) * GRIPPER_MAX_RAD
            self.last_feedback_ns = now; self.have_feedback = True

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
        desired = list(self.positions[8:15]); gripper_rad = self.positions[15]
        if running and fresh and self.run_leader_right is not None:
            remote = self.latest_action
            requested = [follower_zero + (leader_now - leader_zero)
                         for follower_zero, leader_now, leader_zero in zip(
                             self.run_follower_right, remote.right_arm, self.run_leader_right)]
            desired = [max(q - 0.20, min(q + 0.20, target))
                       for q, target in zip(desired, requested)]
            gripper_rad = max(GRIPPER_MAX_RAD, min(0.0,
                self.run_follower_gripper +
                (remote.right_gripper - self.run_leader_gripper)))
            self.applied_session = remote.session_id; self.applied_sequence = remote.sequence
            self.action_timestamp_ns = now_ns
        self.last_target_right = tuple(desired)
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"openarm_right_joint{i}" for i in range(1, 8)] + ["openarm_right_gripper"]
        msg.position = desired + [max(0.0, min(1.0, gripper_rad / GRIPPER_MAX_RAD))]
        self.publisher.publish(msg)

    def handle_commands(self, now_ns):
        try:
            while True:
                raw, peer = self.command.recvfrom(4096); request = json.loads(raw.decode())
                cmd = request.get("command"); response = self.runtime_status()
                if cmd == "status": pass
                elif cmd == "align":
                    if not self.latest_action or not self.have_feedback: raise RuntimeError("missing action or feedback")
                    error = max(abs(a-b) for a,b in zip(self.latest_action.right_arm, self.positions[8:15]))
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
        self.run_leader_gripper = self.run_follower_gripper = None
        self.last_target_right = None

    def runtime_status(self):
        response = dict(self.safety)
        if self.last_target_right is not None:
            response["max_tracking_error_rad"] = max(
                abs(target - actual) for target, actual in zip(
                    self.last_target_right, self.positions[8:15]))
        response["relative_follow_reference_captured"] = self.run_leader_right is not None
        return response

    def send_state(self, now_ns):
        if not self.peer_ip or not self.have_feedback: return
        self.sequence += 1
        state_name = self.safety.get("state", "ALIGNING")
        state = FollowerState(self.session, self.sequence, now_ns, self.last_feedback_ns,
            self.action_timestamp_ns, self.applied_session, self.applied_sequence,
            ControlState[state_name], FaultBits(int(self.safety.get("fault_bits", 0))),
            tuple(self.positions), tuple(self.velocities))
        self.udp.sendto(encode_state(state), (self.peer_ip, STATE_PORT))

    def tick(self):
        now_ns = time.monotonic_ns(); self.receive_action(now_ns)
        if self.latest_action and self.have_feedback:
            aligned = max(abs(a-b) for a,b in zip(
                self.latest_action.right_arm, self.positions[8:15])) <= 0.15
            if aligned and not self.align_since_ns: self.align_since_ns = now_ns
            elif not aligned: self.align_since_ns = 0
        self.heartbeat(now_ns)
        self.handle_commands(now_ns); self.publish_target(now_ns); self.send_state(now_ns)

    def close(self):
        self.udp.close(); self.command.close(); self.watchdog.close()
        if os.path.exists(RUNTIME_SOCKET): os.unlink(RUNTIME_SOCKET)


def main():
    argparse.ArgumentParser().parse_known_args()
    rclpy.init(); node = FollowerGateway()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

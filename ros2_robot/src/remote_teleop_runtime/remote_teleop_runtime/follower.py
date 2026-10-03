from __future__ import annotations

import argparse
import errno
import json
import math
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
from .collection_motion import CollectionMotion, COMMANDS
from .follower_io import FollowerIOWorker

from remote_teleop_follower_safety.local_protocol import encode_heartbeat
from remote_teleop_follower_safety.watchdog import ControllerHeartbeat
from remote_teleop_protocol import (ActionCommand, ControlState, FollowerState, FaultBits,
                                    PacketError, SequenceTracker, decode_message, encode_state)
from .common import (ACTION_PORT, FOLLOWER_DISABLE_SERVICE, FOLLOWER_JOINT_STATES_TOPIC,
                     FOLLOWER_LEFT_COMMAND_TOPIC, FOLLOWER_RIGHT_COMMAND_TOPIC,
                     GRIPPER_MAX_RAD, GRIPPER_OPEN_M,
                     RUNTIME_SOCKET, STATE_PORT, UnixDatagramClient, WATCHDOG_SOCKET,
                     safety_command)

MAX_TRACKING_ERROR_RAD = 0.20
# A short ROS callback delay is not proof that the physical CAN bus failed.
# Pause new tracking targets promptly when feedback is stale, but only report a
# CAN fault after a sustained loss. This avoids latching FAULT during the
# transient CPU/I/O spike at the start of six-stream RGB-D recording.
FEEDBACK_CONTROL_TIMEOUT_NS = 150_000_000
FEEDBACK_CAN_FAULT_TIMEOUT_NS = 1_000_000_000


def bounded_tracking_target(actual, requested):
    """Limit instantaneous following error without limiting total joint travel."""
    return [max(q - MAX_TRACKING_ERROR_RAD, min(q + MAX_TRACKING_ERROR_RAD, target))
            for q, target in zip(actual, requested)]


class FollowerGateway(Node):
    def __init__(self, enable_left: bool, rate: float):
        super().__init__("remote_teleop_follower")
        self.session = secrets.randbits(64) or 1
        self.sequence = self.hb_sequence = 0
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
        self.last_right_feedback_ns = self.last_left_feedback_ns = 0
        self.latest_action = None; self.last_action_rx_ns = 0; self.peer_ip = None
        # Capture both sides at the RUN boundary for diagnostics and a
        # deterministic transition. During RUNNING the follower uses the
        # leader's absolute joint positions, matching the original single-host
        # OpenArm bilateral AdminThread behavior.
        self.run_leader_right = self.run_follower_right = None
        self.run_leader_left = self.run_follower_left = None
        self.run_leader_gripper = self.run_follower_gripper = None
        self.run_leader_left_gripper = self.run_follower_left_gripper = None
        # A non-RUNNING state is a position hold, not a continuously moving
        # snapshot of the measured pose.  Latch once and keep publishing it so
        # gravity or a light disturbance cannot walk a wrist away from the
        # startup alignment pose while the network handshake is in progress.
        self.hold_right = self.hold_left = None
        self.hold_right_gripper = self.hold_left_gripper = None
        self.command_was_running = False
        self.last_target_right = self.last_target_left = None
        self.applied_axes = None
        self.collection = CollectionMotion(os.path.expanduser(
            "~/openarm-rgbd-runtime/right-start-pose.json"))
        self.safety = {"state": "ALIGNING", "fault_bits": 0, "reason": "waiting for watchdog"}
        self.safety_rx_ns = 0; self.align_since_ns = 0
        self.applied_session = self.applied_sequence = self.action_timestamp_ns = 0
        self.command_reply_dropped = 0
        self.stop_intent = None
        self.io = FollowerIOWorker(self.udp, self.watchdog, self.session, rate=min(rate, 100.), kernel_timestamps=True)
        self.create_timer(1.0 / rate, self.tick)
        # Operator/status requests cannot monopolize the control timer through
        # an unbounded socket drain. Their watchdog replies use separate sockets.
        self.create_timer(max(.01, 1.0 / rate), self.command_tick)
        self.io.start()
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
                self.last_right_feedback_ns = now
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
                    self.last_left_feedback_ns = now
            if self.have_feedback:
                self.last_feedback_ns = min(self.last_right_feedback_ns, self.last_left_feedback_ns) if self.enable_left else self.last_right_feedback_ns

    def heartbeat(self, now_ns):
        """Consume I/O evidence; this ROS callback never sends a heartbeat."""
        snapshot = self.io.snapshot()
        action = snapshot["action"]
        self.latest_action = action.action if action else None
        self.last_action_rx_ns = action.safe_rx_ns if action else 0
        self.peer_ip = action.peer_ip if action else None
        previous_state = self.safety.get("state")
        self.safety = snapshot["safety"]
        self.safety_rx_ns = snapshot["safety_rx_ns"]
        self.stop_intent = snapshot.get("stop_intent")
        if previous_state == "RUNNING" and self.safety.get("state") != "RUNNING":
            self.capture_hold_reference()

    def watchdog_command(self, command, **fields):
        """A one-use reply address plus an epoch fence prevents late ACK reuse."""
        self.io.begin_command()
        sent_ns = time.monotonic_ns()
        reply = None
        client = None
        try:
            client = UnixDatagramClient(WATCHDOG_SOCKET)
            reply = safety_command(client, command, **fields)
        finally:
            if client is not None:
                client.close()
            accepted = self.io.finish_command(command, reply, sent_ns)
            self.heartbeat(time.monotonic_ns())
        if not accepted:
            raise RuntimeError("watchdog command acknowledgement was not confirmed; command is not replayed")
        return reply

    def capture_hold_reference(self):
        """Freeze the last published target, preserving load/gripper preload."""
        if not self.have_feedback:
            return

        def finite_snapshot(values, size):
            try:
                snapshot = tuple(values)
                if len(snapshot) == size and all(math.isfinite(q) for q in snapshot):
                    return snapshot
            except (TypeError, ValueError):
                pass
            return None

        # Capturing actual here discards the position error that supports a
        # loaded arm. The 100 ms freshness gate and the later watchdog FAULT
        # can both call this method: both must retain the same commanded pose,
        # rather than relatch the follower again after it has sagged.
        applied = finite_snapshot(getattr(self, "applied_axes", None), 16)
        for side, start in (("right", 8), ("left", 0)):
            if side == "left" and not self.enable_left:
                continue
            reference = applied[start:start + 8] if applied is not None else None
            if reference is None:
                old_joints = getattr(self, f"hold_{side}", None)
                old_gripper = getattr(self, f"hold_{side}_gripper", None)
                if old_joints is not None:
                    reference = finite_snapshot((*old_joints, old_gripper), 8)
            if reference is None:
                # Initialization has no previously published/latched target.
                reference = finite_snapshot(self.positions[start:start + 8], 8)
            if reference is not None:
                setattr(self, f"hold_{side}", reference[:7])
                setattr(self, f"hold_{side}_gripper", reference[7])
        if self.enable_left:
            if getattr(self, "collection", None) and self.collection.left is not None:
                self.hold_left = self.collection.left[:7]
                self.hold_left_gripper = self.collection.left[7]

    def publish_target(self, now_ns):
        if not self.have_feedback: return False
        if self.hold_right is None or (self.enable_left and self.hold_left is None):
            self.capture_hold_reference()
        feedback_fresh = now_ns - self.last_feedback_ns < FEEDBACK_CONTROL_TIMEOUT_NS
        running = (
            self.safety.get("state") == "RUNNING"
            and not self.safety.get("fault_bits", 0)
            and now_ns - self.safety_rx_ns < 100_000_000
            and feedback_fresh
        )
        fresh = self.latest_action is not None and now_ns - self.last_action_rx_ns <= 100_000_000
        if self.command_was_running and (not running or not fresh):
            self.capture_hold_reference()
        if not (running and fresh):
            self.collection.interrupt(self.positions, "控制许可或反馈中断")
        self.command_was_running = running and fresh
        right_desired = list(self.hold_right); right_gripper_rad = self.hold_right_gripper
        left_desired = (list(self.hold_left) if self.enable_left else list(self.positions[0:7]))
        left_gripper_rad = self.hold_left_gripper if self.enable_left else self.positions[7]
        if running and fresh and self.run_leader_right is not None:
            remote = self.latest_action
            requested = list(remote.right_arm)
            # Bound the target around the *current measured follower pose*.
            # Bounding around the latched startup hold pose would incorrectly
            # restrict the arm to a permanent +/-0.20 rad travel window.
            right_desired = bounded_tracking_target(self.positions[8:15], requested)
            # A gripper command is an opening fraction, not a shared arm pose.
            # Use the leader's absolute opening so a closed leader always
            # closes the follower even if their initial finger openings differ.
            right_gripper_rad = max(GRIPPER_MAX_RAD, min(0.0, remote.right_gripper))
            if self.enable_left and self.run_leader_left is not None:
                requested = list(remote.left_arm)
                left_desired = bounded_tracking_target(self.positions[0:7], requested)
                left_gripper_rad = max(GRIPPER_MAX_RAD, min(0.0, remote.left_gripper))
        selected = self.collection.update(
            self.positions, left_desired + [left_gripper_rad] + right_desired + [right_gripper_rad],
            now_ns / 1e9, running and fresh and self.run_leader_right is not None,
            self.latest_action.axes if self.latest_action else None,
            self.latest_action.collection_ack if self.latest_action else 0)
        left_desired, left_gripper_rad = selected[:7], selected[7]
        right_desired, right_gripper_rad = selected[8:15], selected[15]
        right_msg = JointState(); right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.name = [f"openarm_right_joint{i}" for i in range(1, 8)] + ["openarm_right_gripper"]
        right_msg.position = right_desired + [max(0.0, min(1.0, right_gripper_rad / GRIPPER_MAX_RAD))]
        self.right_publisher.publish(right_msg)
        if self.enable_left:
            left_msg = JointState(); left_msg.header.stamp = self.get_clock().now().to_msg()
            left_msg.name = [f"openarm_left_joint{i}" for i in range(1, 8)] + ["openarm_left_gripper"]
            left_msg.position = left_desired + [max(0.0, min(1.0, left_gripper_rad / GRIPPER_MAX_RAD))]
            self.left_publisher.publish(left_msg)
        # Commit only after every enabled side has actually been published.
        # A partial/failed publish is not a new complete control cycle.
        self.applied_axes = tuple(selected)
        self.last_target_right = tuple(right_desired)
        if self.enable_left:
            self.last_target_left = tuple(left_desired)
        if running and fresh and self.run_leader_right is not None:
            self.applied_session = remote.session_id; self.applied_sequence = remote.sequence
            self.action_timestamp_ns = time.monotonic_ns()
        return True

    def capture_run_reference(self):
        """Capture the relative leader/follower pose at the RUNNING boundary.

        The watchdog state is refreshed asynchronously by the heartbeat.  The
        operator's ``run`` command may therefore be acknowledged one timer
        cycle before the gateway observes RUNNING.  Capturing here, from the
        observed state and current action, makes the transition deterministic
        and prevents a RUNNING-but-static follower after a fresh startup.
        """
        if not self.latest_action or not self.have_feedback:
            return
        self.run_leader_right = tuple(self.latest_action.right_arm)
        self.run_follower_right = tuple(self.positions[8:15])
        self.run_leader_gripper = self.latest_action.right_gripper
        self.run_follower_gripper = self.positions[15]
        if self.enable_left:
            self.run_leader_left = tuple(self.latest_action.left_arm)
            self.run_follower_left = tuple(self.positions[0:7])
            self.run_leader_left_gripper = self.latest_action.left_gripper
            self.run_follower_left_gripper = self.positions[7]

    def handle_commands(self, now_ns):
        try:
            for _ in range(1):
                raw, peer = self.command.recvfrom(4096); request = json.loads(raw.decode())
                now_ns = time.monotonic_ns()
                cmd = request.get("command"); response = self.runtime_status()
                if cmd == "status": pass
                elif cmd in COMMANDS:
                    if not self.enable_left:
                        raise RuntimeError("采集姿态控制需要双臂配置")
                    healthy = (self.safety.get("state") == "RUNNING"
                        and not self.safety.get("fault_bits", 0)
                        and now_ns-self.safety_rx_ns < 100_000_000
                        and now_ns-self.last_feedback_ns < FEEDBACK_CONTROL_TIMEOUT_NS
                        and self.latest_action is not None
                        and now_ns-self.last_action_rx_ns < 100_000_000
                        and self.applied_axes is not None)
                    self.collection.command(cmd, request, self.positions,
                        self.applied_axes or self.positions,
                        self.latest_action.axes if self.latest_action else self.positions,
                        self.velocities, now_ns/1e9, healthy,
                        leader_ack=self.latest_action.collection_ack if self.latest_action else 0)
                    response = self.runtime_status()
                elif cmd == "align":
                    if not self.latest_action or not self.have_feedback: raise RuntimeError("missing action or feedback")
                    differences = [
                        (f"right J{index}", float(leader), float(follower))
                        for index, (leader, follower) in enumerate(
                            zip(self.latest_action.right_arm, self.positions[8:15]), start=1)]
                    if self.enable_left:
                        differences.extend(
                            (f"left J{index}", float(leader), float(follower))
                            for index, (leader, follower) in enumerate(
                                zip(self.latest_action.left_arm, self.positions[0:7]), start=1))
                    joint, leader_value, follower_value = max(
                        differences, key=lambda item: abs(item[1] - item[2]))
                    error = abs(leader_value - follower_value)
                    if error > 0.15:
                        raise RuntimeError(
                            f"alignment error {joint}: {error:.3f} rad > 0.15 "
                            f"(leader={leader_value:.3f}, follower={follower_value:.3f})")
                    if now_ns - self.align_since_ns < 1_000_000_000: raise RuntimeError("alignment must remain <=0.15 rad for 1 second")
                    response = self.watchdog_command("alignment_complete", leader_session_id=self.latest_action.session_id)
                elif cmd == "run":
                    if not self.latest_action: raise RuntimeError("no leader session")
                    response = self.watchdog_command("request_run", leader_session_id=self.latest_action.session_id)
                    if response.get("state") == "RUNNING":
                        self.capture_run_reference()
                elif cmd == "hold":
                    response = self.watchdog_command("hold")
                    self.clear_run_reference()
                elif cmd == "reset":
                    response = self.watchdog_command("reset", estop_released=True)
                    self.latest_action = None; self.last_action_rx_ns = 0
                    self.clear_run_reference()
                elif cmd == "disable":
                    watchdog_error = None
                    try:
                        self.watchdog_command("estop")
                    except Exception as exc:
                        # An unavailable watchdog must never prevent the
                        # operator's explicit hardware-disable request.
                        watchdog_error = str(exc)
                    if not self.disable_client.wait_for_service(timeout_sec=1.0): raise RuntimeError("disable service unavailable")
                    self.disable_client.call_async(Trigger.Request())
                    response = {"state": self.safety.get("state", "UNKNOWN"), "reason": "hardware disable requested",
                                "hardware_disable_requested": True, "watchdog_estop_confirmed": watchdog_error is None}
                    if watchdog_error:
                        response["watchdog_confirmation_error"] = watchdog_error
                else: raise RuntimeError("command must be status/align/run/hold/reset/disable")
                self.send_command_reply(response, peer)
        except BlockingIOError: pass
        except Exception as exc:
            if 'peer' in locals(): self.send_command_reply({"error": str(exc)}, peer)

    def send_command_reply(self, response, peer):
        try:
            self.command.sendto(json.dumps(response).encode(), peer)
        except OSError as exc:
            if exc.errno not in {errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN, errno.EWOULDBLOCK}:
                raise
            # A timed-out UI client can remove its reply address at any time.
            # It must not terminate the live control process or trigger replay.
            self.command_reply_dropped = getattr(self, "command_reply_dropped", 0) + 1

    def clear_run_reference(self):
        self.run_leader_right = self.run_follower_right = None
        self.run_leader_left = self.run_follower_left = None
        self.run_leader_gripper = self.run_follower_gripper = None
        self.run_leader_left_gripper = self.run_follower_left_gripper = None
        self.capture_hold_reference()
        self.last_target_right = self.last_target_left = None

    def effective_safety(self, now_ns):
        response = dict(self.safety)
        age = now_ns-self.safety_rx_ns if self.safety_rx_ns else None
        response["safety_reply_age_ms"] = age/1_000_000 if age is not None else None
        if response.get("state") == "RUNNING" and (age is None or not 0 <= age < 100_000_000):
            response["watchdog_state"] = response["state"]
            response["state"] = "READY"  # A temporary gate, not a fabricated hardware fault.
            response["reason"] = ("operator stop acknowledgement is pending; holding last published target"
                if getattr(self, "stop_intent", None) else "watchdog safety reply is stale; holding last published target")
        return response

    def runtime_status(self):
        response = self.effective_safety(time.monotonic_ns())
        if hasattr(self, "io"):
            response["io_diagnostics"] = self.io.snapshot()["diagnostics"]
            response["io_diagnostics"]["command_reply_dropped"] = self.command_reply_dropped
        response["collection"] = self.collection.status(self.positions,
            self.latest_action.axes if self.latest_action else self.positions)
        response["applied_axes"] = self.applied_axes
        if self.last_target_right is not None:
            errors = [target - actual for target, actual in zip(
                self.last_target_right, self.positions[8:15])]
            response["right_target_rad"] = list(self.last_target_right)
            response["right_actual_rad"] = list(self.positions[8:15])
            response["right_tracking_error_rad"] = errors
            response["max_tracking_error_rad"] = max(abs(error) for error in errors)
        if self.latest_action is not None:
            response["leader_axes"] = list(self.latest_action.axes)
            response["leader_right_rad"] = list(self.latest_action.right_arm)
            if self.enable_left:
                response["leader_left_rad"] = list(self.latest_action.left_arm)
            response["action_age_ms"] = (time.monotonic_ns() - self.last_action_rx_ns) / 1_000_000
        if self.have_feedback:
            feedback_age_ns = time.monotonic_ns() - self.last_feedback_ns
            response["feedback_age_ms"] = feedback_age_ns / 1_000_000
            response["feedback_fresh_for_control"] = (
                feedback_age_ns < FEEDBACK_CONTROL_TIMEOUT_NS)
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
        safety = self.effective_safety(now_ns)
        state_name = safety.get("state", "ALIGNING")
        state = FollowerState(self.session, self.sequence, now_ns, self.last_feedback_ns,
            self.action_timestamp_ns, self.applied_session, self.applied_sequence,
            ControlState[state_name], FaultBits(int(safety.get("fault_bits", 0))),
            tuple(self.positions), tuple(self.velocities), tuple(self.efforts), self.collection.flags,
            self.collection.leader_servo_target or (0.,)*8)
        try:
            self.udp.sendto(encode_state(state), (self.peer_ip, STATE_PORT))
        except BlockingIOError:
            pass  # Drop a congested state datagram; never queue/replay old state.

    def command_tick(self):
        self.heartbeat(time.monotonic_ns())
        self.handle_commands(time.monotonic_ns())

    def tick(self):
        now_ns = time.monotonic_ns(); self.heartbeat(now_ns)
        if self.latest_action and self.have_feedback:
            errors = [abs(a-b) for a,b in zip(self.latest_action.right_arm, self.positions[8:15])]
            if self.enable_left:
                errors.extend(abs(a-b) for a,b in zip(self.latest_action.left_arm, self.positions[0:7]))
            aligned = max(errors) <= 0.15
            if aligned and not self.align_since_ns: self.align_since_ns = now_ns
            elif not aligned: self.align_since_ns = 0
        # The next watchdog heartbeat after `run` is the authoritative state
        # transition. Capture here as a second, timing-independent safeguard.
        if self.safety.get("state") == "RUNNING" and self.run_leader_right is None:
            self.capture_run_reference()
        if self.publish_target(time.monotonic_ns()):
            self.io.control_completed(now_ns, time.monotonic_ns(), self.last_feedback_ns, self.have_feedback)
        self.send_state(time.monotonic_ns())

    def close(self):
        self.io.close(); self.command.close()
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

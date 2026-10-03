"""Independent, report-only Jetson watchdog service.

This process never imports or opens CAN. Until a physical reaction backend is
validated, it deliberately refuses the READY -> RUNNING transition.
"""

from __future__ import annotations

import argparse
import errno
import json
import os
import signal
import socket
import tempfile
import time

from remote_teleop_protocol import FaultBits

from .local_protocol import LocalProtocolError, decode_local, encode_command
from .state_machine import SafetyReaction, SafetyStateMachine, TransitionError
from .watchdog import WatchdogConfig, WatchdogSupervisor


WATCHDOG_IO_PROTOCOL_VERSION = 1


def _snapshot_json(supervisor: WatchdogSupervisor, *, reply_to: dict | None = None) -> str:
    # A reply is safety authority. Never acknowledge RUNNING before evaluating
    # the current heartbeat/action ages, including request_run and status.
    now_ns = time.monotonic_ns()
    supervisor.check(now_ns)
    snapshot = supervisor.machine.snapshot()
    return json.dumps(
        {
            "state": snapshot.state.name,
            "fault_bits": int(snapshot.fault_bits),
            "reason": snapshot.reason,
            "leader_session_id": snapshot.leader_session_id,
            "aligned": snapshot.aligned,
            "reaction": snapshot.reaction.value,
            "reaction_verified": snapshot.reaction_verified,
            "hardware_action": "REPORT_ONLY_NO_CAN",
            "watchdog_io_protocol_version": WATCHDOG_IO_PROTOCOL_VERSION,
            "watchdog_session_id": supervisor.watchdog_session_id,
            "snapshot_monotonic_ns": now_ns,
            "reply_to": reply_to,
            "diagnostics": supervisor.diagnostics(now_ns),
            "first_fault": supervisor.first_fault,
        },
        separators=(",", ":"),
        sort_keys=True,
    )


def _handle_datagram(supervisor: WatchdogSupervisor, datagram: bytes) -> str:
    """Apply this request before checking and serializing its correlated reply."""
    reply_to = None
    try:
        kind, message = decode_local(datagram)
        now_ns = time.monotonic_ns()
        machine = supervisor.machine
        if kind == "heartbeat":
            reply_to = {"type": "heartbeat", "controller_session_id": message.controller_session_id,
                        "sequence": message.sequence, "sent_monotonic_ns": message.sent_monotonic_ns}
            supervisor.receive_heartbeat(message, now_ns)
        else:
            command = message["command"]
            reply_to = {"type": "command", "command": command}
            if command == "alignment_complete":
                machine.alignment_complete(int(message["leader_session_id"]))
            elif command == "request_run":
                machine.request_run(int(message["leader_session_id"]))
            elif command == "hold":
                machine.request_hold()
            elif command == "reset":
                supervisor.reset_fault(now_ns, estop_released=message["estop_released"])
            elif command == "estop":
                supervisor.trip(FaultBits.E_STOP_ACTIVE, "local E-stop command", now_ns)
            elif command == "status":
                pass
    except (LocalProtocolError, TransitionError, KeyError, TypeError, ValueError) as exc:
        supervisor.trip(FaultBits.INVALID_COMMAND, f"local watchdog input rejected: {exc}", time.monotonic_ns())
    return _snapshot_json(supervisor, reply_to=reply_to)


def _send_reply(sock: socket.socket, payload: str, peer) -> bool:
    """An expired one-shot client cannot block or terminate supervision."""
    if not peer:
        return False
    try:
        sock.sendto(payload.encode("utf-8"), socket.MSG_DONTWAIT, peer)
        return True
    except OSError as exc:
        if exc.errno in {errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN, errno.EWOULDBLOCK}:
            return False
        raise


def _existing_watchdog_is_live(socket_path: str) -> bool:
    """Return true only if the existing socket answers this watchdog protocol."""
    probe_path = tempfile.mktemp(prefix="openarm_watchdog_probe_", dir="/tmp")
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        probe.bind(probe_path)
        probe.settimeout(0.1)
        probe.sendto(encode_command("status"), socket_path)
        reply = json.loads(probe.recv(4096).decode("utf-8"))
        return isinstance(reply, dict) and isinstance(reply.get("state"), str)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    finally:
        probe.close()
        if os.path.exists(probe_path):
            os.unlink(probe_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Independent Jetson follower watchdog")
    parser.add_argument("--socket", default="/tmp/openarm_follower_watchdog.sock")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until signal")
    parser.add_argument("--startup-grace-s", type=float, default=2.0,
                        help="maximum controller initialization time before a liveness fault")
    parser.add_argument("--control-heartbeat-timeout-ms", type=float, default=300.0,
                        help="maximum follower gateway heartbeat gap")
    parser.add_argument("--control-cycle-timeout-ms", type=float, default=250.0,
                        help="maximum age of the follower gateway control cycle")
    parser.add_argument("--network-action-timeout-ms", type=float, default=150.0,
                        help="maximum leader action age while RUNNING")
    parser.add_argument(
        "--verified-reaction",
        choices=("position_hold",),
        help="operator assertion recorded only after the supervised physical reaction test",
    )
    parser.add_argument("--simulation-verified-reaction", choices=("position_hold",),
                        help=argparse.SUPPRESS)
    # When launched through launch_ros.Node, ROS 2 appends `--ros-args` even
    # though this independent Unix-datagram watchdog has no ROS dependency.
    # Ignore those transport arguments rather than exiting before supervision
    # starts.
    args, _unknown_ros_args = parser.parse_known_args()
    if not args.startup_grace_s > 0.0:
        parser.error("--startup-grace-s must be positive")
    if not args.control_heartbeat_timeout_ms > 0.0:
        parser.error("--control-heartbeat-timeout-ms must be positive")
    if not args.control_cycle_timeout_ms > 0.0:
        parser.error("--control-cycle-timeout-ms must be positive")
    if not args.network_action_timeout_ms > 0.0:
        parser.error("--network-action-timeout-ms must be positive")

    reaction = SafetyReaction.UNDECIDED
    verified = False
    selected_reaction = args.verified_reaction or args.simulation_verified_reaction
    if selected_reaction:
        reaction = SafetyReaction(selected_reaction)
        verified = True

    machine = SafetyStateMachine(reaction, verified)
    supervisor = WatchdogSupervisor(
        machine,
        WatchdogConfig(
            startup_grace_ns=int(args.startup_grace_s * 1_000_000_000),
            control_heartbeat_timeout_ns=int(args.control_heartbeat_timeout_ms * 1_000_000),
            control_cycle_timeout_ns=int(args.control_cycle_timeout_ms * 1_000_000),
            network_action_timeout_ns=int(args.network_action_timeout_ms * 1_000_000),
        ),
    )
    supervisor.boot(time.monotonic_ns())

    socket_path = os.path.abspath(args.socket)
    if os.path.exists(socket_path):
        if _existing_watchdog_is_live(socket_path):
            raise RuntimeError(f"refusing to replace live watchdog socket: {socket_path}")
        # A Jetson reboot can leave an orphan Unix socket.  It is safe to remove
        # only after a protocol probe proves that no watchdog owns it.
        os.unlink(socket_path)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(socket_path)
    os.chmod(socket_path, 0o600)
    sock.settimeout(0.01)

    stopping = False

    def stop_handler(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    previous = machine.snapshot()
    print(_snapshot_json(supervisor), flush=True)
    try:
        while not stopping and (deadline is None or time.monotonic() < deadline):
            try:
                datagram, peer = sock.recvfrom(4096)
                _send_reply(sock, _handle_datagram(supervisor, datagram), peer)
            except socket.timeout:
                pass
            supervisor.check(time.monotonic_ns())
            current = machine.snapshot()
            if current != previous:
                print(_snapshot_json(supervisor), flush=True)
                previous = current
    finally:
        sock.close()
        if os.path.exists(socket_path):
            os.unlink(socket_path)


if __name__ == "__main__":
    main()

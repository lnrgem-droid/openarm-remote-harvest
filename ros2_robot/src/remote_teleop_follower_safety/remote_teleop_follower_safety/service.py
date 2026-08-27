"""Independent, report-only Jetson watchdog service.

This process never imports or opens CAN. Until a physical reaction backend is
validated, it deliberately refuses the READY -> RUNNING transition.
"""

from __future__ import annotations

import argparse
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


def _snapshot_json(supervisor: WatchdogSupervisor) -> str:
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
        },
        separators=(",", ":"),
        sort_keys=True,
    )


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

    reaction = SafetyReaction.UNDECIDED
    verified = False
    selected_reaction = args.verified_reaction or args.simulation_verified_reaction
    if selected_reaction:
        reaction = SafetyReaction(selected_reaction)
        verified = True

    machine = SafetyStateMachine(reaction, verified)
    supervisor = WatchdogSupervisor(machine, WatchdogConfig())
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
                kind, message = decode_local(datagram)
                now_ns = time.monotonic_ns()
                if kind == "heartbeat":
                    supervisor.receive_heartbeat(message, now_ns)
                else:
                    command = message["command"]
                    if command == "alignment_complete":
                        machine.alignment_complete(int(message["leader_session_id"]))
                    elif command == "request_run":
                        machine.request_run(int(message["leader_session_id"]))
                    elif command == "hold":
                        machine.request_hold()
                    elif command == "reset":
                        supervisor.reset_fault(
                            now_ns, estop_released=message["estop_released"]
                        )
                    elif command == "estop":
                        machine.trip(FaultBits.E_STOP_ACTIVE, "local E-stop command")
                    elif command == "status":
                        pass
                if peer:
                    sock.sendto(_snapshot_json(supervisor).encode("utf-8"), peer)
            except socket.timeout:
                pass
            except (LocalProtocolError, TransitionError, KeyError, TypeError, ValueError) as exc:
                machine.trip(FaultBits.INVALID_COMMAND, f"local watchdog input rejected: {exc}")
                if "peer" in locals() and peer:
                    sock.sendto(_snapshot_json(supervisor).encode("utf-8"), peer)
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

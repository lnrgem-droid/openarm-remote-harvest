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
import time

from remote_teleop_protocol import FaultBits

from .local_protocol import LocalProtocolError, decode_local
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Independent Jetson follower watchdog")
    parser.add_argument("--socket", default="/tmp/openarm_follower_watchdog.sock")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means run until signal")
    parser.add_argument(
        "--simulation-verified-reaction",
        choices=("position_hold",),
        help="simulation only; never proves a physical robot reaction",
    )
    args = parser.parse_args()

    reaction = SafetyReaction.UNDECIDED
    verified = False
    if args.simulation_verified_reaction:
        reaction = SafetyReaction(args.simulation_verified_reaction)
        verified = True

    machine = SafetyStateMachine(reaction, verified)
    supervisor = WatchdogSupervisor(machine, WatchdogConfig())
    supervisor.boot(time.monotonic_ns())

    socket_path = os.path.abspath(args.socket)
    if os.path.exists(socket_path):
        raise RuntimeError(f"refusing to replace existing watchdog socket: {socket_path}")
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
                datagram = sock.recv(4096)
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
                    elif command == "reset":
                        supervisor.reset_fault(
                            now_ns, estop_released=message["estop_released"]
                        )
                    elif command == "estop":
                        machine.trip(FaultBits.E_STOP_ACTIVE, "local E-stop command")
            except socket.timeout:
                pass
            except (LocalProtocolError, TransitionError, KeyError, TypeError, ValueError) as exc:
                machine.trip(FaultBits.INVALID_COMMAND, f"local watchdog input rejected: {exc}")
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

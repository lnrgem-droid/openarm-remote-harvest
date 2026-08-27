"""CAN-free heartbeat client for the independent watchdog service."""

from __future__ import annotations

import argparse
import secrets
import socket
import time

from .local_protocol import encode_command, encode_heartbeat
from .watchdog import ControllerHeartbeat


def main() -> None:
    parser = argparse.ArgumentParser(description="CAN-free follower safety simulator")
    parser.add_argument("--socket", default="/tmp/openarm_follower_watchdog.sock")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument(
        "--fault",
        choices=("none", "network_timeout", "control_stall", "can_error", "estop", "overrun"),
        default="none",
    )
    parser.add_argument("--request-run", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0:
        raise ValueError("duration and rate must be greater than zero")

    controller_session = secrets.randbits(64) or 1
    leader_session = secrets.randbits(64) or 1
    sequence = 0
    period = 1.0 / args.rate
    started = time.monotonic()
    deadline = started + args.duration
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def send(raw: bytes) -> None:
        sock.sendto(raw, args.socket)

    try:
        now_ns = time.monotonic_ns()
        send(
            encode_heartbeat(
                ControllerHeartbeat(
                    controller_session,
                    1,
                    now_ns,
                    now_ns,
                    now_ns,
                    leader_session,
                    True,
                    False,
                )
            )
        )
        sequence = 1
        send(encode_command("alignment_complete", leader_session_id=leader_session))
        if args.request_run:
            send(encode_command("request_run", leader_session_id=leader_session))
        next_send = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_send:
                time.sleep(min(next_send - now, 0.001))
                continue
            sequence += 1
            now_ns = time.monotonic_ns()
            elapsed = now - started
            trigger = elapsed >= args.duration / 2.0
            last_action_ns = now_ns
            last_cycle_ns = now_ns
            can_ok = True
            estop = False
            overruns = 0
            if trigger and args.fault == "network_timeout":
                last_action_ns = now_ns - 1_000_000_000
            elif trigger and args.fault == "control_stall":
                last_cycle_ns = now_ns - 1_000_000_000
            elif trigger and args.fault == "can_error":
                can_ok = False
            elif trigger and args.fault == "estop":
                estop = True
            elif trigger and args.fault == "overrun":
                overruns = 5
            heartbeat = ControllerHeartbeat(
                controller_session_id=controller_session,
                sequence=sequence,
                sent_monotonic_ns=now_ns,
                last_control_cycle_ns=last_cycle_ns,
                last_action_rx_ns=last_action_ns,
                leader_session_id=leader_session,
                can_ok=can_ok,
                estop_active=estop,
                consecutive_overruns=overruns,
            )
            send(encode_heartbeat(heartbeat))
            next_send += period
    finally:
        sock.close()
    print(f"safety_sim sent_heartbeats={sequence} fault={args.fault}")


if __name__ == "__main__":
    main()

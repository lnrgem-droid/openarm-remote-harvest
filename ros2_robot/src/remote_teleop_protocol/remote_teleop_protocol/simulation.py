"""Network-only protocol simulator. This module cannot access CAN or motors."""

from __future__ import annotations

import argparse
import secrets
import socket
import time

from .protocol import (
    AXIS_COUNT,
    ActionCommand,
    ControlState,
    FollowerState,
    MessageType,
    PacketError,
    SequenceTracker,
    decode_message,
    encode_action,
    encode_state,
)


DEFAULT_ACTION_PORT = 51010
DEFAULT_STATE_PORT = 51011


def _nonnegative_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return number


def follower_main() -> None:
    parser = argparse.ArgumentParser(description="CAN-free follower protocol simulator")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--action-port", type=int, default=DEFAULT_ACTION_PORT)
    parser.add_argument("--state-port", type=int, default=DEFAULT_STATE_PORT)
    parser.add_argument("--duration", type=_nonnegative_float, default=10.0)
    args = parser.parse_args()

    session_id = secrets.randbits(64)
    tracker = SequenceTracker()
    received = 0
    invalid = 0
    started = time.monotonic()
    deadline = started + args.duration
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.action_port))
    sock.settimeout(0.1)
    try:
        while time.monotonic() < deadline:
            try:
                datagram, peer = sock.recvfrom(2048)
                message = decode_message(datagram)
            except socket.timeout:
                continue
            except PacketError:
                invalid += 1
                continue
            if not isinstance(message, ActionCommand):
                invalid += 1
                continue
            if not tracker.accept(message.session_id, message.sequence):
                continue
            received += 1
            now_ns = time.monotonic_ns()
            state = FollowerState(
                session_id=session_id,
                sequence=received,
                sender_monotonic_ns=now_ns,
                obs_timestamp_ns=now_ns,
                action_timestamp_ns=0,
                applied_action_sequence=0,
                control_state=ControlState.ALIGNING,
                fault_bits=0,
                positions=(0.0,) * AXIS_COUNT,
                velocities=(0.0,) * AXIS_COUNT,
            )
            sock.sendto(encode_state(state), (peer[0], args.state_port))
    finally:
        sock.close()
    elapsed = time.monotonic() - started
    print(f"sim_follower received={received} invalid={invalid} elapsed_s={elapsed:.3f}")


def leader_main() -> None:
    parser = argparse.ArgumentParser(description="CAN-free leader protocol simulator")
    parser.add_argument("--peer", default="127.0.0.1")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--action-port", type=int, default=DEFAULT_ACTION_PORT)
    parser.add_argument("--state-port", type=int, default=DEFAULT_STATE_PORT)
    parser.add_argument("--rate", type=_nonnegative_float, default=100.0)
    parser.add_argument("--duration", type=_nonnegative_float, default=5.0)
    args = parser.parse_args()

    session_id = secrets.randbits(64)
    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration
    next_send = time.monotonic()
    sent = 0
    received = 0
    invalid = 0
    tracker = SequenceTracker()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.state_port))
    sock.setblocking(False)
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                sent += 1
                now_ns = time.monotonic_ns()
                command = ActionCommand(
                    session_id=session_id,
                    sequence=sent,
                    sender_monotonic_ns=now_ns,
                    axes=(0.0,) * AXIS_COUNT,
                    valid_for_ns=100_000_000,
                )
                sock.sendto(encode_action(command), (args.peer, args.action_port))
                next_send += period
            try:
                while True:
                    datagram, _ = sock.recvfrom(2048)
                    message = decode_message(datagram)
                    if isinstance(message, FollowerState) and tracker.accept(
                        message.session_id, message.sequence
                    ):
                        received += 1
                    else:
                        invalid += 1
            except BlockingIOError:
                pass
            except PacketError:
                invalid += 1
            time.sleep(min(period / 4.0, 0.001))
    finally:
        sock.close()
    print(f"sim_leader sent={sent} received={received} invalid={invalid}")

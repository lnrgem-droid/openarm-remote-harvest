#!/usr/bin/env python3
"""Passive, bounded SocketCAN drive-enable check. NEVER sends CAN frames.

Exit 0: all enabled; 10: all disabled; 11: mixed; 12: unknown/fault.
With --jetson, check both leader buses locally and both follower buses by SSH.
The source is streamed over SSH, so stale remote installations cannot pass.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import time

SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)


def feedback_receive_time(ancillary, flags, realtime_ns, monotonic_s):
    """Kernel RX time, not dequeue time: a socket backlog is not live feedback."""
    if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC): return None
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(data) >= 16:
            sec, ns = struct.unpack_from('=qq', data)
            age_ns = realtime_ns - (sec*1_000_000_000 + ns)
            if not 0 <= ns < 1_000_000_000 or not 0 <= age_ns <= 250_000_000:
                return None
            return monotonic_s-age_ns/1e9
    return None


def decode_feedback(packet):
    if len(packet) not in (16, 72):
        return None
    can_id, length = struct.unpack_from('=IB', packet)
    # Reject extended, RTR and error frames, commands and parameter replies.
    if can_id & 0xE0000000 or not 0x11 <= can_id <= 0x18 or length != 8:
        return None
    data = packet[8:16]
    motor_id = can_id - 0x10
    if data[0] & 0x0F != motor_id:
        return None
    return motor_id, data[0] >> 4


def summarize(observations, interfaces, now):
    motors = {}
    for interface in interfaces:
        for motor in range(1, 9):
            values = observations.get((interface, motor), [])
            states = sorted({state for _, state in values})
            fresh = bool(values) and now - values[-1][0] <= .25
            valid = fresh and len(values) >= 3 and len(states) == 1
            motors[f'{interface}/{motor}'] = {
                'states': states, 'samples': len(values), 'fresh': fresh,
                'state': states[0] if valid else None,
            }
    states = [entry['state'] for entry in motors.values()]
    if any(state is None for state in states):
        result = 'UNKNOWN'
    elif any(state not in (0, 1) for state in states):
        result = 'FAULT'
    elif all(state == 1 for state in states):
        result = 'ENABLED'
    elif all(state == 0 for state in states):
        result = 'DISABLED'
    else:
        result = 'MIXED'
    return {'result': result, 'motors': motors}


def probe(interfaces):
    observations = {}
    sockets = {}
    try:
        for interface in interfaces:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sockets[sock] = interface
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                            # CAN_ERR_FLAG in a kernel filter mask selects
                            # error-frame handling, not normal data frames.
                            b''.join(struct.pack('=II', i, 0xC00007FF)
                                     for i in range(0x11, 0x19)))
            sock.bind((interface,))
            sock.setblocking(False)
        deadline = time.monotonic() + .75
        while time.monotonic() < deadline:
            ready, _, _ = select.select(list(sockets), [], [],
                                       max(0., deadline - time.monotonic()))
            for sock in ready:
                try:
                    packet, ancillary, flags, _ = sock.recvmsg(72, socket.CMSG_SPACE(16))
                    received = feedback_receive_time(ancillary, flags, time.time_ns(), time.monotonic())
                    decoded = decode_feedback(packet) if received is not None else None
                except BlockingIOError:
                    continue
                if decoded is not None:
                    motor, state = decoded
                    observations.setdefault((sockets[sock], motor), []).append(
                        (received, state))
        return summarize(observations, interfaces, time.monotonic())
    except OSError as exc:
        return {'result': 'UNKNOWN', 'error': str(exc)}
    finally:
        for sock in sockets:
            sock.close()


def combined(host, follower):
    results = {host.get('result'), follower.get('result')}
    result = ('FAULT' if 'FAULT' in results else next(iter(results)) if len(results) == 1 else
              'MIXED' if results <= {'ENABLED', 'DISABLED', 'MIXED'} else 'UNKNOWN')
    return {'result': result, 'host': host, 'follower': follower}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interfaces', nargs='+', default=['can0', 'can1'])
    parser.add_argument('--jetson')
    args = parser.parse_args()
    report = probe(args.interfaces)
    if args.jetson:
        if args.jetson.startswith('-'):
            parser.error('invalid SSH destination')
        try:
            remote = subprocess.run(
                ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                 args.jetson, '/usr/bin/python3 - --interfaces can1 can2'],
                input=Path(__file__).read_text(), text=True, capture_output=True,
                timeout=10, check=False)
            follower = json.loads(remote.stdout)
            if not isinstance(follower, dict):
                raise ValueError('invalid probe reply')
        except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
            follower = {'result': 'UNKNOWN', 'error': str(exc)}
        report = combined(report, follower)
    print(json.dumps(report, ensure_ascii=False))
    return {'ENABLED': 0, 'DISABLED': 10, 'MIXED': 11}.get(report['result'], 12)


if __name__ == '__main__':
    sys.exit(main())

"""Isolated left-arm transport and passive drive-health gate. Never sends CAN."""
import select
import socket
import struct
import threading
import time
from collections import deque

LEFT_ACTION_PORT = 50110
LEFT_STATE_PORT = 50111
LEFT_RUNTIME_SOCKET = '/tmp/openarm_left_runtime.sock'
LEFT_WATCHDOG_SOCKET = '/tmp/openarm_left_watchdog.sock'
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)


def decode_drive(packet):
    if len(packet) not in (16, 72):
        return None
    cid, length = struct.unpack_from('=IB', packet)
    if cid & 0xE0000000 or not 0x11 <= cid <= 0x18 or length != 8:
        return None
    motor = cid - 0x10
    if packet[8] & 15 != motor:
        return None
    return motor, packet[8] >> 4


class MotorEvidence:
    """Three fresh enabled replies for every drive; any subsequent loss latches."""
    def __init__(self):
        self.samples = {i: deque(maxlen=3) for i in range(1, 9)}
        self.armed = False
        self.fault = None

    def observe(self, motor, state, received):
        if state not in (0, 1):
            self.fault = f'motor {motor} abnormal state {state}'
        elif self.armed and state != 1:
            self.fault = f'motor {motor} disabled during session'
        self.samples[motor].append((received, state))

    def healthy(self, now):
        valid = all(len(v) == 3 and all(state == 1 for _, state in v)
                    and all(0 <= now-t <= .25 for t, _ in v) for v in self.samples.values())
        if self.armed and not valid and self.fault is None:
            self.fault = 'left motor feedback missing or stale'
        if valid:
            self.armed = True
        return valid and self.fault is None


class PassiveMotorHealth:
    def __init__(self, bus):
        self.bus = bus
        self.evidence = MotorEvidence()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
            b''.join(struct.pack('=II', i, 0xC00007FF) for i in range(0x11, 0x19)))
        self.sock.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack('=I', 0x1fffffff))
        self.sock.bind((bus,))
        self.sock.setblocking(False)
        self.thread = threading.Thread(target=self._run, daemon=True, name='left-drive-health')
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                if not select.select([self.sock], [], [], .02)[0]:
                    continue
                packet, ancillary, flags, _ = self.sock.recvmsg(72, socket.CMSG_SPACE(16))
                if struct.unpack_from('=I', packet)[0] & 0x20000000:
                    with self.lock: self.evidence.fault = 'CAN error frame'
                    continue
                decoded = decode_drive(packet)
                if decoded is None or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                    continue
                now, wall = time.monotonic(), time.time_ns()
                for level, kind, stamp in ancillary:
                    if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(stamp) >= 16:
                        sec, ns = struct.unpack_from('=qq', stamp)
                        age = wall - (sec * 1_000_000_000 + ns)
                        if 0 <= ns < 1_000_000_000 and 0 <= age <= 250_000_000:
                            with self.lock:
                                self.evidence.observe(*decoded, now - age / 1e9)
                        break
        except Exception as exc:
            with self.lock: self.evidence.fault = f'CAN listener failed: {exc}'

    def healthy(self):
        with self.lock:
            return self.evidence.healthy(time.monotonic())

    def report(self):
        with self.lock:
            return {'bus': self.bus, 'healthy': self.evidence.healthy(time.monotonic()),
                    'armed': self.evidence.armed, 'fault': self.evidence.fault}

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.sock.close()

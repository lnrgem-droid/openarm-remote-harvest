"""Linux receive timestamps with a fixed, conservative monotonic conversion.

The kernel timestamps arrival, not when Python eventually drains the socket.
CLOCK_REALTIME is not monotonic: a fixed offset with a 1ms lower-bound guard
is checked on every packet. A clock step outside that envelope fails closed;
the offset is NEVER rebased, so buffered packets cannot be made young again.
See https://docs.kernel.org/networking/timestamping.html (SO_TIMESTAMPNS_NEW).
"""
import socket
import struct
import time

SO_TIMESTAMPNS_NEW = 64  # Linux UAPI: __kernel_timespec, two signed 64-bit fields.
CLOCK_GUARD_NS = 1_000_000


class KernelReceiveClock:
    def __init__(self, sock, *, monotonic=time.monotonic_ns, realtime=time.time_ns):
        self.monotonic, self.realtime = monotonic, realtime
        samples = [self._sample() for _ in range(3)]
        self.low, self.high, _ = min(samples, key=lambda x: x[1]-x[0])
        if self.high-self.low > CLOCK_GUARD_NS:
            raise RuntimeError('cannot establish bounded kernel receive clock')
        sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS_NEW, 1)

    def _sample(self):
        before = self.monotonic()
        wall = self.realtime()
        after = self.monotonic()
        return before-wall, after-wall, wall

    def timestamp(self, ancillary, flags):
        low, high, wall = self._sample()
        if high-low > CLOCK_GUARD_NS:
            return None  # Scheduling uncertainty: never pretend the packet is new.
        if low < self.low-CLOCK_GUARD_NS or high > self.high+CLOCK_GUARD_NS:
            raise RuntimeError('kernel receive clock changed; explicit restart required')
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            return None
        stamps = [data for level, kind, data in ancillary
                  if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS_NEW]
        if len(stamps) != 1 or len(stamps[0]) != 16:
            return None
        sec, ns = struct.unpack('=qq', stamps[0])
        if sec <= 0 or not 0 <= ns < 1_000_000_000:
            return None
        realtime_ns = sec*1_000_000_000+ns
        if realtime_ns > wall:
            return None
        return realtime_ns+self.low-CLOCK_GUARD_NS

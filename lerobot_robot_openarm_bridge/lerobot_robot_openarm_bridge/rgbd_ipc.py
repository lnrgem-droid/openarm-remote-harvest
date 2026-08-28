"""Latest-frame client for the Jetson-owned OpenArm RGB-D service."""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RGBDFrame:
    role: str
    header: dict
    rgb: np.ndarray
    depth: np.ndarray


class RGBDHub:
    """Subscribe to atomic RGB-D pairs; never opens an Orbbec device."""

    def __init__(self, endpoint: str, roles: tuple[str, ...]):
        self.endpoint, self.roles = endpoint, roles
        self._frames: dict[str, RGBDFrame] = {}
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._socket = self._ctx = None

    def connect(self, startup_timeout_s: float = 3.0) -> None:
        import zmq
        self._ctx = zmq.Context(); self._socket = self._ctx.socket(zmq.SUB)
        # Three 30 Hz cameras arrive as independent publications. Keep a
        # short complete set, then drain to latest below; an HWM of two can
        # systematically starve the third role under scheduler contention.
        self._socket.setsockopt(zmq.RCVHWM, 12)
        for role in self.roles:
            self._socket.setsockopt(zmq.SUBSCRIBE, f"rgbd/{role}".encode())
        self._socket.connect(self.endpoint); self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="openarm-rgbd-ipc")
        self._thread.start()
        # ZMQ subscriptions are asynchronous.  Do not let LeRobot begin an
        # episode before every camera has supplied one fresh atomic RGB-D pair.
        deadline = time.monotonic() + startup_timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                ready = all(role in self._frames for role in self.roles)
            if ready:
                return
            time.sleep(0.01)
        self.disconnect()
        raise TimeoutError(f"RGB-D startup timed out; missing roles: {self._missing_roles()}")

    def _missing_roles(self) -> list[str]:
        with self._lock:
            return [role for role in self.roles if role not in self._frames]

    @property
    def is_connected(self) -> bool:
        return self._running and self._thread is not None and self._thread.is_alive()

    def _loop(self) -> None:
        import zmq
        assert self._socket is not None
        while self._running:
            try:
                topic, raw_header, raw_rgb, raw_depth = self._socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.001); continue
            try:
                header = json.loads(raw_header)
                role = topic.decode().split("/", 1)[1]
                h, w = int(header["height"]), int(header["width"])
                rgb = np.frombuffer(raw_rgb, dtype=np.uint8).reshape(h, w, 3).copy()
                depth = np.frombuffer(raw_depth, dtype=np.uint16).reshape(h, w, 1).copy()
                with self._lock:
                    self._frames[role] = RGBDFrame(role, header, rgb, depth)
            except Exception:
                continue

    def snapshot(self, max_age_ms: int = 100) -> dict[str, RGBDFrame]:
        now = time.monotonic_ns()
        with self._lock:
            result = dict(self._frames)
        missing = [role for role in self.roles if role not in result]
        if missing:
            raise TimeoutError(f"missing RGB-D frames: {missing}")
        stale = [role for role, frame in result.items()
                 if now - int(frame.header["host_monotonic_ns"]) > max_age_ms * 1_000_000]
        if stale:
            raise TimeoutError(f"stale RGB-D frames: {stale}")
        return result

    def disconnect(self) -> None:
        self._running = False
        if self._thread: self._thread.join(timeout=1.0)
        if self._socket: self._socket.close(0)
        if self._ctx: self._ctx.term()

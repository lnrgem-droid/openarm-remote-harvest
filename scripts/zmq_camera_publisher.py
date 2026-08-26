#!/usr/bin/env python3
"""Stream USB cameras over ZMQ for LeRobot ZMQCamera clients.

Protocol (ZMQ PUB, JSON string):
    {"timestamps": {"cam_left": 123.4}, "images": {"cam_left": "<base64-jpeg>"}}

Deps (camera host only, no lerobot):
    pip install opencv-python pyzmq numpy

Examples:
    python scripts/zmq_camera_publisher.py
    python scripts/zmq_camera_publisher.py --port 5555 --fps 30 --width 640 --height 480
    python scripts/zmq_camera_publisher.py \\
        --camera cam_left:/dev/video0 \\
        --camera cam_right:/dev/video2 \\
        --camera cam_head:/dev/video4
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import json
import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import zmq

logger = logging.getLogger("zmq_camera")


@dataclass
class CameraSpec:
    name: str
    device: str | int


def parse_camera_arg(value: str) -> CameraSpec:
    name, _, device = value.partition(":")
    name, device = name.strip(), device.strip()
    if not name or not device:
        raise argparse.ArgumentTypeError(f"expected NAME:DEVICE, got {value!r}")
    return CameraSpec(name, int(device) if device.isdigit() else device)


class CameraCapture:
    """Background capture thread; keeps only the latest encoded frame."""

    def __init__(self, spec: CameraSpec, fps: int, width: int, height: int, fourcc: str, quality: int):
        self.spec = spec
        self.fps = fps
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self.quality = quality

        self._encoded: str | None = None
        self._captured_at = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._cap: cv2.VideoCapture | None = None

    def start(self) -> None:
        cap = cv2.VideoCapture(self.spec.device)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera {self.spec.name} ({self.spec.device})")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        self._cap = cap

        actual = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        logger.info("camera %s (%s) opened, actual %dx%d", self.spec.name, self.spec.device, *actual)

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name=f"cap-{self.spec.name}")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._cap:
            self._cap.release()

    def latest(self) -> tuple[str | None, float]:
        with self._lock:
            return self._encoded, self._captured_at

    def _loop(self) -> None:
        assert self._cap is not None
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                logger.warning("camera %s read failed, retrying", self.spec.name)
                time.sleep(0.01)
                continue
            # Match lerobot OpenCVCamera RGB convention
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            with self._lock:
                self._encoded = base64.b64encode(buf).decode("utf-8")
                self._captured_at = time.time()


class ZmqCameraPublisher:
    """Publish all cameras' latest frames over ZMQ PUB at a fixed rate."""

    def __init__(self, cameras: list[CameraSpec], port: int, fps: int, log_interval_s: float, **cam_kwargs):
        if not cameras:
            raise ValueError("at least one camera is required")
        self.fps = fps
        self.log_interval_s = log_interval_s
        self.captures = {c.name: CameraCapture(c, fps=fps, **cam_kwargs) for c in cameras}

        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 20)  # drop when send buffer full
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{port}")
        logger.info("ZMQ publisher listening on tcp://*:%d", port)

    def run(self) -> None:
        for cap in self.captures.values():
            cap.start()

        logger.info("waiting for first frames...")
        for name, cap in self.captures.items():
            while cap.latest()[0] is None:
                time.sleep(0.01)
        logger.info("ready: %d cameras @ %d fps", len(self.captures), self.fps)

        sent_count = 0
        window_start = time.time()

        try:
            while True:
                loop_start = time.time()
                # Always include every camera. ZMQCamera falls back to the first
                # image if its key is missing, which causes cross-camera flicker.
                message: dict[str, dict] = {"timestamps": {}, "images": {}}
                latencies: list[float] = []

                for name, cap in self.captures.items():
                    encoded, captured_at = cap.latest()
                    if encoded is None:
                        continue
                    message["timestamps"][name] = captured_at
                    message["images"][name] = encoded
                    latencies.append((loop_start - captured_at) * 1000.0)

                if len(message["images"]) == len(self.captures):
                    with contextlib.suppress(zmq.Again):
                        self.socket.send_string(json.dumps(message), zmq.NOBLOCK)
                    sent_count += 1

                elapsed = time.time() - window_start
                if elapsed >= self.log_interval_s:
                    fps = sent_count / elapsed if elapsed > 0 else 0.0
                    lat = self._format_latency(latencies)
                    logger.info("publish FPS %.1f | latency %s", fps, lat)
                    sent_count, window_start = 0, time.time()

                sleep_s = 1.0 / self.fps - (time.time() - loop_start)
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except KeyboardInterrupt:
            logger.info("interrupted")
        finally:
            self.shutdown()

    @staticmethod
    def _format_latency(latencies: list[float]) -> str:
        if not latencies:
            return "no frames"
        return f"avg {sum(latencies) / len(latencies):.0f}ms max {max(latencies):.0f}ms"

    def shutdown(self) -> None:
        for cap in self.captures.values():
            cap.stop()
        self.socket.close()
        self.ctx.term()
        logger.info("publisher stopped")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Stream USB cameras over ZMQ for LeRobot ZMQCamera")
    p.add_argument("--port", type=int, default=5555, help="ZMQ PUB port (default: 5555)")
    p.add_argument("--fps", type=int, default=30, help="publish rate (default: 30)")
    p.add_argument("--width", type=int, default=640, help="frame width (default: 640)")
    p.add_argument("--height", type=int, default=480, help="frame height (default: 480)")
    p.add_argument("--fourcc", default="MJPG", help="V4L2 fourcc (default: MJPG)")
    p.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100 (default: 80)")
    p.add_argument("--log-interval", type=float, default=2.0, help="status log interval in seconds (default: 2.0)")
    p.add_argument(
        "--camera",
        action="append",
        type=parse_camera_arg,
        metavar="NAME:DEVICE",
        help="camera mapping, repeatable (default: cam_left/right/head -> /dev/video0/2/4)",
    )
    p.add_argument("-v", "--verbose", action="store_true", help="enable debug logging")
    return p


DEFAULT_CAMERAS = [
    CameraSpec("cam_left", "/dev/video0"),
    CameraSpec("cam_right", "/dev/video2"),
    CameraSpec("cam_head", "/dev/video4"),
]


def main() -> None:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    ZmqCameraPublisher(
        cameras=args.camera or DEFAULT_CAMERAS,
        port=args.port,
        fps=args.fps,
        log_interval_s=args.log_interval,
        width=args.width,
        height=args.height,
        fourcc=args.fourcc,
        quality=args.quality,
    ).run()


if __name__ == "__main__":
    main()

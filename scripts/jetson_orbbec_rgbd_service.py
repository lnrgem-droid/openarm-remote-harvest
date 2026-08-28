#!/usr/bin/env python3
"""Jetson-only, single-owner three-camera Orbbec RGB-D service.

Local subscribers receive raw paired RGB-D frames over IPC.  Remote subscribers
receive only best-effort JPEG RGB preview frames over TCP.  This process is the
only component allowed to open the physical Orbbec devices.
"""
from __future__ import annotations

import argparse, base64, json, logging, os, threading, time
from dataclasses import dataclass

import cv2
import numpy as np
import yaml
import zmq

LOG = logging.getLogger("openarm_rgbd")


@dataclass(frozen=True)
class CameraSpec:
    role: str
    serial: str


class OrbbecCamera:
    def __init__(self, spec: CameraSpec, width: int, height: int, fps: int):
        self.spec, self.width, self.height, self.fps = spec, width, height, fps
        self.latest = None; self.lock = threading.Lock(); self.running = False
        self.count = 0; self.dropped = 0; self.thread = None

    def start(self) -> None:
        from pyorbbecsdk import (AlignFilter, Config, Context, OBFormat, OBFrameAggregateOutputMode,
                                 OBSensorType, OBStreamType, Pipeline)
        ctx = Context(); devices = ctx.query_devices(); device = None
        for idx in range(devices.get_count()):
            candidate = devices.get_device_by_index(idx)
            if candidate.get_device_info().get_serial_number() == self.spec.serial:
                device = candidate; break
        if device is None: raise RuntimeError(f"{self.spec.role}: serial {self.spec.serial} not found")
        pipe = Pipeline(device); cfg = Config()
        color = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_video_stream_profile(
            self.width, self.height, OBFormat.RGB, self.fps)
        depth = pipe.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        cfg.enable_stream(color); cfg.enable_stream(depth)
        cfg.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        pipe.start(cfg); align = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self.running = True
        def loop():
            try:
                while self.running:
                    frames = pipe.wait_for_frames(100)
                    if not frames: continue
                    frames = align.process(frames)
                    if not frames: continue
                    rgb, dep = frames.get_color_frame(), frames.get_depth_frame()
                    if not rgb or not dep: continue
                    rgb_np = np.frombuffer(rgb.get_data(), np.uint8).reshape(rgb.get_height(), rgb.get_width(), 3).copy()
                    raw = np.frombuffer(dep.get_data(), np.uint16).reshape(dep.get_height(), dep.get_width())
                    depth_mm = np.rint(raw.astype(np.float32) * dep.get_depth_scale()).clip(0, 65535).astype(np.uint16)[..., None]
                    if rgb_np.shape[:2] != depth_mm.shape[:2]:
                        self.dropped += 1; continue
                    header = {"schema_version": 1, "role": self.spec.role, "frame_sequence": self.count,
                              "width": int(rgb_np.shape[1]), "height": int(rgb_np.shape[0]),
                              "aligned_depth_to_rgb": True, "rgb_device_timestamp_us": int(rgb.get_timestamp_us()),
                              "depth_device_timestamp_us": int(dep.get_timestamp_us()),
                              "host_monotonic_ns": time.monotonic_ns(), "host_realtime_ns": time.time_ns(),
                              "depth_unit": "mm", "rgb_dtype": "uint8", "depth_dtype": "uint16"}
                    with self.lock: self.latest = (header, rgb_np, depth_mm)
                    self.count += 1
            finally:
                pipe.stop()
        self.thread = threading.Thread(target=loop, daemon=True, name=f"orbbec-{self.spec.role}")
        self.thread.start()

    def get(self):
        with self.lock: return self.latest

    def stop(self):
        self.running = False
        if self.thread: self.thread.join(timeout=2.0)


def load_specs(path: str) -> list[CameraSpec]:
    with open(path, encoding="utf-8") as f: raw = yaml.safe_load(f)
    roles = raw.get("cameras", {})
    expected = ("left_wrist", "right_wrist", "chest")
    if set(roles) != set(expected): raise ValueError(f"camera roles must be {expected}")
    serials = [roles[role].get("serial") for role in expected]
    if any(not value for value in serials) or len(set(serials)) != 3: raise ValueError("camera serials must be complete and unique")
    return [CameraSpec(role, roles[role]["serial"]) for role in expected]


def main() -> None:
    p = argparse.ArgumentParser(); p.add_argument("--config", required=True)
    p.add_argument("--ipc", default="ipc:///tmp/openarm_rgbd_raw.ipc")
    p.add_argument("--preview-port", type=int, default=5556); p.add_argument("--fps", type=int, default=30)
    p.add_argument("--preview-fps", type=int, default=15); p.add_argument("--quality", type=int, default=75)
    p.add_argument("--metadata-dir", default="/tmp/openarm-rgbd-metadata")
    args = p.parse_args(); logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try: os.unlink(args.ipc.removeprefix("ipc://"))
    except FileNotFoundError: pass
    cameras = [OrbbecCamera(s, 640, 480, args.fps) for s in load_specs(args.config)]
    for camera in cameras: camera.start()
    os.makedirs(args.metadata_dir, exist_ok=True)
    metadata = {camera.spec.role: open(os.path.join(args.metadata_dir, f"{camera.spec.role}.jsonl"), "a", buffering=1)
                for camera in cameras}
    ctx = zmq.Context(); raw = ctx.socket(zmq.PUB); raw.setsockopt(zmq.SNDHWM, 2); raw.bind(args.ipc)
    preview = ctx.socket(zmq.PUB); preview.setsockopt(zmq.SNDHWM, 1); preview.setsockopt(zmq.LINGER, 0)
    preview.bind(f"tcp://*:{args.preview_port}")
    last_seq = {c.spec.role: -1 for c in cameras}; next_preview = 0.0; sent = 0; report = time.monotonic()
    try:
        while True:
            latest = {}
            for camera in cameras:
                item = camera.get()
                if not item: continue
                header, rgb, depth = item; latest[camera.spec.role] = item
                if header["frame_sequence"] != last_seq[camera.spec.role]:
                    try:
                        raw.send_multipart([f"rgbd/{camera.spec.role}".encode(), json.dumps(header).encode(), rgb.tobytes(), depth.tobytes()], flags=zmq.NOBLOCK)
                    except zmq.Again:
                        camera.dropped += 1
                    metadata[camera.spec.role].write(json.dumps(header) + "\n")
                    last_seq[camera.spec.role] = header["frame_sequence"]
            now = time.monotonic()
            if now >= next_preview and len(latest) == 3:
                message = {"schema_version": 1, "timestamps": {}, "frame_seq": {}, "images": {}}
                for role, (header, rgb, _) in latest.items():
                    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, args.quality])
                    if ok:
                        message["timestamps"][role] = header["host_realtime_ns"] / 1e9
                        message["frame_seq"][role] = header["frame_sequence"]
                        message["images"][role] = base64.b64encode(buf).decode()
                if len(message["images"]) == 3:
                    try: preview.send_string(json.dumps(message), flags=zmq.NOBLOCK); sent += 1
                    except zmq.Again: pass
                next_preview = now + 1.0 / args.preview_fps
            if now - report >= 2:
                LOG.info("capture=%s preview=%.1ffps", {c.spec.role: c.count for c in cameras}, sent / (now-report))
                report, sent = now, 0
            time.sleep(0.001)
    except KeyboardInterrupt: pass
    finally:
        for camera in cameras: camera.stop()
        for stream in metadata.values(): stream.close()
        raw.close(0); preview.close(0); ctx.term()

if __name__ == "__main__": main()

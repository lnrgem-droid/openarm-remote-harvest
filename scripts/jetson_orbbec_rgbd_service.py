#!/usr/bin/env python3
"""Jetson-only, single-owner three-camera Orbbec RGB-D service.

Local subscribers receive raw paired RGB-D frames over IPC.  Remote subscribers
receive only best-effort JPEG RGB preview frames over TCP.  This process is the
only component allowed to open the physical Orbbec devices.
"""
from __future__ import annotations

import argparse, base64, json, logging, os, queue, threading, time
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
        self.context = None
        self.last_error: str | None = None

    def start(self) -> None:
        from pyorbbecsdk import (AlignFilter, Config, Context, OBFormat, OBFrameAggregateOutputMode,
                                 OBSensorType, OBStreamType, Pipeline)
        device = None
        # USB cameras may appear a few seconds after a service restart.
        for _ in range(20):
            self.context = Context()
            devices = self.context.query_devices()
            for idx in range(devices.get_count()):
                candidate = devices.get_device_by_index(idx)
                if candidate.get_device_info().get_serial_number() == self.spec.serial:
                    device = candidate
                    break
            if device is not None:
                break
            time.sleep(0.5)
        if device is None: raise RuntimeError(f"{self.spec.role}: serial {self.spec.serial} not found after USB discovery retry")
        pipe = Pipeline(device); cfg = Config()
        color = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_video_stream_profile(
            self.width, self.height, OBFormat.RGB, self.fps)
        # Do not use the device's default depth profile: on Gemini 335 it can
        # be a larger mode which forces an extra align/resize pass and starves
        # the third camera.  The dataset contract is explicitly 640x480@30.
        depth = pipe.get_stream_profile_list(OBSensorType.DEPTH_SENSOR).get_video_stream_profile(
            self.width, self.height, OBFormat.Y16, self.fps)
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
                    depth_scale_mm = float(dep.get_depth_scale())
                    header = {"schema_version": 1, "role": self.spec.role, "frame_sequence": self.count,
                              "width": int(rgb_np.shape[1]), "height": int(rgb_np.shape[0]),
                              "aligned_depth_to_rgb": True, "rgb_device_timestamp_us": int(rgb.get_timestamp_us()),
                              "depth_device_timestamp_us": int(dep.get_timestamp_us()),
                              "host_monotonic_ns": time.monotonic_ns(), "host_realtime_ns": time.time_ns(),
                              "rgb_format": "rgb8", "depth_unit": "mm", "rgb_dtype": "uint8", "depth_dtype": "uint16",
                              "rgb_stride_bytes": int(rgb_np.strides[0]), "depth_stride_bytes": int(depth_mm.strides[0]),
                              "depth_scale_mm_per_unit": depth_scale_mm,
                              "rgb_depth_pair_delta_us": abs(int(rgb.get_timestamp_us()) - int(dep.get_timestamp_us()))}
                    with self.lock: self.latest = (header, rgb_np, depth_mm)
                    self.count += 1
            except Exception as exc:
                self.last_error = repr(exc)
                LOG.exception("%s capture thread stopped", self.spec.role)
            finally:
                pipe.stop()
        self.thread = threading.Thread(target=loop, daemon=True, name=f"orbbec-{self.spec.role}")
        self.thread.start()

    def get(self):
        with self.lock: return self.latest

    def stop(self):
        self.running = False
        if self.thread: self.thread.join(timeout=2.0)


class DepthSpooler:
    """Append lossless RGB-D frames to NVMe without blocking camera dispatch.

    HEVC lossless depth encoding is too CPU-heavy for three 30 Hz streams on
    this Jetson.  Raw sequential chunks sustain NVMe throughput and retain
    every source frame; a later offline converter can inject them into a
    LeRobot depth-video representation without affecting teleoperation.
    """
    def __init__(self, roles: tuple[str, ...]):
        self.roles = roles
        self.root: str | None = None
        self.data: dict[str, object] = {}
        self.rgb: dict[str, object] = {}
        self.meta: dict[str, object] = {}
        self.queues: dict[str, queue.Queue] = {}
        self.workers: dict[str, threading.Thread] = {}
        self.last_sequence = {role: -1 for role in roles}
        self.written = {role: 0 for role in roles}
        self.dropped = {role: 0 for role in roles}

    def _close(self) -> None:
        for item_queue in self.queues.values():
            item_queue.put(None)
        for worker in self.workers.values():
            worker.join(timeout=10.0)
        for stream in [*self.data.values(), *self.rgb.values(), *self.meta.values()]:
            stream.close()
        self.root = None; self.data = {}; self.rgb = {}; self.meta = {}; self.queues = {}; self.workers = {}

    def _writer(self, role: str) -> None:
        item_queue = self.queues[role]
        while True:
            item = item_queue.get()
            if item is None:
                return
            header, rgb, depth = item
            rgb_offset = self.rgb[role].tell(); depth_offset = self.data[role].tell()
            self.rgb[role].write(rgb.tobytes()); self.data[role].write(depth.tobytes())
            record = dict(header)
            record.update({"rgb_storage": "rgb24", "rgb_offset_bytes": rgb_offset,
                           "rgb_nbytes": int(rgb.nbytes), "storage": "u16le",
                           "offset_bytes": depth_offset, "nbytes": int(depth.nbytes)})
            self.meta[role].write(json.dumps(record) + "\n")
            self.written[role] += 1

    def update(self, latest: dict) -> None:
        marker = "/tmp/openarm-rgbd-recording.active"
        try:
            with open(marker, encoding="utf-8") as f:
                root = f.read().strip()
        except FileNotFoundError:
            if self.root is not None:
                self._close()
            return
        if not root:
            return
        if root != self.root:
            if self.root is not None:
                self._close()
            out = os.path.join(root, "depth_raw")
            os.makedirs(out, exist_ok=True)
            rgb_out = os.path.join(root, "rgb_raw")
            os.makedirs(rgb_out, exist_ok=True)
            self.root = root
            self.data = {role: open(os.path.join(out, f"{role}.u16le"), "ab", buffering=1024 * 1024) for role in self.roles}
            self.rgb = {role: open(os.path.join(rgb_out, f"{role}.rgb24"), "ab", buffering=1024 * 1024) for role in self.roles}
            self.meta = {role: open(os.path.join(out, f"{role}.jsonl"), "a", buffering=1024 * 1024) for role in self.roles}
            self.queues = {role: queue.Queue(maxsize=8) for role in self.roles}
            self.workers = {role: threading.Thread(target=self._writer, args=(role,), daemon=True,
                                                    name=f"rgbd-spool-{role}") for role in self.roles}
            for worker in self.workers.values(): worker.start()
            self.last_sequence = {role: -1 for role in self.roles}
            self.written = {role: 0 for role in self.roles}
            self.dropped = {role: 0 for role in self.roles}
        for role, (header, rgb, depth) in latest.items():
            if header["frame_sequence"] == self.last_sequence[role]:
                continue
            try:
                self.queues[role].put_nowait((header, rgb, depth))
                self.last_sequence[role] = header["frame_sequence"]
            except queue.Full:
                # Never stall capture.  A queue overflow is explicit in the
                # health log and makes the episode invalid for training.
                self.dropped[role] += 1


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
    # Preview is deliberately best-effort.  Ten FPS remains responsive for
    # teleoperation while leaving the 30 FPS RGB-D recording path priority.
    p.add_argument("--preview-fps", type=int, default=15); p.add_argument("--record-preview-fps", type=int, default=10)
    p.add_argument("--quality", type=int, default=75)
    p.add_argument("--metadata-dir", default="/tmp/openarm-rgbd-metadata")
    args = p.parse_args(); logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try: os.unlink(args.ipc.removeprefix("ipc://"))
    except FileNotFoundError: pass
    cameras = [OrbbecCamera(s, 640, 480, args.fps) for s in load_specs(args.config)]
    for camera in cameras: camera.start()
    os.makedirs(args.metadata_dir, exist_ok=True)
    metadata = {camera.spec.role: open(os.path.join(args.metadata_dir, f"{camera.spec.role}.jsonl"), "a", buffering=1)
                for camera in cameras}
    depth_spooler = DepthSpooler(tuple(camera.spec.role for camera in cameras))
    ctx = zmq.Context(); raw = ctx.socket(zmq.PUB); raw.setsockopt(zmq.SNDHWM, 12); raw.bind(args.ipc)
    preview = ctx.socket(zmq.PUB); preview.setsockopt(zmq.SNDHWM, 1); preview.setsockopt(zmq.LINGER, 0)
    preview.bind(f"tcp://*:{args.preview_port}")
    last_seq = {c.spec.role: -1 for c in cameras}; next_preview = 0.0; sent = 0; report = time.monotonic()
    report_counts = {c.spec.role: 0 for c in cameras}
    try:
        while True:
            latest = {}
            for camera in cameras:
                item = camera.get()
                if not item: continue
                header, rgb, depth = item; latest[camera.spec.role] = item
                if header["frame_sequence"] != last_seq[camera.spec.role]:
                    try:
                        # RGB-only subscribers are the production recording
                        # path.  They must never pay for a second Python copy
                        # of Depth: the camera owner spools exact uint16 data
                        # directly to NVMe below.
                        raw.send_multipart([f"rgb/{camera.spec.role}".encode(), json.dumps(header).encode(), rgb.tobytes()], flags=zmq.NOBLOCK)
                    except zmq.Again:
                        camera.dropped += 1
                    metadata[camera.spec.role].write(json.dumps(header) + "\n")
                    last_seq[camera.spec.role] = header["frame_sequence"]
            now = time.monotonic()
            depth_spooler.update(latest)
            recording = os.path.exists("/tmp/openarm-rgbd-recording.active")
            effective_preview_fps = args.record_preview_fps if recording else args.preview_fps
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
                next_preview = now + 1.0 / effective_preview_fps
            if now - report >= 2:
                elapsed = now - report
                status = {}
                for camera in cameras:
                    item = camera.get()
                    header = item[0] if item else None
                    age_ms = None if header is None else (time.monotonic_ns() - header["host_monotonic_ns"]) / 1e6
                    status[camera.spec.role] = {
                        "rgbd_fps": round((camera.count - report_counts[camera.spec.role]) / elapsed, 1),
                        "age_ms": None if age_ms is None else round(age_ms, 1),
                        "pair_us": None if header is None else header["rgb_depth_pair_delta_us"],
                        "drop": camera.dropped, "healthy": age_ms is not None and age_ms <= 100,
                        "error": camera.last_error,
                    }
                    report_counts[camera.spec.role] = camera.count
                LOG.info("capture=%s preview_fps=%.1f rgbd_spool=%s spool_drop=%s", status,
                         sent / elapsed, depth_spooler.written, depth_spooler.dropped)
                report, sent = now, 0
            time.sleep(0.001)
    except KeyboardInterrupt: pass
    finally:
        for camera in cameras: camera.stop()
        depth_spooler._close()
        for stream in metadata.values(): stream.close()
        raw.close(0); preview.close(0); ctx.term()

if __name__ == "__main__": main()

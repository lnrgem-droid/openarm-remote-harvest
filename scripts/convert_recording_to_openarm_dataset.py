#!/usr/bin/env python3
"""Convert one Jetson RGB-D recording into the official OpenArmDataset v0.4.

The source recording is intentionally left untouched.  The official OpenArm
format stores RGB camera frames as timestamped JPEG files; its current schema
does not define a depth-camera feature.  Depth therefore remains lossless in
the source recording and is referenced by ``depth_sidecar_manifest.json``.

Run the official second step after this conversion:

  openarm-dataset-validate <openarm_dataset_dir>
  openarm-dataset-convert <openarm_dataset_dir> <lerobot_v3_dir> \
    --format lerobot_v3.0 --fps 30 --state qpos
"""
from __future__ import annotations

import argparse
import json
import shutil
import warnings
from bisect import bisect_left
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import pyarrow.parquet as pq
import yaml


ROLES = ("left_wrist", "right_wrist", "chest")
CAMERA_NAMES = {"left_wrist": "wrist_left", "right_wrist": "wrist_right", "chest": "chest"}
JOINTS = [f"joint{i}" for i in range(1, 8)] + ["gripper"]


def recording_time_origin(source: Path, camera_times: dict[str, list[int]]) -> tuple[int, str]:
    """Use the clock anchor that belongs to the recorded robot timestamps."""
    info_path = source / "meta" / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8")) if info_path.exists() else {}
    if "started_unix_ns" in info:
        origin = info["started_unix_ns"]
        if type(origin) is not int or origin <= 0:
            raise ValueError("meta/info.json started_unix_ns must be a positive integer")
        return origin, "meta/info.json:started_unix_ns"
    if info.get("format") in {"openarm_rgbd_staging_v1", "openarm_left_rgbd_staging_v1"}:
        raise ValueError("staging recording is missing meta/info.json started_unix_ns")
    # Older LeRobot recordings have no absolute robot clock anchor. Keep the
    # historical approximation explicit; never silently replace a broken anchor.
    warnings.warn("Legacy recording has no robot time origin; using first camera "
                  "timestamp as an approximation.", RuntimeWarning, stacklevel=2)
    return min(values[0] for values in camera_times.values()), "first_aligned_camera_host_realtime_ns"


def read_camera_index(source: Path, role: str) -> list[dict]:
    entries: list[dict] = []
    with (source / "depth_raw" / f"{role}.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("role") != role or not item.get("aligned_depth_to_rgb"):
                continue
            entries.append(item)
    if not entries:
        raise ValueError(f"{role}: no aligned RGB-D frames in sidecar")
    return entries


def nearest(entries: list[dict], timestamps_ns: list[int], target_ns: int) -> dict:
    index = bisect_left(timestamps_ns, target_ns)
    candidates = entries[max(0, index - 1): index + 1]
    return min(candidates, key=lambda item: abs(int(item["host_realtime_ns"]) - target_ns))


def read_rgb(source: Path, role: str, item: dict) -> np.ndarray:
    height, width = int(item["height"]), int(item["width"])
    offset, nbytes = int(item["rgb_offset_bytes"]), int(item["rgb_nbytes"])
    with (source / "rgb_raw" / f"{role}.rgb24").open("rb") as stream:
        stream.seek(offset)
        raw = stream.read(nbytes)
    expected = height * width * 3
    if len(raw) != expected:
        raise ValueError(f"{role}: RGB frame has {len(raw)} bytes, expected {expected}")
    return np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)


def recording_camera_roles(source: Path, *, left_only: bool = False) -> tuple[str, ...]:
    config = source / "capture-config.json"
    if config.exists():
        roles = json.loads(config.read_text(encoding="utf-8"))["camera_roles"]
        if (not isinstance(roles, list) or not roles or any(not isinstance(r, str) for r in roles)
                or len(set(roles)) != len(roles) or not set(roles).issubset(ROLES)):
            raise ValueError("invalid recording camera_roles")
        return tuple(roles)
    # Legacy recordings retain their original required camera contract.
    return ("left_wrist", "chest") if left_only else ROLES


def metadata(task: str, camera_fps: float, success: bool, *, left_only: bool = False,
             camera_roles: tuple[str, ...] | None = None) -> dict:
    sides = ("left",) if left_only else ("right", "left")
    roles = camera_roles if camera_roles is not None else (("left_wrist", "chest") if left_only else ROLES)
    cameras = tuple(CAMERA_NAMES[role] for role in roles)
    return {
        "version": "0.4.0",
        "operator": "openarm remote teleoperation",
        "operation_type": "teleop",
        "location": "Jetson follower workstation",
        "tasks": [{"prompt": task, "description": "Single-left mushroom-log teleoperation." if left_only else "Bimanual mushroom-harvesting teleoperation."}],
        "episodes": [{"id": "0", "task_index": 0, "success": success}],
        "equipment": {
            "id": "OpenArmRemoteHarvest",
            "version": "1.0",
            "embodiments": {"arms": {"id": "OpenArm", "version": "2.0"}},
            "perceptions": {"cameras": {name: {} for name in cameras}},
        },
        "frequencies": {
            "obs": {"arms": {side: 30.0 for side in sides}},
            "action": {"arms": {side: 30.0 for side in sides}},
            "cameras": {name: camera_fps for name in cameras},
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="one /home/nvidia/datasets/openarm_rgbd_* directory")
    parser.add_argument("output", type=Path, help="new empty OpenArmDataset v0.4 directory")
    parser.add_argument("--task", default=None)
    parser.add_argument("--success", action="store_true", help="mark this episode successful (default: false)")
    parser.add_argument("--max-camera-skew-ms", type=float, default=100.0,
                        help="reject a robot/RGB nearest-frame match beyond this limit")
    parser.add_argument("--camera-time-offset-ms", type=float, default=0.0,
                        help="manual correction if visual validation finds a constant RGB/robot offset")
    args = parser.parse_args()

    source, output = args.source.resolve(), args.output.resolve()
    info_path = source / "meta/info.json"
    info = json.loads(info_path.read_text()) if info_path.exists() else {}
    left_only = info.get("format") == "openarm_left_rgbd_staging_v1"
    roles = recording_camera_roles(source, left_only=left_only)
    sides = ("left",) if left_only else ("left", "right")
    vector_size = 8 if left_only else 16
    if left_only and (info.get("active_arms") != ["left"] or info.get("vector_size") != 8):
        raise SystemExit("invalid single-left recording contract")
    task = args.task or info.get("task") or "bimanual mushroom harvesting teleoperation"
    if not (source / "data" / "chunk-000" / "file-000.parquet").is_file():
        raise SystemExit(f"not a supported Jetson recording: {source}")
    if output.exists():
        raise SystemExit(f"refusing to overwrite existing output: {output}")

    table = pq.read_table(source / "data" / "chunk-000" / "file-000.parquet")
    rows = table.select(["timestamp", "observation.state", "action"]).to_pylist()
    if not rows:
        raise SystemExit("recording has no robot rows")
    for row in rows:
        if len(row["observation.state"]) != vector_size or len(row["action"]) != vector_size:
            raise SystemExit(f"expected {vector_size}-D state/action vectors")
        if not np.isfinite([row["timestamp"], *row["observation.state"], *row["action"]]).all():
            raise SystemExit("non-finite robot sample")

    indexes = {role: read_camera_index(source, role) for role in roles}
    times = {role: [int(item["host_realtime_ns"]) for item in indexes[role]] for role in roles}
    origin_ns, origin_source = recording_time_origin(source, times)
    offset_ns = int(args.camera_time_offset_ms * 1_000_000)

    target_ns = [origin_ns + int(float(row["timestamp"]) * 1_000_000_000) + offset_ns for row in rows]
    selected: dict[str, list[dict]] = {role: [nearest(indexes[role], times[role], t) for t in target_ns] for role in roles}
    max_skew_ns = int(args.max_camera_skew_ms * 1_000_000)
    skew = {role: [abs(int(frame["host_realtime_ns"]) - target) for frame, target in zip(selected[role], target_ns)] for role in roles}
    failures = {role: sum(value > max_skew_ns for value in values) for role, values in skew.items()}
    if any(failures.values()):
        raise SystemExit(f"camera matching exceeds {args.max_camera_skew_ms} ms: {failures}")

    # Do not create a partial official dataset until all alignment checks pass.
    episode = output / "episodes" / "0"
    camera_dirs = {role: episode / "cameras" / CAMERA_NAMES[role] for role in roles}
    for path in [*(episode / kind / "arms" / side for kind in ("obs", "action") for side in sides),
                 *camera_dirs.values()]:
        path.mkdir(parents=True, exist_ok=False)

    timestamps = pd.to_datetime(np.asarray(target_ns, dtype=np.int64), unit="ns")
    for kind, vector_key in (("obs", "observation.state"), ("action", "action")):
        vector = np.asarray([row[vector_key] for row in rows], dtype=np.float32)
        # The OpenArmBridge feature insertion order is LEFT_JOINTS followed by
        # RIGHT_JOINTS.  The official dataset stores each side in a named
        # directory, so split the staging vector using that actual contract.
        # Reversing these slices silently produces a structurally valid but
        # semantically unusable training dataset.
        for index, side in enumerate(sides):
            values = vector[:, index * 8:(index + 1) * 8]
            frame = pd.DataFrame({"timestamp": timestamps, "qpos": list(values)})
            frame.to_parquet(episode / kind / "arms" / side / "state.parquet", index=False)

    for role in roles:
        for target, item in zip(target_ns, selected[role]):
            rgb = read_rgb(source, role, item)
            # OpenCV encodes BGR; raw Orbbec data is RGB.
            ok, encoded = cv2.imencode(".jpeg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError(f"{role}: JPEG encoding failed")
            (camera_dirs[role] / f"{target}.jpeg").write_bytes(encoded.tobytes())

    with (output / "metadata.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata(task, 30.0, args.success, left_only=left_only, camera_roles=roles), stream, sort_keys=False, allow_unicode=True)
    manifest = {
        "schema_version": 1,
        "source_recording": str(source),
        "camera_roles": list(roles),
        "depth_storage": "source depth_raw/*.u16le with matching JSONL indexes; unmodified",
        "rgb_time_anchor": f"{origin_source} + LeRobot relative timestamp + camera_time_offset_ms",
        "robot_time_origin_ns": origin_ns,
        "robot_time_origin_source": origin_source,
        "time_origin_is_approximate": origin_source == "first_aligned_camera_host_realtime_ns",
        "camera_time_offset_ms": args.camera_time_offset_ms,
        "max_camera_skew_ms": args.max_camera_skew_ms,
        "frames": len(rows),
        "camera_match": {
            role: {"max_skew_ms": max(values) / 1e6, "mean_skew_ms": float(np.mean(values)) / 1e6,
                   "over_limit": failures[role]} for role, values in skew.items()
        },
        "note": "Official OpenArmDataset v0.4 carries RGB cameras only. Depth remains a lossless sidecar."
    }
    (output / "depth_sidecar_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"openarm_dataset": str(output), **manifest}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

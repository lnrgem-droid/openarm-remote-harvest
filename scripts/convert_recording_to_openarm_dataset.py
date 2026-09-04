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


def metadata(task: str, camera_fps: float, success: bool) -> dict:
    return {
        "version": "0.4.0",
        "operator": "openarm remote teleoperation",
        "operation_type": "teleop",
        "location": "Jetson follower workstation",
        "tasks": [{"prompt": task, "description": "Bimanual mushroom-harvesting teleoperation."}],
        "episodes": [{"id": "0", "task_index": 0, "success": success}],
        "equipment": {
            "id": "OpenArmRemoteHarvest",
            "version": "1.0",
            "embodiments": {"arms": {"id": "OpenArm", "version": "2.0"}},
            "perceptions": {"cameras": {name: {} for name in CAMERA_NAMES.values()}},
        },
        "frequencies": {
            "obs": {"arms": {"right": 30.0, "left": 30.0}},
            "action": {"arms": {"right": 30.0, "left": 30.0}},
            "cameras": {name: camera_fps for name in CAMERA_NAMES.values()},
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="one /home/nvidia/datasets/openarm_rgbd_* directory")
    parser.add_argument("output", type=Path, help="new empty OpenArmDataset v0.4 directory")
    parser.add_argument("--task", default="bimanual mushroom harvesting teleoperation")
    parser.add_argument("--success", action="store_true", help="mark this episode successful (default: false)")
    parser.add_argument("--max-camera-skew-ms", type=float, default=100.0,
                        help="reject a robot/RGB nearest-frame match beyond this limit")
    parser.add_argument("--camera-time-offset-ms", type=float, default=0.0,
                        help="manual correction if visual validation finds a constant RGB/robot offset")
    args = parser.parse_args()

    source, output = args.source.resolve(), args.output.resolve()
    if not (source / "data" / "chunk-000" / "file-000.parquet").is_file():
        raise SystemExit(f"not a supported Jetson recording: {source}")
    if output.exists():
        raise SystemExit(f"refusing to overwrite existing output: {output}")

    table = pq.read_table(source / "data" / "chunk-000" / "file-000.parquet")
    rows = table.select(["timestamp", "observation.state", "action"]).to_pylist()
    if not rows:
        raise SystemExit("recording has no robot rows")
    for row in rows:
        if len(row["observation.state"]) != 16 or len(row["action"]) != 16:
            raise SystemExit("expected 16-D right[8] + left[8] state/action vectors")

    indexes = {role: read_camera_index(source, role) for role in ROLES}
    times = {role: [int(item["host_realtime_ns"]) for item in indexes[role]] for role in ROLES}
    # LeRobot timestamps are relative.  For recordings made before an explicit
    # robot absolute-time sidecar exists, use the first RGB-D frame as t=0 and
    # record this assumption in the report; later recordings can provide a
    # validated constant correction with --camera-time-offset-ms.
    origin_ns = min(times[role][0] for role in ROLES)
    offset_ns = int(args.camera_time_offset_ms * 1_000_000)

    target_ns = [origin_ns + int(float(row["timestamp"]) * 1_000_000_000) + offset_ns for row in rows]
    selected: dict[str, list[dict]] = {role: [nearest(indexes[role], times[role], t) for t in target_ns] for role in ROLES}
    max_skew_ns = int(args.max_camera_skew_ms * 1_000_000)
    skew = {role: [abs(int(frame["host_realtime_ns"]) - target) for frame, target in zip(selected[role], target_ns)] for role in ROLES}
    failures = {role: sum(value > max_skew_ns for value in values) for role, values in skew.items()}
    if any(failures.values()):
        raise SystemExit(f"camera matching exceeds {args.max_camera_skew_ms} ms: {failures}")

    # Do not create a partial official dataset until all alignment checks pass.
    episode = output / "episodes" / "0"
    camera_dirs = {role: episode / "cameras" / CAMERA_NAMES[role] for role in ROLES}
    for path in [episode / "obs" / "arms" / "right", episode / "obs" / "arms" / "left",
                 episode / "action" / "arms" / "right", episode / "action" / "arms" / "left",
                 *camera_dirs.values()]:
        path.mkdir(parents=True, exist_ok=False)

    timestamps = pd.to_datetime(np.asarray(target_ns, dtype=np.int64), unit="ns")
    for kind, vector_key in (("obs", "observation.state"), ("action", "action")):
        vector = np.asarray([row[vector_key] for row in rows], dtype=np.float32)
        for side, values in (("right", vector[:, :8]), ("left", vector[:, 8:])):
            frame = pd.DataFrame({"timestamp": timestamps, "qpos": list(values)})
            frame.to_parquet(episode / kind / "arms" / side / "state.parquet", index=False)

    for role in ROLES:
        for target, item in zip(target_ns, selected[role]):
            rgb = read_rgb(source, role, item)
            # OpenCV encodes BGR; raw Orbbec data is RGB.
            ok, encoded = cv2.imencode(".jpeg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError(f"{role}: JPEG encoding failed")
            (camera_dirs[role] / f"{target}.jpeg").write_bytes(encoded.tobytes())

    with (output / "metadata.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata(args.task, 30.0, args.success), stream, sort_keys=False, allow_unicode=True)
    manifest = {
        "schema_version": 1,
        "source_recording": str(source),
        "depth_storage": "source depth_raw/*.u16le with matching JSONL indexes; unmodified",
        "rgb_time_anchor": "first_aligned_camera_host_realtime_ns + LeRobot relative timestamp + camera_time_offset_ms",
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

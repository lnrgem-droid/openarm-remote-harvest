#!/usr/bin/env python3
"""Lightweight 30 Hz OpenArm state/action staging recorder.

RGB-D bytes are written by the independent camera owner after the recording
manager publishes its active marker.  This process only subscribes to the
read-only follower bridge and records the exact vectors required by the
offline OpenArmDataset converter.  It deliberately avoids importing Torch or
LeRobot, which took 10-12 seconds on Jetson for every episode.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import signal
import termios
import time
import tty
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq
from websockets.sync.client import connect


GRIPPER_OPEN_M = 0.044
stop_requested = False


def request_stop(*_args: object) -> None:
    global stop_requested
    stop_requested = True


def normalized_state(data: dict) -> list[float]:
    left = list(data["left_arm"]["position"][:8])
    right = list(data["right_arm"]["position"][:8])
    if len(left) != 8 or len(right) != 8:
        raise ValueError("follower state must contain left[8] + right[8]")
    left[7] = max(0.0, min(1.0, float(left[7]) / GRIPPER_OPEN_M))
    right[7] = max(0.0, min(1.0, float(right[7]) / GRIPPER_OPEN_M))
    return [float(v) for v in (*left, *right)]


def applied_action(data: dict) -> list[float]:
    action = data.get("teleop_action", {})
    if action.get("valid") is not True:
        raise ValueError("teleoperation action is not valid")
    left, right = list(action.get("left", [])), list(action.get("right", []))
    if len(left) != 8 or len(right) != 8:
        raise ValueError("action must contain left[8] + right[8]")
    return [float(v) for v in (*left, *right)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--ws-url", default="ws://127.0.0.1:9000")
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()
    if args.root.exists():
        raise SystemExit(f"refusing to overwrite existing root: {args.root}")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    # The manager sends a single `q` byte through a PTY. Canonical terminal
    # mode buffers it until Enter, which would force the 12 s shutdown
    # watchdog to intervene. Match LeRobot's keyboard listener and consume
    # control characters immediately.
    if os.isatty(0):
        tty.setcbreak(0, termios.TCSANOW)
    period = 1.0 / args.fps
    rows: list[dict] = []
    started_mono: float | None = None
    started_unix_ns: int | None = None

    with connect(args.ws_url, open_timeout=5, close_timeout=2) as ws:
        # Do not claim the episode directory until both state and the applied
        # action are valid. The manager uses this mkdir as its RECORDING gate.
        while not stop_requested:
            raw = ws.recv(timeout=5)
            message = json.loads(raw)
            if message.get("type") != "state":
                continue
            data = message.get("data", {})
            try:
                state = normalized_state(data)
                action = applied_action(data)
            except (KeyError, TypeError, ValueError):
                continue
            args.root.mkdir(parents=True, exist_ok=False)
            (args.root / "data" / "chunk-000").mkdir(parents=True)
            (args.root / "meta").mkdir()
            started_mono = time.monotonic()
            started_unix_ns = time.time_ns()
            break

        next_sample = time.monotonic()
        latest: tuple[list[float], list[float]] | None = (state, action) if started_mono is not None else None
        while not stop_requested and started_mono is not None:
            if select.select([0], [], [], 0)[0] and os.read(0, 64).lower().find(b"q") >= 0:
                break
            timeout = max(0.0, min(period, next_sample - time.monotonic()))
            try:
                raw = ws.recv(timeout=timeout)
                message = json.loads(raw)
                if message.get("type") == "state":
                    data = message.get("data", {})
                    latest = (normalized_state(data), applied_action(data))
            except TimeoutError:
                pass
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue
            now = time.monotonic()
            if now < next_sample or latest is None:
                continue
            state, action = latest
            index = len(rows)
            rows.append({
                "timestamp": now - started_mono,
                "observation.state": state,
                "action": action,
                "episode_index": 0,
                "frame_index": index,
                "index": index,
                "task_index": 0,
            })
            next_sample += period
            if next_sample < now - period:
                next_sample = now + period

    if not rows:
        return 2
    vector = pa.list_(pa.float32(), 16)
    table = pa.Table.from_arrays(
        [
            pa.array([r["timestamp"] for r in rows], type=pa.float32()),
            pa.array([r["observation.state"] for r in rows], type=vector),
            pa.array([r["action"] for r in rows], type=vector),
            pa.array([r["episode_index"] for r in rows], type=pa.int64()),
            pa.array([r["frame_index"] for r in rows], type=pa.int64()),
            pa.array([r["index"] for r in rows], type=pa.int64()),
            pa.array([r["task_index"] for r in rows], type=pa.int64()),
        ],
        names=["timestamp", "observation.state", "action", "episode_index", "frame_index", "index", "task_index"],
    )
    pq.write_table(table, args.root / "data" / "chunk-000" / "file-000.parquet", compression="zstd")
    info = {
        "format": "openarm_rgbd_staging_v1",
        "fps": args.fps,
        "task": args.task,
        "total_frames": len(rows),
        "started_unix_ns": started_unix_ns,
        "duration_s": float(rows[-1]["timestamp"]),
        "vector_order": "left_joint1..7,left_gripper,right_joint1..7,right_gripper",
        "conversion": "convert_recording_to_openarm_dataset.py then official openarm-dataset-convert",
    }
    (args.root / "meta" / "info.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(info, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

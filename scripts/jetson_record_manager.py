#!/usr/bin/env python3
"""Jetson-local lifecycle manager for LeRobot RGB-D recording.

The host viewer can only request start/stop/status through this narrow ZMQ
control socket.  It cannot access CAN, camera devices, or ROS commands.
"""
from __future__ import annotations

import argparse
import json
import os
import pty
import select
import subprocess
import time
from pathlib import Path

import zmq


class Recorder:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.process: subprocess.Popen[bytes] | None = None
        self.pty_master: int | None = None
        self.started: float | None = None
        self.last_log = "idle"
        self.dataset_root: str | None = None
        self.log_path = Path("/home/nvidia/openarm-rgbd-runtime/record-last.log")

    def status(self) -> dict:
        running = self.process is not None and self.process.poll() is None
        return {"running": running, "started_unix_s": self.started,
                "dataset_root": self.dataset_root, "last_log": self.last_log[-240:]}

    def _drain(self) -> None:
        if self.pty_master is None:
            return
        while select.select([self.pty_master], [], [], 0)[0]:
            try:
                chunk = os.read(self.pty_master, 4096).decode(errors="replace")
            except OSError:
                break
            self.last_log = (self.last_log + chunk)[-4000:]
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as log:
                log.write(chunk)

    def start(self) -> dict:
        self._drain()
        if self.status()["running"]:
            return {"ok": False, "error": "recording is already running", **self.status()}
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dataset_root = f"/home/nvidia/datasets/openarm_rgbd_{stamp}"
        master, slave = pty.openpty()
        env = os.environ | {"DATASET_ID": "openarm/mushroom-rgbd", "DATASET_ROOT": self.dataset_root}
        command = [str(self.root / "scripts" / "record_jetson_rgbd_dataset.sh")]
        self.process = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave,
                                        cwd=self.root, env=env, start_new_session=True)
        os.close(slave)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text("", encoding="utf-8")
        self.pty_master, self.started, self.last_log = master, time.time(), "starting"
        return {"ok": True, **self.status()}

    def stop(self) -> dict:
        self._drain()
        if not self.status()["running"]:
            return {"ok": False, "error": "recording is not running", **self.status()}
        assert self.pty_master is not None
        os.write(self.pty_master, b"q")
        return {"ok": True, "message": "stop requested; saving episode", **self.status()}


def main() -> None:
    parser = argparse.ArgumentParser()
    # Private wired robot LAN: host UI can request recording, but this service
    # only manages LeRobot persistence and has no CAN/ROS control interface.
    parser.add_argument("--bind", default="tcp://*:5557")
    parser.add_argument("--root", default="/home/nvidia/dev/openarm-rgbd-preview")
    args = parser.parse_args()
    recorder = Recorder(Path(args.root))
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.bind)
    try:
        while True:
            command = socket.recv_json().get("command")
            recorder._drain()
            if command == "start":
                response = recorder.start()
            elif command == "stop":
                response = recorder.stop()
            elif command == "status":
                response = {"ok": True, **recorder.status()}
            else:
                response = {"ok": False, "error": "commands: start, stop, status"}
            socket.send_json(response)
    finally:
        socket.close(0)
        context.term()


if __name__ == "__main__":
    main()

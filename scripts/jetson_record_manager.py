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
import shutil
import signal
import subprocess
import threading
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
        self.active_marker = Path("/tmp/openarm-rgbd-recording.active")
        self.error_marker = Path("/tmp/openarm-rgbd-recording.error")
        self.phase = "idle"
        self.stop_reason: str | None = None
        self._generation = 0
        self._stop_requested = threading.Event()
        self._lock = threading.RLock()
        self.min_free_gb = 10.0

    def status(self) -> dict:
        process = self.process
        running = process is not None and process.poll() is None
        if not running:
            self.active_marker.unlink(missing_ok=True)
            if self.phase in {"starting", "recording", "stopping"}:
                returncode = None if process is None else process.returncode
                self.phase = "idle" if returncode in {None, 0} else "error"
        free_gb = shutil.disk_usage("/home/nvidia/datasets").free / (1024 ** 3)
        return {"running": running, "phase": self.phase,
                "stop_reason": self.stop_reason, "free_gb": round(free_gb, 1),
                "started_unix_s": self.started, "dataset_root": self.dataset_root,
                "last_log": self.last_log[-240:]}

    def _drain_loop(self, process: subprocess.Popen[bytes], master: int, generation: int) -> None:
        """Continuously drain the PTY so logging can never block recording."""
        while process.poll() is None or select.select([master], [], [], 0)[0]:
            if generation != self._generation:
                return
            if not select.select([master], [], [], 0.25)[0]:
                continue
            try:
                chunk = os.read(master, 4096).decode(errors="replace")
            except OSError:
                break
            with self._lock:
                self.last_log = (self.last_log + chunk)[-4000:]
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as log:
                log.write(chunk)

    def _activate_depth_spool(self, dataset_path: Path, generation: int) -> None:
        """Enable depth only after LeRobot has claimed its empty root."""
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if (generation != self._generation or self._stop_requested.is_set()
                    or self.process is None or self.process.poll() is not None):
                return
            if dataset_path.is_dir():
                if self._stop_requested.is_set() or generation != self._generation:
                    return
                self.active_marker.write_text(str(dataset_path) + "\n", encoding="utf-8")
                with self._lock:
                    self.phase = "recording"
                return
            time.sleep(0.05)

    def _clear_marker_when_recording_exits(self, process: subprocess.Popen[bytes], generation: int) -> None:
        """Never leave a camera spool attached to a completed episode."""
        returncode = process.wait()
        if generation == self._generation:
            self.active_marker.unlink(missing_ok=True)
            with self._lock:
                self.phase = "idle" if returncode == 0 else "error"

    def _force_stop_after_timeout(self, process: subprocess.Popen[bytes], generation: int) -> None:
        """Escalate only the recorder process group if graceful q is ignored."""
        try:
            process.wait(timeout=15.0)
            return
        except subprocess.TimeoutExpired:
            pass
        if generation != self._generation:
            return
        with self._lock:
            self.last_log = (self.last_log + "\nRecorder ignored q for 15 s; sent SIGINT.\n")[-4000:]
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    def _request_stop(self, reason: str) -> bool:
        process = self.process
        if process is None or process.poll() is not None:
            self.active_marker.unlink(missing_ok=True)
            return False
        first_request = not self._stop_requested.is_set()
        self._stop_requested.set()
        self.active_marker.unlink(missing_ok=True)
        with self._lock:
            self.phase = "stopping"
            self.stop_reason = reason
        if first_request and self.pty_master is not None:
            try:
                os.write(self.pty_master, b"q")
            except OSError:
                os.killpg(process.pid, signal.SIGINT)
            threading.Thread(target=self._force_stop_after_timeout,
                             args=(process, self._generation), daemon=True,
                             name="record-stop-timeout").start()
        return True

    def _monitor(self, process: subprocess.Popen[bytes], dataset_path: Path, generation: int) -> None:
        """Stop on low disk, camera/spool error, or deletion of an active root."""
        root_seen = False
        while process.poll() is None and generation == self._generation:
            root_seen = root_seen or dataset_path.exists()
            if root_seen and not dataset_path.exists():
                self._request_stop("dataset directory was removed while recording")
                return
            free_gb = shutil.disk_usage("/home/nvidia/datasets").free / (1024 ** 3)
            if free_gb < self.min_free_gb:
                self._request_stop(f"automatic stop: only {free_gb:.1f} GB free")
                return
            if self.error_marker.exists():
                try:
                    reason = self.error_marker.read_text(encoding="utf-8").strip()
                except OSError:
                    reason = "camera or RGB-D spool error"
                self._request_stop(reason or "camera or RGB-D spool error")
                return
            self._stop_requested.wait(1.0)
            if self._stop_requested.is_set():
                return

    def start(self) -> dict:
        if self.status()["running"]:
            return {"ok": False, "error": "recording is already running", **self.status()}
        free_gb = shutil.disk_usage("/home/nvidia/datasets").free / (1024 ** 3)
        if free_gb < 20.0:
            return {"ok": False, "error": f"only {free_gb:.1f} GB free; 20 GB required", **self.status()}
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.dataset_root = f"/home/nvidia/datasets/openarm_rgbd_{stamp}"
        self._generation += 1
        generation = self._generation
        self._stop_requested.clear()
        self.active_marker.unlink(missing_ok=True)
        self.error_marker.unlink(missing_ok=True)
        self.phase, self.stop_reason = "starting", None
        master, slave = pty.openpty()
        plugin_root = str(self.root / "lerobot_robot_openarm_bridge")
        python_path = os.environ.get("PYTHONPATH", "")
        env = os.environ | {
            "DATASET_ID": "openarm/mushroom-rgbd",
            "DATASET_ROOT": self.dataset_root,
            # LeRobot discovers locally developed robot types from import
            # paths. Make this explicit for manager-spawned (non-shell) jobs.
            "PYTHONPATH": plugin_root + (":" + python_path if python_path else ""),
        }
        command = [str(self.root / "scripts" / "record_jetson_rgbd_dataset.sh")]
        self.process = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave,
                                        cwd=self.root, env=env, start_new_session=True)
        os.close(slave)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text("", encoding="utf-8")
        self.pty_master, self.started, self.last_log = master, time.time(), "starting"
        process = self.process
        assert process is not None
        threading.Thread(target=self._drain_loop, args=(process, master, generation),
                         daemon=True, name="record-log-drain").start()
        # Do not block the UI/start request: package imports can take longer
        # than camera setup.  The helper waits in the background and activates
        # raw depth only once LeRobot has created the root itself.
        dataset_path = Path(self.dataset_root)
        threading.Thread(target=self._activate_depth_spool, args=(dataset_path, generation),
                         daemon=True, name="depth-spool-activation").start()
        threading.Thread(target=self._clear_marker_when_recording_exits, args=(process, generation),
                         daemon=True, name="depth-spool-cleanup").start()
        threading.Thread(target=self._monitor, args=(process, dataset_path, generation),
                         daemon=True, name="record-health-monitor").start()
        return {"ok": True, **self.status()}

    def stop(self) -> dict:
        if not self.status()["running"]:
            return {"ok": False, "error": "recording is not running", **self.status()}
        first_request = not self._stop_requested.is_set()
        self._request_stop("operator requested stop")
        message = "stop requested; finalizing" if first_request else "already stopping; duplicate ignored"
        return {"ok": True, "message": message, **self.status()}


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

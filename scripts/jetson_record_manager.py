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
        self.session_root: Path | None = None
        self.session_id: str | None = None
        self.session_started: float | None = None
        self.next_episode = 1
        self.active_episode: dict | None = None
        self.last_episode: dict | None = None
        self.session_base = Path("/home/nvidia/datasets/openarm_harvest_sessions")
        self.camera_status_path = Path("/tmp/openarm-rgbd-camera-status.json")
        self.session_state_path = Path("/home/nvidia/openarm-rgbd-runtime/active-session.json")
        self._restore_session()

    def _restore_session(self) -> None:
        """Recover an idle session after a recorder-manager restart.

        The active *episode* is never resumed after a service restart, but a
        completed-session folder remains usable so the operator can continue
        with the next numbered episode instead of creating stray roots.
        """
        try:
            state = json.loads(self.session_state_path.read_text(encoding="utf-8"))
            root = Path(state["session_root"])
            if not root.is_dir() or not (root / "episodes").is_dir():
                return
            self.session_root = root
            self.session_id = str(state["session_id"])
            self.session_started = float(state["session_started_unix_s"])
            existing = list((root / "episodes").glob("episode_*/episode.json"))
            self.next_episode = len(existing) + 1
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    def _persist_session(self) -> None:
        if self.session_root is None:
            self.session_state_path.unlink(missing_ok=True)
            return
        self._write_json(self.session_state_path, {
            "session_id": self.session_id, "session_root": str(self.session_root),
            "session_started_unix_s": self.session_started, "next_episode": self.next_episode,
        })

    def status(self) -> dict:
        process = self.process
        running = process is not None and process.poll() is None
        if not running:
            self.active_marker.unlink(missing_ok=True)
            if self.dataset_root and not Path(self.dataset_root).exists():
                self.dataset_root = None
            if self.phase in {"starting", "recording", "stopping"}:
                returncode = None if process is None else process.returncode
                cancelled_start = bool(self.stop_reason and self.stop_reason.endswith(" during startup"))
                self.phase = "idle" if returncode in {None, 0} or cancelled_start else "error"
        free_gb = shutil.disk_usage("/home/nvidia/datasets").free / (1024 ** 3)
        return {"running": running, "phase": self.phase,
                "stop_reason": self.stop_reason, "free_gb": round(free_gb, 1),
                "started_unix_s": self.started, "dataset_root": self.dataset_root,
                "last_log": self.last_log[-240:],
                "session_id": self.session_id,
                "session_root": None if self.session_root is None else str(self.session_root),
                "session_started_unix_s": self.session_started,
                "next_episode": self.next_episode,
                "active_episode": self.active_episode,
                "last_episode": self.last_episode,
                "camera_health": self._camera_health()}

    def _camera_health(self) -> dict:
        try:
            value = json.loads(self.camera_status_path.read_text(encoding="utf-8"))
            age_s = time.time() - float(value.get("updated_unix_s", 0))
            cameras = value.get("cameras", {})
            healthy = age_s <= 5 and set(cameras) == {"left_wrist", "right_wrist", "chest"} and all(
                bool(cameras[role].get("healthy")) for role in cameras)
            return {"ok": healthy, "age_s": round(age_s, 1), "detail": value}
        except Exception as exc:
            return {"ok": False, "age_s": None, "error": str(exc)}

    def _write_json(self, path: Path, value: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def _write_session_manifest(self) -> None:
        if self.session_root is None:
            return
        episodes = []
        for metadata in sorted((self.session_root / "episodes").glob("*/episode.json")):
            try:
                episodes.append(json.loads(metadata.read_text(encoding="utf-8")))
            except (OSError, json.JSONDecodeError):
                continue
        self._write_json(self.session_root / "session.json", {
            "schema_version": 1,
            "session_id": self.session_id,
            "session_started_unix_s": self.session_started,
            "session_ended_unix_s": None,
            "storage_contract": "Each episode contains a native LeRobot staging dataset plus raw RGB-D sidecars.",
            "camera_roles": ["left_wrist", "right_wrist", "chest"],
            "episodes": episodes,
        })

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

    def _finalize_episode(self, returncode: int | None) -> None:
        episode = self.active_episode
        if episode is None:
            return
        ended = time.time()
        requested = episode.get("requested_result")
        result = requested if requested in {"success", "failure", "aborted"} else "aborted"
        end_health = self._camera_health()
        spool_written = end_health.get("detail", {}).get("spool_written", {})
        spool_drop = end_health.get("detail", {}).get("spool_drop", {})
        frame_values = [int(spool_written.get(role, 0)) for role in ("left_wrist", "right_wrist", "chest")]
        frame_spread = max(frame_values) - min(frame_values) if frame_values else 0
        # "valid" is a collection eligibility flag, not a claim that the
        # grasp succeeded.  A successful operator label is necessary but not
        # sufficient: all three source streams must have frames, be healthy,
        # and report no camera-owner queue loss.  Full OpenArm validation and
        # RGB-D-to-LeRobot conversion remain an explicit offline step.
        valid = (result == "success" and returncode == 0 and end_health.get("ok")
                 and min(frame_values, default=0) >= 30
                 and all(int(spool_drop.get(role, 0)) == 0 for role in ("left_wrist", "right_wrist", "chest")))
        episode.update({
            "ended_unix_s": ended,
            "duration_s": round(ended - float(episode["started_unix_s"]), 3),
            "result": result,
            "valid": valid,
            "recorder_returncode": returncode,
            "stop_reason": self.stop_reason,
            "camera_health_at_end": end_health,
            "rgbd_spool_frame_count": dict(spool_written),
            "rgbd_spool_frame_spread": frame_spread,
            "collection_eligibility": "eligible" if valid else "needs_offline_review",
        })
        if result == "failure" and not episode.get("failure_code"):
            episode["failure_code"] = "unspecified_failure"
        self._write_json(Path(episode["episode_root"]) / "episode.json", episode)
        self.last_episode = dict(episode)
        self.active_episode = None
        self.next_episode += 1
        self._write_session_manifest()
        self._persist_session()

    def _clear_marker_when_recording_exits(self, process: subprocess.Popen[bytes], generation: int) -> None:
        """Never leave a camera spool attached to a completed episode."""
        returncode = process.wait()
        if generation == self._generation:
            self.active_marker.unlink(missing_ok=True)
            with self._lock:
                cancelled_start = bool(self.stop_reason and self.stop_reason.endswith(" during startup"))
                self.phase = "idle" if returncode == 0 or cancelled_start else "error"
                self._finalize_episode(returncode)

    def _force_stop_after_timeout(self, process: subprocess.Popen[bytes], generation: int) -> None:
        """Escalate only the recorder process group if graceful q is ignored."""
        try:
            process.wait(timeout=5.0)
            return
        except subprocess.TimeoutExpired:
            pass
        if generation != self._generation:
            return
        with self._lock:
            self.last_log = (self.last_log + "\nRecorder ignored q for 5 s; sent SIGINT.\n")[-4000:]
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=2.0)
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
        with self._lock:
            was_starting = self.phase == "starting"
        first_request = not self._stop_requested.is_set()
        self._stop_requested.set()
        self.active_marker.unlink(missing_ok=True)
        with self._lock:
            self.phase = "stopping"
            self.stop_reason = f"{reason} during startup" if was_starting else reason
        if first_request and was_starting:
            # LeRobot initializes its keyboard handler late and flushes input;
            # a q sent during STARTING can therefore be lost.  There is no
            # active episode to finalize yet, so cancel the exact recorder
            # process group immediately instead of making the operator wait.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            return True
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

    def start(self, *, dataset_root: str | None = None, task: str | None = None) -> dict:
        if self.status()["running"]:
            return {"ok": False, "error": "recording is already running", **self.status()}
        free_gb = shutil.disk_usage("/home/nvidia/datasets").free / (1024 ** 3)
        if free_gb < 20.0:
            return {"ok": False, "error": f"only {free_gb:.1f} GB free; 20 GB required", **self.status()}
        if dataset_root is None:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.dataset_root = f"/home/nvidia/datasets/openarm_rgbd_{stamp}"
        else:
            self.dataset_root = dataset_root
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
            "TASK": task or "bimanual mushroom harvesting teleoperation",
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

    def start_session(self) -> dict:
        if self.status()["running"]:
            return {"ok": False, "error": "cannot create a session while an episode is running", **self.status()}
        if self.session_root is not None:
            return {"ok": True, "message": "existing session retained", **self.status()}
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_id = f"mushroom_harvest_{stamp}"
        self.session_root = self.session_base / self.session_id
        self.session_started = time.time()
        self.next_episode = 1
        self.last_episode = None
        self.session_root.mkdir(parents=True, exist_ok=False)
        (self.session_root / "episodes").mkdir()
        self._write_session_manifest()
        self._persist_session()
        return {"ok": True, "message": "session ready; no recording yet", **self.status()}

    def start_episode(self, task: str, target: str = "") -> dict:
        if self.session_root is None:
            return {"ok": False, "error": "create a session first", **self.status()}
        if not task:
            return {"ok": False, "error": "task is required", **self.status()}
        health = self._camera_health()
        if not health.get("ok"):
            return {"ok": False, "error": "three RGB-D cameras are not healthy", **self.status()}
        episode_id = f"episode_{self.next_episode:04d}"
        episode_root = self.session_root / "episodes" / episode_id
        dataset_root = episode_root / "lerobot"
        episode_root.mkdir(parents=True, exist_ok=False)
        self.active_episode = {
            "schema_version": 1, "episode_id": episode_id,
            "episode_root": str(episode_root), "lerobot_root": str(dataset_root),
            "task": task, "target": target, "started_unix_s": time.time(),
            "camera_health_at_start": health, "result": None, "valid": None,
            "failure_code": None,
        }
        response = self.start(dataset_root=str(dataset_root), task=task)
        if not response.get("ok"):
            self.active_episode = None
            try: episode_root.rmdir()
            except OSError: pass
        return response

    def stop_episode(self, result: str, failure_code: str = "") -> dict:
        if result not in {"success", "failure", "aborted"}:
            return {"ok": False, "error": "result must be success, failure, or aborted", **self.status()}
        if self.active_episode is None:
            return {"ok": False, "error": "no active episode", **self.status()}
        self.active_episode["requested_result"] = result
        self.active_episode["failure_code"] = failure_code.strip() or None
        return self.stop()

    def close_session(self) -> dict:
        if self.status()["running"]:
            return {"ok": False, "error": "stop the active episode before closing the session", **self.status()}
        if self.session_root is None:
            return {"ok": True, "message": "no active session", **self.status()}
        manifest = self.session_root / "session.json"
        value = json.loads(manifest.read_text(encoding="utf-8"))
        value["session_ended_unix_s"] = time.time()
        self._write_json(manifest, value)
        response = {"ok": True, "message": "session closed", **self.status()}
        self.session_root = None; self.session_id = None; self.session_started = None
        self._persist_session()
        return response

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
            request = socket.recv_json()
            command = request.get("command")
            if command == "start":
                response = recorder.start()
            elif command == "stop":
                response = recorder.stop()
            elif command == "session_start":
                response = recorder.start_session()
            elif command == "episode_start":
                response = recorder.start_episode(str(request.get("task", "")), str(request.get("target", "")))
            elif command == "episode_stop":
                response = recorder.stop_episode(str(request.get("result", "aborted")), str(request.get("failure_code", "")))
            elif command == "session_close":
                response = recorder.close_session()
            elif command == "status":
                response = {"ok": True, **recorder.status()}
            else:
                response = {"ok": False, "error": "commands: status, session_start, episode_start, episode_stop, session_close"}
            socket.send_json(response)
    finally:
        socket.close(0)
        context.term()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Hardware-free tests for left/right episode storage contracts."""
from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "jetson_record_manager.py"
SPEC = importlib.util.spec_from_file_location("jetson_record_manager", SCRIPT)
manager = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(manager)


class RecorderLayoutTest(unittest.TestCase):
    def test_new_select_and_continue_session_modes_are_explicit(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            root = Path(temporary)
            recorder = manager.Recorder(root)
            recorder.allowed_storage_root = root
            recorder.session_base = root / "sessions"
            recorder.session_state_path = root / "active-session.json"

            first = recorder.start_session("new", str(root / "greenhouse_a"))
            first_root = Path(first["session_root"])
            self.assertTrue(first_root.is_dir())
            self.assertEqual(first["next_episode_by_task"]["left"], 1)
            recorder.close_session()

            selected = recorder.start_session("select", session_root=str(first_root))
            self.assertEqual(selected["session_root"], str(first_root))
            resumed = recorder.start_session("continue")
            self.assertEqual(resumed["session_root"], str(first_root))
            catalog = recorder.session_catalog()
            self.assertTrue(any(item["session_root"] == str(first_root) for item in catalog["sessions"]))

            with self.assertRaises(ValueError):
                recorder.start_session("new", "/etc")

    def test_separate_left_right_numbering_and_manifest(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            root = Path(temporary)
            recorder = manager.Recorder(root)
            recorder.allowed_storage_root = root
            recorder.session_base = root / "sessions"
            recorder.session_state_path = root / "active-session.json"
            state = recorder.start_session(); session = Path(state["session_root"])
            self.assertTrue((session / "episodes/left").is_dir())
            self.assertTrue((session / "episodes/right").is_dir())
            self.assertEqual(recorder._task_group("LEFT_GRASP_LOG"), "left")
            self.assertEqual(recorder._task_group("RIGHT_PICK_ONE"), "right")
            self.assertEqual(recorder._task_group("TEST_LEFT_GRASP_LOG"), "test")
            self.assertEqual(recorder.next_episode_by_task["left"], 1)
            self.assertEqual(recorder.next_episode_by_task["right"], 1)
            manifest = (session / "session.json").read_text(encoding="utf-8")
            self.assertIn("episodes/left/episode_NNNN", manifest)
            self.assertIn("episodes/right/episode_NNNN", manifest)

            recorder.active_episode = {"task": "LEFT_GRASP_LOG"}
            recorder.phase = "starting"
            response = recorder.stop_episode("success")
            self.assertFalse(response["ok"])
            self.assertIn("still starting", response["error"])

    def test_stale_directory_is_preserved_and_next_number_is_used(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            root = Path(temporary)
            recorder = manager.Recorder(root)
            recorder.allowed_storage_root = root
            recorder.session_base = root / "sessions"
            recorder.session_state_path = root / "active-session.json"
            recorder.start_session()
            stale = recorder.session_root / "episodes/left/episode_0001"
            stale.mkdir()
            recorder._camera_health = lambda: {"ok": True}
            captured = {}

            def fake_start(*, dataset_root=None, task=None):
                captured.update(dataset_root=dataset_root, task=task)
                return {"ok": True, **recorder.status()}

            recorder.start = fake_start
            response = recorder.start_episode("LEFT_GRASP_LOG")
            self.assertTrue(response["ok"])
            self.assertEqual(recorder.active_episode["episode_number"], 2)
            self.assertIn("episodes/left/episode_0002/lerobot", captured["dataset_root"])
            self.assertTrue(stale.is_dir())

    def test_duplicate_stop_is_idempotent_and_start_is_single_flight(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            root = Path(temporary)
            recorder = manager.Recorder(root)
            recorder.allowed_storage_root = root
            recorder.session_base = root / "sessions"
            recorder.session_state_path = root / "active-session.json"
            recorder.start_session()

            class FakeProcess:
                pid = 12345
                returncode = None
                def poll(self): return None

            recorder.process = FakeProcess()
            recorder.phase = "recording"
            recorder.active_episode = {"task": "LEFT_GRASP_LOG"}
            first = recorder.stop_episode("aborted", "test")
            second = recorder.stop_episode("aborted", "test")
            self.assertTrue(first["ok"])
            self.assertTrue(second["ok"])
            self.assertIn("duplicate", second["message"])

            start = recorder.start_episode("RIGHT_PICK_ONE")
            self.assertFalse(start["ok"])
            self.assertIn("already active", start["error"])

    def test_dispatch_bad_request_returns_error_without_crashing(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            recorder = manager.Recorder(Path(temporary))
            response = manager.dispatch_request(recorder, ["not", "an", "object"])
            self.assertFalse(response["ok"])
            self.assertIn("JSON object", response["error"])

    def test_recording_monitor_stops_on_stale_camera_health(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            recorder = manager.Recorder(Path(temporary))
            recorder.error_marker = Path(temporary) / "no-error"
            recorder._camera_health = lambda: {"ok": False}
            reasons = []
            recorder._request_stop = lambda reason: reasons.append(reason) or True

            class FakeProcess:
                def poll(self): return None

            recorder._monitor(FakeProcess(), Path(temporary) / "dataset", recorder._generation)
            self.assertEqual(reasons, ["automatic stop: RGB-D camera health lost"])

    def test_episode_duration_excludes_recorder_startup(self) -> None:
        manager.shutil.disk_usage = lambda _path: type("Usage", (), {"free": 100 * 1024 ** 3})()
        with tempfile.TemporaryDirectory(prefix="openarm-recorder-test-") as temporary:
            root = Path(temporary)
            recorder = manager.Recorder(root)
            recorder.allowed_storage_root = root
            recorder.session_base = root / "sessions"
            recorder.session_state_path = root / "active-session.json"
            recorder.start_session()
            episode_root = recorder.session_root / "episodes/test/episode_0001"
            episode_root.mkdir(parents=True)
            recorder.active_episode = {
                "episode_id": "test_episode_0001", "episode_number": 1,
                "episode_root": str(episode_root), "task_group": "test",
                "task": "TEST", "started_unix_s": 100.0,
                "recording_started_unix_s": 115.0, "requested_result": "aborted",
                "stop_requested_unix_s": 119.0,
            }
            recorder._camera_health = lambda: {
                "ok": True, "detail": {
                    "spool_written": {"left_wrist": 90, "right_wrist": 90, "chest": 90},
                    "spool_drop": {"left_wrist": 0, "right_wrist": 0, "chest": 0},
                },
            }
            original_time = manager.time.time
            manager.time.time = lambda: 120.0
            try:
                recorder._finalize_episode(0)
            finally:
                manager.time.time = original_time
            self.assertEqual(recorder.last_episode["startup_duration_s"], 15.0)
            self.assertEqual(recorder.last_episode["duration_s"], 4.0)
            self.assertEqual(recorder.last_episode["finalization_duration_s"], 1.0)


if __name__ == "__main__":
    unittest.main()

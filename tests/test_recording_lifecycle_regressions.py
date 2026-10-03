"""Regression tests with temporary files and synthetic frames only (no CAN/USB)."""
import importlib.util
import ast
import logging
import os
import queue
import threading
import time
import types
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    value = importlib.util.module_from_spec(spec)
    sys.modules[name] = value
    spec.loader.exec_module(value)
    return value

manager = module("audit_manager", "jetson_record_manager.py")
# Test the real spool class without loading the USB/OpenCV SDK on the host.
tree = ast.parse((ROOT / "scripts/jetson_orbbec_rgbd_service.py").read_text())
camera = types.ModuleType("audit_camera")
camera.__dict__.update(json=json, os=os, queue=queue, threading=threading, time=time,
    Path=Path, LOG=logging.getLogger("spool_test"), ACTIVE_MARKER=None, ERROR_MARKER=None)
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DepthSpooler"],
                       type_ignores=[]), str(ROOT / "scripts/jetson_orbbec_rgbd_service.py"), "exec"), camera.__dict__)

class LifecycleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.r = manager.Recorder(self.root, runtime_dir=self.root / "runtime", storage_root=self.root)
        self.r.active_marker = self.root / "active"
        self.r.error_marker = self.root / "error"
        self.r.camera_status_path = self.root / "health"
        self.r._free_gb = lambda: 100
        self.r._camera_health = lambda: {"ok": True, "detail": {
            "spool_written": dict.fromkeys(camera_roles, 100), "spool_drop": dict.fromkeys(camera_roles, 0)}}
        self.r._motion_request = lambda *args, **kwargs: {}
        self.r.start_session("new")

    def active(self, result="success"):
        path = self.r.session_root / "episodes/right/episode_0001"
        path.mkdir()
        self.r.active_episode = {"episode_root": str(path), "lerobot_root": str(path / "lerobot"),
            "episode_id": "right_episode_0001", "task": "RIGHT_PICK_ONE", "task_group": "right",
            "started_unix_s": 1., "requested_result": result}
        return path

    def test_exited_process_is_busy_until_metadata_finalizes(self):
        self.active(); self.r.phase = "stopping"
        self.assertTrue(self.r.status()["running"])
        self.assertEqual(self.r.status()["phase"], "stopping")
        self.assertFalse(self.r.start_session("new")["ok"])
        self.assertFalse(self.r.close_session()["ok"])

    def test_spawn_failure_waits_for_operator_before_advancing(self):
        response = self.r.start_episode("LEFT_GRASP_LOG")
        self.assertFalse(response["ok"])
        self.assertEqual(self.r.phase, "awaiting_result")
        self.assertEqual(self.r.next_episode_by_task["left"], 1)
        self.assertIsNone(self.r.motion_token)
        self.assertFalse(self.r.status()["recording_active"])
        self.assertTrue(self.r.stop_episode("failure")["ok"])
        self.assertEqual(self.r.last_episode["failure_code"], "recorder_start_failed")
        self.assertEqual(self.r.next_episode_by_task["left"], 2)
        self.assertTrue(self.r.start_session("new")["ok"])

    def exited(self, returncode=0):
        process = types.SimpleNamespace(poll=lambda: returncode, wait=lambda: returncode, returncode=returncode)
        self.r.process = process
        self.r.phase = "recording"
        self.r._finalizing = True
        self.r._clear_marker_when_recording_exits(process, self.r._generation)

    def receipt(self, path):
        dataset = path / "lerobot"
        dataset.mkdir(exist_ok=True)
        (dataset / "rgbd-complete.json").write_text(json.dumps({"dataset_root": str(dataset),
            "complete": True, "written": dict.fromkeys(camera_roles, 90), "dropped": {}}))

    def test_unrequested_exit_preserves_number_and_data_until_operator_saves(self):
        path = self.active(result=None)
        self.receipt(path)
        self.exited()
        state = self.r.status()
        self.assertEqual(state["phase"], "awaiting_result")
        self.assertTrue(state["running"])
        self.assertFalse(state["recording_active"])
        self.assertIn("recorder exited", state["capture_interrupted"])
        self.assertEqual(state["next_episode_by_task"]["right"], 1)
        self.assertTrue((path / "episode.pending.json").exists())
        self.assertFalse((path / "episode.json").exists())
        self.assertFalse(self.r.start_episode("LEFT_GRASP_LOG")["ok"])
        self.assertFalse(self.r.start_session("new")["ok"])
        self.assertFalse(self.r.close_session()["ok"])
        self.assertTrue(self.r.stop_episode("success")["ok"])
        saved = json.loads((path / "episode.json").read_text())
        self.assertEqual(saved["result"], "success")
        self.assertFalse(saved["valid"])
        self.assertEqual(self.r.next_episode_by_task["right"], 2)
        self.assertFalse((path / "episode.pending.json").exists())
        self.assertTrue(self.r.stop_episode("failure")["ok"])
        self.assertEqual(self.r.next_episode_by_task["right"], 2)
        self.assertEqual(json.loads((path / "episode.json").read_text()), saved)

    def test_clean_operator_result_still_saves_once_and_can_be_valid(self):
        path = self.active("success")
        self.receipt(path)
        self.exited()
        self.assertIsNone(self.r.active_episode)
        self.assertTrue(self.r.last_episode["valid"])
        self.assertEqual(self.r.next_episode_by_task["right"], 2)
        self.r._finalize_episode(0)
        self.assertEqual(self.r.next_episode_by_task["right"], 2)

    def test_failure_and_explicit_abort_can_close_interrupted_capture(self):
        for result in ("failure", "aborted"):
            with self.subTest(result=result):
                self.r.start_session("new")
                self.active(None)
                self.exited(3)
                response = self.r.stop_episode(result, "operator_choice")
                self.assertTrue(response["ok"])
                self.assertEqual(self.r.last_episode["result"], result)
                self.assertEqual(self.r.next_episode_by_task["right"], 2)

    def test_result_during_capture_cleanup_is_accepted_once(self):
        self.active(None)
        self.r.active_episode["capture_interrupted"] = "camera disconnected"
        self.r.phase = "stopping"
        self.r._finalizing = True
        self.assertTrue(self.r.stop_episode("failure")["ok"])
        self.assertEqual(self.r.active_episode["requested_result"], "failure")
        self.assertTrue(self.r.stop_episode("success")["ok"])
        self.assertEqual(self.r.active_episode["requested_result"], "failure")
        self.r._finalize_episode(0)
        self.assertEqual(self.r.last_episode["result"], "failure")
        self.assertEqual(self.r.next_episode_by_task["right"], 2)

    def test_episode_save_failure_retains_result_and_number_for_retry(self):
        path = self.active(None)
        self.exited()
        original = self.r._write_json
        def fail_episode(target, value):
            if target.name == "episode.json":
                raise OSError("disk write failed")
            return original(target, value)
        with patch.object(self.r, "_write_json", side_effect=fail_episode):
            response = self.r.stop_episode("failure")
        self.assertFalse(response["ok"])
        self.assertEqual(self.r.phase, "awaiting_result")
        self.assertEqual(self.r.next_episode_by_task["right"], 1)
        self.assertIsNotNone(self.r.active_episode)
        self.assertFalse((path / "episode.json").exists())
        self.assertTrue(self.r.stop_episode("success")["ok"])
        self.assertEqual(self.r.last_episode["result"], "failure")
        self.assertEqual(self.r.next_episode_by_task["right"], 2)

    def test_pending_episode_survives_restart_without_advancing(self):
        path = self.active(None)
        self.exited(3)
        restored = manager.Recorder(self.root, runtime_dir=self.root / "runtime", storage_root=self.root)
        self.assertEqual(restored.phase, "awaiting_result")
        self.assertEqual(restored.next_episode_by_task["right"], 1)
        self.assertEqual(restored.active_episode["episode_root"], str(path))
        self.assertEqual(restored.active_episode["recorder_returncode"], 3)
        restored._camera_health = lambda: {"ok": True}
        restored.active_marker = self.root / "restored-marker"
        self.assertTrue(restored.stop_episode("failure")["ok"])
        self.assertEqual(restored.next_episode_by_task["right"], 2)

    def test_committed_episode_remains_successful_when_batch_summary_write_fails(self):
        path = self.active(None)
        self.exited()
        with patch.object(self.r, "_persist_session", side_effect=OSError("runtime disk unavailable")):
            response = self.r.stop_episode("failure")
        self.assertTrue(response["ok"])
        self.assertIsNone(self.r.active_episode)
        self.assertEqual(self.r.next_episode_by_task["right"], 2)
        self.assertTrue((path / "episode.json").exists())
        self.assertIn("本条已保存", self.r.last_log)
        self.assertTrue(self.r.stop_episode("failure")["ok"])
        self.assertEqual(self.r.next_episode_by_task["right"], 2)

    def test_automatic_safety_stop_keeps_first_reason_and_waits_for_result(self):
        self.active(None)
        self.r.process = types.SimpleNamespace(poll=lambda: None)
        self.r.phase = "recording"
        self.assertTrue(self.r._request_stop("automatic stop: only 9.0 GB free"))
        self.r._request_stop("automatic stop: RGB-D camera health lost")
        self.assertIn("9.0 GB", self.r.stop_reason)
        self.assertIn("9.0 GB", self.r.active_episode["capture_interrupted"])
        self.assertIsNone(self.r.active_episode["requested_result"])
        self.exited()
        self.assertEqual(self.r.phase, "awaiting_result")
        self.assertEqual(self.r.next_episode_by_task["right"], 1)

    def test_runtime_pointer_recovers_pending_when_episode_metadata_write_failed(self):
        path = self.active(None)
        self.exited()
        (path / "episode.pending.json").unlink()
        restored = manager.Recorder(self.root, runtime_dir=self.root / "runtime", storage_root=self.root)
        self.assertEqual(restored.phase, "awaiting_result")
        self.assertEqual(restored.next_episode_by_task["right"], 1)
        self.assertEqual(restored.active_episode["episode_root"], str(path))

    def test_restart_prefers_latest_operator_choice_over_stale_episode_snapshot(self):
        path = self.active(None)
        self.exited()
        older_snapshot = (path / "episode.pending.json").read_text()
        self.r.active_episode["requested_result"] = "failure"
        self.r._persist_pending_episode()
        (path / "episode.pending.json").write_text(older_snapshot)
        restored = manager.Recorder(self.root, runtime_dir=self.root / "runtime", storage_root=self.root)
        self.assertEqual(restored.phase, "awaiting_result")
        self.assertEqual(restored.active_episode["requested_result"], "failure")
        self.assertEqual(restored.next_episode_by_task["right"], 1)

    def test_committed_episode_does_not_reopen_from_stale_pending_metadata(self):
        path = self.active(None)
        self.exited()
        pending = (path / "episode.pending.json").read_text()
        stale_runtime = self.r.session_state_path.read_text()
        self.r.stop_episode("failure")
        (path / "episode.pending.json").write_text(pending)
        self.r.session_state_path.write_text(stale_runtime)
        restored = manager.Recorder(self.root, runtime_dir=self.root / "runtime", storage_root=self.root)
        self.assertIsNone(restored.active_episode)
        self.assertEqual(restored.next_episode_by_task["right"], 2)

    def test_previous_episode_global_statistics_do_not_prove_current_validity(self):
        self.active(); self.r._finalize_episode(0)
        self.assertFalse(self.r.last_episode["valid"])

    def test_dataset_receipt_is_authoritative(self):
        path = self.active() / "lerobot"; path.mkdir()
        (path / "rgbd-complete.json").write_text(json.dumps({"dataset_root": str(path),
            "complete": True, "written": dict.fromkeys(camera_roles, 90), "dropped": {}}))
        self.r._finalize_episode(0)
        self.assertTrue(self.r.last_episode["valid"])

    def test_stop_before_activation_cannot_resurrect_marker(self):
        path = self.root / "dataset"; path.mkdir()
        self.r._stop_requested.set()
        self.r._activate_depth_spool(path, self.r._generation)
        self.assertFalse(self.r.active_marker.exists())

    def test_spool_shutdown_preserves_all_queued_frames(self):
        import numpy as np
        marker = self.root / "marker"; marker.write_text(str(self.root / "dataset"))
        with patch.object(camera, "ACTIVE_MARKER", marker), patch.object(camera, "ERROR_MARKER", self.root / "error"):
            spool = camera.DepthSpooler(camera_roles)
            for sequence in range(5):
                spool.update({r: ({"frame_sequence": sequence}, np.zeros((2,2,3),np.uint8),
                                  np.zeros((2,2,1),np.uint16)) for r in camera_roles})
            marker.unlink()
            self.assertTrue(spool._close())
        receipt = json.loads((self.root / "dataset/rgbd-complete.json").read_text())
        self.assertTrue(receipt["complete"])
        self.assertEqual(receipt["written"], dict.fromkeys(camera_roles, 5))
        for r in camera_roles:
            self.assertEqual((self.root / f"dataset/depth_raw/{r}.u16le").stat().st_size, 40)

camera_roles = ("left_wrist", "right_wrist", "chest")

if __name__ == "__main__":
    unittest.main()

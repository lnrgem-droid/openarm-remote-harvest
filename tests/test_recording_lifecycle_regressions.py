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

    def test_spawn_failure_does_not_permanently_block_future_episodes(self):
        with self.assertRaises(FileNotFoundError):
            self.r.start_episode("LEFT_GRASP_LOG")
        self.assertIsNone(self.r.active_episode)
        self.assertIsNone(self.r.motion_token)
        self.assertFalse(self.r.status()["running"])
        saved = list(self.r.session_root.rglob("episode.json"))
        self.assertEqual(json.loads(saved[0].read_text())["failure_code"], "recorder_start_failed")
        self.assertTrue(self.r.start_session("new")["ok"])

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

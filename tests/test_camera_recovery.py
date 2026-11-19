"""Camera disconnect regressions without opening any hardware."""
import base64
import importlib.util
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

spec = importlib.util.spec_from_file_location(
    "recovery_camera", Path(__file__).parents[1] / "scripts/jetson_orbbec_rgbd_service.py")
camera = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = camera
spec.loader.exec_module(camera)


class CameraRecoveryTest(unittest.TestCase):
    def test_stalled_camera_retries_only_after_recording_and_writers_finish(self):
        policy = camera.RecoveryPolicy()
        self.assertFalse(policy.due(0, False, True, True))
        self.assertFalse(policy.due(20, False, True, False))
        self.assertFalse(policy.due(21, False, False, True))
        self.assertTrue(policy.due(22, False, False, False))

    def test_short_gap_does_not_restart_and_recovery_resets_stall_timer(self):
        policy = camera.RecoveryPolicy()
        self.assertFalse(policy.due(0, False, False, False))
        self.assertFalse(policy.due(7, False, False, False))
        self.assertFalse(policy.due(8, True, False, False))
        self.assertFalse(policy.due(20, False, False, False))
        self.assertFalse(policy.due(27, False, False, False))
        self.assertTrue(policy.due(28, False, False, False))

    def test_stable_stream_resets_retry_backoff(self):
        with patch.dict(os.environ, {"OPENARM_RGBD_RECOVERY_ATTEMPT": "4"}):
            policy = camera.RecoveryPolicy()
            policy.due(0, True, False, False)
            policy.due(29, True, False, False)
            self.assertIn("OPENARM_RGBD_RECOVERY_ATTEMPT", os.environ)
            policy.due(30, True, False, False)
            self.assertNotIn("OPENARM_RGBD_RECOVERY_ATTEMPT", os.environ)

    def test_stale_or_missing_camera_does_not_hide_healthy_preview(self):
        now = 2_000_000_000
        def sample(age):
            return ({"host_monotonic_ns": now - age,
                     "host_realtime_ns": 1_000_000_000, "frame_sequence": 7},
                    np.full((480, 640, 3), 255, np.uint8), None)
        fresh, stale = sample(10_000_000), sample(1_000_000_000)
        cameras = [SimpleNamespace(spec=camera.CameraSpec(role, role), get=lambda item=item: item)
                   for role, item in [("left_wrist", fresh), ("right_wrist", None), ("chest", stale)]]
        packet = camera.preview_message(cameras, 75, clock_ns=lambda: now)
        self.assertEqual(set(packet["images"]), {"left_wrist", "right_wrist", "chest"})
        self.assertTrue(packet["camera_status"]["left_wrist"]["healthy"])
        for role in ("right_wrist", "chest"):
            self.assertFalse(packet["camera_status"][role]["healthy"])
            self.assertEqual(packet["timestamps"][role], 0)
            self.assertEqual(packet["frame_seq"][role], -1)
            image = cv2.imdecode(np.frombuffer(base64.b64decode(packet["images"][role]), np.uint8), 1)
            self.assertLess(float(image.mean()), 20)  # Not the old white frame.

    def test_future_timestamp_is_not_treated_as_fresh(self):
        self.assertFalse(camera.frame_is_fresh(({"host_monotonic_ns": 2}, None, None), 1))

    def test_frames_arriving_during_preview_encoding_are_not_blackened(self):
        # Each camera delivers a frame newer than the snapshot for the prior
        # camera. The observation clock must be taken after each get().
        clock = [2_000_000_000]
        def get():
            clock[0] += 30_000_000
            return ({"host_monotonic_ns": clock[0],
                     "host_realtime_ns": 1_000_000_000, "frame_sequence": 9},
                    np.full((4, 4, 3), 255, np.uint8), None)
        cameras = [SimpleNamespace(spec=camera.CameraSpec(role, role), get=get)
                   for role in ("left_wrist", "right_wrist", "chest")]
        packet = camera.preview_message(cameras, 75, clock_ns=lambda: clock[0])
        self.assertTrue(all(v["healthy"] for v in packet["camera_status"].values()))
        self.assertEqual(set(packet["frame_seq"].values()), {9})


if __name__ == "__main__":
    unittest.main()

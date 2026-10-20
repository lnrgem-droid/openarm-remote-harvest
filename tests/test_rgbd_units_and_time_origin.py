"""Numeric payload and real converter regression tests; no hardware access."""
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

ROOT = Path(__file__).parents[1]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


camera = load("units_camera", "jetson_orbbec_rgbd_service.py")
converter = load("time_converter", "convert_recording_to_openarm_dataset.py")


class UnitsAndOriginTest(unittest.TestCase):
    def test_native_tenth_mm_becomes_mm_exactly_once(self):
        depth, metadata = camera.depth_in_millimeters(
            np.array([[0, 1000, 12340]], dtype=np.uint16), .1)
        np.testing.assert_array_equal(depth[..., 0], [[0, 100, 1234]])
        np.testing.assert_array_equal(
            depth[..., 0] * metadata["depth_scale_mm_per_unit"], [[0, 100, 1234]])
        self.assertEqual(metadata["source_depth_scale_mm_per_unit"], .1)
        self.assertEqual(depth.dtype, np.uint16)

    def test_native_mm_is_unchanged(self):
        raw = np.array([[0, 1234, 65535]], dtype=np.uint16)
        depth, metadata = camera.depth_in_millimeters(raw, 1.)
        np.testing.assert_array_equal(depth[..., 0], raw)
        self.assertEqual(metadata["depth_scale_mm_per_unit"], 1.)

    def test_invalid_depth_scale_fails(self):
        for scale in (0, -1, float("nan"), float("inf")):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                camera.depth_in_millimeters(np.zeros((1, 1), dtype=np.uint16), scale)

    def test_bad_or_missing_modern_anchor_is_not_silently_guessed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "meta").mkdir()
            for value in (None, 0, -1, True, "123", 1.5):
                (root / "meta/info.json").write_text(json.dumps({"started_unix_ns": value}))
                with self.subTest(value=value), self.assertRaises(ValueError):
                    converter.recording_time_origin(root, {"chest": [123]})
            (root / "meta/info.json").write_text('{"format":"openarm_rgbd_staging_v1"}')
            with self.assertRaises(ValueError):
                converter.recording_time_origin(root, {"chest": [123]})

    def test_legacy_anchor_is_explicitly_approximate(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertWarns(RuntimeWarning):
                origin, name = converter.recording_time_origin(Path(directory), {"a": [123], "b": [456]})
            self.assertEqual(origin, 123)
            self.assertEqual(name, "first_aligned_camera_host_realtime_ns")

    def test_conversion_uses_robot_origin_for_timestamps_and_image_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "source"
            for subdir in ("meta", "data/chunk-000", "depth_raw", "rgb_raw"):
                (root / subdir).mkdir(parents=True)
            origin = 1_790_000_000_000_000_000
            (root / "meta/info.json").write_text(json.dumps({"started_unix_ns": origin}))
            table = pa.table({"timestamp": [0., .05],
                              "observation.state": [list(range(16))] * 2,
                              "action": [list(range(16, 32))] * 2})
            pq.write_table(table, root / "data/chunk-000/file-000.parquet")
            # Camera begins 50 ms before robot. The old origin picked the wrong
            # color frame even though its nearest-time skew looked perfect.
            for role in converter.ROLES:
                frames = [np.full((2, 2, 3), v, np.uint8) for v in (20, 100, 200)]
                (root / "rgb_raw" / f"{role}.rgb24").write_bytes(b"".join(f.tobytes() for f in frames))
                entries = [{"role": role, "aligned_depth_to_rgb": True,
                            "host_realtime_ns": origin + (i - 1) * 50_000_000,
                            "height": 2, "width": 2, "rgb_offset_bytes": i * 12,
                            "rgb_nbytes": 12} for i in range(3)]
                (root / "depth_raw" / f"{role}.jsonl").write_text(
                    "".join(json.dumps(e) + "\n" for e in entries))
            output = Path(directory) / "output"
            with patch.object(sys, "argv", ["convert", str(root), str(output)]):
                converter.main()
            manifest = json.loads((output / "depth_sidecar_manifest.json").read_text())
            self.assertEqual(manifest["robot_time_origin_ns"], origin)
            self.assertFalse(manifest["time_origin_is_approximate"])
            saved = pq.read_table(output / "episodes/0/obs/arms/left/state.parquet")
            self.assertEqual(saved["timestamp"].cast(pa.int64()).to_pylist(),
                             [origin, origin + 50_000_000])
            self.assertEqual(saved["qpos"].to_pylist(), [list(range(8))] * 2)
            for role in converter.CAMERA_NAMES.values():
                for ns, value in ((origin, 100), (origin + 50_000_000, 200)):
                    image = cv2.imread(str(output / f"episodes/0/cameras/{role}/{ns}.jpeg"))
                    self.assertTrue(np.all(np.abs(image.astype(int) - value) <= 1))


if __name__ == "__main__":
    unittest.main()

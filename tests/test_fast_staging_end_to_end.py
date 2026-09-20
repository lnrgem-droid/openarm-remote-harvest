"""Local fake WebSocket bridge; never touches ROS, cameras, or CAN."""
import json
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).parents[1]

class FastStagingTest(unittest.TestCase):
    def scenario(self, stall):
        try:
            from websockets.sync.server import serve
            import pyarrow.parquet as pq
        except ImportError:
            self.skipTest("run in the Jetson lerobot environment (websockets.sync + pyarrow)")
        closing = threading.Event()
        def bridge(ws):
            start = time.monotonic()
            while not closing.wait(.015):
                if stall and time.monotonic() - start > .35:
                    continue
                q = [0.] * 8
                data = {"left_arm": {"position": q}, "right_arm": {"position": q},
                        "teleop_action": {"left": q, "right": q, "valid": True, "recv_time": time.time()}}
                try:
                    ws.send(json.dumps({"type": "state", "data": data}))
                except Exception:
                    return
        with tempfile.TemporaryDirectory(prefix="openarm-fake-record-") as directory:
            dataset = Path(directory) / "sample"
            with serve(bridge, "127.0.0.1", 0) as server:
                thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
                port = server.socket.getsockname()[1]
                process = subprocess.Popen([sys.executable, str(ROOT / "scripts/record_openarm_fast_staging.py"),
                    "--root", str(dataset), "--task", "TEST_ONLY", "--ws-url", f"ws://127.0.0.1:{port}"],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                try:
                    deadline = time.monotonic()+8
                    while not dataset.exists() and process.poll() is None and time.monotonic()<deadline:
                        time.sleep(.02)
                    if not stall:
                        time.sleep(.5); process.send_signal(signal.SIGINT)
                    output, _ = process.communicate(timeout=8)
                    self.assertEqual(process.returncode, 3 if stall else 0, output)
                    info = json.loads((dataset / "meta/info.json").read_text())
                    rows = pq.read_table(dataset / "data/chunk-000/file-000.parquet")
                    self.assertEqual(rows.num_rows, info["total_frames"])
                    self.assertGreater(rows.num_rows, 3)
                    self.assertEqual(bool(info["recording_error"]), stall)
                    if stall:
                        self.assertIn("stream stopped", info["recording_error"])
                        self.assertLess(info["duration_s"], 1.)
                finally:
                    closing.set()
                    if process.poll() is None:
                        process.kill(); process.wait()
                    server.shutdown(); thread.join(timeout=2)

    def test_normal_stop_keeps_parquet(self):
        self.scenario(False)

    def test_stalled_bridge_seals_partial_data_with_error(self):
        self.scenario(True)

if __name__ == "__main__":
    unittest.main()

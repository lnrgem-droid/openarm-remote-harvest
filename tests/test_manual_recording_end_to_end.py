"""Real recorder subprocess + manager, using only synthetic loopback data.

Run with Jetson's lerobot Python (pyarrow/websockets). No robot, camera,
production service, live dataset, or hardware control endpoint is accessed.
"""
import importlib.util
import json
from pathlib import Path
import shlex
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location('manual_manager', ROOT / 'scripts/jetson_record_manager.py')
manager = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(manager)


class ManualRecordingEndToEnd(unittest.TestCase):
    def wait_until(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.02)
        self.fail('timed out waiting for synthetic recording transition')

    def scenario(self, side, result, interrupted=False):
        try:
            from websockets.sync.server import serve
            import pyarrow.parquet as pq
        except ImportError:
            self.skipTest('requires lerobot Python with pyarrow and websockets.sync')
        closing = threading.Event()

        def bridge(ws):
            started = time.monotonic()
            while not closing.wait(.015):
                elapsed = time.monotonic() - started
                if interrupted and elapsed > .8:
                    continue
                # Both sides change, so a repeated cached vector cannot pass.
                left = [elapsed * .01] * 7 + [.022]
                right = [elapsed * .02] * 7 + [.033]
                data = dict(left_arm=dict(position=left), right_arm=dict(position=right),
                            teleop_action=dict(left=left[:7] + [.5], right=right[:7] + [.75],
                                               valid=True, recv_time=time.time()))
                try:
                    ws.send(json.dumps(dict(type='state', data=data)))
                except Exception:
                    return

        with tempfile.TemporaryDirectory(prefix='openarm-manual-synthetic-') as directory:
            root = Path(directory)
            with serve(bridge, '127.0.0.1', 0) as server:
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()
                port = server.socket.getsockname()[1]
                (root / 'scripts').mkdir()
                wrapper = root / 'scripts/record_jetson_rgbd_dataset.sh'
                wrapper.write_text('#!/usr/bin/env bash\nexec ' + shlex.quote(sys.executable) + ' ' +
                    shlex.quote(str(ROOT / 'scripts/record_openarm_fast_staging.py')) +
                    ' --root "$DATASET_ROOT" --task "$TASK" --ws-url ws://127.0.0.1:' + str(port) + '\n')
                wrapper.chmod(0o755)
                recorder = manager.Recorder(root, runtime_dir=root/'runtime', storage_root=root)
                recorder.active_marker = root/'recording.active'
                recorder.error_marker = root/'recording.error'
                recorder._free_gb = lambda: 100.
                recorder._camera_health = lambda: {'ok': True}
                recorder._motion_request = lambda *a, **kw: dict(state='RUNNING', fault_bits=0,
                    collection=dict(recording=dict(token=recorder.motion_token)))
                recorder.start_session('new')
                task = 'LEFT_GRASP_LOG' if side == 'left' else 'RIGHT_PICK_ONE'
                try:
                    self.assertTrue(recorder.start_episode(task)['ok'])
                    self.wait_until(lambda: recorder.phase == 'recording')
                    episode = dict(recorder.active_episode)
                    dataset = Path(episode['lerobot_root'])
                    # Synthetic writer receipt; no USB/camera service is used.
                    (dataset/'rgbd-complete.json').write_text(json.dumps(dict(
                        dataset_root=str(dataset), complete=True,
                        written=dict(left_wrist=150, right_wrist=150, chest=150), dropped={})))
                    if interrupted:
                        self.wait_until(lambda: recorder.phase == 'awaiting_result')
                        self.assertTrue(recorder.status()['running'])
                        self.assertIsNotNone(recorder.active_episode)
                        self.assertEqual(recorder.next_episode_by_task[side], 1)
                        time.sleep(.3)
                        self.assertEqual(recorder.next_episode_by_task[side], 1)
                    else:
                        time.sleep(4.2)  # Longer than the old three-second auto-abort.
                        self.assertEqual(recorder.phase, 'recording')
                        self.assertIsNone(recorder.process.poll())
                        self.assertEqual(recorder.next_episode_by_task[side], 1)
                        self.assertFalse((Path(episode['episode_root'])/'episode.json').exists())
                    saved = recorder.stop_episode(result, 'operator_marked_failure' if result == 'failure' else '')
                    self.assertTrue(saved['ok'], saved)
                    self.wait_until(lambda: not recorder.status()['running'])
                    self.assertEqual(recorder.next_episode_by_task[side], 2)
                    finished = json.loads((Path(episode['episode_root'])/'episode.json').read_text())
                    self.assertEqual(finished['result'], result)
                    self.assertEqual(finished['valid'], result == 'success' and not interrupted)
                    self.assertEqual(recorder.last_episode['episode_number'], 1)
                    rows = pq.read_table(dataset/'data/chunk-000/file-000.parquet')
                    self.assertGreater(rows.num_rows, 10 if interrupted else 100)
                    states = rows['observation.state'].to_pylist()
                    joint = 0 if side == 'left' else 8
                    self.assertGreater(states[-1][joint] - states[0][joint], .005)
                    if not interrupted:
                        self.assertGreater(rows['timestamp'].to_pylist()[-1], 4.)
                    # A duplicate result cannot save another numbered episode.
                    recorder.stop_episode(result)
                    self.assertEqual(recorder.next_episode_by_task[side], 2)
                finally:
                    closing.set()
                    process = recorder.process
                    if process is not None and process.poll() is None:
                        process.kill(); process.wait(timeout=3)
                    server.shutdown(); thread.join(timeout=2)

    def test_both_arms_wait_for_each_manual_save_result(self):
        for side in ('left', 'right'):
            for result in ('success', 'failure'):
                with self.subTest(side=side, result=result):
                    self.scenario(side, result)

    def test_stream_error_keeps_number_until_operator_saves_failure(self):
        self.scenario('right', 'failure', interrupted=True)


if __name__ == '__main__':
    unittest.main()

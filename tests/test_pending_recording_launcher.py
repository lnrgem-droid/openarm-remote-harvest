"""Exercise the launcher's recorder gate with in-memory status replies only."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
LAUNCHER = ROOT/'daily_start_teleop_rgbd.sh'
if not LAUNCHER.exists():
    LAUNCHER = ROOT.parent/'openarm-remote-harvest/scripts/daily_start_teleop_rgbd.sh'


class PendingRecordingLauncherTest(unittest.TestCase):
    def check_gate(self, replies, commands):
        source = LAUNCHER.read_text().split('ensure_recording_idle() {', 1)[1]
        code = source[source.index('import time, zmq\n'):source.index("\nPY'")]
        sent = []
        pending = iter(replies)
        class Socket:
            def setsockopt(self, *args): pass
            def connect(self, *args): pass
            def send_json(self, payload): sent.append(payload['command'])
            def recv_json(self): return next(pending)
            def close(self, *args): pass
        class Context:
            def socket(self, *args): return Socket()
            def term(self): pass
        zmq = SimpleNamespace(Context=Context, REQ=0, LINGER=0, SNDTIMEO=0, RCVTIMEO=0)
        with patch.dict(sys.modules, zmq=zmq, time=SimpleNamespace(sleep=lambda _:None)):
            try:
                exec(code, {})
            except SystemExit as exc:
                self.assertEqual(exc.code, 0)
        self.assertEqual(sent, commands)

    def test_existing_pending_result_opens_without_stop_request(self):
        self.check_gate([dict(running=True, phase='awaiting_result', recording_active=False)], ['status'])

    def test_orphan_capture_stops_then_opens_pending_result_without_clearing_it(self):
        self.check_gate([dict(running=True, phase='recording'), dict(ok=True),
                         dict(running=True, phase='awaiting_result', recording_active=False)],
                        ['status', 'stop', 'status'])

    def test_idle_recorder_is_unchanged(self):
        self.check_gate([dict(running=False, phase='idle')], ['status'])


if __name__ == '__main__':
    unittest.main()

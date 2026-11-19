"""Exercise the real read-only transport against fake Unix datagram peers."""
import importlib.util
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location("fast_status", ROOT / "scripts/read_remote_teleop_status.py")
status = importlib.util.module_from_spec(spec)
spec.loader.exec_module(status)


class FastStatusTest(unittest.TestCase):
    def exchange(self, response):
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "runtime")
            requests = []
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as server:
                server.bind(path)
                server.settimeout(2)
                def reply():
                    data, peer = server.recvfrom(65536)
                    requests.append(json.loads(data))
                    server.sendto(response, peer)
                thread = threading.Thread(target=reply)
                thread.start()
                result = subprocess.run([sys.executable, "-c", status.remote_source(path, .5)],
                                        capture_output=True, text=True, timeout=3)
                thread.join(2)
            self.assertEqual(requests, [{"command": "status"}])
            return result

    def test_preserves_fault_and_session_without_reset_or_retry(self):
        value = {"state": "FAULT", "fault_bits": 1, "leader_session_id": 42,
                 "reason": "leader action receive timeout", "collection": {"left_mode": "HOLD"}}
        result = self.exchange(json.dumps(value).encode())
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), value)

    def test_rejected_malformed_and_incomplete_status_fail_closed(self):
        for response in (b'{"error":"unavailable"}', b'[]', b'bad json',
                         b'{"state":"RUNNING"}', b'{"state":"RUNNING","fault_bits":false}'):
            with self.subTest(response=response):
                self.assertNotEqual(self.exchange(response).returncode, 0)

    def test_missing_and_silent_runtime_do_not_produce_ready(self):
        with tempfile.TemporaryDirectory() as d:
            path = str(Path(d) / "runtime")
            for listening in (False, True):
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as server:
                    if listening:
                        server.bind(path)
                    result = subprocess.run([sys.executable, "-c", status.remote_source(path, .05)],
                                            capture_output=True, text=True, timeout=2)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertEqual(result.stdout, "")

    def test_failed_ssh_is_not_retried(self):
        with patch.object(status.subprocess, "run", side_effect=subprocess.TimeoutExpired("ssh", 8)) as run:
            with self.assertRaises(subprocess.TimeoutExpired):
                status.read_status("test-peer")
        self.assertEqual(run.call_count, 1)

    def test_option_as_peer_is_rejected_before_execution(self):
        with patch.object(status.subprocess, "run") as run:
            with self.assertRaises(ValueError):
                status.read_status("-oProxyCommand=bad")
            run.assert_not_called()

    def test_launcher_only_accelerates_status_not_motion_commands(self):
        source = (ROOT / "scripts/run_bimanual_remote_feedback.sh").read_text()
        function = source[source.index("remote_control() {"):source.index("\nstatus_is_healthy_running()")]
        for command in ("status", "align", "run", "hold", "reset", "disable"):
            with tempfile.TemporaryDirectory() as d:
                root = Path(d)
                (root / "scripts").mkdir()
                (root / "scripts/read_remote_teleop_status.py").write_text("print('READ_ONLY_PATH')\n")
                shell = 'set -eu\nROOT_DIR="$1"\nJETSON_HOST=test-peer\nJETSON_ROOT=/fake\n'
                shell += 'ssh() { printf "CLI_PATH %s\\n" "$*"; }\n' + function + '\nremote_control "$2"\n'
                result = subprocess.run(["bash", "-c", shell, "test", d, command],
                                        text=True, capture_output=True, check=True)
                self.assertIn("READ_ONLY_PATH" if command == "status" else "CLI_PATH", result.stdout)
                if command != "status":
                    self.assertIn("remote-teleop-control " + command, result.stdout)


if __name__ == "__main__":
    unittest.main()

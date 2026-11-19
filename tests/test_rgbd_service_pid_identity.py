"""PID reuse after reboot must not suppress camera/recorder startup."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ServicePidIdentityTests(unittest.TestCase):
    def test_pid_file_requires_matching_service_process(self):
        source = (ROOT / 'scripts/daily_start_teleop_rgbd.sh').read_text()
        function = source.split('alive() {', 1)[1].split('\n}', 1)[0]
        function = 'alive() {' + function + '\n}'
        function = function.replace('/home/nvidia/miniconda3/envs/lerobot/bin/python', sys.executable)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'scripts').mkdir()
            script = root / 'scripts/jetson_orbbec_rgbd_service.py'
            script.write_text('import time\ntime.sleep(30)\n')
            camera = subprocess.Popen([sys.executable, str(script)])
            unrelated = subprocess.Popen(['sleep', '30'])
            try:
                for name, pid, expected in (
                    ('camera-service.pid', camera.pid, 0),
                    ('record-manager.pid', camera.pid, 1),
                    ('camera-service.pid', unrelated.pid, 1),
                    ('record-manager.pid', unrelated.pid, 1),
                    ('camera-service.pid', 'not-a-pid', 1),
                    ('camera-service.pid', '', 1),
                    ('camera-service.pid', '999999999', 1),
                ):
                    with self.subTest(name=name, pid=pid):
                        pid_file = root / name
                        pid_file.write_text(str(pid) + '\n')
                        result = subprocess.run(
                            ['bash', '-c', 'runtime="$1"; root="$1"\n' + function + '\nalive "$2"',
                             'test', directory, str(pid_file)], capture_output=True, text=True)
                        self.assertEqual(result.returncode, expected, result.stderr)
            finally:
                for process in (camera, unrelated):
                    process.terminate()
                    process.wait(timeout=3)


class ServicePreparationTests(unittest.TestCase):
    """Run the complete remote preparation against isolated fake services."""
    def setUp(self):
        import socket
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.runtime = self.root / 'runtime'
        self.runtime.mkdir()
        self.scripts = self.root / 'scripts'
        self.scripts.mkdir()
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.ipc = self.root / 'camera.ipc'
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            self.port = sock.getsockname()[1]
        source = (ROOT / 'scripts/daily_start_teleop_rgbd.sh').read_text()
        start = source.index('ensure_jetson_rgbd_services() {')
        end = source.index('\nstop_recording_on_exit()', start)
        # Capture the exact shell argument sent to SSH; catch quoting regressions too.
        capture = 'TEST_HOST_MODE=false\nJETSON_HOST=fake\nssh() { printf "%s" "$2"; }\n'
        capture += source[start:end] + '\nensure_jetson_rgbd_services\n'
        self.code = subprocess.check_output(['bash'], input=capture, text=True)
        for old, new in (
            ('/home/nvidia/openarm-rgbd-runtime', str(self.runtime)),
            ('/home/nvidia/dev/openarm-rgbd-preview', str(self.root)),
            ('/home/nvidia/miniconda3/envs/lerobot/bin/python', sys.executable),
            ('/tmp/openarm_rgbd_raw.ipc', str(self.ipc)),
            ('tcp://127.0.0.1:5557', f'tcp://127.0.0.1:{self.port}'),
            ('seq 1 20', 'seq 1 2'), ('seq 1 30', 'seq 1 2'),
            ('sleep 1\n', 'sleep 0.05\n'),
            ('time.monotonic() + 20', 'time.monotonic() + 0.5'),
            ('time.sleep(0.5)', 'time.sleep(0.02)'),
            ('TIMEO, 1000', 'TIMEO, 50'),
            ('flock -w 15', 'flock -w 0.1'),
        ):
            self.code = self.code.replace(old, new)
        for name, code in {'ss': 'echo LISTEN', 'taskset': 'exit 0', 'renice': 'exit 0'}.items():
            path = self.bin / name
            path.write_text('#!/bin/sh\n' + code + '\n')
            path.chmod(0o755)
        import os
        self.env = dict(os.environ, PATH=str(self.bin) + ':' + os.environ['PATH'])
        self.camera = self.scripts / 'jetson_orbbec_rgbd_service.py'
        self.camera.write_text('import socket,time\ns=socket.socket(socket.AF_UNIX)\n'
                               + f's.bind({str(self.ipc)!r})\ntime.sleep(30)\n')
        self.manager = self.scripts / 'jetson_record_manager.py'
        self.manager.write_text('import zmq\nc=zmq.Context(); s=c.socket(zmq.REP)\n'
                                + f's.bind("tcp://127.0.0.1:{self.port}")\n'
                                + 'while True:\n s.recv_json(); s.send_json({"ok":True,"camera_health":{"ok":True}})\n')
        for starter, service in (('start_jetson_rgbd_service.sh', self.camera),
                                 ('start_jetson_record_manager.sh', self.manager)):
            (self.scripts / starter).write_text(f'exec {sys.executable} {service}\n')
        self.unrelated = subprocess.Popen(['sleep', '30'])

    def tearDown(self):
        import os, signal
        for name in ('camera-service.pid', 'record-manager.pid'):
            path = self.runtime / name
            if path.exists():
                try:
                    pid = int(path.read_text())
                    cmdline = Path(f'/proc/{pid}/cmdline').read_bytes()
                    if str(self.scripts).encode() in cmdline:
                        os.kill(pid, signal.SIGTERM)
                except (OSError, ValueError):
                    pass
        self.unrelated.terminate()
        self.unrelated.wait(timeout=3)
        self.tmp.cleanup()

    def run_prepare(self):
        return subprocess.run(['bash'], input=self.code, text=True, capture_output=True,
                              env=self.env, timeout=5)

    def test_reboot_pid_reuse_starts_services_and_repeat_reuses_them(self):
        for name in ('camera-service.pid', 'record-manager.pid'):
            (self.runtime / name).write_text(str(self.unrelated.pid) + '\n')
        first = self.run_prepare()
        self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
        pids = [(self.runtime / name).read_text() for name in ('camera-service.pid', 'record-manager.pid')]
        second = self.run_prepare()
        self.assertEqual(second.returncode, 0, second.stdout + second.stderr)
        self.assertNotIn('启动 Jetson', second.stdout)
        self.assertEqual(pids, [(self.runtime / name).read_text() for name in ('camera-service.pid', 'record-manager.pid')])
        self.assertIsNone(self.unrelated.poll())

    def test_camera_crash_reports_camera_log(self):
        self.camera.write_text('raise SystemExit("FAKE_CAMERA_FAILURE")\n')
        result = self.run_prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('相机服务未就绪', result.stderr)
        self.assertIn('FAKE_CAMERA_FAILURE', result.stderr)

    def test_recorder_crash_does_not_pass_on_camera_socket_alone(self):
        self.manager.write_text('raise SystemExit("FAKE_RECORDER_FAILURE")\n')
        result = self.run_prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('录制服务状态查询失败', result.stderr)
        self.assertIn('FAKE_RECORDER_FAILURE', result.stderr)

    def test_unhealthy_camera_blocks_success_with_specific_reason(self):
        self.manager.write_text(self.manager.read_text().replace('"camera_health":{"ok":True}',
                                                               '"camera_health":{"ok":False}'))
        result = self.run_prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('相机健康检查未通过', result.stderr)

    def test_startup_lock_prevents_parallel_launch(self):
        import fcntl
        with (self.runtime / 'startup.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            result = self.run_prepare()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('另一窗口正在准备采集服务', result.stderr)
        self.assertFalse((self.runtime / 'camera-service.pid').exists())


if __name__ == '__main__':
    unittest.main()

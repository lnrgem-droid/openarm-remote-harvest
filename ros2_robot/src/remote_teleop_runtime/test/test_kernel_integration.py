"""Real local transports and watchdog process; no ROS nodes or robot endpoints."""
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

from remote_teleop_protocol import ActionCommand, FaultBits, encode_action
from remote_teleop_follower_safety.local_protocol import encode_command
from remote_teleop_runtime.common import UnixDatagramClient
from remote_teleop_runtime.follower_io import FollowerIOWorker


class LocalRig:
    def __init__(self, directory):
        self.socket_path = str(directory / 'watchdog.sock')
        self.log = (directory / 'watchdog.log').open('w+')
        self.process = subprocess.Popen([
            sys.executable, '-m', 'remote_teleop_follower_safety.service',
            '--socket', self.socket_path, '--simulation-verified-reaction', 'position_hold',
        ], stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 3.
        while not Path(self.socket_path).exists():
            assert self.process.poll() is None, 'isolated watchdog exited'
            assert time.monotonic() < deadline, 'isolated watchdog startup timeout'
            time.sleep(.005)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(('127.0.0.1', 0))
        self.rx.setblocking(False)
        self.endpoint = self.rx.getsockname()
        assert self.endpoint[0] == '127.0.0.1'
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.worker = FollowerIOWorker(self.rx, UnixDatagramClient(self.socket_path), 4242, rate=100., kernel_timestamps=True)
        self.sequence = 0
        self.worker.start()

    def pump(self, duration, *, actions=True, control=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            now = time.monotonic_ns()
            if actions:
                self.sequence += 1
                self.sender.sendto(encode_action(ActionCommand(
                    3131, self.sequence, now, (0.,) * 16, 100_000_000)), self.endpoint)
            if control:
                self.worker.control_completed(now, time.monotonic_ns(), now, True)
            time.sleep(.004)
        return self.worker.snapshot()

    def command(self, name, **fields):
        client = UnixDatagramClient(self.socket_path)
        self.worker.begin_command()
        sent = time.monotonic_ns()
        try:
            reply = client.exchange(encode_command(name, **fields), .05)
        finally:
            client.close()
        assert self.worker.finish_command(name, reply, sent), reply
        return reply

    def run(self):
        snapshot = self.pump(.12)
        assert snapshot['action'] is not None
        assert snapshot['safety']['state'] == 'ALIGNING', snapshot
        assert self.command('alignment_complete', leader_session_id=3131)['state'] == 'READY'
        assert self.command('request_run', leader_session_id=3131)['state'] == 'RUNNING'
        assert self.pump(.04)['safety']['state'] == 'RUNNING'

    def close(self):
        self.worker.close()
        self.sender.close()
        self.process.terminate()
        self.process.wait(timeout=3)
        self.log.close()


@pytest.fixture
def rig(tmp_path):
    value = LocalRig(tmp_path)
    try:
        yield value
    finally:
        value.close()


def test_executor_pause_does_not_fake_network_loss_but_real_loss_latches(rig):
    rig.run()
    # I/O continues while ROS publishes no cycles for 180 ms: below the actual
    # 250 ms control deadline, but beyond the 150 ms network deadline.
    snapshot = rig.pump(.18, control=False)
    assert snapshot['safety']['state'] == 'RUNNING', snapshot
    assert snapshot['safety']['fault_bits'] == 0
    assert rig.pump(.04)['safety']['state'] == 'RUNNING'
    # Genuine silence still faults at the unchanged network deadline.
    snapshot = rig.pump(.19, actions=False)
    assert snapshot['safety']['state'] == 'FAULT', snapshot
    assert snapshot['safety']['fault_bits'] & int(FaultBits.NETWORK_TIMEOUT)
    evidence = snapshot['safety']['first_fault']
    assert evidence['action_age_ms'] > 150.
    assert evidence['control_age_ms'] < 250.
    assert rig.pump(.05)['safety']['state'] == 'FAULT'
    assert rig.worker.snapshot()['safety']['first_fault'] == evidence


def test_live_network_cannot_hide_a_stopped_control_loop(rig):
    rig.run()
    snapshot = rig.pump(.29, control=False)
    assert snapshot['safety']['state'] == 'FAULT', snapshot
    assert snapshot['safety']['fault_bits'] & int(FaultBits.CONTROL_CYCLE_TIMEOUT)
    evidence = snapshot['safety']['first_fault']
    assert evidence['control_age_ms'] > 250.
    assert evidence['action_age_ms'] < 150.
    assert rig.pump(.05)['safety']['state'] == 'FAULT'

"""Interrupt every post-disable boundary; preserve proof without relaxing start gates."""
import copy
import json
import subprocess
import sys
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import inspect_teleop_recovery as recovery
import power_down_arms as stop
import start_teleop_from_console as start
from test_disconnect_recovery import normal_power_down


def initial():
    e = normal_power_down()
    e['host_unit'].update(ActiveState='active', MainPID='100', ControlGroup='/teleop')
    e['local_control_processes'] = [101, 102]
    return e


def with_checkpoint(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', tmp_path / 'receipt.json')
    before = initial()
    receipt = recovery.save_disable_checkpoint(before, before['power_down_receipt']['devices'])
    before['power_down_receipt'] = receipt
    after = copy.deepcopy(before)
    after['host_unit'].update(ActiveState='failed', MainPID='0', ControlGroup='')
    after['local_control_processes'] = []
    return before, after, receipt


def test_checkpoint_survives_process_exit_before_final_receipt(tmp_path, monkeypatch):
    before, after, receipt = with_checkpoint(tmp_path, monkeypatch)
    assert recovery.read_power_down_receipt() == receipt
    assert not recovery.known_power_down_eligible(before)
    assert recovery.known_power_down_eligible(before, allow_active_checkpoint=True)
    assert recovery.classify(before)['code'] == 'STOP_INCOMPLETE'
    assert recovery.classify(after)['code'] == 'OPERATOR_POWER_DOWN_CONFIRM'
    result = recovery.complete_disable_checkpoint(receipt['shutdown_id'], after)
    assert result['phase'] == 'complete'
    assert result['devices'] == receipt['devices']


@pytest.mark.parametrize('change', ['session', 'boot', 'process', 'invocation', 'enabled', 'estop', 'invalid_command', 'recorder', 'orphan_host'])
def test_checkpoint_never_admits_changed_or_faulted_evidence(tmp_path, monkeypatch, change):
    before, e, receipt = with_checkpoint(tmp_path, monkeypatch)
    if change == 'session': e['remote']['status']['leader_session_id'] += 1
    if change == 'boot': e['remote']['boot_id'] = 'new'
    if change == 'process': e['remote']['process_identities']['234']['start_ticks'] += 1
    if change == 'invocation': e['host_unit']['InvocationID'] = 'new'
    if change == 'enabled': e['motor_report']['host']['motors']['can0/1']['states'] = [1]
    if change == 'estop': e['remote']['status'].update(state='E_STOP', fault_bits=32)
    if change == 'invalid_command': e['remote']['status']['fault_bits'] = 10
    if change == 'recorder': e['remote']['recorder_idle'] = False
    if change == 'orphan_host': e['local_control_processes'] = [999]
    assert not recovery.known_power_down_eligible(e, allow_active_checkpoint=True)
    with pytest.raises(RuntimeError):
        recovery.complete_disable_checkpoint(receipt['shutdown_id'], e)
    assert recovery.read_power_down_receipt() == receipt


def test_retry_rechecks_identity_before_stopping_service(tmp_path, monkeypatch):
    before, after, receipt = with_checkpoint(tmp_path, monkeypatch)
    changed = copy.deepcopy(before)
    changed['remote']['process_identities']['234']['start_ticks'] += 1
    reads = iter([before, changed])
    with patch.object(stop, 'seal_recording'), patch.object(stop, 'RecorderClient'), \
            patch.object(stop, 'stop_service') as service:
        with pytest.raises(stop.ShutdownError):
            stop.resume_checkpoint(recovery.connection_config(), lambda _: {'evidence': next(reads)})
    service.assert_not_called()


def test_partial_motor_proof_is_never_checkpointed(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', tmp_path / 'receipt.json')
    e = initial(); reports = e['power_down_receipt']['devices']
    reports['host']['motors']['can0/1']['states'] = [0, 0]
    with pytest.raises(RuntimeError): recovery.save_disable_checkpoint(e, reports)
    assert not recovery.POWER_DOWN_RECEIPT.exists()


@pytest.mark.parametrize('already_stopped', [False, True])
def test_retry_finishes_from_checkpoint_without_motor_commands(tmp_path, monkeypatch, already_stopped):
    before, after, receipt = with_checkpoint(tmp_path, monkeypatch)
    current = after if already_stopped else before
    reads = iter([current, copy.deepcopy(current), after])
    with patch.object(stop, 'seal_recording'), patch.object(stop, 'RecorderClient'), \
            patch.object(stop, 'stop_service') as service, patch.object(stop, 'emit'), \
            patch.object(stop, 'DeviceProcess') as device, patch.object(stop, 'hold_following') as hold:
        result = stop.resume_checkpoint(recovery.connection_config(), lambda _: {'evidence': next(reads)})
    assert result['ok']
    assert service.call_count == (0 if already_stopped else 1)
    device.assert_not_called(); hold.assert_not_called()
    assert recovery.read_power_down_receipt()['phase'] == 'complete'


def test_repeated_completed_stop_is_idempotent(tmp_path, monkeypatch):
    before, after, receipt = with_checkpoint(tmp_path, monkeypatch)
    after['power_down_receipt'] = recovery.complete_disable_checkpoint(receipt['shutdown_id'], after)
    with patch.object(stop, 'seal_recording'), patch.object(stop, 'RecorderClient'), \
            patch.object(stop, 'stop_service') as service, patch.object(stop, 'emit'), \
            patch.object(stop, 'DeviceProcess') as device:
        result = stop.resume_checkpoint(recovery.connection_config(), lambda _: {'evidence': after})
    assert result['ok']
    service.assert_not_called(); device.assert_not_called()


def test_stop_failure_keeps_checkpoint_and_retry_is_available(tmp_path, monkeypatch):
    before, after, receipt = with_checkpoint(tmp_path, monkeypatch)
    with patch.object(stop, 'seal_recording'), patch.object(stop, 'RecorderClient'), \
            patch.object(stop, 'stop_service', side_effect=subprocess.TimeoutExpired('stop', 25)):
        with pytest.raises(subprocess.TimeoutExpired):
            stop.resume_checkpoint(recovery.connection_config(), lambda _: {'evidence': before})
    assert recovery.read_power_down_receipt() == receipt


@pytest.mark.parametrize('fail_at', ['stop', 'finish', 'checkpoint'])
def test_real_coordinator_checkpoints_before_service_exit(tmp_path, monkeypatch, fail_at):
    before = initial(); reports = before['power_down_receipt']['devices']; events = []
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', tmp_path / 'receipt.json')
    class Device:
        def __init__(self, side): self.side = side
        def arm(self): pass
        def go(self): pass
        def result(self): return reports[self.side]
        def close(self): pass
    def checkpoint(identity, proof):
        events.append('checkpoint')
        if fail_at == 'checkpoint': raise OSError('disk full')
        recovery.save_disable_checkpoint(identity, proof)
    def stop_service():
        events.append('stop')
        assert recovery.read_power_down_receipt()['phase'] == 'drives_disabled'
        if fail_at == 'stop': raise RuntimeError('interrupted stop')
    def finish(*_):
        events.append('finish'); raise RuntimeError('SSH unavailable after stop')
    with patch.object(stop, 'seal_recording'):
        result = stop.coordinate(None, lambda: None, Device, stop_service, lambda _: None,
            capture=lambda: before, checkpoint=checkpoint, finish=finish)
    assert not result['ok']
    if fail_at == 'checkpoint':
        assert events == ['checkpoint'] and not recovery.POWER_DOWN_RECEIPT.exists()
    else:
        assert events[:2] == ['checkpoint', 'stop']
        assert recovery.read_power_down_receipt()['devices'] == reports


@pytest.mark.parametrize('failure', ['systemctl', 'no_new_invocation'])
def test_failed_start_does_not_consume_valid_stop_receipt(tmp_path, monkeypatch, failure):
    _, after, receipt = with_checkpoint(tmp_path, monkeypatch)
    inspected = dict(recovery.classify(after), evidence=after)
    unit = dict(after['host_unit'], InvocationID='unit-instance', StandardOutput='journal')
    run = Mock(side_effect=RuntimeError('systemctl failed')) if failure == 'systemctl' else Mock()
    with patch.object(start, 'RecorderClient', return_value=lambda _: {'ok': True, 'running': False, 'active_episode': None}), \
            patch.object(start, 'unit_status', return_value=unit), patch.object(start, 'emit'), \
            patch.object(start, 'verify_motor_preflight'), \
            patch.object(start.subprocess, 'run', run), patch.object(start, 'consume_power_down_receipt') as consume:
        with pytest.raises(RuntimeError):
            start.start(recovery.connection_config(), inspect_fn=lambda _: inspected)
    consume.assert_not_called()
    assert recovery.read_power_down_receipt() == receipt

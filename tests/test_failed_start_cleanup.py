"""Offline recovery admission and exact-process cleanup; no robot commands."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import cleanup_rebooted_teleop as cleanup
import inspect_teleop_recovery as recovery
import terminate_isolated_follower as terminator
from start_teleop_from_console import start_decision


def failed():
    root = '/home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/'
    argv = [
        ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'remote_teleop_runtime', 'bimanual_follower.launch.py', ''],
        [root+'openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node', '--ros-args', '-r', '__node:=follower_gravity_pd', ''],
        ['/usr/bin/python3', root+'remote_teleop_runtime/lib/remote_teleop_runtime/remote-teleop-follower', ''],
        ['/usr/bin/python3', root+'remote_teleop_follower_safety/lib/remote_teleop_follower_safety/remote-teleop-follower-watchdog', ''],
    ]
    motors = {'result': 'UNKNOWN'}
    for side, buses in recovery.buses_by_side():
        motors[side] = {'result': 'UNKNOWN', 'motors': {
            f'{bus}/{j}': dict(states=[], samples=0, state=None, fresh=False)
            for bus in buses for j in range(1, 9)}}
    e = dict(ssh_ok=True, host_can_ok=True, host_boot_id='host-boot', local_control_processes=[],
        motor_report=motors, host_unit=dict(LoadState='loaded', ActiveState='failed', MainPID='0',
        ControlGroup='', InvocationID='failed-start'), remote=dict(boot_id='peer-boot', can_ok=True,
        recorder_idle=True, processes=[101, 102, 103, 104], runtime_socket_exists=True,
        process_identities={str(101+i): dict(start_ticks=100+i, argv=a) for i, a in enumerate(argv)},
        status=dict(state='FAULT', fault_bits=1, reason='leader action receive timeout', leader_session_id=123,
                    collection=dict(recording=None, transitioning_arms=[], return_phase='idle',
                                    left_static_hold_phase='idle', left_mode='FOLLOW', right_mode='FOLLOW'))))
    return dict(recovery.classify(e), evidence=e)


def emptied(before):
    after = copy.deepcopy(before)
    after['evidence']['remote'].update(processes=[], process_identities={}, status=None, runtime_socket_exists=False)
    return after


def test_power_off_cleanup_is_not_start_permission():
    value = failed()
    assert value['code'] == 'FAILED_START_POWER_OFF_REQUIRED'
    assert value['cleanup_available'] and not value['confirmation_required']
    with pytest.raises(RuntimeError):
        start_decision(value)
    assert start_decision(emptied(value)) == 'cold'


def interrupted_alignment():
    value = failed()
    value['evidence']['remote']['status'].update(
        state='ALIGNING', fault_bits=0, aligned=False, leader_session_id=None,
        reason='explicit HOLD accepted; fresh RUN request required')
    return dict(recovery.classify(value['evidence']), evidence=value['evidence'])


def test_failed_homing_alignment_can_only_be_cleaned_after_power_off(tmp_path):
    value = interrupted_alignment()
    assert value['code'] == 'FAILED_START_POWER_OFF_REQUIRED'
    assert cleanup.cleanup_kind(value) == 'failed_start'
    with pytest.raises(RuntimeError):
        start_decision(value)
    with pytest.raises(RuntimeError, match='电源'):
        cleanup.cleanup(recovery.connection_config(), inspect_fn=lambda _: value)
    result, calls = run_cleanup([value, copy.deepcopy(value), emptied(value)], tmp_path)
    assert result['ok'] and len(calls) == 1
    assert json.loads((tmp_path/'before.json').read_text())['evidence']['remote']['status']['state'] == 'ALIGNING'


@pytest.mark.parametrize('change', ['running', 'aligned', 'session', 'missing_session',
    'not_held', 'fault', 'active_host', 'recording', 'motor_feedback', 'can_down'])
def test_alignment_cleanup_rejects_live_or_uncertain_startup(change):
    e = interrupted_alignment()['evidence']; s = e['remote']['status']
    if change == 'running': s['state'] = 'RUNNING'
    elif change == 'aligned': s['aligned'] = True
    elif change == 'session': s['leader_session_id'] = 123
    elif change == 'missing_session': del s['leader_session_id']
    elif change == 'not_held': s['reason'] = 'restart requires alignment'
    elif change == 'fault': s['fault_bits'] = 1
    elif change == 'active_host': e['host_unit']['ActiveState'] = 'active'
    elif change == 'recording': e['remote']['recorder_idle'] = False
    elif change == 'motor_feedback': e['motor_report']['follower']['motors']['can1/1']['samples'] = 1
    elif change == 'can_down': e['remote']['can_ok'] = False
    assert cleanup.cleanup_kind(dict(evidence=e)) is None


@pytest.mark.parametrize('change', ['ssh', 'can', 'local_process', 'active', 'recording', 'motion',
    'estop', 'hardware_fault', 'wrong_reason', 'missing_probe', 'probe_error', 'positive_feedback',
    'unrelated_process', 'duplicate_role', 'missing_identity', 'invalid_ticks'])
def test_contrary_or_incomplete_evidence_rejects_cleanup(change):
    e = failed()['evidence']; r = e['remote']; s = r['status']
    if change == 'ssh': e['ssh_ok'] = False
    elif change == 'can': r['can_ok'] = False
    elif change == 'local_process': e['local_control_processes'] = [555]
    elif change == 'active': e['host_unit']['ActiveState'] = 'active'
    elif change == 'recording': r['recorder_idle'] = False
    elif change == 'motion': s['collection']['right_mode'] = 'RETURNING'
    elif change == 'estop': s['state'] = 'E_STOP'
    elif change == 'hardware_fault': s['fault_bits'] = 8
    elif change == 'wrong_reason': s['reason'] = 'unexpected failure'
    elif change == 'missing_probe': e['motor_report']['host']['motors'].pop('can0/1')
    elif change == 'probe_error': e['motor_report']['follower']['error'] = 'socket failed'
    elif change == 'positive_feedback': e['motor_report']['host']['motors']['can0/1']['states'] = [1]
    elif change == 'unrelated_process': r['process_identities']['102']['argv'][0] = '/opt/ros/humble/lib/rviz2/rviz2'
    elif change == 'duplicate_role': r['process_identities']['103'] = copy.deepcopy(r['process_identities']['102'])
    elif change == 'missing_identity': del r['process_identities']['102']
    elif change == 'invalid_ticks': r['process_identities']['102']['start_ticks'] = None
    assert not recovery.failed_start_cleanup_available(e)


def test_partial_cleanup_with_gateway_gone_can_be_retried():
    e = failed()['evidence']; e['remote']['status'] = None
    del e['remote']['process_identities']['103']; e['remote']['processes'].remove(103)
    assert recovery.failed_start_cleanup_available(e)


def bus_off(value):
    r = value['evidence']['remote']
    r['can_ok'] = False
    r['can'] = [dict(ifname=bus, linkinfo=dict(info_kind='can', info_data=dict(state=state)))
                for bus, state in zip(recovery.connection_config()['follower_interfaces'],
                                      ('ERROR-PASSIVE', 'BUS-OFF'))]
    return value


def test_busoff_cleanup_stops_only_old_processes_and_does_not_grant_restart(tmp_path):
    before = bus_off(interrupted_alignment())
    assert cleanup.cleanup_kind(before) == 'failed_start'
    with pytest.raises(RuntimeError):
        start_decision(before)
    after = emptied(before)
    result, calls = run_cleanup([before, copy.deepcopy(before), after], tmp_path)
    assert result['ok'] and result['can_recovery_required']
    assert '保持电机电源关闭' in result['message']
    assert len(calls) == 1
    assert cleanup.cleanup_kind(after) == 'already_empty'
    with pytest.raises(RuntimeError):
        start_decision(after)


@pytest.mark.parametrize('change', ['missing_bus', 'wrong_bus', 'wrong_kind', 'unknown_state',
                                  'feedback', 'probe_error', 'missing_probe'])
def test_busoff_cleanup_still_requires_interfaces_and_complete_silent_probes(change):
    value = bus_off(interrupted_alignment()); e = value['evidence']; r = e['remote']
    if change == 'missing_bus': r['can'].pop()
    elif change == 'wrong_bus': r['can'][0]['ifname'] = 'can0'
    elif change == 'wrong_kind': r['can'][0]['linkinfo']['info_kind'] = 'vcan'
    elif change == 'unknown_state': r['can'][0]['linkinfo']['info_data']['state'] = 'UNKNOWN'
    elif change == 'feedback': e['motor_report']['follower']['motors']['can1/1']['states'] = [1]
    elif change == 'probe_error': e['motor_report']['follower']['error'] = 'socket unavailable'
    elif change == 'missing_probe': del e['motor_report']['follower']['motors']['can1/1']
    assert cleanup.cleanup_kind(value) is None
    assert cleanup.cleanup_kind(emptied(value)) is None


def run_cleanup(reads, tmp_path, unit=None):
    reads = iter(reads)
    with patch.object(cleanup, 'emit'), patch.object(cleanup, 'RecorderClient') as recorder, \
         patch.object(cleanup, 'unit_status', return_value=unit or failed()['evidence']['host_unit']), \
         patch.object(cleanup, 'stop_isolated_follower') as stop, \
         patch.object(cleanup.tempfile, 'mkdtemp', return_value=str(tmp_path)), \
         patch.object(cleanup.subprocess, 'run', side_effect=AssertionError('no host service mutation')):
        recorder.return_value.return_value = dict(ok=True, running=False, active_episode=None, phase='idle')
        result = cleanup.cleanup(recovery.connection_config(), power_off_confirmed=True,
                                 inspect_fn=lambda _: next(reads))
        return result, stop.call_args_list


def test_cleanup_preserves_fault_evidence_and_never_starts(tmp_path):
    before = failed()
    result, calls = run_cleanup([before, copy.deepcopy(before), emptied(before)], tmp_path)
    assert result['ok']
    assert len(calls) == 1 and calls[0].args[0] == 'openarm-jetson'
    assert calls[0].args[1]['remote']['process_identities'] == before['evidence']['remote']['process_identities']
    saved = json.loads((tmp_path/'before.json').read_text())
    assert saved['evidence']['remote']['status']['reason'] == 'leader action receive timeout'


def test_already_empty_cleanup_succeeds_without_process_signals(tmp_path):
    before = emptied(failed())
    assert cleanup.cleanup_kind(before) == 'already_empty'
    result, calls = run_cleanup([before, copy.deepcopy(before), copy.deepcopy(before)], tmp_path)
    assert result['ok'] and '无需重复清理' in result['message']
    assert calls == []
    assert not (tmp_path/'before.json').exists()


@pytest.mark.parametrize('change', ['enabled', 'error', 'missing', 'bad_samples', 'recording', 'orphan', 'socket'])
def test_empty_stack_does_not_bypass_evidence_checks(change):
    value = emptied(failed()); e = value['evidence']; r = e['remote']
    if change == 'enabled': e['motor_report']['host']['motors']['can0/1']['states'] = [1]
    elif change == 'error': e['motor_report']['host']['error'] = 'probe failed'
    elif change == 'missing': del e['motor_report']['host']['motors']['can0/1']
    elif change == 'bad_samples': e['motor_report']['host']['motors']['can0/1']['samples'] = False
    elif change == 'recording': r['recorder_idle'] = False
    elif change == 'orphan': e['local_control_processes'] = [999]
    elif change == 'socket': r['runtime_socket_exists'] = True
    assert cleanup.cleanup_kind(value) is None


def test_empty_stack_identity_change_aborts_without_signals(tmp_path):
    first = emptied(failed()); second = copy.deepcopy(first)
    second['evidence']['remote']['boot_id'] = 'another-boot'
    with pytest.raises(RuntimeError, match='改变'):
        run_cleanup([first, second], tmp_path)


@pytest.mark.parametrize('change', ['boot', 'process', 'service', 'enabled', 'recording'])
def test_change_between_checks_sends_no_signals(change, tmp_path):
    first = failed(); second = copy.deepcopy(first); e = second['evidence']
    if change == 'boot': e['remote']['boot_id'] = 'new-boot'
    elif change == 'process': e['remote']['process_identities']['102']['start_ticks'] += 1
    elif change == 'service': e['host_unit']['InvocationID'] = 'new'
    elif change == 'enabled': e['motor_report']['follower']['motors']['can1/1']['samples'] = 1
    elif change == 'recording': e['remote']['recorder_idle'] = False
    with patch.object(cleanup, 'stop_isolated_follower') as stop, patch.object(cleanup, 'emit'):
        reads = iter([first, second])
        with pytest.raises(RuntimeError, match='改变'):
            cleanup.cleanup(recovery.connection_config(), power_off_confirmed=True, inspect_fn=lambda _: next(reads))
        stop.assert_not_called()


def test_unconfirmed_power_off_never_contacts_robot():
    with pytest.raises(RuntimeError, match='电源'):
        cleanup.cleanup(recovery.connection_config(), inspect_fn=lambda _: pytest.fail('unexpected read'))


def test_real_pidfd_signals_only_the_named_dummy_process():
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        pytest.skip('Linux pidfd required')
    target = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
        pid = str(target.pid)
        evidence = dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                        process_identities={pid: terminator.process_identity(pid)})
        wrong = copy.deepcopy(evidence); wrong['process_identities'][pid]['start_ticks'] += 1
        with pytest.raises(RuntimeError, match='替换'):
            terminator.terminate_exact(wrong)
        assert target.poll() is None and unrelated.poll() is None
        assert terminator.terminate_exact(evidence)['stopped_pids'] == [pid]
        assert target.wait(timeout=3) == -signal.SIGTERM
        assert unrelated.poll() is None
    finally:
        for p in (target, unrelated):
            if p.poll() is None: p.terminate()
            p.wait(timeout=3)

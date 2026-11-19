"""Drive-power recovery through the real start/stop coordinators, with fake I/O."""
import copy
import json
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import inspect_teleop_recovery as recovery
import power_down_arms as stop
import preflight_motor_feedback as preflight
import recover_powered_session as powered
import start_teleop_from_console as start
import stop_without_gateway as missing
import teleop_readiness
from test_start_preflight_recovery import abandoned, good_reports


def lost_power():
    result = abandoned()
    e = result['evidence']
    e.update(profile='jetson', peer='openarm-jetson', local_control_processes=[200, 201])
    e['host_unit'].update(ActiveState='active', MainPID='199', ControlGroup='/teleop')
    e['remote']['status'].update(state='FAULT', fault_bits=2,
                                reason='controller reports CAN failure', leader_session_id=123)
    return result


@pytest.fixture
def rig(monkeypatch, tmp_path):
    monkeypatch.setenv('OPENARM_TELEOP_PROFILE', 'jetson')
    cfg = recovery.connection_config()
    e = lost_power()['evidence']
    query = good_reports()
    effects, events = [], []
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', tmp_path / 'normal.json')
    monkeypatch.setattr(missing, 'JOURNAL', tmp_path / 'missing.json')
    monkeypatch.setattr(start, 'emit', events.append)
    monkeypatch.setattr(preflight, 'both_endpoints', lambda *a, **k: copy.deepcopy(query))
    monkeypatch.setattr(stop, 'RecorderClient', Mock())
    monkeypatch.setattr(stop, 'seal_recording', lambda _: effects.append('seal'))
    def inspect(_):
        current = copy.deepcopy(e)
        current['power_down_receipt'] = recovery.read_power_down_receipt()
        return dict(recovery.classify(current), evidence=current)
    def hold(_):
        effects.append('hold')
        if e['remote']['status']['state'] != 'FAULT':
            e['remote']['status'].update(state='READY', fault_bits=0)
    monkeypatch.setattr(stop, 'hold_following', hold)
    class Device:
        def __init__(self, side, *args): self.side = side
        def arm(self): effects.append('arm:' + self.side)
        def go(self):
            assert {'arm:host', 'arm:follower'} <= set(effects)
            effects.append('disable:' + self.side)
            query[self.side] = good_reports()[self.side]
        def result(self):
            return {'ok': False, 'verified': False, 'acknowledged': True,
                    'error': '缺少连续失能反馈：already disabled',
                    'motors': {key: {'states': [], 'disabled': False}
                               for key in query[self.side]['motors']}}
        def close(self): effects.append('close:' + self.side)
    monkeypatch.setattr(stop, 'DeviceProcess', Device)
    def stop_service(_):
        effects.append('stop_service')
        e['host_unit'].update(ActiveState='inactive', MainPID='0', ControlGroup='')
        e['local_control_processes'] = []
    monkeypatch.setattr(stop, 'stop_service', stop_service)
    def terminate(_, evidence):
        effects.append('terminate')
        e['remote'].update(processes=[], process_identities={})
    monkeypatch.setattr(missing, 'terminate_follower', terminate)
    return cfg, e, query, effects, events, inspect


@pytest.mark.parametrize('side,bus', [('host', 'can0'), ('host', 'can1'),
                                    ('follower', 'can1'), ('follower', 'can2'), ('both', None)])
@pytest.mark.parametrize('watchdog', ['RUNNING', 'FAULT'])
def test_each_arm_or_both_power_cycled_recovers_through_real_stop(rig, side, bus, watchdog):
    cfg, e, query, effects, events, inspect = rig
    for role in query:
        for key in query[role]['motors']:
            query[role]['motors'][key] = [0, 0, 0] if side == 'both' or (role == side and key.startswith(bus+'/')) else [1, 1, 1]
    if watchdog == 'RUNNING': e['remote']['status'].update(state='RUNNING', fault_bits=0)
    result = start.prepare_start(cfg, inspect(''), inspect)
    assert start.start_decision(result) == 'restart'
    assert {'disable:host', 'disable:follower', 'stop_service'} <= set(effects)
    assert recovery.read_power_down_receipt()['phase'] == 'complete'
    assert all(value == [0, 0, 0] for report in query.values() for value in report['motors'].values())
    assert all(event['event'] == 'progress' for event in events)


def test_gateway_crash_after_power_cycle_becomes_verified_cold_start(rig):
    cfg, e, _, effects, events, inspect = rig
    e['remote'].update(status=None, runtime_socket_exists=False)
    del e['remote']['process_identities']['103']
    e['remote']['processes'].remove(103)
    result = start.prepare_start(cfg, inspect(''), inspect)
    assert start.start_decision(result) == 'cold'
    assert 'hold' not in effects and 'terminate' in effects
    assert missing.read_journal()['phase'] == 'complete'


@pytest.mark.parametrize('side', ['host', 'follower'])
@pytest.mark.parametrize('failure', ['off', 'fault', 'stale', 'mixed', 'boolean', 'missing_side'])
def test_bad_or_missing_drive_feedback_never_disables_or_restarts(rig, side, failure, monkeypatch):
    cfg, _, query, effects, _, inspect = rig
    key = next(iter(query[side]['motors']))
    if failure == 'off': query[side].update(ok=False, error='电源未接通')
    if failure == 'fault': query[side]['motors'][key] = [8, 8, 8]
    if failure == 'stale': query[side]['motors'][key] = []
    if failure == 'mixed': query[side]['motors'][key] = [0, 1, 0]
    if failure == 'boolean': query[side]['motors'][key] = [False] * 3
    if failure == 'missing_side': del query[side]
    restart = Mock(); monkeypatch.setattr(start.subprocess, 'run', restart)
    with pytest.raises(RuntimeError): start.start(cfg, inspect_fn=inspect)
    assert effects == []
    restart.assert_not_called()


@pytest.mark.parametrize('change', ['estop', 'tracking_fault', 'unknown_fault', 'recording', 'returning',
                                  'aligning', 'wrong_controller', 'duplicate_controller', 'socket',
                                  'unknown_status', 'can', 'profile', 'no_session', 'transition'])
def test_recovery_admission_rejects_unsafe_or_ambiguous_state(change):
    r = lost_power(); e = r['evidence']; remote = e['remote']; s = remote['status']; c = s['collection']
    if change == 'estop': s.update(state='E_STOP', fault_bits=32)
    if change == 'tracking_fault': s['fault_bits'] = 8
    if change == 'unknown_fault': s['reason'] = 'unknown hardware fault'
    if change == 'recording': c['recording'] = 'episode'
    if change == 'returning': c['return_phase'] = 'moving'
    if change == 'aligning': c['left_align_active'] = True
    if change == 'wrong_controller': remote['process_identities']['102']['argv'] = ['/unrelated/controller']
    if change == 'duplicate_controller':
        remote['process_identities']['105'] = copy.deepcopy(remote['process_identities']['102'])
        remote['processes'].append(105)
    if change == 'socket': remote['runtime_socket_exists'] = False
    if change == 'unknown_status': s['state'] = 'UNKNOWN'
    if change == 'can': e['host_can_ok'] = False
    if change == 'profile': e['profile'] = 'wrong'
    if change == 'no_session': s['leader_session_id'] = None
    if change == 'transition': c['transitioning_arms'] = ['right']
    assert not powered.eligible(r, recovery.connection_config())


@pytest.mark.parametrize('when', ['query', 'after_stop'])
def test_changed_session_during_recovery_never_restarts(rig, monkeypatch, when):
    cfg, e, _, effects, _, inspect = rig
    original = start.verify_motor_preflight
    if when == 'query':
        def query(*args, **kwargs):
            value = original(*args, **kwargs)
            e['host_unit']['InvocationID'] = 'new'
            return value
        monkeypatch.setattr(start, 'verify_motor_preflight', query)
    else:
        def stopped(*args, **kwargs):
            e['remote']['boot_id'] = 'new'
            return {'ok': True}
        monkeypatch.setattr(stop, 'perform_stop', stopped)
    with pytest.raises(RuntimeError, match='改变|未核验'):
        start.prepare_start(cfg, inspect(''), inspect)
    assert 'stop_service' not in effects


def test_failed_disable_blocks_power_cycle_restart(rig, monkeypatch):
    cfg, _, _, effects, _, inspect = rig
    monkeypatch.setattr(stop, 'perform_stop', Mock(return_value={'ok': False, 'message': '从臂失能服务未确认'}))
    with pytest.raises(RuntimeError, match='失能未完成'):
        start.prepare_start(cfg, inspect(''), inspect)
    assert 'stop_service' not in effects


@pytest.mark.parametrize('gateway_missing', [False, True])
def test_start_continues_after_recovery_and_requires_new_initialized_session(rig, monkeypatch, gateway_missing):
    cfg, e, _, effects, events, inspect = rig
    if gateway_missing:
        e['remote'].update(status=None, runtime_socket_exists=False)
        del e['remote']['process_identities']['103']
        e['remote']['processes'].remove(103)
    original_unit = copy.deepcopy(e['host_unit'])
    units = iter([original_unit, {'InvocationID': 'new-session', 'ActiveState': 'active'},
                  {'InvocationID': 'new-session', 'ActiveState': 'active'}])
    monkeypatch.setattr(start, 'unit_status', lambda _: next(units))
    monkeypatch.setattr(start, 'startup_log_cursor', lambda *a: None)
    monkeypatch.setattr(start, 'RecorderClient', lambda _: lambda command: {
        'ok': True, 'running': False, 'active_episode': None})
    monkeypatch.setattr(start, 'follower_status', lambda _: {'leader_session_id': 124})
    monkeypatch.setattr(start, 'apply_current_report', lambda _: {'teleop_ready': True})
    monkeypatch.setattr(teleop_readiness, 'read_report', lambda: {'probe_started_monotonic_ns': 1001})
    monkeypatch.setattr(start.time, 'monotonic_ns', lambda: 1000)
    real_path = Path
    class Acceptance:
        def read_text(self): return json.dumps({'accepted': True, 'leader_session_id': 124})
    monkeypatch.setattr(start, 'Path', lambda path: Acceptance() if str(path).endswith('alignment-following.json') else real_path(path))
    def restart(args, **kwargs):
        assert args == ['systemctl', '--user', 'restart', cfg['service']]
        assert 'stop_service' in effects
        assert all(v == [0, 0, 0] for report in rig[2].values() for v in report['motors'].values())
        effects.append('restart')
    monkeypatch.setattr(start.subprocess, 'run', restart)
    result = start.start(cfg, inspect_fn=inspect)
    assert result['ok'] and '归位和对齐通过' in result['message']
    assert effects.count('restart') == 1
    assert effects.index('stop_service') < effects.index('restart')
    assert all(event['event'] == 'progress' for event in events)

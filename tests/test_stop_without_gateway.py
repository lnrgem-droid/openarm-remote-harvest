"""Offline failure injection: never connects to ROS, SSH, CAN or systemd."""
import copy
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import inspect_teleop_recovery as recovery
import power_down_arms as stop
import preflight_motor_feedback as feedback
import stop_without_gateway as fallback


def snapshot():
    roles = [('bimanual_follower.launch.py', []), ('remote-teleop-follower-watchdog', []),
             ('openarm_gravity_pd_node', ['__node:=follower_gravity_pd'])]
    ids = {str(i): {'start_ticks': i * 10, 'argv': [f'/opt/test/{role}', *args]}
           for i, (role, args) in enumerate(roles, 200)}
    return {'ssh_ok': True, 'profile': 'jetson', 'peer': 'openarm-jetson',
            'host_can_ok': True, 'host_boot_id': 'host-boot',
            'host_unit': {'LoadState': 'loaded', 'ActiveState': 'active', 'MainPID': '100',
                          'ControlGroup': '/teleop', 'InvocationID': 'old-session'},
            'local_control_processes': [101, 102],
            'remote': {'boot_id': 'remote-boot', 'processes': [200, 201, 202],
                       'process_identities': ids, 'status': None, 'runtime_socket_exists': False,
                       'recorder_idle': True, 'can_ok': True}}


def proof(config, *, quiet=False):
    return {side: {'ok': not quiet, 'verified': not quiet, 'acknowledged': True,
                   'error': '缺少连续失能反馈：all' if quiet else None,
                   'motors': {f'{bus}/{i}': {'states': [] if quiet else [0, 0, 0],
                                           'disabled': not quiet}
                              for bus in config[side + '_interfaces'] for i in range(1, 9)}}
            for side in ('host', 'follower')}


@pytest.fixture
def rig(monkeypatch, tmp_path):
    monkeypatch.setenv('OPENARM_TELEOP_PROFILE', 'jetson')
    config = recovery.connection_config()
    state = snapshot()
    events, effects = [], []
    reports = proof(config)
    query = {side: {'ok': True, 'motors': {k: [0, 0, 0] for k in report['motors']}}
             for side, report in reports.items()}
    monkeypatch.setattr(fallback, 'JOURNAL', tmp_path / 'gateway-stop.json')
    monkeypatch.setattr(stop, 'RecorderClient', Mock())
    monkeypatch.setattr(stop, 'seal_recording', lambda _: effects.append('seal'))
    monkeypatch.setattr(stop, 'hold_following', Mock(side_effect=AssertionError('No gateway call')))
    monkeypatch.setattr(feedback, 'verify', lambda *a, **k: copy.deepcopy(query))
    class Device:
        def __init__(self, side, *args): self.side = side
        def arm(self): effects.append('arm:' + self.side)
        def go(self):
            assert {'arm:host', 'arm:follower'} <= set(effects)
            effects.append('go:' + self.side)
        def result(self): return copy.deepcopy(reports[self.side])
        def close(self): effects.append('close:' + self.side)
    monkeypatch.setattr(stop, 'DeviceProcess', Device)
    def stop_host(_):
        assert fallback.read_journal()['phase'] == 'drives_disabled'
        effects.append('stop_host')
        state['host_unit'].update(ActiveState='inactive', MainPID='0', ControlGroup='')
        state['local_control_processes'] = []
    monkeypatch.setattr(stop, 'stop_service', stop_host)
    def terminate(config, e):
        assert fallback.host_stopped(e)
        effects.append('terminate')
        state['remote'].update(processes=[], process_identities={})
    monkeypatch.setattr(fallback, 'terminate_follower', terminate)
    inspect_fn = lambda _: {'evidence': copy.deepcopy(state)}
    return config, state, reports, query, events, effects, inspect_fn


def run(rig):
    c, _, _, _, events, _, inspect_fn = rig
    return fallback.try_stop_without_gateway(c, inspect_fn, events.append)


@pytest.mark.parametrize('quiet', [False, True])
def test_missing_gateway_stops_both_and_can_retry_without_disable(rig, quiet):
    c, state, reports, _, events, effects, _ = rig
    reports.update(proof(c, quiet=quiet))
    result = run(rig)
    assert result['ok'] and recovery.verified_disable_reports(result['devices'])
    assert effects.index('go:host') < effects.index('stop_host') < effects.index('terminate')
    assert fallback.read_journal()['phase'] == 'complete'
    effects.clear()
    assert run(rig)['ok']
    assert effects == ['seal']


@pytest.mark.parametrize('failure', ['host_stop', 'remote_stop', 'remote_partial'])
def test_cleanup_failure_retains_checkpoint_and_retry_uses_fresh_queries(rig, monkeypatch, failure):
    original_host, original_remote = stop.stop_service, fallback.terminate_follower
    def fail(*args):
        if failure == 'remote_partial':
            rig[1]['remote']['processes'].remove(201)
            del rig[1]['remote']['process_identities']['201']
        raise RuntimeError('injected cleanup failure')
    if failure == 'host_stop': monkeypatch.setattr(stop, 'stop_service', fail)
    else: monkeypatch.setattr(fallback, 'terminate_follower', fail)
    assert not run(rig)['ok']
    assert fallback.read_journal()['phase'] == 'drives_disabled'
    monkeypatch.setattr(stop, 'stop_service', original_host)
    monkeypatch.setattr(fallback, 'terminate_follower', original_remote)
    rig[5].clear()
    assert run(rig)['ok']
    assert not any(e.startswith(('go:', 'arm:')) for e in rig[5])


@pytest.mark.parametrize('change', ['ack', 'timeout', 'nonzero', 'fault', 'missing_key',
                                  'fresh_missing', 'fresh_enabled', 'fresh_boolean'])
def test_no_cleanup_or_success_without_real_complete_disable(rig, change):
    reports, query = rig[2:4]
    if change == 'ack': reports['host']['acknowledged'] = False
    if change == 'timeout': reports['host'].update(ok=False, error='服务超时')
    if change == 'nonzero': reports['follower']['motors']['can1/1']['states'] = [1]
    if change == 'fault': reports['follower']['motors']['can1/1']['states'] = [8]
    if change == 'missing_key': del reports['host']['motors']['can0/1']
    if change == 'fresh_missing': del query['follower']['motors']['can1/1']
    if change == 'fresh_enabled': query['follower']['motors']['can1/1'] = [1, 1, 1]
    if change == 'fresh_boolean': query['host']['motors']['can0/1'] = [False] * 3
    assert not run(rig)['ok']
    assert not {'stop_host', 'terminate'} & set(rig[5])
    assert not fallback.JOURNAL.exists()


@pytest.mark.parametrize('change', ['gateway', 'socket', 'status', 'ssh'])
def test_never_bypass_live_or_unreachable_gateway(rig, change):
    e = rig[1]
    if change == 'gateway': e['remote']['process_identities']['203'] = {
        'start_ticks': 20, 'argv': ['/opt/bin/remote-teleop-follower']}
    if change == 'socket': e['remote']['runtime_socket_exists'] = True
    if change == 'status': e['remote']['status'] = {'state': 'RUNNING'}
    if change == 'ssh': e['ssh_ok'] = False
    assert run(rig) is None
    assert rig[5] == []


@pytest.mark.parametrize('change', ['boot', 'invocation', 'pid_reuse', 'new_process', 'host', 'gateway'])
def test_identity_change_after_arming_prevents_disable(rig, change):
    c, state, _, _, events, effects, inspect_fn = rig
    def inspect(_):
        if 'arm:follower' in effects:
            if change == 'boot': state['remote']['boot_id'] = 'new'
            if change == 'invocation': state['host_unit']['InvocationID'] = 'new'
            if change == 'pid_reuse': state['remote']['process_identities']['202']['start_ticks'] += 1
            if change == 'new_process': state['remote']['processes'].append(999)
            if change == 'host': state['local_control_processes'] = [999]
            if change == 'gateway': state['remote']['status'] = {'state': 'RUNNING'}
        return inspect_fn(_)
    result = fallback.try_stop_without_gateway(c, inspect, events.append)
    assert not result['ok']
    assert not any(e.startswith('go:') for e in effects)
    assert 'close:host' in effects and 'close:follower' in effects


def test_retry_does_not_trust_old_disabled_proof_when_motor_reenabled(rig, monkeypatch):
    monkeypatch.setattr(stop, 'stop_service', Mock(side_effect=RuntimeError('interrupted')))
    assert not run(rig)['ok']
    rig[5].clear()
    rig[3]['host']['motors']['can0/1'] = [1, 1, 1]
    with pytest.raises(RuntimeError): run(rig)
    assert rig[5] == ['seal']


def test_perform_stop_routes_real_entrypoint_to_gateway_fallback(rig, monkeypatch):
    monkeypatch.setattr(stop, 'resume_checkpoint', lambda *a, **k: None)
    c, _, _, _, events, _, inspect_fn = rig
    assert stop.perform_stop(c, inspect_fn, events.append)['ok']


def test_one_unavailable_disable_service_sends_no_commands(rig, monkeypatch):
    device = stop.DeviceProcess
    class Broken(device):
        def arm(self):
            if self.side == 'follower': raise RuntimeError('no ROS service')
            super().arm()
    monkeypatch.setattr(stop, 'DeviceProcess', Broken)
    assert not run(rig)['ok']
    assert not any(e.startswith('go:') for e in rig[5])

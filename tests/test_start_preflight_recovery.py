"""No robot access: pre-start checks cannot turn an error into motion."""
import copy
import json
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import start_teleop_from_console as start
import preflight_motor_feedback as pre
import power_down_arms as stop
import inspect_teleop_recovery as recovery
from test_console_teleop_lifecycle import cold
from test_failed_start_cleanup import failed, interrupted_alignment
from test_stopped_leader_shutdown import fake_can, recovered_alignment


def abandoned():
    result = interrupted_alignment()
    result['evidence']['remote']['status']['collection'].update(
        left_align_active=False, left_master_align_active=False, right_master_align_active=False)
    return result


def good_reports():
    cfg = recovery.connection_config()
    return {side: {'ok': True, 'motors': {f'{bus}/{j}': [0, 0, 0]
            for bus in cfg[side+'_interfaces'] for j in range(1, 9)}} for side in ('host', 'follower')}


@pytest.mark.parametrize('field', ['estop', 'recording', 'returning', 'active_host', 'unknown_process',
                                  'missing_role', 'alignment', 'hold_task', 'no_identity', 'fault'])
def test_auto_stop_rejects_live_tasks_and_unknown_owners(field):
    r = abandoned(); e = r['evidence']; s = e['remote']['status']; c = s['collection']
    if field == 'estop': s.update(state='E_STOP', fault_bits=32)
    if field == 'recording': c['recording'] = 'episode'
    if field == 'returning': c['return_phase'] = 'moving'
    if field == 'active_host': e['host_unit']['ActiveState'] = 'active'
    if field == 'unknown_process': e['remote']['process_identities']['102']['argv'] = ['/other/controller']
    if field == 'missing_role': del e['remote']['process_identities']['104']
    if field == 'alignment': c['left_align_active'] = True
    if field == 'hold_task': c['left_mode'] = 'HOLD'
    if field == 'no_identity': e['host_unit']['InvocationID'] = ''
    if field == 'fault': s.update(state='FAULT', fault_bits=8)
    assert not start.powered_recovery_eligible(r, recovery.connection_config())


def test_cold_start_missing_power_never_starts_service(monkeypatch):
    run = Mock(); monkeypatch.setattr(start.subprocess, 'run', run)
    monkeypatch.setattr(start, 'emit', lambda _: None)
    monkeypatch.setattr(start, 'verify_motor_preflight', Mock(side_effect=RuntimeError('主臂电源无反馈')))
    with pytest.raises(RuntimeError, match='主臂电源'):
        start.start(recovery.connection_config(), inspect_fn=lambda _: cold())
    run.assert_not_called()


def test_identity_change_after_motor_queries_never_starts_service(monkeypatch):
    values = [cold(), cold(), cold()]; values[-1]['evidence']['remote']['boot_id'] = 'new'
    reads = iter(values); run = Mock()
    monkeypatch.setattr(start.subprocess, 'run', run)
    monkeypatch.setattr(start, 'emit', lambda _: None)
    monkeypatch.setattr(start, 'verify_motor_preflight', Mock(return_value=good_reports()))
    with pytest.raises(RuntimeError, match='查询期间'):
        start.start(recovery.connection_config(), inspect_fn=lambda _: next(reads))
    run.assert_not_called()


def test_abandoned_start_is_stopped_once_without_nested_ui_completion(monkeypatch):
    before = abandoned(); after = copy.deepcopy(before)
    after.update(code='OPERATOR_POWER_DOWN_CONFIRM', confirmation_required=True)
    assert start.powered_recovery_eligible(before, recovery.connection_config())
    reads = iter([before, before, after]); events = []
    monkeypatch.setattr(start, 'emit', events.append)
    monkeypatch.setattr(start, 'verify_motor_preflight', Mock(return_value=good_reports()))
    calls = []
    def recover(*args, **kwargs):
        calls.append('stop')
        kwargs['progress']({'event': 'complete', 'ok': True, 'message': '失能完成'})
        return {'ok': True}
    monkeypatch.setattr(stop, 'perform_stop', recover)
    assert start.prepare_start(recovery.connection_config(), before, lambda _: next(reads)) == after
    assert calls == ['stop']
    assert all(e['event'] == 'progress' for e in events)


@pytest.mark.parametrize('cause', ['missing_power', 'enabled_host', 'identity_change', 'stop_failure'])
def test_abandoned_start_does_not_bypass_failed_recovery(monkeypatch, cause):
    before = abandoned(); second = copy.deepcopy(before); third = copy.deepcopy(before)
    if cause == 'identity_change': third['evidence']['remote']['boot_id'] = 'new'
    reads = iter([second, third]); reports = good_reports()
    if cause == 'enabled_host': reports['host']['motors']['can0/1'] = [1, 1, 1]
    probe = Mock(return_value=reports)
    if cause == 'missing_power': probe.side_effect = RuntimeError('无反馈')
    monkeypatch.setattr(start, 'verify_motor_preflight', probe)
    monkeypatch.setattr(start, 'emit', lambda _: None)
    shutdown = Mock(return_value={'ok': False, 'message': '失能未确认'})
    monkeypatch.setattr(stop, 'perform_stop', shutdown)
    with pytest.raises(RuntimeError):
        start.prepare_start(recovery.connection_config(), before, lambda _: next(reads))
    assert shutdown.call_count == (1 if cause == 'stop_failure' else 0)


def test_down_interfaces_rechecked_after_repair_before_admission(monkeypatch):
    before = cold(); before['evidence']['remote']['can_ok'] = False
    probe = Mock(); monkeypatch.setattr(start, 'verify_motor_preflight', probe)
    monkeypatch.setattr(start, 'emit', lambda _: None)
    after = cold()
    assert start.prepare_start(recovery.connection_config(), before, lambda _: after) == after
    probe.assert_called_once_with(recovery.connection_config(), repair=True)


def test_early_follower_failure_can_record_disable_without_inventing_leader_session(tmp_path, monkeypatch):
    e = recovered_alignment(tmp_path, monkeypatch)
    e['remote']['status']['leader_session_id'] = None
    receipt = recovery.save_disable_checkpoint(e, e['power_down_receipt']['devices'])
    assert receipt['no_leader_session'] is True and receipt['leader_session_id'] is None
    e['power_down_receipt'] = receipt
    assert recovery.known_power_down_eligible(e)
    e['remote']['status'].update(state='FAULT', fault_bits=2, reason='controller reports CAN failure')
    assert recovery.complete_disable_checkpoint(receipt['shutdown_id'], e)['phase'] == 'complete'
    e['remote']['status']['leader_session_id'] = 99
    assert not recovery.known_power_down_eligible(e)


@pytest.mark.parametrize('change', ['no_method', 'not_hold', 'enabled', 'active_host'])
def test_no_session_receipt_cannot_be_created_without_narrow_proof(tmp_path, monkeypatch, change):
    e = recovered_alignment(tmp_path, monkeypatch); devices = e['power_down_receipt']['devices']
    e['remote']['status']['leader_session_id'] = None
    if change == 'no_method': devices['host'].pop('verification_method')
    if change == 'not_hold': e['remote']['status']['reason'] = 'restart requires alignment'
    if change == 'enabled': devices['host']['motors']['can0/1']['states'] = [1, 1, 1]
    if change == 'active_host': e['host_unit']['ActiveState'] = 'active'
    with pytest.raises(RuntimeError): recovery.save_disable_checkpoint(e, devices)


def mock_links(monkeypatch, *, down=False, wrong_adapter=False, active=False):
    real_path = Path
    class FakePath:
        def __init__(self, path): self.path = str(path)
        def __truediv__(self, value): return FakePath(self.path+'/'+str(value))
        def resolve(self): return FakePath('/usb/box/interface')
        @property
        def parent(self): return FakePath('/usb/box')
        def __str__(self): return self.path
        def read_text(self): return ('3562' if wrong_adapter else '0c72') if self.path.endswith('idVendor') else '0011'
        def glob(self, _): return [FakePath('/proc/1/cmdline')] if active else []
        def read_bytes(self): return b'/path/openarm_gravity_pd_node\0'
    monkeypatch.setattr(pre, 'Path', lambda p: FakePath(p) if str(p).startswith(('/sys/', '/proc')) else real_path(p))
    mutations = []
    def run(args, **kwargs):
        if args[0] == 'sudo':
            mutations.append(args); return type('R', (), {'returncode': 0})()
        bus = args[-1]
        return type('R', (), {'stdout': json.dumps([{'ifindex': int(bus[-1])+1,
            'flags': [] if down else ['UP'], 'linkinfo': {'info_data': {
                'state': 'STOPPED' if down else 'ERROR-ACTIVE', 'ctrlmode': ['FD'],
                'bittiming': {'bitrate': 1000000}, 'data_bittiming': {'bitrate': 5000000}}}}])})()
    monkeypatch.setattr(pre.subprocess, 'run', run)
    return mutations


def test_endpoint_queries_only_refresh_frames_and_requires_all_motors(monkeypatch):
    sent = fake_can(monkeypatch); mock_links(monkeypatch)
    report = pre.endpoint('host', ['can0', 'can1'])
    assert report['ok'] and len(sent) == 48 and len(report['motors']) == 16


@pytest.mark.parametrize('mode', ['missing', 'error', 'stale', 'wrong_motor'])
def test_endpoint_rejects_missing_or_bad_feedback(monkeypatch, mode):
    fake_can(monkeypatch, mode); mock_links(monkeypatch)
    assert pre.endpoint('host', ['can0', 'can1'])['ok'] is False


def test_enabled_feedback_is_visible_but_blocks_start_verification(monkeypatch):
    fake_can(monkeypatch, 'enabled'); mock_links(monkeypatch)
    report = pre.endpoint('host', ['can0', 'can1']); assert report['ok']
    reports = good_reports(); reports['host'] = report
    monkeypatch.setattr(pre, 'both_endpoints', lambda *a, **k: reports)
    with pytest.raises(RuntimeError, match='仍有电机使能'): pre.verify(recovery.connection_config())
    assert pre.verify(recovery.connection_config(), require_disabled=False) == reports


@pytest.mark.parametrize('bad', ['wrong_adapter', 'active'])
def test_repair_never_touches_other_devices_or_active_controller(monkeypatch, bad):
    mutations = mock_links(monkeypatch, down=True, **{bad: True})
    assert pre.endpoint('host', ['can0', 'can1'], repair=True)['ok'] is False
    assert mutations == []


def test_repair_only_uses_the_two_validated_arm_interfaces(monkeypatch):
    mutations = mock_links(monkeypatch, down=True)
    assert pre.endpoint('follower', ['can1', 'can2'], repair=True)['ok']
    assert [a[5] for a in mutations] == ['can1', 'can1', 'can2', 'can2']


def test_permission_failure_is_explicit_not_silent_success(monkeypatch):
    mock_links(monkeypatch, down=True)
    original = pre.subprocess.run
    monkeypatch.setattr(pre.subprocess, 'run', lambda args, **kw:
        type('R', (), {'returncode': 1})() if args[0] == 'sudo' else original(args, **kw))
    r = pre.endpoint('host', ['can0', 'can1'], repair=True)
    assert not r['ok'] and '权限' in r['error']


def rebooted():
    r = cold(); e = r['evidence']
    e.update(host_boot_id='host-boot', previous_boot_id='11111111-1111-1111-1111-111111111111',
             host_service_age_s=500, host_service_pid=99, local_control_processes=[100, 101])
    e['host_unit'].update(ActiveState='active', MainPID='99', ControlGroup='/old-teleop', InvocationID='old')
    e['remote'].update(boot_id='22222222-2222-2222-2222-222222222222', uptime_s=100, process_identities={})
    return r


@pytest.mark.parametrize('bad', ['same_boot', 'no_boot', 'younger_host', 'active_peer', 'socket',
                               'recording', 'network', 'missing_owner', 'different_pid'])
def test_peer_reboot_needs_positive_identity_and_empty_follower(bad):
    r = rebooted(); e = r['evidence']; peer = e['remote']
    assert start.rebooted_peer_recovery_eligible(r)
    if bad == 'same_boot': e['previous_boot_id'] = peer['boot_id']
    if bad == 'no_boot': e['previous_boot_id'] = None
    if bad == 'younger_host': e['host_service_age_s'] = 50
    if bad == 'active_peer': peer['processes'] = [100]
    if bad == 'socket': peer['runtime_socket_exists'] = True
    if bad == 'recording': peer['recorder_idle'] = False
    if bad == 'network': e['ssh_ok'] = False
    if bad == 'missing_owner': e['local_control_processes'] = []
    if bad == 'different_pid': e['host_service_pid'] = 101
    assert not start.rebooted_peer_recovery_eligible(r)


@pytest.mark.parametrize('failure', [None, 'follower_enabled', 'partial_host_proof', 'query_failure', 'changed_peer'])
def test_peer_reboot_disables_only_leader_and_rechecks_before_stopping(monkeypatch, failure):
    before = rebooted(); after = cold()
    after['evidence']['host_boot_id'] = before['evidence']['host_boot_id']
    after['evidence']['remote']['boot_id'] = before['evidence']['remote']['boot_id']
    latest = copy.deepcopy(before)
    if failure == 'changed_peer': latest['evidence']['remote']['boot_id'] = '33333333-3333-3333-3333-333333333333'
    reads = iter([before, latest, before, before, after])
    reports = good_reports()
    if failure == 'follower_enabled': reports['follower']['motors']['can1/1'] = [1, 1, 1]
    probe = Mock(side_effect=[reports, RuntimeError('反馈丢失') if failure == 'query_failure' else good_reports()])
    monkeypatch.setattr(start, 'verify_motor_preflight', probe)
    monkeypatch.setattr(start, 'emit', lambda _: None)
    calls = []
    class Device:
        def __init__(self, side, *_): calls.append(side)
        def arm(self): calls.append('arm')
        def go(self): calls.append('go')
        def result(self):
            proof = {'ok': True, 'verified': True, 'acknowledged': True,
                     'motors': {k: {'states': v, 'disabled': True} for k, v in reports['host']['motors'].items()}}
            if failure == 'partial_host_proof': proof['motors'].pop('can0/1')
            return proof
        def close(self): calls.append('close')
    monkeypatch.setattr(stop, 'DeviceProcess', Device)
    stopped = Mock(); monkeypatch.setattr(stop, 'stop_service', stopped)
    if failure:
        with pytest.raises(RuntimeError):
            start.recover_rebooted_peer(recovery.connection_config(), before, lambda _: next(reads))
        stopped.assert_not_called()
        if failure in ('follower_enabled', 'changed_peer'): assert 'go' not in calls
    else:
        assert start.recover_rebooted_peer(recovery.connection_config(), before, lambda _: next(reads)) == after
        assert calls == ['host', 'arm', 'go', 'close']
        stopped.assert_called_once_with(recovery.connection_config()['service'])


def test_terminal_entry_checks_motors_before_touching_old_controllers():
    source = (Path(__file__).parents[1]/'scripts/run_bimanual_remote_feedback.sh').read_text()
    assert source.index('scripts/preflight_motor_feedback.py') < source.index('trap cleanup INT TERM EXIT')
    assert source.index('scripts/preflight_motor_feedback.py') < source.index('show_stage 1')


@pytest.mark.parametrize('case', ['silent', 'partial_zero', 'unacknowledged', 'listener_error',
    'passive_enabled', 'passive_fault', 'missing_key', 'query_missing', 'query_enabled',
    'query_fault', 'query_boolean', 'query_stale', 'identity_changed', 'recording_started'])
def test_reboot_repeat_disable_uses_fresh_queries_without_weakening_stop_gate(monkeypatch, case):
    before = rebooted(); after = cold()
    after['evidence']['host_boot_id'] = before['evidence']['host_boot_id']
    after['evidence']['remote']['boot_id'] = before['evidence']['remote']['boot_id']
    final = copy.deepcopy(before)
    if case == 'identity_changed': final['evidence']['host_unit']['InvocationID'] = 'replacement'
    if case == 'recording_started': final['evidence']['remote']['recorder_idle'] = False
    reads = iter([before, before, before, final, after])
    proof = {'ok': False, 'verified': False, 'acknowledged': True,
        'error': '缺少连续失能反馈：can0/1',
        'motors': {k: {'states': [], 'disabled': False} for k in good_reports()['host']['motors']}}
    if case == 'partial_zero': proof['motors']['can0/1']['states'] = [0, 0]
    if case == 'unacknowledged': proof['acknowledged'] = False
    if case == 'listener_error': proof['error'] = 'CAN listener failed'
    if case == 'passive_enabled': proof['motors']['can0/1']['states'] = [1]
    if case == 'passive_fault': proof['motors']['can0/1']['states'] = [8]
    if case == 'missing_key': del proof['motors']['can0/1']
    fresh = good_reports()
    if case == 'query_missing': del fresh['host']['motors']['can0/1']
    if case == 'query_enabled': fresh['host']['motors']['can0/1'] = [1, 1, 1]
    if case == 'query_fault': fresh['host']['motors']['can0/1'] = [8, 8, 8]
    if case == 'query_boolean': fresh['host']['motors']['can0/1'] = [False, False, False]
    if case == 'query_stale': fresh['host'].update(ok=False, error='无本次新反馈')
    endpoint_results = iter([good_reports(), fresh])
    monkeypatch.setattr(pre, 'both_endpoints', lambda *a, **kw: next(endpoint_results))
    monkeypatch.setattr(start, 'verify_motor_preflight', pre.verify)
    events = []; monkeypatch.setattr(start, 'emit', events.append)
    actions = []
    class Device:
        def __init__(self, side, *_): assert side == 'host'
        def arm(self): actions.append('arm')
        def go(self): actions.append('disable')
        def result(self): return proof
        def close(self): actions.append('close')
    monkeypatch.setattr(stop, 'DeviceProcess', Device)
    stopped = Mock(); monkeypatch.setattr(stop, 'stop_service', stopped)
    if case in ('silent', 'partial_zero'):
        assert start.recover_rebooted_peer(recovery.connection_config(), before, lambda _: next(reads)) == after
        stopped.assert_called_once()
        assert any('只读复核' in e['message'] for e in events)
    else:
        with pytest.raises(RuntimeError):
            start.recover_rebooted_peer(recovery.connection_config(), before, lambda _: next(reads))
        stopped.assert_not_called()
    assert actions == ['arm', 'disable', 'close']

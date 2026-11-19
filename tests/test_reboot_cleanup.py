"""Hardware-free regression for a rebooted peer and silent, stale leader."""
import copy
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import cleanup_rebooted_teleop as cleanup
import inspect_teleop_recovery as recovery
from start_teleop_from_console import start_decision


def stale():
    motors = {'result': 'UNKNOWN'}
    for side, buses in recovery.buses_by_side():
        motors[side] = {'result': 'UNKNOWN', 'motors': {
            f'{bus}/{motor}': dict(states=[], samples=0, fresh=False, state=None)
            for bus in buses for motor in range(1, 9)}}
    e = dict(ssh_ok=True, host_can_ok=True, host_motors='UNKNOWN', host_boot_id='host-boot',
        previous_boot_id='11111111-1111-1111-1111-111111111111', host_service_age_s=500,
        local_control_processes=[102], motor_report=motors,
        host_unit=dict(LoadState='loaded', ActiveState='active', MainPID='101',
                       ControlGroup='/user/teleop.service', InvocationID='old'),
        remote=dict(boot_id='22222222-2222-2222-2222-222222222222', uptime_s=100,
                    processes=[], process_identities={}, runtime_socket_exists=False, status=None,
                    can_ok=True, recorder_idle=True))
    return dict(recovery.classify(e), evidence=e)


def stopped(before):
    after = copy.deepcopy(before)
    after['evidence']['host_unit'].update(ActiveState='inactive', MainPID='0', ControlGroup='')
    after['evidence']['local_control_processes'] = []
    return after


def test_silent_reboot_offers_cleanup_but_never_restart():
    value = stale()
    assert value['code'] == 'PEER_REBOOT_POWER_OFF_REQUIRED'
    assert value['cleanup_available'] is True
    assert value['confirmation_required'] is False
    assert value['automatic_restart_allowed'] is False
    with pytest.raises(RuntimeError):
        start_decision(value)


@pytest.mark.parametrize('location,key,value', [
    ('host', 'ssh_ok', False), ('host', 'host_can_ok', False),
    ('host', 'previous_boot_id', 'bad'),
    ('host', 'previous_boot_id', '22222222-2222-2222-2222-222222222222'),
    ('host', 'host_service_age_s', 50), ('host', 'host_service_age_s', float('nan')),
    ('remote', 'can_ok', False), ('remote', 'recorder_idle', False),
    ('remote', 'processes', [33]), ('remote', 'process_identities', {'33': {}}),
    ('remote', 'runtime_socket_exists', True), ('remote', 'status', {'state': 'FAULT'}),
    ('remote', 'boot_id', 'bad'), ('remote', 'uptime_s', float('inf')),
    ('unit', 'ActiveState', 'inactive'), ('unit', 'MainPID', '0'),
    ('unit', 'InvocationID', ''), ('unit', 'ControlGroup', ''),
])
def test_unsafe_or_unproven_states_do_not_offer_cleanup(location, key, value):
    e = stale()['evidence']
    target = e if location == 'host' else e['remote' if location == 'remote' else 'host_unit']
    target[key] = value
    assert not recovery.peer_reboot_cleanup_available(e)


@pytest.mark.parametrize('key,value', [('states', [1]), ('states', [8]), ('states', [0]),
    ('samples', 1), ('samples', False), ('fresh', True), ('state', 0)])
def test_any_motor_response_blocks_power_off_cleanup(key, value):
    for side, buses in recovery.buses_by_side():
        e = stale()['evidence']
        e['motor_report'][side]['motors'][f'{buses[-1]}/8'][key] = value
        assert not recovery.peer_reboot_cleanup_available(e)


def test_missing_motor_probe_or_error_blocks_cleanup():
    for side, _ in recovery.buses_by_side():
        for mode in ('error', 'missing'):
            e = stale()['evidence']; p = e['motor_report'][side]
            if mode == 'error': p['error'] = 'CAN socket failed'
            else: p['motors'].pop(next(iter(p['motors'])))
            assert not recovery.peer_reboot_cleanup_available(e)


def test_confirmation_required_before_read_or_mutation():
    with patch.object(cleanup.subprocess, 'run') as run:
        with pytest.raises(RuntimeError, match='电源'):
            cleanup.cleanup(recovery.connection_config(), inspect_fn=lambda _: pytest.fail('unexpected probe'))
    run.assert_not_called()


def run_cleanup(reads, *, recorder=None, unit=None, stop_error=None):
    reads = iter(reads)
    with patch.object(cleanup, 'emit'), patch.object(cleanup, 'RecorderClient') as client, \
         patch.object(cleanup, 'unit_status', return_value=unit or stale()['evidence']['host_unit']), \
         patch.object(cleanup.subprocess, 'run', side_effect=stop_error) as run:
        client.return_value.return_value = recorder or dict(ok=True, running=False, phase='idle', active_episode=None)
        result = cleanup.cleanup(recovery.connection_config(), power_off_confirmed=True,
                                 inspect_fn=lambda _: next(reads))
        return result, run.call_args_list


def test_confirmed_cleanup_only_stops_service_never_starts_or_disables():
    before = stale()
    result, calls = run_cleanup([before, copy.deepcopy(before), stopped(before)])
    assert result['ok'] and result['operation'] == 'cleanup'
    assert [c.args[0] for c in calls] == [['systemctl', '--user', 'stop', 'openarm-remote-teleop.service']]
    assert start_decision(stopped(before)) == 'cold'


@pytest.mark.parametrize('change', ['boot', 'pid', 'invocation', 'recording', 'feedback'])
def test_changed_observation_aborts_before_stop(change):
    first = stale(); second = copy.deepcopy(first); e = second['evidence']
    if change == 'boot': e['remote']['boot_id'] = '33333333-3333-3333-3333-333333333333'
    if change == 'pid': e['local_control_processes'] = [103]
    if change == 'invocation': e['host_unit']['InvocationID'] = 'new'
    if change == 'recording': e['remote']['recorder_idle'] = False
    if change == 'feedback': e['motor_report']['host']['motors']['can0/1']['states'] = [1]
    with patch.object(cleanup, 'emit'), patch.object(cleanup.subprocess, 'run') as run:
        reads = iter([first, second])
        with pytest.raises(RuntimeError, match='变化|改变'):
            cleanup.cleanup(recovery.connection_config(), power_off_confirmed=True,
                            inspect_fn=lambda _: next(reads))
    run.assert_not_called()


def test_late_recorder_or_service_change_blocks_stop():
    for recorder, unit in [
        (dict(ok=True, running=True, phase='recording', active_episode={}), None),
        (None, dict(InvocationID='new', MainPID='101', ActiveState='active'))]:
        with pytest.raises(RuntimeError):
            run_cleanup([stale(), stale()], recorder=recorder, unit=unit,
                        stop_error=AssertionError('unexpected stop'))


def test_post_stop_incomplete_or_peer_reboot_is_not_success():
    for change in ('live', 'boot'):
        first = stale(); after = stopped(first)
        if change == 'live': after['evidence']['local_control_processes'] = [102]
        else: after['evidence']['remote']['boot_id'] = '33333333-3333-3333-3333-333333333333'
        with pytest.raises(RuntimeError, match='未完全确认'):
            run_cleanup([first, copy.deepcopy(first), after])


def test_stop_failure_never_starts_another_service():
    with pytest.raises(subprocess.TimeoutExpired):
        run_cleanup([stale(), stale()], stop_error=subprocess.TimeoutExpired('stop', 25))


@pytest.mark.parametrize('flags', [[], ['--confirm'], ['--confirm-power-off', '--jetson', 'wrong']])
def test_cli_requires_specific_power_off_confirmation_and_matching_target(flags):
    command = [sys.executable, str(Path(cleanup.__file__)), '--jetson', 'openarm-jetson',
               '--recorder', 'tcp://192.168.50.2:5557', *flags]
    result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'progress' not in result.stdout

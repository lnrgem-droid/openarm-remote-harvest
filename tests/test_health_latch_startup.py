"""Offline startup recovery tests; all hardware and service boundaries are mocked."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import inspect_teleop_recovery as recovery


def enabled_motors():
    report = {'result': 'ENABLED'}
    for role, buses in (('host', ('can0', 'can1')), ('follower', ('can1', 'can2'))):
        report[role] = {'result': 'ENABLED', 'motors': {
            f'{bus}/{motor}': dict(state=1, states=[1], fresh=True, samples=50)
            for bus in buses for motor in range(1, 9)}}
    return report


def running_status():
    return dict(state='RUNNING', fault_bits=0, leader_session_id=45,
        action_age_ms=1., feedback_age_ms=2., feedback_fresh_for_control=True,
        relative_follow_reference_captured=True, enabled_arms=['left', 'right'],
        max_tracking_error_rad=.01, left_max_tracking_error_rad=.01,
        collection=dict(left_mode='FOLLOW', right_mode='FOLLOW', recording=None,
                        return_phase='idle', transitioning_arms=[]))


def latched_evidence():
    status = running_status()
    motors = enabled_motors()
    return dict(ssh_ok=True, host_can_ok=True, host_motors='ENABLED',
        host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        host_service_pid=123, host_service_age_s=500,
        host_unit=dict(ActiveState='active', MainPID='123', InvocationID='service-1',
                       ControlGroup='/mock-teleop'),
        remote=dict(boot_id='22222222-2222-2222-2222-222222222222',
            uptime_s=100, can_ok=True, recorder_idle=True,
            processes=[234], process_identities={'234': {'start_ticks': 1000,
                'argv': ['mock-follower']}}, runtime_socket_exists=True, status=status),
        motor_report=motors, health_report_fresh=True,
        health_report=dict(schema_version=1,
            host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            probe_started_monotonic_ns=time.monotonic_ns(),
            status=copy.deepcopy(status), motors=copy.deepcopy(motors),
            invalidated_session=status['leader_session_id']))


def mutate(value, path, replacement):
    for key in path[:-1]:
        value = value[key]
    value[path[-1]] = replacement


def test_enabled_old_latch_offers_confirmation_without_automatic_restart():
    evidence = latched_evidence()
    assert recovery.health_latch_recovery_eligible(evidence)
    decision = recovery.classify(evidence)
    assert decision['code'] == 'HEALTH_LATCH_CONFIRM'
    assert decision['confirmation_required'] is True
    assert decision['automatic_restart_allowed'] is False


@pytest.mark.parametrize('path,value', [
    (('remote', 'status', 'collection', 'left_mode'), 'HOLD'),
    (('remote', 'status', 'collection', 'right_mode'), 'HOLD'),
    (('remote', 'status', 'collection', 'left_mode'), 'RETURNING'),
    (('remote', 'status', 'collection', 'right_mode'), 'RETURNING'),
    (('remote', 'status', 'collection', 'recording'), {'token': 'active'}),
    (('remote', 'status', 'collection', 'return_phase'), 'moving'),
    (('remote', 'status', 'collection', 'transitioning_arms'), ['left']),
    (('remote', 'status', 'state'), 'FAULT'),
    (('remote', 'status', 'state'), 'E_STOP'),
    (('remote', 'status', 'fault_bits'), 8),
    (('remote', 'status', 'feedback_fresh_for_control'), False),
    (('remote', 'status', 'action_age_ms'), 100),
    (('remote', 'status', 'feedback_age_ms'), 100),
    (('remote', 'status', 'relative_follow_reference_captured'), False),
    (('remote', 'recorder_idle'), False),
    (('remote', 'can_ok'), False),
    (('host_can_ok',), False),
    (('ssh_ok',), False),
    (('health_report_fresh',), False),
    (('health_report', 'invalidated_session'), None),
    (('health_report', 'invalidated_session'), 44),
    (('health_report', 'status', 'leader_session_id'), 44),
    (('host_unit', 'ActiveState'), 'inactive'),
    (('host_unit', 'MainPID'), '0'),
    (('motor_report',), {'result': 'ENABLED'}),
    (('motor_report', 'follower', 'motors', 'can2/8', 'fresh'), False),
    (('motor_report', 'host', 'motors', 'can0/1', 'states'), [0, 1]),
])
def test_current_unsafe_or_uncertain_evidence_never_offers_latch_recovery(path, value):
    evidence = latched_evidence()
    mutate(evidence, path, value)
    assert not recovery.health_latch_recovery_eligible(evidence)
    assert recovery.classify(evidence)['confirmation_required'] is False


@pytest.mark.parametrize('state', [0, 8, 12])
def test_disabled_or_faulted_physical_motor_overrides_enabled_summary(state):
    evidence = latched_evidence()
    evidence['motor_report']['host']['motors']['can1/8'].update(state=state, states=[state])
    assert not recovery.health_latch_recovery_eligible(evidence)
    assert not recovery.classify(evidence)['confirmation_required']


@pytest.mark.parametrize('phase', ['idle', 'error'])
def test_recorder_idle_includes_finished_error_without_active_work(phase):
    assert recovery.recorder_is_idle(dict(ok=True, running=False,
                                         phase=phase, active_episode=None))


@pytest.mark.parametrize('reply', [
    {}, None, [],
    dict(ok=False, running=False, phase='idle', active_episode=None),
    dict(ok=True, running=True, phase='error', active_episode=None),
    dict(ok=True, running=False, phase='error', active_episode={'token': 'active'}),
    dict(ok=True, running=False, phase='idle'),
    dict(ok=True, phase='idle', active_episode=None),
    dict(ok=True, running=False, active_episode=None),
    dict(ok=True, running=False, phase='finalizing', active_episode=None),
    dict(ok=True, running=False, phase='recording', active_episode=None),
    dict(ok=True, running=False, phase='unknown', active_episode=None),
])
def test_recorder_active_or_incomplete_status_is_not_idle(reply):
    assert not recovery.recorder_is_idle(reply)


def shell_fixture(tmp_path, before=None, after=None):
    """Run the real launcher decision and confirmation code, without I/O to robots."""
    before = latched_evidence() if before is None else before
    after = copy.deepcopy(before) if after is None else after
    for name, evidence in (('first.json', before), ('second.json', after)):
        (tmp_path / name).write_text(json.dumps(dict(recovery.classify(evidence), evidence=evidence)))
    (tmp_path / 'status.json').write_text(json.dumps(before['remote']['status']))
    (tmp_path / 'motors.json').write_text(json.dumps(before['motor_report']))
    (tmp_path / 'live-health.json').write_text(json.dumps(before['health_report']))
    (tmp_path / 'alignment-following.json').write_text(json.dumps(dict(
        accepted=True, phase='following', leader_session_id=45)))

    source = (ROOT / 'scripts/daily_start_teleop_rgbd.sh').read_text()
    function = source[source.index('recover_restarted_peer() {'):
                      source.index('\nensure_host_can()')]
    invocation = ('/usr/bin/python3 "$TELEOP_ROOT/scripts/inspect_teleop_recovery.py" '
                  '\\\n    --jetson "$JETSON_HOST" --output "$report"')
    assert function.count(invocation) == 2
    function = function.replace(invocation, 'probe_fixture "$report"')
    branch = source[source.index('\nensure_teleop_service\nstatus='):
                    source.index("\necho '  正在核实本会话两端")]
    invocation = '/usr/bin/python3 "$TELEOP_ROOT/scripts/check_motor_enable.py" --jetson "$JETSON_HOST"'
    assert branch.count(invocation) == 2
    branch = branch.replace(invocation, 'cat "$LOG_DIR/motors.json"')
    branch = branch.replace('</dev/tty', '</dev/stdin')
    setup = r'''
set -euo pipefail
TELEOP_ROOT="$1"; LOG_DIR="$2"; JETSON_HOST=mock; TELEOP_SERVICE=mock
export HOME="$LOG_DIR/home"
COUNT=0
probe_fixture() {
  COUNT=$((COUNT+1))
  printf '%s\n' "$COUNT" >"$LOG_DIR/probe-count"
  if [[ "$COUNT" == 1 ]]; then
    cp "$LOG_DIR/first.json" "$1"
  else
    cp "$LOG_DIR/second.json" "$1"
  fi
}
ensure_teleop_service() { :; }
say() { printf '%s\n' "$*"; }
teleop_status() { cat "$LOG_DIR/status.json"; }
status_is_safe_to_reuse() {
  /usr/bin/python3 "$TELEOP_ROOT/scripts/check_remote_running_status.py" \
    --max-tracking-error-rad 0.20 --health-report "$LOG_DIR/live-health.json" \
    --startup-report "$LOG_DIR/alignment-following.json" "${@:2}" <<<"$1"
}
refresh_healthy_running_status() { status_is_safe_to_reuse "$status"; }
status_is_healthy_running() {
  /usr/bin/python3 "$TELEOP_ROOT/scripts/check_remote_running_status.py" <<<"$1"
}
ssh() { echo UNEXPECTED_SSH >&2; return 99; }
sleep() { :; }
systemctl() {
  case "$*" in
    '--user is-active --quiet mock') return 0 ;;
    '--user restart mock')
      echo MOCK_RESTART
      echo restarted >>"$LOG_DIR/restarts"
      printf '启动完成：状态 RUNNING\n' >"$LOG_DIR/teleop.log" ;;
    '--user show mock --property=MainPID --value') echo 12345 ;;
    *) echo UNEXPECTED_SYSTEMCTL >&2; return 99 ;;
  esac
}
'''
    return setup + function + branch


def run_launcher(tmp_path, answer, before=None, after=None):
    result = subprocess.run(['bash', '-c', shell_fixture(tmp_path, before, after),
        'test', str(ROOT), str(tmp_path)], input=answer, text=True,
        capture_output=True, timeout=5)
    assert 'UNEXPECTED_SSH' not in result.stderr
    assert 'UNEXPECTED_SYSTEMCTL' not in result.stderr
    return result


@pytest.mark.parametrize('answer,expected', [('r\n', 0), ('恢复\n', 0), ('R\n', 0),
    ('取消\n', 2), ('\n', 2), ('', 2)])
def test_real_running_enabled_shell_requires_explicit_latch_confirmation(tmp_path, answer, expected):
    result = run_launcher(tmp_path, answer)
    assert result.returncode == expected, result.stdout + result.stderr
    assert '本次恢复将建立新会话' in result.stdout
    assert ('MOCK_RESTART' in result.stdout) is (expected == 0)
    assert (tmp_path / 'probe-count').read_text().strip() == ('2' if expected == 0 else '1')
    assert (tmp_path / 'restarts').exists() is (expected == 0)


@pytest.mark.parametrize('path,value', [
    (('remote', 'status', 'leader_session_id'), 46),
    (('host_service_pid',), 124),
    (('host_unit', 'InvocationID'), 'service-2'),
    (('remote', 'process_identities', '234', 'start_ticks'), 2000),
    (('remote', 'boot_id'), '33333333-3333-3333-3333-333333333333'),
    (('remote', 'status', 'collection', 'left_mode'), 'HOLD'),
    (('remote', 'status', 'collection', 'right_mode'), 'RETURNING'),
    (('remote', 'status', 'collection', 'recording'), {'token': 'active'}),
    (('remote', 'recorder_idle'), False),
    (('health_report_fresh',), False),
    (('motor_report', 'follower', 'motors', 'can2/8', 'fresh'), False),
])
def test_real_shell_rechecks_identity_and_conditions_after_confirmation(tmp_path, path, value):
    before = latched_evidence(); after = copy.deepcopy(before)
    mutate(after, path, value)
    # A genuinely new but internally consistent session must also be rejected.
    if path == ('remote', 'status', 'leader_session_id'):
        after['health_report']['status']['leader_session_id'] = value
        after['health_report']['invalidated_session'] = value
    result = run_launcher(tmp_path, 'r\n', before, after)
    assert result.returncode == 2, result.stdout + result.stderr
    assert '二次检查通过' not in result.stdout
    assert 'MOCK_RESTART' not in result.stdout
    assert not (tmp_path / 'restarts').exists()
    assert (tmp_path / 'probe-count').read_text().strip() == '2'


def test_ineligible_old_latch_does_not_fall_through_to_generic_restart(tmp_path):
    evidence = latched_evidence()
    evidence['remote']['status']['collection']['left_mode'] = 'HOLD'
    result = run_launcher(tmp_path, 'r\n', evidence)
    assert result.returncode == 2, result.stdout + result.stderr
    assert '等待你的确认' not in result.stdout
    assert 'MOCK_RESTART' not in result.stdout
    assert 'REINITIALIZE_REQUIRED' in result.stderr
    assert '启动对齐验收未通过' not in result.stderr


def test_cli_explains_actual_latch_without_misreporting_valid_alignment(tmp_path):
    evidence = latched_evidence()
    health = tmp_path / 'health.json'; health.write_text(json.dumps(evidence['health_report']))
    startup = tmp_path / 'startup.json'
    startup.write_text(json.dumps(dict(accepted=True, phase='following', leader_session_id=45)))
    command = [sys.executable, str(ROOT / 'scripts/check_remote_running_status.py'),
        '--health-report', str(health), '--startup-report', str(startup),
        '--max-tracking-error-rad', '0.20']
    for explain in (False, True):
        result = subprocess.run(command + (['--explain'] if explain else []),
            input=json.dumps(evidence['remote']['status']), text=True, capture_output=True, timeout=5)
        assert result.returncode == 1
        assert not result.stdout
        if explain:
            assert 'REINITIALIZE_REQUIRED' in result.stderr
            assert str(health) in result.stderr
            assert '启动对齐验收未通过' not in result.stderr
            assert '无法读取启动对齐验收' not in result.stderr
            assert '控制状态/会话/跟踪误差未通过' not in result.stderr
        else:
            assert not result.stderr

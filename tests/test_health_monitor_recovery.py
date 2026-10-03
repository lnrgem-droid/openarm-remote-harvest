"""Synthetic samples only: health recovery must never command robot hardware."""
import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from teleop_readiness import apply_report, evaluate, motor_health
from watch_teleop_link import HealthMonitor


def status(sid=123):
    return dict(state='RUNNING', fault_bits=0, leader_session_id=sid,
                action_age_ms=1., feedback_age_ms=2., feedback_fresh_for_control=True,
                relative_follow_reference_captured=True, enabled_arms=['left', 'right'],
                collection=dict(left_mode='FOLLOW', right_mode='FOLLOW', transitioning_arms=[]))


def motors():
    return dict(result='ENABLED', **{
        role: dict(result='ENABLED', motors={f'{bus}/{j}':
            dict(state=1, states=[1], fresh=True, samples=50)
            for bus in buses for j in range(1, 9)})
        for role, buses in [('host', ['can0', 'can1']), ('follower', ['can1', 'can2'])]})


def unknown():
    value = motors()
    value['follower'] = dict(result='UNKNOWN', error='synthetic passive acquisition failure')
    return value


def sample(monitor, s=None, m=None, now=1.):
    s = status() if s is None else s
    m = motors() if m is None else m
    result = monitor.sample(s, m, now=now)
    return dict(result, host_boot_id='boot', status=s, motors=m)


def ready_monitor():
    monitor = HealthMonitor()
    for _ in range(3): result = sample(monitor)
    assert result['teleop_ready']
    return monitor


def test_one_unknown_blocks_then_three_good_samples_recover_without_latch():
    monitor = ready_monitor()
    bad = sample(monitor, m=unknown(), now=10.)
    assert not bad['teleop_ready'] and bad['diagnostic'] == 'MOTOR_UNKNOWN'
    assert bad['invalidated_session'] is None
    for good in range(1, 4):
        result = sample(monitor, now=10. + good)
        assert result['recovery_good_samples'] == good
        assert result['teleop_ready'] is (good == 3)
        assert apply_report(status(), result)['teleop_ready'] is (good == 3)
        assert result['invalidated_session'] is None


def test_recovery_window_restarts_after_another_bad_sample():
    monitor = ready_monitor()
    sample(monitor, m=unknown())
    sample(monitor)
    sample(monitor)
    result = sample(monitor, m=unknown())
    assert result['recovery_good_samples'] == 0
    assert result['consecutive_failures'] == 1
    assert not sample(monitor)['teleop_ready']
    assert not sample(monitor)['teleop_ready']
    assert sample(monitor)['teleop_ready']


def test_three_unknowns_latch_first_complete_evidence():
    monitor = ready_monitor()
    first = unknown()
    for now in (10., 11., 12.): result = sample(monitor, m=first, now=now)
    assert result['invalidated_session'] == 123
    evidence = result['invalidation_evidence']
    assert evidence['diagnostic'] == 'MOTOR_UNKNOWN'
    assert evidence['observed_unix_s'] == 10.
    assert evidence['latched_unix_s'] == 12.
    assert evidence['status'] == status() and evidence['motors'] == first
    first['follower']['error'] = 'changed after sampling'
    for _ in range(5): result = sample(monitor)
    assert result['diagnostic'] == 'REINITIALIZE_REQUIRED'
    assert result['invalidation_evidence'] == evidence


def test_complete_ssh_status_loss_counts_against_last_confirmed_session():
    monitor = ready_monitor()
    m = dict(result='UNKNOWN', error='SSH timed out')
    for now in (10., 11., 12.): result = monitor.sample(None, m, now)
    assert result['invalidated_session'] == 123
    assert result['invalidation_evidence']['status'] is None
    assert result['invalidation_evidence']['motors']['error'] == 'SSH timed out'
    assert not sample(monitor)['teleop_ready']


@pytest.mark.parametrize('states,expected', [([0], 'MOTORS_DISABLED'),
    ([0, 1], 'MOTORS_DISABLED'), ([1, 12], 'MOTOR_FAULT')])
def test_real_drive_evidence_is_immediate_even_amid_unknown_motors(states, expected):
    monitor = ready_monitor()
    m = unknown()
    m['host']['motors']['can0/1'].update(state=None, states=states, samples=1, fresh=False)
    result = sample(monitor, m=m)
    assert motor_health(m)[0] == expected
    assert result['invalidated_session'] == 123
    assert result['invalidation_evidence']['diagnostic'] == expected
    assert not sample(monitor)['teleop_ready']


def test_pure_evaluator_does_not_treat_unknown_as_proven_drive_loss():
    result = evaluate(status(), unknown())
    assert not result['teleop_ready'] and result['invalidated_session'] is None


@pytest.mark.parametrize('state,bits', [('FAULT', 8), ('E_STOP', 0), ('RUNNING', 8)])
def test_explicit_control_fault_cannot_recover_in_same_session(state, bits):
    monitor = ready_monitor()
    s = status(); s.update(state=state, fault_bits=bits)
    bad = sample(monitor, s=s)
    assert bad['invalidated_session'] == 123
    assert bad['invalidation_evidence']['diagnostic'] == 'FAULT'
    for _ in range(4):
        result = sample(monitor)
        assert not result['teleop_ready']
        assert not apply_report(status(), result)['teleop_ready']


def test_restart_inherits_failure_count_and_first_evidence():
    monitor = ready_monitor()
    first = sample(monitor, m=unknown(), now=10.)
    second = sample(monitor, m=unknown(), now=11.)
    restored = HealthMonitor(second, boot_id='boot')
    result = sample(restored, m=unknown(), now=12.)
    assert result['invalidated_session'] == 123
    assert result['invalidation_evidence']['observed_unix_s'] == 10.
    assert result['invalidation_evidence']['motors'] == first['motors']


def test_restart_cannot_clear_legacy_latch_or_claim_a_known_historical_cause():
    previous = dict(host_boot_id='boot', invalidated_session=123, status=status())
    restored = HealthMonitor(previous, boot_id='boot')
    for _ in range(4): result = sample(restored)
    assert not result['teleop_ready'] and result['invalidated_session'] == 123
    assert result['invalidation_evidence']['diagnostic'] == 'HISTORY_UNKNOWN'
    assert result['invalidation_evidence']['observed_unix_s'] is None
    restarted = HealthMonitor(result, boot_id='boot')
    assert sample(restarted)['invalidation_evidence'] == result['invalidation_evidence']


def test_new_session_requires_its_own_window_and_retains_previous_latch():
    monitor = ready_monitor()
    m = motors(); m['host']['motors']['can0/1'].update(state=0, states=[0])
    old = sample(monitor, m=m)
    for count in range(1, 4):
        result = sample(monitor, s=status(124))
        assert result['teleop_ready'] is (count == 3)
        assert result['invalidated_session'] == 123
        assert result['invalidation_evidence'] == old['invalidation_evidence']


def test_failure_counts_do_not_cross_sessions():
    monitor = ready_monitor()
    sample(monitor, m=unknown()); sample(monitor, m=unknown())
    result = sample(monitor, s=status(124), m=unknown())
    assert result['consecutive_failures'] == 1 and result['invalidated_session'] is None


def test_new_latch_archives_old_evidence_and_old_session_still_cannot_pass():
    monitor = ready_monitor()
    m = motors(); m['host']['motors']['can0/1'].update(state=0, states=[0])
    old = sample(monitor, m=m)
    new = sample(monitor, s=status(124), m=m)
    assert new['invalidation_history'] == [old['invalidation_evidence']]
    result = sample(monitor)
    assert not result['teleop_ready']
    assert not apply_report(status(), result)['teleop_ready']


def test_restart_requires_three_fresh_good_samples_even_if_previous_was_ready():
    monitor = ready_monitor()
    restored = HealthMonitor(sample(monitor), boot_id='boot')
    for count in range(1, 4):
        assert sample(restored)['teleop_ready'] is (count == 3)


def test_ui_explains_first_latch_without_dumping_motor_evidence():
    monitor = ready_monitor()
    for _ in range(3): sample(monitor, m=unknown(), now=10.)
    result = apply_report(status(), sample(monitor))
    assert '首次记录：MOTOR_UNKNOWN' in result['readiness_reason']
    assert '1970-' in result['readiness_reason']
    assert 'follower:can1/1' not in result['readiness_reason']
    legacy = dict(status=status(), motors=motors(), invalidated_session=123)
    assert '历史首次原因未知' in apply_report(status(), legacy)['readiness_reason']


@pytest.mark.parametrize('field,value', [('recovery_pending', 'false'),
    ('recovery_good_samples', True), ('recovery_good_samples', 0),
    ('monitor_session_id', 999)])
def test_invalid_recovery_metadata_cannot_bypass_window(field, value):
    monitor = ready_monitor()
    result = sample(monitor)
    result[field] = value
    assert not apply_report(status(), result)['teleop_ready']

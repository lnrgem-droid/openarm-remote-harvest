"""Synthetic startup tests: no ROS node, CAN traffic or motor enable."""
import copy
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import left_only_session as session
from left_only_alignment_fixture import status, POLICY, Clock, alignment


def hanging(phase='aligning'):
    s = status(phase)
    s.update(mode='left_only', enabled_arms=['left'], motor_health={'healthy': True})
    pose = [0.1, -0.9, 0.1, 1.5, 0.2, -0.4, 0.1]
    for key in ('leader_left_rad', 'left_actual_rad', 'left_target_rad'):
        s[key] = pose[:]
    s['collection']['right_mode'] = 'DISABLED'
    return s


def check(s):
    clock = Clock()
    return alignment.verify(lambda: s, 'aligning', POLICY, clock.time, clock.sleep,
                            arms=('left',), measured_start=True)


def test_matching_hanging_pose_passes_without_fixed_home():
    s = hanging()
    assert check(s)['accepted']
    assert not alignment.assess(s, 'aligning', POLICY, ('left',))['ok']


@pytest.mark.parametrize('failure', ['mismatch', 'stale', 'fault', 'unhealthy', 'nan'])
def test_current_pose_does_not_bypass_protection(failure):
    s = hanging()
    if failure == 'mismatch': s['left_actual_rad'][6] += .12
    if failure == 'stale': s['action_age_ms'] = 101
    if failure == 'fault': s.update(state='FAULT', fault_bits=1)
    if failure == 'unhealthy': s['motor_health']['healthy'] = False
    if failure == 'nan': s['left_actual_rad'][0] = float('nan')
    assert not check(s)['accepted']


def test_measured_start_cannot_relax_other_arm_mode():
    with pytest.raises(ValueError):
        alignment.assess(status(), 'aligning', POLICY, measured_start=True)


def test_following_still_requires_absolute_targets():
    s = hanging('following')
    assert alignment.assess(s, 'following', POLICY, ('left',), True)['ok']
    s['left_target_rad'][3] += .03
    assert not alignment.assess(s, 'following', POLICY, ('left',), True)['ok']


def test_moving_hanging_pose_is_not_accepted():
    clock = Clock()
    def moving():
        s = hanging()
        for key in ('leader_left_rad', 'left_actual_rad'):
            s[key][0] += .03 if int(clock.now*10) % 2 else 0.
        return s
    assert not alignment.verify(moving, 'aligning', POLICY, clock.time, clock.sleep,
                                arms=('left',), measured_start=True)['accepted']


def test_waits_for_both_common_home_holds_not_measured_hold_logs(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(session, 'time', SimpleNamespace(monotonic=clock.time, sleep=clock.sleep))
    monkeypatch.setattr(session, 'control', lambda *a: hanging())
    def log(c, role):
        if role == 'follower' and clock.now < 1.:
            return 'Startup measured-pose hold ready; fixed homing skipped.'
        return 'Startup homing reached upstream initial pose'
    monkeypatch.setattr(session, 'home_log', log)
    session.await_startup_hold({})
    assert clock.now >= 1.


def test_holds_are_released_only_after_checked_run(monkeypatch):
    monkeypatch.setattr(session,'verify_versions',lambda c: None)
    monkeypatch.setattr(session,'prepare_start',lambda c, supported=False: None)
    calls = []
    monkeypatch.setattr(session, 'preflight', lambda c: calls.append('preflight'))
    monkeypatch.setattr(session, 'remote', lambda *a: '')
    monkeypatch.setattr(session, 'run', lambda *a: '')
    monkeypatch.setattr(session, 'await_startup_hold', lambda c: calls.append('hold_ready'))
    monkeypatch.setattr(session, 'verify_alignment', lambda c, phase: calls.append(phase))
    def control(c, cmd):
        calls.append(cmd)
        return {'state': 'READY' if cmd == 'align' else 'RUNNING'}
    monkeypatch.setattr(session, 'control', control)
    monkeypatch.setattr(session, 'left_status', lambda *a, **kw: {})
    monkeypatch.setattr(session, 'ros_call', lambda c, role, *a: calls.append('release_'+role))
    session.start({})
    assert calls == ['preflight','hold_ready','aligning','align','run',
                     'release_leader','release_follower','following','status']

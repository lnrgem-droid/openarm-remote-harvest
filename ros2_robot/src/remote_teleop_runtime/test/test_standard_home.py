"""Bilateral zero return trajectories and failure gates; no hardware access."""
import json
from pathlib import Path

import pytest
from remote_teleop_runtime.collection_motion import CollectionMotion, LIMITS, LEFT_LIMITS
from remote_teleop_protocol.protocol import COLLECTION_ACK_LEFT_CAPABLE

Q = [.1, 0., 0., .5, -.1, 0., .1, -.4]*2
FREE = COLLECTION_ACK_LEFT_CAPABLE


def command(m, side, now=0., healthy=True, **kw):
    m.command(side, kw, Q, Q, Q, [0.]*16, now, healthy, leader_ack=FREE)


def ack(m):
    return FREE | (4 if m.left_master_target is not None else 0) | (1 if m.leader_target is not None else 0)


@pytest.mark.parametrize('side,offset', [('left', 0), ('right', 8)])
def test_return_is_smooth_independent_zero_and_preserves_grippers(tmp_path, side, offset):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = list(Q), list(Q)
    command(m, side+'_home')
    max_speed = 0.
    for step in range(1, 1401):
        out = m.update(actual, leader, step*.01, True, leader, ack(m))
        max_speed = max(max_speed, max(abs(a-b)/.01 for a, b in zip(out, actual)))
        target = m.leader_servo_target
        if target is not None:
            leader[offset:offset+8] = target
        actual = out
    assert m.standard_home.phase == 'completed'
    assert max_speed <= .10001
    assert actual[offset:offset+7] == [0.]*7
    assert leader[offset:offset+7] == [0.]*7
    assert actual[7] == actual[15] == leader[7] == leader[15] == -.4
    other = 8-offset
    assert actual[other:other+8] == Q[other:other+8]
    assert getattr(m, side) is None
    assert m.leader_servo_target is None
    assert m.saved is None and not m.path.exists()
    leader[offset] = .02
    out = m.update(actual, leader, 14.01, True, leader, FREE)
    assert out[offset] > actual[offset]  # Teleoperation really resumes.
    assert out[other:other+8] == Q[other:other+8]


@pytest.mark.parametrize('side,offset', [('left', 0), ('right', 8)])
@pytest.mark.parametrize('fault', ['stop', 'master_drift', 'gripper', 'nan', 'disconnect', 'ack'])
def test_release_never_resumes_when_stopped_or_unaligned(tmp_path, side, offset, fault):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = list(Q), list(Q)
    command(m, side+'_home')
    for step in range(1, 1401):
        actual = m.update(actual, leader, step*.01, True, leader, ack(m))
        target = m.leader_servo_target
        if target is not None:
            leader[offset:offset+8] = target
        if m.standard_home.phase == 'releasing':
            break
    assert m.standard_home.phase == 'releasing'
    held = getattr(m, side)
    if fault == 'stop':
        command(m, side+'_home_stop', now=step*.01)
    if fault == 'master_drift': leader[offset] += .25
    if fault == 'gripper': leader[offset+7] += .15
    if fault == 'nan': actual[offset] = float('nan')
    for i in range(1, 230):
        m.update(actual, leader, (step+i)*.01, fault != 'disconnect', leader,
                 FREE | (4 if side == 'left' else 1) if fault == 'ack' else FREE)
    assert getattr(m, side) == held
    assert m.transition[side] is None
    assert m.standard_home.phase == ('paused' if fault == 'stop' else
                                    'failed' if fault in {'disconnect', 'ack'} else 'alignment_required')


@pytest.mark.parametrize('side', ['left', 'right'])
def test_ack_missing_never_moves_and_stop_needs_release_ack(tmp_path, side):
    m = CollectionMotion(tmp_path/'saved.json'); command(m, side+'_home')
    for i in range(1, 250):
        assert m.update(Q, Q, i*.01, True, Q, FREE) == Q
    assert m.standard_home.phase == 'failed'
    command(m, side+'_home_stop', now=3.)
    assert m.leader_servo_target is None
    m.update(Q, Q, 3.01, True, Q, FREE)
    assert m.standard_home.busy
    for i in range(2, 20):
        m.update(Q, Q, 3.+i*.01, True, Q, FREE)
    assert m.standard_home.phase == 'paused'


@pytest.mark.parametrize('fault', ['disconnect', 'tracking', 'ack', 'nan'])
def test_failure_freezes_last_command_and_never_restarts(tmp_path, fault):
    m = CollectionMotion(tmp_path/'saved.json'); command(m, 'left_home')
    actual, leader = list(Q), list(Q)
    for i in range(1, 30):
        actual = m.update(actual, Q, i*.01, True, leader, ack(m))
        leader[:8] = m.left_master_target
    held = m.left
    if fault == 'tracking': actual[0] += .3
    if fault == 'nan': actual[0] = float('nan')
    m.update(actual, Q, .3, fault != 'disconnect', leader, FREE if fault == 'ack' else ack(m))
    assert m.standard_home.phase == 'failed'
    assert m.left == held
    for i in range(31, 60):
        m.update(Q, Q, i*.01, True, Q, ack(m))
    assert m.left == held and m.standard_home.phase == 'failed'


def test_recording_other_motion_and_old_peer_block_start(tmp_path):
    m = CollectionMotion(tmp_path/'saved.json')
    with pytest.raises(ValueError): command(m, 'left_home', healthy=False)
    m.recording = {'token': 'episode'}
    with pytest.raises(ValueError): command(m, 'left_home')
    m.recording = None
    with pytest.raises(ValueError):
        m.command('left_home', {}, Q, Q, Q, [0.]*16, 0., True, leader_ack=0)
    command(m, 'left_home')
    for name in ['right_home', 'left_home', 'right_return', 'left_master_align', 'collection_begin']:
        with pytest.raises(ValueError): command(m, name)


def test_startup_configs_and_acceptance_agree_on_zero():
    import yaml
    root = Path(__file__).resolve().parents[4]
    policy = json.loads((root/'scripts/startup_alignment_policy.json').read_text())
    assert policy['home_rad'] == [0.]*7
    for side in ['leader', 'follower']:
        p = root/f'ros2_robot/src/remote_teleop_runtime/config/bimanual_{side}.yaml'
        config = yaml.safe_load(p.read_text())['/**']['ros__parameters']
        assert config['startup_home_target'] == policy['home_rad']


@pytest.mark.parametrize('side,offset', [('left', 0), ('right', 8)])
@pytest.mark.parametrize('tick_s', [.01, .0245, .027])
def test_long_return_has_separate_settling_budget(tmp_path, side, offset, tick_s):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = list(Q), list(Q)
    actual[offset+3] = leader[offset+3] = 1.86
    m.command(side+'_home', {}, actual, actual, leader, [0.]*16, 0., True, leader_ack=FREE)
    settling_at = None
    for step in range(1, 10001):
        now = step*tick_s
        out = m.update(actual, leader, now, True, leader, ack(m))
        target = m.leader_servo_target
        if target is not None:
            leader[offset:offset+8] = target
        actual = out
        if m.standard_home.phase == 'settling':
            if settling_at is None: settling_at = now
            # Real joints take time to settle after the command reaches zero.
            actual[offset] = .06 if now-settling_at < 1. else .022
        if m.standard_home.phase in {'completed', 'failed', 'alignment_required'}:
            break
    assert m.standard_home.phase == 'completed', m.standard_home.status()
    assert settling_at is not None and now-settling_at >= 1.5
    assert getattr(m, side) is None and m.leader_servo_target is None


@pytest.mark.parametrize('case', ['never_settles', 'excessive_tick_delay'])
def test_separate_timeouts_still_hold_and_report_diagnostics(tmp_path, case):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = list(Q), list(Q)
    command(m, 'right_home')
    tick_s = .08 if case == 'excessive_tick_delay' else .01
    for step in range(1, 5001):
        actual = m.update(actual, leader, step*tick_s, True, leader, ack(m))
        if m.leader_servo_target is not None:
            leader[8:16] = m.leader_servo_target
        if m.standard_home.phase == 'settling': actual[8] = .06
        if m.standard_home.phase == 'failed': break
    assert m.standard_home.phase == 'failed'
    assert m.right is not None and m.transition['right'] is None
    detail = m.standard_home.status()
    if case == 'never_settles':
        assert 8. < detail['settling_elapsed_s'] < 8.02
        assert detail['follower_worst_axis'] == 'J1'
        assert detail['follower_goal_error_rad'] == .06
        assert '0.050' in detail['message']
    else:
        assert detail['progress'] < 1.
        assert '轨迹执行超时' in detail['message']


@pytest.mark.parametrize('side,offset,limits', [('left', 0, LEFT_LIMITS), ('right', 8, LIMITS)])
def test_zero_encoder_quantization_never_sends_out_of_limit_servo_target(tmp_path, side, offset, limits):
    m = CollectionMotion(tmp_path/'saved.json')
    actual = [0.]*16
    actual[7] = actual[15] = -.17
    leader = list(actual)
    leader[offset+3] = -.0001907377737087046  # Captured physical J4 at zero.
    m.command(side+'_home', {}, actual, leader, leader, [0.]*16, 0., True, leader_ack=FREE)
    for i in range(1, 500):
        target = m.leader_servo_target
        if target is not None:
            # Match the C++ controller: a single invalid axis rejects the packet.
            assert all(lo <= v <= hi for v, (lo, hi) in zip(target, limits))
            leader[offset:offset+8] = target
        actual = m.update(actual, leader, i*.01, True, leader, ack(m))
    assert m.standard_home.phase == 'completed'
    assert getattr(m, side) is None
    assert leader[offset+7] == -.17


def test_large_limit_violation_is_rejected_without_starting(tmp_path):
    m = CollectionMotion(tmp_path/'saved.json')
    leader = list(Q); leader[11] = -.02
    with pytest.raises(ValueError, match='限位'):
        m.command('right_home', {}, Q, Q, leader, [0.]*16, 0., True, leader_ack=FREE)
    assert m.standard_home.phase == 'idle' and m.leader_servo_target is None


def run_to_release(m, side, offset):
    actual, leader = list(Q), list(Q)
    command(m, side+'_home')
    for step in range(1, 1401):
        actual = m.update(actual, leader, step*.01, True, leader, ack(m))
        if m.leader_servo_target is not None:
            leader[offset:offset+8] = m.leader_servo_target
        if m.standard_home.phase == 'releasing':
            return actual, leader, step*.01
    raise AssertionError('release not reached')


@pytest.mark.parametrize('side,offset', [('left', 0), ('right', 8)])
def test_small_master_drift_after_release_hands_over_slowly(tmp_path, side, offset):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader, now = run_to_release(m, side, offset)
    leader[offset+6] = .11  # Physical report: wrist drift after servo release.
    phases = set()
    for i in range(1, 151):
        phase = m.standard_home.phase
        out = m.update(actual, leader, now+i*.01, True, leader, FREE)
        phases.add(m.standard_home.phase)
        speed = max(abs(a-b)/.01 for a, b in zip(actual, out))
        assert speed <= (.15001 if m.standard_home.phase == 'completed' else .10001)
        if phase == 'releasing': assert out == actual  # No motion before free ACK.
        assert out[7] == out[15] == -.4
        other = 8-offset
        assert out[other:other+8] == Q[other:other+8]
        actual = out
    assert 'handover' in phases and m.standard_home.phase == 'completed'
    assert getattr(m, side) is None and actual[offset+6] == .11


@pytest.mark.parametrize('fault', ['stop', 'disconnect', 'ack', 'nan', 'drift', 'tracking', 'gripper', 'timeout'])
def test_handover_interrupts_never_resume(tmp_path, fault):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader, now = run_to_release(m, 'right', 8)
    leader[14] = .15
    for i in range(1, 20):
        actual = m.update(actual, leader, now+i*.01, True, leader, FREE)
        if m.standard_home.phase == 'handover': break
    assert m.standard_home.phase == 'handover'
    now += i*.01
    for i in range(1, 20): actual = m.update(actual, leader, now+i*.01, True, leader, FREE)
    now += .19
    held = m.right
    if fault == 'stop': command(m, 'right_home_stop', now=now)
    if fault == 'nan': leader[14] = float('nan')
    if fault == 'drift': leader[14] = .25
    if fault == 'tracking': actual[8] = .3
    if fault == 'gripper': leader[15] += .15
    if fault == 'timeout': m.standard_home.started = now-4.01
    m.update(actual, leader, now+.01, fault != 'disconnect', leader, FREE|1 if fault == 'ack' else FREE)
    for i in range(2, 50):
        m.update(actual, actual, now+i*.01, True, actual, FREE)
    assert m.right == held and m.transition['right'] is None
    assert m.standard_home.phase in {'paused', 'failed', 'alignment_required'}


@pytest.mark.parametrize('side,offset', [('left', 0), ('right', 8)])
@pytest.mark.parametrize('follower_grip,master_grip', [(-.751, -.069), (-.069, -.751)])
def test_mismatched_grippers_align_master_without_moving_follower_gripper(tmp_path, side, offset, follower_grip, master_grip):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = [0.]*16, [0.]*16
    actual[offset+7], leader[offset+7] = follower_grip, master_grip
    m.command(side+'_home', {}, actual, actual, leader, [0.]*16, 0., True, leader_ack=FREE)
    peak = 0.
    for step in range(1, 1501):
        out = m.update(actual, leader, step*.01, True, leader, ack(m))
        target = m.left_master_target if side == 'left' else m.leader_servo_target
        if target is not None:
            peak = max(peak, abs(target[7]-leader[offset+7])/.01)
            leader[offset:offset+8] = target
        assert out[offset+7] == pytest.approx(follower_grip, abs=1e-12)
        actual = out
        if m.standard_home.phase == 'completed':break
    assert m.standard_home.phase == 'completed',m.standard_home.status()
    assert .149 < peak <= .15001
    assert leader[offset+7] == pytest.approx(follower_grip)
    assert getattr(m, side) is None


def test_gripper_must_settle_before_release(tmp_path):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = list(Q), list(Q)
    command(m, 'right_home')
    for i in range(1, 2001):
        actual = m.update(actual, leader, i*.01, True, leader, ack(m))
        if m.leader_servo_target is not None:
            leader[8:16] = m.leader_servo_target
            leader[15] += .08  # Within tracking guard, outside follow alignment.
        if m.standard_home.phase == 'failed':break
    assert m.standard_home.phase == 'failed'
    assert m.right is not None
    assert '夹爪误差' in m.standard_home.status()['message']


@pytest.mark.parametrize('side,offset', [('left', 0), ('right', 8)])
@pytest.mark.parametrize('error,expected', [(.03032730601968403, 'completed'), (.03528648813611035, 'completed'), (.05, 'completed'), (.0501, 'failed')])
def test_zero_arrival_accepts_measured_pd_residual_but_blocks_above_limit(tmp_path, side, offset, error, expected):
    m = CollectionMotion(tmp_path/'saved.json')
    actual, leader = [0.]*16, [0.]*16
    m.command(side+'_home', {}, actual, actual, leader, [0.]*16, 0., True, leader_ack=FREE)
    for i in range(1, 1501):
        actual = m.update(actual, leader, i*.01, True, leader, ack(m))
        if m.leader_servo_target is not None:
            leader[offset:offset+8] = m.leader_servo_target
            leader[offset] = error
        if m.standard_home.phase in ('completed', 'failed'):break
    assert m.standard_home.phase == expected, m.standard_home.status()
    assert (getattr(m, side) is None) == (expected == 'completed')

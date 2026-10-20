"""Dual-end hold coordination without hardware or motion."""
import pytest
from remote_teleop_runtime.collection_motion import CollectionMotion
from remote_teleop_protocol.protocol import decode_collection_ack

Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2
FREE, HELD, FAULT = 0x30, 0x34, 0x38


def command(m, name, now=1., ack=FREE, leader=None, **fields):
    m.command(name, fields, Q, Q, leader or Q, [0.]*16, now, True, leader_ack=ack)


def held(tmp_path):
    m = CollectionMotion(tmp_path / 'pose.json')
    command(m, 'right_save')
    command(m, 'left_lock_both')
    assert m.left_static_phase == 'preparing'
    m.update(Q, Q, 1.02, True, Q, HELD)
    assert m.status(Q, Q)['left_static_hold_confirmed']
    return m


def test_old_gateway_rejected_before_mutating_targets(tmp_path):
    m = CollectionMotion(tmp_path / 'pose.json')
    with pytest.raises(ValueError, match='不支持'):
        command(m, 'left_lock_both', ack=0x10)
    assert m.left is None and m.flags == 0


def test_hold_does_not_relatch_or_follow_displaced_master(tmp_path):
    m = held(tmp_path)
    moved = [v+.1 for v in Q]
    command(m, 'left_lock_both', ack=HELD, leader=moved)
    output = m.update(Q, moved, 1.03, True, moved, HELD)
    assert output[:8] == Q[:8]
    assert m.flags == 17
    assert decode_collection_ack(HELD) == (0, 1)


@pytest.mark.parametrize('ack,healthy', [(FAULT, True), (FREE, True), (HELD, False), (0x14, True)])
def test_fault_stays_failed_until_explicit_release(tmp_path, ack, healthy):
    m = held(tmp_path)
    m.update(Q, Q, 1.1, healthy, Q, ack)
    assert m.left_static_phase == 'failed' and m.flags == 17
    m.update(Q, Q, 1.2, True, Q, HELD)
    assert not m.status(Q, Q)['left_static_hold_confirmed']
    with pytest.raises(ValueError):
        command(m, 'collection_begin', ack=HELD, side='right', token='x')


def test_timeout_does_not_claim_success(tmp_path):
    m = CollectionMotion(tmp_path / 'pose.json')
    command(m, 'left_lock_both')
    m.update(Q, Q, 3.1, True, Q, FREE)
    assert m.left_static_phase == 'failed'


@pytest.mark.parametrize('name', ['left_follow', 'left_master_align', 'left_align_follow'])
def test_hold_cannot_implicitly_resume_or_align(tmp_path, name):
    m = held(tmp_path)
    with pytest.raises(ValueError, match='解除'):
        command(m, name, ack=HELD)
    assert m.flags == 17


def test_release_waits_for_continuous_physical_ack_and_keeps_follower(tmp_path):
    m = held(tmp_path)
    command(m, 'left_hold_release', ack=HELD, now=1.1)
    assert m.flags == 1 and m.left_static_phase == 'releasing'
    for now, ack in [(1.2, FREE), (1.25, HELD), (1.3, FREE), (1.35, FREE)]:
        m.update(Q, Q, now, True, Q, ack)
        assert m.left_static_phase == 'releasing'
    m.update(Q, Q, 1.41, True, Q, FREE)
    assert m.left_static_phase == 'idle' and m.left == tuple(Q[:8])
    assert m.status(Q, Q)['left_mode'] == 'HOLD'


def test_right_recording_allowed_only_after_hold_ack(tmp_path):
    m = held(tmp_path)
    command(m, 'collection_begin', ack=HELD, side='right', token='x')
    assert m.recording['side'] == 'right'
    with pytest.raises(ValueError, match='录制'):
        command(m, 'left_hold_release', ack=HELD)


def test_right_return_keeps_left_static_hold_through_completion(tmp_path):
    m = held(tmp_path)
    command(m, 'right_return', ack=HELD)
    for step in range(401):
        ack = HELD | (1 if m.leader_target is not None else 0)
        m.update(Q, Q, 1.1+step*.01, True, Q, ack)
        assert m.left_static_phase == 'held'
        assert m.flags & 16 and m.left == tuple(Q[:8])
    assert not m.returning and m.right is None


def test_release_blocked_during_right_servo(tmp_path):
    m = held(tmp_path)
    command(m, 'right_return', ack=HELD)
    with pytest.raises(ValueError, match='右主臂'):
        command(m, 'left_hold_release', ack=HELD)
    assert m.left_static_hold


def test_right_alignment_can_complete_while_left_stays_held(tmp_path):
    m = held(tmp_path)
    m.right = tuple(Q[8:])
    command(m, 'right_master_align', ack=HELD)
    for step in range(500):
        ack = HELD | (1 if m.leader_target is not None else 0)
        m.update(Q, Q, 1.1+step*.01, True, Q, ack)
    assert m.right_master_align_phase == 'completed'
    assert m.right is None and m.left_static_phase == 'held'


def test_failed_hold_during_return_can_release_right_then_left(tmp_path):
    m = held(tmp_path)
    command(m, 'right_return', ack=HELD)
    m.update(Q, Q, 1.1, True, Q, HELD)
    m.update(Q, Q, 1.2, True, Q, HELD | 1)
    m.update(Q, Q, 1.3, True, Q, FAULT | 2)
    assert not m.returning and m.left_static_phase == 'failed'
    command(m, 'right_pause', now=1.4, ack=FAULT | 2, release_servo=True)
    for now in (1.5, 1.61):
        m.update(Q, Q, now, True, Q, FAULT)
    assert not m.returning and not m.right_servo_release_pending
    command(m, 'left_hold_release', now=1.7, ack=FAULT)
    for now in (1.8, 1.91):
        m.update(Q, Q, now, True, Q, FREE)
    assert m.left_static_phase == 'idle'
    assert m.left is not None and m.right is not None

"""Synthetic ACKs/poses only: no node, socket, service, CAN or device is opened."""
import threading
from types import SimpleNamespace

import pytest

from remote_teleop_runtime.collection_motion import CollectionMotion

Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2
FREE, RIGHT_SERVO, RIGHT_FAULT = 0x10, 0x11, 0x12


def command(m, name="right_pause", *, now=1., ack=RIGHT_FAULT, healthy=True,
            leader=None, **request):
    m.command(name, request, Q, Q, Q if leader is None else leader,
              [0.] * 16, now, healthy, leader_ack=ack)


def held(tmp_path, *, target=True):
    m = CollectionMotion(tmp_path / "saved.json")
    m.left = tuple(Q[:8])
    m.right = tuple(Q[8:])
    m.leader_target = tuple(Q[8:]) if target else None
    m.leader_now = tuple(Q)
    m.phase = "hold"
    m.collection_ack = RIGHT_FAULT
    return m


def cycle(m, now, ack=FREE, healthy=True, leader=None):
    master = Q if leader is None else leader
    return m.update(Q, master, now, healthy, master, ack)


@pytest.mark.parametrize("gap", [0., .3])
@pytest.mark.parametrize("target", [False, True])
def test_confirmed_release_keeps_both_holds_even_when_right_not_aligned(tmp_path, gap, target):
    m = held(tmp_path, target=target)
    master = Q[:]; master[8] += gap
    before = (m.left, m.right, m.left_master_target, dict(m.transition))
    command(m, release_servo=True, leader=master)
    assert m.right_servo_release_pending and m.returning and m.phase == "releasing"
    assert m.leader_target is None and not m.resume_on_release
    assert m.flags == 3
    status = m.status(Q, master)
    assert status["right_pause_release_supported"] is True
    assert status["right_servo_release_detail"]["source_leader_mode"] == 2
    assert status["right_servo_release_detail"]["source_target_present"] is target
    for now in (1.01, 1.05, 1.09):
        assert cycle(m, now, leader=master) == Q
        assert m.right_servo_release_pending
    assert cycle(m, 1.12, leader=master) == Q
    assert not m.right_servo_release_pending and not m.returning and m.phase == "hold"
    assert (m.left, m.right, m.left_master_target, m.transition) == before
    assert "不会自动跟随" in m.note
    assert not m.status(Q, master)["right_servo_release_required"]


@pytest.mark.parametrize("release", [None, False, 1, "true"])
def test_old_pause_or_untrusted_confirmation_never_relaxes_master(tmp_path, release):
    m = held(tmp_path)
    target = m.leader_target
    for now in (1., 1.1, 1.2):
        command(m, now=now, release_servo=release)
    assert m.leader_target == target and not m.right_servo_release_pending
    assert m.status(Q, Q)["right_servo_release_required"]


def test_first_pause_during_return_only_stops_and_repeated_stop_never_releases(tmp_path):
    m = held(tmp_path); m.returning = True; m.phase = "moving"
    command(m, ack=RIGHT_SERVO, release_servo=True)
    assert not m.returning and m.phase == "hold" and m.leader_target is not None
    command(m, now=1.1)
    assert m.leader_target is not None and not m.right_servo_release_pending
    command(m, now=1.2, release_servo=True)
    assert m.leader_target is None and m.right_servo_release_pending


def test_repeated_release_does_not_reset_deadline_or_free_stability_timer(tmp_path):
    m = held(tmp_path)
    command(m, release_servo=True)
    cycle(m, 1.01)
    since = m.right_servo_free_since
    command(m, now=1.05, ack=FREE, release_servo=True)
    assert m.started == 1. and m.right_servo_free_since == since
    cycle(m, 1.12)
    assert not m.returning and m.right is not None


def test_free_ack_must_be_continuous_and_new_after_withdrawal(tmp_path):
    m = held(tmp_path)
    command(m, ack=FREE, release_servo=True)  # Residual target with an old ACK0.
    cycle(m, 1.001)
    cycle(m, 1.09, RIGHT_FAULT)
    cycle(m, 1.10)
    cycle(m, 1.19)
    assert m.right_servo_release_pending
    cycle(m, 1.21)
    assert not m.right_servo_release_pending and m.right is not None


@pytest.mark.parametrize("bad_ack", [3, 0x13, 0x1c, None])
def test_unknown_ack_cannot_confirm_release(tmp_path, bad_ack):
    m = held(tmp_path); command(m, release_servo=True)
    before = (m.left, m.right)
    cycle(m, 1.1, bad_ack)
    assert not m.returning and m.phase == "hold"
    assert m.leader_target is None and (m.left, m.right) == before
    assert "无效" in m.note and "尚未确认" in m.note
    with pytest.raises(ValueError):
        command(m, "left_master_align", now=1.2, ack=0x13)


def test_timeout_keeps_preload_and_requires_explicit_retry(tmp_path):
    m = held(tmp_path); command(m, release_servo=True)
    left, right = m.left, m.right
    cycle(m, 3.01, RIGHT_FAULT)
    assert not m.right_servo_release_pending and not m.returning
    assert m.phase == "hold" and m.leader_target is None
    assert "超时" in m.note and (m.left, m.right) == (left, right)
    for now in (3.1, 3.2):
        cycle(m, now, RIGHT_FAULT)
    assert not m.returning
    command(m, now=4., release_servo=True)
    assert m.right_servo_release_pending and m.started == 4.
    cycle(m, 4.01); cycle(m, 4.12)
    assert not m.returning and (m.left, m.right) == (left, right)


def test_fault_neither_rearms_master_nor_resumes_release_or_follower(tmp_path):
    m = held(tmp_path); command(m, release_servo=True)
    cycle(m, 1.01)
    cycle(m, 1.05, FREE, healthy=False)
    assert not m.right_servo_release_pending and m.leader_target is None
    assert "控制许可或反馈中断" in m.note
    cycle(m, 1.3, RIGHT_FAULT)
    assert m.phase == "hold" and not m.returning and m.right is not None
    with pytest.raises(ValueError, match="RUNNING"):
        command(m, now=1.4, healthy=False, release_servo=True)
    assert not m.returning


@pytest.mark.parametrize("blocked", ["recording", "left_servo", "left_master_active",
                                      "left_target", "left_alignment", "transition", "unknown"])
def test_release_gate_preserves_all_targets_on_rejection(tmp_path, blocked):
    m = held(tmp_path); ack = RIGHT_FAULT
    if blocked == "recording": m.recording = {"side": "left", "token": "test"}
    if blocked == "left_servo": ack = 0x16
    if blocked == "left_master_active": m.left_master_align_active = True
    if blocked == "left_target": m.left_master_target = tuple(Q[:8])
    if blocked == "left_alignment": m.left_align_active = True
    if blocked == "transition": m.transition["left"] = list(Q[:8])
    if blocked == "unknown": ack = 0x13
    before = (m.left, m.right, m.leader_target, m.left_master_target)
    with pytest.raises(ValueError, match="录制|运动切换"):
        command(m, ack=ack, release_servo=True)
    assert (m.left, m.right, m.leader_target, m.left_master_target) == before
    assert not m.right_servo_release_pending


def test_record_and_left_start_wait_for_release_completion(tmp_path):
    m = held(tmp_path); command(m, release_servo=True)
    with pytest.raises(ValueError, match="运动切换"):
        command(m, "collection_begin", ack=FREE, side="left", token="test")
    with pytest.raises(ValueError, match="已有回位、伺服或跟随切换"):
        command(m, "left_master_align", ack=FREE)


def test_completed_release_duplicate_and_plain_pause_keep_both_holds(tmp_path):
    m = held(tmp_path); command(m, release_servo=True)
    cycle(m, 1.01); cycle(m, 1.12)
    before = (m.left, m.right, m.leader_target, m.started, dict(m.right_servo_release_detail))
    command(m, now=1.2, ack=FREE, release_servo=True)
    command(m, now=1.3, ack=FREE)
    assert (m.left, m.right, m.leader_target, m.started, m.right_servo_release_detail) == before
    assert not m.returning and not m.right_servo_release_pending


def test_left_mode_conflict_during_release_stops_without_new_right_servo_target(tmp_path):
    m = held(tmp_path); command(m, release_servo=True)
    before = (m.left, m.right)
    cycle(m, 1.1, 0x14)
    assert not m.returning and not m.right_servo_release_pending
    assert m.leader_target is None and (m.left, m.right) == before
    assert "左主臂伺服状态冲突" in m.note


@pytest.mark.parametrize("failure", ["timeout", "health_loss"])
@pytest.mark.parametrize("ack", [RIGHT_SERVO, RIGHT_FAULT, 0x13])
def test_failed_release_cannot_bypass_physical_free_gate_via_right_follow(tmp_path, failure, ack):
    m = held(tmp_path); command(m, release_servo=True)
    if failure == "timeout":
        cycle(m, 3.01, RIGHT_FAULT)
    else:
        cycle(m, 1.1, RIGHT_FAULT, healthy=False)
    assert m.leader_target is None and not m.returning
    before = (m.left, m.right, dict(m.transition))
    with pytest.raises(ValueError, match="伺服尚未"):
        command(m, "right_follow", now=4., ack=ack)
    assert (m.left, m.right, m.transition) == before
    assert m.phase == "hold" and m.leader_target is None
    # A genuinely free, healthy ACK plus another explicit follow command is
    # allowed. This is never an automatic recovery from either failure.
    cycle(m, 4.1, FREE)
    assert m.right is not None
    command(m, "right_follow", now=4.2, ack=FREE)
    assert m.right is None and m.transition["right"] == list(before[1])


def test_pending_release_other_commands_cannot_resume_any_motion(tmp_path):
    m = held(tmp_path); command(m, release_servo=True)
    before = (m.left, m.right, m.leader_target, m.started)
    for name in ("left_follow", "right_follow"):
        with pytest.raises(ValueError, match="右臂仍在回位"):
            command(m, name, now=1.1, ack=FREE)
    command(m, "right_return", now=1.2, ack=FREE)
    assert (m.left, m.right, m.leader_target, m.started) == before
    assert m.right_servo_release_pending and not m.resume_on_release


def test_real_leader_router_releases_only_right_then_accepts_left_alignment(tmp_path, monkeypatch):
    from remote_teleop_runtime import leader
    monkeypatch.setattr(leader.time, "monotonic", lambda: 10.)
    gateway = object.__new__(leader.LeaderGateway)
    gateway.lock = threading.Lock()
    gateway.enable_left = gateway.have_left = gateway.have_right = True
    gateway.left_rx = gateway.right_rx = 10.
    gateway.collection_ack = 2; gateway.left_collection_ack = 0
    gateway.left_collection_mode_invalid = False
    gateway.return_requested = True; gateway.left_return_requested = False
    right_messages, left_messages = [], []
    gateway.return_pub = SimpleNamespace(publish=lambda msg: right_messages.append(list(msg.position)))
    gateway.left_return_pub = SimpleNamespace(publish=lambda msg: left_messages.append(list(msg.position)))
    m = held(tmp_path)
    master = Q[:]; master[0] = .2; master[8] = .3
    with pytest.raises(ValueError):
        command(m, "left_master_align", leader=master)
    command(m, release_servo=True, leader=master)

    def route():
        gateway.publish_return(SimpleNamespace(
            collection_flags=m.flags, control_state=SimpleNamespace(name="RUNNING"), fault_bits=0,
            sender_monotonic_ns=1_000_000_000, obs_timestamp_ns=990_000_000,
            leader_return_target=m.leader_servo_target or (0.,)*8))

    route()
    assert right_messages == [[]] and left_messages == []
    assert not gateway.return_requested
    gateway.collection_ack = 0  # Simulated physical controller free ACK.
    ack = gateway.collection_acknowledgement()
    cycle(m, 1.01, ack, leader=master); cycle(m, 1.12, ack, leader=master)
    assert m.right == tuple(Q[8:]) and m.left == tuple(Q[:8])
    command(m, "left_master_align", now=1.2, ack=ack, leader=master)
    for step in range(1, 20):
        cycle(m, 1.2 + step*.01, ack, leader=master)
        if m.left_master_target is not None:
            break
    assert m.flags == 11 and m.right == tuple(Q[8:])
    route()
    assert left_messages == [list(m.left_master_target)]
    assert right_messages == [[]]  # No right target or implicit right FOLLOW.

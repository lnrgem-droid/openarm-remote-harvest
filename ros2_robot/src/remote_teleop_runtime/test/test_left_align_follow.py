"""Synthetic-only tests: no ROS node, sockets, CAN, or physical movement."""
import math

import pytest

from remote_teleop_runtime.collection_motion import (
    CollectionMotion, LEFT_ALIGN_SPEED_RAD_S, LEFT_LIMITS,
)


Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2


def command(m, name, *, actual=None, applied=None, leader=None, now=1., healthy=True, **fields):
    m.command(name, fields, Q if actual is None else actual,
              Q if applied is None else applied, Q if leader is None else leader,
              [0.] * 16, now, healthy)


def begin(tmp_path, axis=0, change=.3, measured=None):
    m = CollectionMotion(tmp_path / "pose.json")
    goal = Q[:]
    goal[axis] += change
    command(m, "left_lock", actual=measured)
    command(m, "left_align_follow", leader=goal, actual=measured)
    return m, goal


@pytest.mark.parametrize("axis", range(8))
def test_all_eight_axes_reach_fixed_goal_then_follow_with_bounded_speed(tmp_path, axis):
    change = -.5 if axis in (1, 7) else .5
    m, goal = begin(tmp_path, axis, change)
    actual = Q[:]
    peak = 0.
    saw_settling = False
    for step in range(1, int((m.left_align_duration + 2.) / .01)):
        before = actual[:]
        actual = m.update(actual, goal, 1. + step*.01, True, goal)
        peak = max(peak, abs(actual[axis]-before[axis])/.01)
        assert actual[8:] == goal[8:]
        if m.left_align_active:
            assert m.flags & 1
            assert m.status(actual, goal)["left_mode"] == "HOLD"
        if m.left_align_phase == "settling":
            saw_settling = True
        if not m.left_align_active:
            break
    assert saw_settling
    assert peak <= LEFT_ALIGN_SPEED_RAD_S[axis] + 1e-9
    assert peak > .99 * LEFT_ALIGN_SPEED_RAD_S[axis]
    assert m.left_align_phase == "completed"
    assert not m.flags & 1
    assert actual[:8] == pytest.approx(goal[:8])
    assert m.left is None
    assert m.status(actual, goal)["transitioning_arms"] == []


def test_starts_at_applied_target_and_preserves_preload(tmp_path):
    measured = Q[:]
    measured[7] += .15
    m, goal = begin(tmp_path, measured=measured)
    assert m.left_align_start == tuple(Q[:8])
    assert m.update(measured, goal, 1., True, goal)[:8] == Q[:8]


def test_duplicate_start_does_not_recapture_moving_master_or_restart_clock(tmp_path):
    m, goal = begin(tmp_path)
    m.update(Q, goal, 1.01, True, goal)
    before = (m.left_align_goal, m.left_align_started, m.left_align_elapsed)
    moved = goal[:]
    moved[0] += .04
    command(m, "left_align_follow", leader=moved, now=1.02)
    assert (m.left_align_goal, m.left_align_started, m.left_align_elapsed) == before


@pytest.mark.parametrize("command_name", ["left_align_pause", "left_lock"])
def test_cancel_keeps_last_applied_target_and_never_restarts(tmp_path, command_name):
    m, goal = begin(tmp_path)
    actual = Q[:]
    for step in range(1, 50):
        actual = m.update(actual, goal, 1.+step*.01, True, goal)
    held = m.left
    command(m, command_name, actual=actual, applied=actual, leader=goal, now=1.5, healthy=False)
    assert m.left_align_phase == "paused"
    assert m.left == held
    assert not m.left_align_active
    later = m.update(actual, goal, 20., True, goal)
    assert later[:8] == list(held)
    assert m.flags & 1


def test_control_failure_preserves_left_hold_without_restarting(tmp_path):
    m, goal = begin(tmp_path)
    actual = m.update(Q, goal, 1.02, True, goal)
    held = m.left
    actual = m.update(actual, goal, 1.04, False, goal)
    assert actual[:8] == list(held)
    assert m.left_align_phase == "failed"
    assert not m.left_align_active
    assert m.update(actual, goal, 2., True, goal)[:8] == list(held)


@pytest.mark.parametrize("finish_transition", [False, True])
def test_late_pause_after_completion_latches_latest_applied_without_jump(tmp_path, finish_transition):
    m, goal = begin(tmp_path)
    leader = goal[:]
    leader[0] += .04
    applied = Q[:]
    now = 1.
    while m.left_align_active:
        now += .01
        applied = m.update(applied, leader, now, True, leader)
    assert m.left_align_phase == "completed"
    assert m.left is None
    assert m.transition["left"] is not None
    if finish_transition:
        for _ in range(100):
            now += .01
            applied = m.update(applied, leader, now, True, leader)
        assert m.transition["left"] is None
        leader[0] += .02
        now += .01
        applied = m.update(applied, leader, now, True, leader)
    assert applied[0] > goal[0]
    before = applied[:]
    command(m, "left_align_pause", applied=applied, actual=applied, leader=leader, now=now, healthy=False)
    assert m.left_align_phase == "paused"
    assert m.left == tuple(before[:8])
    assert m.transition["left"] is None
    assert m.flags & 1
    output = m.update(applied, leader, now+.01, True, leader)
    assert output == before


def test_late_pause_rejects_invalid_applied_snapshot(tmp_path):
    m, goal = begin(tmp_path, change=.05)
    applied = Q[:]
    for step in range(1, 500):
        applied = m.update(applied, goal, 1.+step*.01, True, goal)
    invalid = applied[:]
    invalid[7] = float("nan")
    with pytest.raises(ValueError, match="有限"):
        command(m, "left_align_pause", applied=invalid, healthy=False)
    assert m.left is None


def test_master_motion_stops_fixed_goal_and_preserves_right_state(tmp_path):
    m, goal = begin(tmp_path)
    m.right = tuple(Q[8:])
    m.phase = "hold"
    m.return_detail = {"prior_right_event": "retained"}
    right_state = (m.right, m.phase, dict(m.return_detail), m.leader_target)
    actual = m.update(Q, goal, 1.02, True, goal)
    held = m.left
    moved = goal[:]
    moved[7] += .11
    output = m.update(actual, moved, 1.04, True, moved)
    assert m.left_align_phase == "failed"
    assert "移离固定目标" in m.left_align_detail["message"]
    assert m.left == held
    assert output[:8] == list(held)
    assert (m.right, m.phase, m.return_detail, m.leader_target) == right_state


@pytest.mark.parametrize("axis,change", [(0, .5), (7, -.5)])
def test_arm_or_gripper_tracking_failure_retains_last_target(tmp_path, axis, change):
    m, goal = begin(tmp_path, axis, change)
    for step in range(1, 1200):
        held = m.left
        output = m.update(Q, goal, 1.+step*.01, True, goal)
        if not m.left_align_active:
            break
    assert m.left_align_phase == "failed"
    assert m.left == held
    assert output[:8] == list(held)
    assert "跟踪偏差" in m.left_align_detail["message"]
    assert m.left_align_detail["worst_axis"] == ("夹爪" if axis == 7 else "J1")


def test_timeout_requires_measured_goal_and_keeps_hold(tmp_path):
    m, goal = begin(tmp_path, change=.15)
    for step in range(1, 1400):
        m.update(Q, goal, 1.+step*.01, True, goal)
        if not m.left_align_active:
            break
    assert m.left_align_phase == "failed"
    assert "超时" in m.left_align_detail["message"]
    assert m.left is not None


def test_gripper_must_be_within_goal_tolerance_for_stable_release(tmp_path):
    m, goal = begin(tmp_path, axis=7, change=-.15)
    for step in range(1, 1400):
        m.update(Q, goal, 1.+step*.01, True, goal)
        if not m.left_align_active:
            break
    assert m.left_align_phase == "failed"
    assert "超时" in m.left_align_detail["message"]
    assert m.flags & 1


def test_small_master_drift_above_alignment_tolerance_cannot_release(tmp_path):
    m, goal = begin(tmp_path)
    moved = goal[:]
    moved[0] += .08
    actual = Q[:]
    for step in range(1, 1500):
        actual = m.update(actual, moved, 1.+step*.01, True, moved)
        if not m.left_align_active:
            break
    assert m.left_align_phase == "failed"
    assert "超时" in m.left_align_detail["message"]
    assert m.left_align_goal == tuple(goal[:8])


def test_delayed_cycle_never_catches_up_with_a_large_target_step(tmp_path):
    m, goal = begin(tmp_path, change=.5)
    actual = Q[:]
    now = 1.
    while m.left_align_elapsed < m.left_align_duration/2:
        now += .01
        actual = m.update(actual, goal, now, True, goal)
    before = m.left[0]
    old_elapsed = m.left_align_elapsed
    actual = m.update(actual, goal, now+.4, True, goal)
    assert m.left_align_elapsed-old_elapsed == pytest.approx(.02)
    assert abs(actual[0]-before) <= LEFT_ALIGN_SPEED_RAD_S[0]*.02 + 1e-9


def test_settling_requires_observed_stable_samples_not_wall_clock_gap(tmp_path):
    m, goal = begin(tmp_path, change=.05)
    actual = Q[:]
    now = 1.
    while m.left_align_phase != "settling":
        now += .01
        actual = m.update(actual, goal, now, True, goal)
    m.update(actual, goal, now+1., True, goal)
    assert m.left_align_active
    assert m.left_align_stable_s < .1


@pytest.mark.parametrize("conflict", ["left_follow", "right_save", "right_return", "right_follow", "collection_begin"])
def test_new_motion_and_both_recording_sides_are_blocked_during_alignment(tmp_path, conflict):
    m, goal = begin(tmp_path)
    for side in ("left", "right"):
        with pytest.raises(ValueError, match="左臂正在"):
            command(m, conflict, leader=goal, side=side, token="synthetic")
    assert m.left_align_active


@pytest.mark.parametrize("blocked_by", ["right_return", "left_transition", "right_transition", "recording", "unhealthy", "not_held"])
def test_start_rejects_conflicting_or_unhealthy_state(tmp_path, blocked_by):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    if blocked_by == "right_return":
        m.returning = True
    elif blocked_by.endswith("transition"):
        m.transition[blocked_by.split("_")[0]] = Q[:8]
    elif blocked_by == "recording":
        m.recording = {"side": "right", "token": "synthetic"}
    elif blocked_by == "not_held":
        m.left = None
    with pytest.raises(ValueError):
        command(m, "left_align_follow", healthy=blocked_by != "unhealthy")
    assert not m.left_align_active


@pytest.mark.parametrize("field,index,value", [
    ("leader", 0, 1.5), ("leader", 1, .2), ("leader", 7, -1.1),
    ("leader", 3, float("nan")), ("actual", 7, float("inf")), ("applied", 0, float("nan")),
])
def test_left_specific_limits_and_finite_feedback_are_required(tmp_path, field, index, value):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    vector = Q[:]
    vector[index] = value
    with pytest.raises(ValueError):
        command(m, "left_align_follow", **{field: vector})
    assert not m.left_align_active


def test_left_limits_use_actual_left_urdf_offsets():
    assert LEFT_LIMITS[0] == pytest.approx((-3.490659, 1.396263))
    assert LEFT_LIMITS[1] == pytest.approx((-1.745329-math.pi/2, 1.745329-math.pi/2))


def test_existing_manual_resume_still_rejects_misalignment(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    leader = Q[:]
    leader[0] += .1
    with pytest.raises(ValueError, match="对齐"):
        command(m, "left_follow", leader=leader)
    assert m.left is not None
    assert not m.left_align_active


def test_completed_alignment_then_hold_can_start_and_complete_another_alignment(tmp_path):
    m, goal = begin(tmp_path, change=.1)
    actual = Q[:]
    now = 1.
    while m.left_align_active:
        now += .01
        actual = m.update(actual, goal, now, True, goal)
    assert m.left_align_phase == "completed"
    command(m, "left_lock", actual=actual, applied=actual, leader=goal, now=now)
    assert m.left_align_phase == "idle"
    assert m.left_align_detail == {}
    assert m.left == tuple(actual[:8])
    goal[0] += .1
    command(m, "left_align_follow", actual=actual, applied=actual, leader=goal, now=now)
    while m.left_align_active:
        now += .01
        actual = m.update(actual, goal, now, True, goal)
    assert m.left_align_phase == "completed"
    assert m.left is None
    assert actual[:8] == pytest.approx(goal[:8])


@pytest.mark.parametrize("previous_phase", ["paused", "failed"])
def test_manual_resume_clears_prior_automatic_alignment_message(tmp_path, previous_phase):
    m, goal = begin(tmp_path)
    if previous_phase == "paused":
        command(m, "left_align_pause")
    else:
        m.update(Q, goal, 1.01, False, goal)
    assert m.left_align_phase == previous_phase
    command(m, "left_follow", leader=Q)
    assert m.left is None
    assert m.left_align_phase == "idle"
    assert m.left_align_detail == {}

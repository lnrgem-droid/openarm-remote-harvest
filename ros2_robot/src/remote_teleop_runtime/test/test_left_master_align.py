"""Left master servo coordination, using synthetic arrays and ACKs only."""
import pytest

from remote_teleop_runtime.collection_motion import CollectionMotion, LEFT_ALIGN_SPEED_RAD_S

Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2
FREE = 0x10
LEFT_SERVO = 0x14
LEFT_FAULT = 0x18


def command(m, name, *, leader=None, actual=None, applied=None, now=1., ack=FREE, healthy=True, **request):
    m.command(name, request, Q if actual is None else actual,
              Q if applied is None else applied, Q if leader is None else leader,
              [0.]*16, now, healthy, leader_ack=ack)


def begin(tmp_path, axis=0, change=.3, right_held=False, actual=None):
    m = CollectionMotion(tmp_path / "pose.json")
    master = Q[:]
    master[axis] += change
    command(m, "left_lock", leader=master)
    if right_held:
        m.right = tuple(Q[8:])
        m.phase = "hold"
        m.return_detail = {"right_event": "preserved"}
    command(m, "left_master_align", leader=master, actual=actual)
    return m, master


def cycle(m, master, now, *, actual=None, ack=None, healthy=True, track=True):
    actual = Q[:] if actual is None else actual
    ack = (LEFT_SERVO if m.left_master_target is not None else FREE) if ack is None else ack
    output = m.update(actual, master, now, healthy, master, ack)
    if track and m.left_master_target is not None:
        master = list(m.left_master_target) + master[8:]
    return output, master


def enter_moving(m, master):
    now = 1.
    while m.left_master_align_phase != "moving":
        now += .01
        _, master = cycle(m, master, now)
        assert now < 2.
    return now, master


@pytest.mark.parametrize("axis", range(8))
@pytest.mark.parametrize("right_held", [False, True])
def test_master_eight_axis_curve_keeps_follower_target_fixed_until_free_ack(tmp_path, axis, right_held):
    change = -.5 if axis in (1, 7) else .5
    m, master = begin(tmp_path, axis, change, right_held)
    right_state = (m.right, m.phase, dict(m.return_detail), m.leader_target)
    held = m.left
    now, master = enter_moving(m, master)
    peak = 0.
    phases = set()
    for _ in range(1800):
        before = master[:]
        now += .01
        output, master = cycle(m, master, now)
        phases.add(m.left_master_align_phase)
        peak = max(peak, abs(master[axis]-before[axis])/.01)
        assert output[8:] == Q[8:]
        assert (m.right, m.phase, m.return_detail, m.leader_target) == right_state
        if m.left_master_align_phase == "completed":
            break
        assert m.left == held
        assert output[:8] == list(held)
        assert m.flags & 1
        if m.left_master_target is not None:
            assert m.flags == (11 if right_held else 9)
    assert {"moving", "settling", "releasing", "completed"} <= phases
    assert peak <= LEFT_ALIGN_SPEED_RAD_S[axis]+1e-9
    assert peak >= .99*LEFT_ALIGN_SPEED_RAD_S[axis]
    assert master[:8] == pytest.approx(Q[:8])
    assert m.left is None
    assert not m.left_master_align_active
    assert m.status(Q, master)["left_master_align_detail"]["servo_released"]


def test_capability_is_real_peer_mode_support_and_old_peer_cannot_start(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    assert not m.status(Q, Q)["left_master_align_supported"]
    command(m, "left_lock", ack=0)
    for ack in (0, 1, 2, 3, 0x13, 0x1c, 0x20):
        with pytest.raises(ValueError):
            command(m, "left_master_align", ack=ack)
        assert not m.status(Q, Q)["left_master_align_supported"]
    command(m, "left_master_align", ack=FREE)
    assert m.status(Q, Q)["left_master_align_supported"]


@pytest.mark.parametrize("ack", [0x11, 0x12, LEFT_SERVO, LEFT_FAULT, 0x15, 0x19])
def test_existing_physical_servo_or_fault_requires_explicit_stop_before_start(tmp_path, ack):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    with pytest.raises(ValueError):
        command(m, "left_master_align", ack=ack)
    assert not m.left_master_align_active


def test_explicit_pause_can_release_orphan_left_fault_without_changing_right(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    m.right = tuple(Q[8:])
    held = m.left
    command(m, "left_master_pause", ack=LEFT_FAULT)
    assert m.left_master_align_phase == "releasing"
    assert m.left == held
    for step in range(1, 15):
        cycle(m, Q[:], 1.+step*.01, ack=FREE)
    assert m.left_master_align_phase == "paused"
    assert m.left == held and m.right == tuple(Q[8:])


@pytest.mark.parametrize("name", ["collection_begin", "right_return", "right_follow", "right_save", "left_follow", "left_align_follow"])
@pytest.mark.parametrize("ack", [LEFT_SERVO, LEFT_FAULT, 0x13])
def test_physical_left_nonfree_or_unknown_blocks_recording_and_other_motion(tmp_path, name, ack):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    with pytest.raises(ValueError):
        command(m, name, ack=ack, side="left", token="synthetic")


def test_duplicate_start_cannot_recapture_goal_or_reset_running_curve(tmp_path):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    _, master = cycle(m, master, now+.02)
    before = (m.left_master_goal, m.left_master_start, m.left_master_elapsed, m.left)
    moved = master[:]
    moved[0] += .1
    command(m, "left_master_align", leader=moved, now=now+.03, ack=LEFT_SERVO)
    assert (m.left_master_goal, m.left_master_start, m.left_master_elapsed, m.left) == before


@pytest.mark.parametrize("ack", [0, 0x11, LEFT_FAULT, FREE])
def test_lost_capability_wrong_side_fault_or_missing_servo_ack_stops_path(tmp_path, ack):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    held = m.left
    cycle(m, master, now+.01, ack=ack)
    assert m.left_master_align_phase == "failed"
    assert m.left_master_align_active
    assert m.left == held
    assert m.left_master_target == tuple(master[:8])


@pytest.mark.parametrize("index,value", [(0, 1.5), (1, .2), (7, -1.1), (3, float("nan"))])
def test_master_start_requires_left_limits_and_finite_eight_axis_pose(tmp_path, index, value):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    master = Q[:]
    master[index] = value
    with pytest.raises(ValueError):
        command(m, "left_master_align", leader=master)
    assert not m.left_master_align_active
    assert m.left_master_target is None


@pytest.mark.parametrize("axis,change", [(0, .5), (7, -.5)])
def test_arm_and_gripper_tracking_failure_latches_master_actual_and_follower_hold(tmp_path, axis, change):
    m, master = begin(tmp_path, axis, change)
    now, master = enter_moving(m, master)
    held = m.left
    for _ in range(1500):
        now += .01
        output, master = cycle(m, master, now, track=False)
        if m.left_master_align_phase == "failed":
            break
    assert m.left_master_align_phase == "failed"
    assert m.left_master_align_active
    assert m.left_master_target == tuple(master[:8])
    assert m.left == held and output[:8] == list(held)
    assert m.left_master_align_detail["worst_axis"] == ("夹爪" if axis == 7 else "J1")
    saved_target = m.left_master_target
    cycle(m, master, now+1., track=False)
    assert m.left_master_target == saved_target
    assert m.left_master_align_phase == "failed"


def test_pause_stops_master_then_waits_for_explicit_free_without_releasing_follower(tmp_path):
    m, master = begin(tmp_path, right_held=True)
    now, master = enter_moving(m, master)
    held = m.left
    right = m.right
    for _ in range(100):
        now += .01
        _, master = cycle(m, master, now)
    current = tuple(master[:8])
    command(m, "left_master_pause", leader=master, now=now, ack=LEFT_SERVO)
    assert m.left_master_target == current
    assert m.left_master_align_phase == "stopping"
    for _ in range(20):
        now += .01
        cycle(m, master, now, ack=LEFT_SERVO)
    assert m.left_master_align_phase == "releasing"
    assert m.left_master_target is None
    assert m.left_master_align_active
    assert m.left == held and m.right == right
    for _ in range(300):
        now += .01
        cycle(m, master, now, ack=LEFT_SERVO)
    assert m.left_master_align_phase == "releasing" and m.left_master_align_active
    assert not m.status(Q, master)["left_master_align_detail"]["servo_released"]
    for _ in range(12):
        now += .01
        cycle(m, master, now, ack=FREE)
    assert m.left_master_align_phase == "paused"
    assert m.left == held and m.right == right
    assert not m.left_master_align_active


def test_control_loss_never_restarts_trajectory_and_pause_can_release_after_recovery(tmp_path):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    held = m.left
    cycle(m, master, now+.01, healthy=False)
    assert m.left_master_align_phase == "failed"
    target = m.left_master_target
    cycle(m, master, now+.02)
    assert m.left_master_target == target
    assert m.left_master_align_phase == "failed"
    command(m, "left_master_pause", leader=master, now=now+.03, healthy=False, ack=LEFT_SERVO)
    for step in range(4, 50):
        _, master = cycle(m, master, now+step*.01)
    assert m.left_master_align_phase == "paused"
    assert m.left == held


def test_duplicate_pause_does_not_recapture_or_reset_free_ack_timer(tmp_path):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    command(m, "left_master_pause", leader=master, now=now, ack=LEFT_SERVO)
    target = m.left_master_target
    started = m.left_master_phase_started
    moved = master[:]
    moved[0] += .03
    command(m, "left_master_pause", leader=moved, now=now+.03, ack=LEFT_SERVO)
    assert m.left_master_target == target
    assert m.left_master_phase_started == started
    while m.left_master_align_phase != "releasing":
        now += .01
        _, master = cycle(m, master, now)
    now += .02
    cycle(m, master, now, ack=FREE)
    observed = m.left_master_free_s
    command(m, "left_master_pause", leader=master, now=now, ack=FREE)
    assert m.left_master_free_s == observed


def test_completion_rechecks_alignment_after_servo_is_free(tmp_path):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    while m.left_master_align_phase != "releasing":
        now += .01
        _, master = cycle(m, master, now)
        assert now < 12.
    master[7] += .07
    for _ in range(12):
        now += .01
        cycle(m, master, now, ack=FREE)
    assert m.left_master_align_phase == "failed"
    assert m.left == tuple(Q[:8])
    assert not m.left_master_align_active
    assert m.status(Q, master)["left_master_align_detail"]["servo_released"]


def test_control_loss_during_release_does_not_treat_cached_free_ack_as_confirmed(tmp_path):
    m, master = begin(tmp_path)
    command(m, "left_master_pause", leader=master)
    assert m.left_master_align_phase == "releasing"
    cycle(m, master, 1.01, ack=FREE, healthy=False)
    assert m.left_master_align_phase == "failed"
    assert m.left_master_align_active
    assert not m.status(Q, master)["left_master_align_detail"]["servo_released"]


@pytest.mark.parametrize("axis", [0, 7])
def test_follower_disturbance_during_release_cannot_resume_follow(tmp_path, axis):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    while m.left_master_align_phase != "releasing":
        now += .01
        _, master = cycle(m, master, now)
    disturbed = Q[:]
    disturbed[axis] += .15
    now += .01
    cycle(m, master, now, actual=disturbed, ack=FREE)
    # Even if the follower springs back before the free confirmation arrives,
    # a disturbed HOLD must not silently turn into FOLLOW.
    for _ in range(12):
        now += .01
        cycle(m, master, now, ack=FREE)
    assert m.left_master_align_phase == "failed"
    assert m.left == tuple(Q[:8])
    assert not m.left_master_align_active
    assert m.status(Q, master)["left_master_align_detail"]["servo_released"]
    assert "显著位移" in m.left_master_align_detail["message"]


def test_late_pause_after_completion_latches_new_applied_not_old_hold_goal(tmp_path):
    m, master = begin(tmp_path)
    now, master = enter_moving(m, master)
    while m.left_master_align_phase != "completed":
        now += .01
        _, master = cycle(m, master, now)
    applied = Q[:]
    applied[0] += .04
    command(m, "left_master_pause", leader=applied, applied=applied, now=now)
    for _ in range(12):
        now += .01
        cycle(m, applied, now, actual=applied)
    assert m.left == tuple(applied[:8])
    assert m.left_master_align_phase == "paused"


def test_delayed_control_tick_never_catches_up_with_large_master_step(tmp_path):
    m, master = begin(tmp_path, change=.5)
    now, master = enter_moving(m, master)
    while m.left_master_elapsed < m.left_master_duration/2:
        now += .01
        _, master = cycle(m, master, now)
    before = m.left_master_target[0]
    elapsed = m.left_master_elapsed
    _, master = cycle(m, master, now+.4)
    assert m.left_master_elapsed-elapsed == pytest.approx(.02)
    assert abs(m.left_master_target[0]-before) <= .10*.02 + 1e-9


def test_follower_preload_is_preserved_and_measured_drift_aborts_master_motion(tmp_path):
    actual = Q[:]
    actual[7] += .3
    m, master = begin(tmp_path, actual=actual)
    now = 1.
    while m.left_master_align_phase != "moving":
        now += .01
        _, master = cycle(m, master, now, actual=actual)
    assert m.left == tuple(Q[:8])
    actual[0] += .11
    cycle(m, master, now+.01, actual=actual)
    assert m.left_master_align_phase == "failed"
    assert m.left == tuple(Q[:8])


@pytest.mark.parametrize("block", ["recording", "right_return", "right_target", "old_left_align", "transition"])
def test_conflicting_operations_reject_start(tmp_path, block):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "left_lock")
    if block == "recording":
        m.recording = {"side": "right", "token": "synthetic"}
    elif block == "right_return":
        m.returning = True
    elif block == "right_target":
        m.leader_target = tuple(Q[8:])
    elif block == "old_left_align":
        m.left_align_active = True
    else:
        m.transition["right"] = Q[8:]
    with pytest.raises(ValueError):
        command(m, "left_master_align")


@pytest.mark.parametrize("side", ["left", "right"])
def test_recording_cannot_start_during_any_master_align_phase(tmp_path, side):
    m, master = begin(tmp_path)
    with pytest.raises(ValueError):
        command(m, "collection_begin", leader=master, side=side, token="synthetic")


def test_new_capability_ack_preserves_old_right_return_modes(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    command(m, "right_save")
    command(m, "left_lock")
    command(m, "right_return")
    master = Q[:]
    actual = Q[:]
    for step in range(1, 401):
        ack = 0x11 if m.leader_target is not None else FREE
        actual = m.update(actual, master, 1.+step*.01, True, master, ack)
        if m.leader_target is not None:
            master[8:] = m.leader_target
    assert not m.returning
    assert m.right is None
    assert m.left == tuple(Q[:8])

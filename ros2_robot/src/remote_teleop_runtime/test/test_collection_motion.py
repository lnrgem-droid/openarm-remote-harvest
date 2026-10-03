import json
import pytest
from remote_teleop_runtime.collection_motion import CollectionMotion, RETURN_SPEED_RAD_S

Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2


def execute(m, name, actual=None, applied=None, leader=None, now=1., **fields):
    m.command(name, fields, actual or Q, applied or Q, leader or Q, [0.]*16, now, True)


def test_hold_preserves_grip_preload_and_ignores_master(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    actual = list(Q); actual[7] = -.7
    execute(m, "left_lock", actual=actual)
    moved = [v+.1 for v in Q]
    assert m.update(actual, moved, 2., True)[:8] == Q[:8]
    execute(m, "left_lock", applied=moved)  # double click must not relatch
    assert list(m.left) == Q[:8]
    assert m.flags == 1


def test_resume_requires_alignment_and_ramps(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json"); execute(m, "left_lock")
    shifted = list(Q); shifted[0] += .3
    with pytest.raises(ValueError, match="对齐"):
        execute(m, "left_follow", leader=shifted)
    shifted[0] = .05
    execute(m, "left_follow", leader=shifted)
    assert m.status(Q, shifted)["transitioning_arms"] == ["left"]
    m.update(Q, shifted, 1., True)
    first = m.update(Q, shifted, 1.01, True)
    assert first[0] <= .001501
    for step in range(100):
        m.update(Q, shifted, 1.02 + step * .01, True)
    assert m.status(Q, shifted)["transitioning_arms"] == []


def test_saved_pose_survives_restart_and_return_is_bounded(tmp_path):
    path = tmp_path / "pose.json"
    m = CollectionMotion(path); execute(m, "right_save")
    m = CollectionMotion(path); execute(m, "left_lock")
    displaced = list(Q); displaced[14] = .1
    execute(m, "right_return", actual=displaced, applied=displaced, leader=displaced)
    prev = list(displaced); max_speed = 0.
    master = list(displaced)
    for step in range(1, 401):
        out = m.update(prev, master, 1.+step*.01, True, master,
                       1 if m.leader_target is not None else 0)
        max_speed = max(max_speed, abs(out[14]-prev[14])/.01)
        if m.leader_target is not None:
            master[8:16] = m.leader_target
        prev = out
    assert max_speed <= .15
    assert m.right is None and not m.returning
    assert prev[8:16] == Q[8:16]
    assert master[8:16] == Q[8:16]
    assert m.flags == 1


@pytest.mark.parametrize('joint', range(8))
@pytest.mark.parametrize('longer', ['leader', 'follower'])
def test_faster_saved_return_limits_both_arms_and_preserves_gripper(tmp_path, joint, longer):
    m = CollectionMotion(tmp_path / 'pose.json')
    execute(m, 'right_save'); execute(m, 'left_lock')
    actual = list(Q); master = list(Q)
    # Long enough that the 2 s minimum cannot hide an incorrect speed limit.
    direction = -1. if joint == 7 else 1.
    actual[8+joint] += direction * (.5 if longer == 'follower' else .4)
    master[8+joint] += direction * (.5 if longer == 'leader' else .4)
    execute(m, 'right_return', actual=actual, applied=actual, leader=master)
    speed = .25 if joint == 7 else .20
    assert RETURN_SPEED_RAD_S[joint] == speed
    assert m.duration == pytest.approx(1.875 * .5 / speed)
    if joint != 7:
        assert m.duration == pytest.approx(.75 * (1.875 * .5 / .15))
    peaks = [0., 0.]
    dt = .01
    for step in range(1, int((m.duration + 1.) / dt) + 1):
        previous = actual[:]; previous_master = master[:]
        actual = m.update(actual, master, 1.+step*dt, True, master,
                          1 if m.leader_target is not None else 0)
        if m.leader_target is not None:
            master[8:16] = m.leader_target
        for i, (current, old) in enumerate(((actual, previous), (master, previous_master))):
            peaks[i] = max(peaks[i], abs(current[8+joint]-old[8+joint])/dt)
        assert actual[:8] == Q[:8]  # Left HOLD must not change.
    assert max(peaks) <= speed + 1e-9
    assert max(peaks) > speed * .99
    assert actual[8:16] == Q[8:16] and master[8:16] == Q[8:16]
    assert m.phase == 'idle' and not m.returning


def test_return_rechecks_leader_distance_after_reset_handshake(tmp_path):
    m = CollectionMotion(tmp_path / 'pose.json')
    execute(m, 'right_save'); execute(m, 'left_lock'); execute(m, 'right_return')
    master = list(Q); master[8] += .8
    m.update(Q, master, 1.01, True, master, 0)
    assert m.phase == 'preparing'
    assert m.duration == pytest.approx(1.875 * .8 / .20)


@pytest.mark.parametrize('residual,expected_phase',[(.119,'hold'),(.0524,'idle')])
def test_return_static_residual_blocks_then_corrected_residual_releases(tmp_path, residual, expected_phase):
    m = CollectionMotion(tmp_path / 'pose.json')
    execute(m, 'right_save'); execute(m, 'left_lock')
    actual = list(Q); actual[10] = -.3
    master = actual[:]
    execute(m, 'right_return', actual=actual, applied=actual, leader=master)
    for step in range(1, 1501):
        actual = m.update(actual, master, 1.+step*.01, True, master,
                          1 if m.leader_target is not None else 0)
        if m.leader_target is not None:
            target = list(m.leader_target)
            # Approximate measured steady-state offset with gradual onset;
            # this tests the state machine, not a complete dynamics model.
            target[2] -= residual * min(step*.01, 1.)
            master[8:16] = target
        if m.phase == 'hold' or (m.phase == 'idle' and not m.returning):
            break
    assert m.phase == expected_phase
    if expected_phase == 'hold':
        assert '到位超时' in m.note
        assert m.return_detail['leader_worst_axis'] == 'J3'
        assert m.return_detail['leader_error_rad'] == pytest.approx(residual)
        assert m.right is not None and not m.returning
    else:
        assert m.right is None and not m.returning
        assert '恢复跟随' in m.note


def test_return_velocity_matches_controller_cap():
    from pathlib import Path
    import re
    header=Path(__file__).resolve().parents[2]/'openarm_gravity_pd_control/include/openarm_gravity_pd_control/tracking_assist.hpp'
    cap=float(re.search(r'kCollectionReturnMaxVelocity = ([0-9.]+);', header.read_text())[1])
    assert all(v == cap for v in RETURN_SPEED_RAD_S[:7])


def test_no_recording_during_return_or_mutation_during_recording(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save")
    with pytest.raises(ValueError, match="左臂保持"):
        execute(m, "collection_begin", side="right", token="1")
    execute(m, "left_lock"); execute(m, "right_return")
    with pytest.raises(ValueError, match="运动切换"):
        execute(m, "collection_begin", side="right", token="1")
    execute(m, "right_pause"); execute(m, "right_follow")
    m.update(Q, Q, 2., True)
    execute(m, "collection_begin", side="right", token="1")
    execute(m, "collection_begin", side="right", token="1")
    for command in ("right_return", "right_save", "left_follow"):
        with pytest.raises(ValueError, match="正在录制"):
            execute(m, command)
    with pytest.raises(ValueError, match="不属于"):
        execute(m, "collection_end", token="2")
    execute(m, "collection_end", token="1")


def test_disconnect_cancels_return_and_does_not_resume(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save"); execute(m, "left_lock")
    moved = list(Q); moved[14] = .15
    execute(m, "right_return", applied=moved, actual=moved)
    m.update(moved, moved, 1.1, False)
    assert not m.returning
    assert m.update(moved, Q, 2., True)[14] == .15


def test_corrupted_or_out_of_bounds_pose_rejected(tmp_path):
    path = tmp_path / "pose.json"; path.write_text('{"bad": 1}')
    m = CollectionMotion(path); assert m.saved is None
    bad = list(Q); bad[14] = 4.
    with pytest.raises(ValueError, match="限位"):
        execute(m, "right_save", actual=bad, applied=bad)
    assert json.loads(path.read_text()) == {"bad": 1}


def test_valid_json_with_wrong_root_type_does_not_crash_gateway(tmp_path):
    path = tmp_path / "pose.json"
    for value in ([], None, "wrong", 42):
        path.write_text(json.dumps(value))
        motion = CollectionMotion(path)
        assert motion.saved is None and "不可用" in motion.note


def test_tracking_error_aborts_motion(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save"); execute(m, "left_lock"); execute(m, "right_return")
    m.update(Q, Q, 1.01, True, Q, 0)
    wrong = list(Q); wrong[10] += .3
    m.update(wrong, Q, 1.2, True, Q, 1)
    assert not m.returning and "超限" in m.note


def test_no_follower_motion_until_physical_leader_ack(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save"); execute(m, "left_lock")
    moved = list(Q); moved[14] += .1
    execute(m, "right_return", actual=moved, applied=moved, leader=moved)
    assert m.update(moved, Q, 2., True, moved, 0)[8:] == moved[8:]
    m.update(moved, Q, 4.1, True, moved, 0)
    assert not m.returning and m.phase == "hold"


def test_master_tracking_error_and_fault_abort_both(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save"); execute(m, "left_lock"); execute(m, "right_return")
    m.update(Q, Q, 1.01, True, Q, 0)
    wrong = list(Q); wrong[14] += .3
    m.update(Q, wrong, 1.1, True, wrong, 1)
    assert not m.returning and m.leader_target[6] == .3
    execute(m, "right_return")
    m.update(Q, Q, 1.02, True, Q, 0)
    m.update(Q, Q, 2., True, Q, 2)
    assert not m.returning and "控制器故障" in m.note


def test_record_waits_for_leader_release_ack(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save"); execute(m, "left_lock"); execute(m, "right_return")
    m.update(Q, Q, .99, True, Q, 0)
    for now in (1., 3., 3.6):
        m.update(Q, Q, now, True, Q, 1)
    assert m.phase == "releasing" and m.returning
    with pytest.raises(ValueError, match="运动切换"):
        execute(m, "collection_begin", side="right", token="test")
    m.update(Q, Q, 3.7, True, Q, 0)
    execute(m, "collection_begin", side="right", token="test")


def test_fault_can_only_retry_after_explicit_release_handshake(tmp_path):
    m = CollectionMotion(tmp_path / "pose.json")
    execute(m, "right_save"); execute(m, "left_lock"); execute(m, "right_return")
    m.update(Q, Q, 1.1, True, Q, 2)
    assert m.phase == "resetting" and m.leader_target is None
    m.update(Q, Q, 1.2, True, Q, 0)
    assert m.phase == "preparing" and m.leader_target is not None
    m.update(Q, Q, 1.3, True, Q, 1)
    assert m.phase == "moving"
    execute(m, "right_pause")
    wrong = list(Q); wrong[14] += .1
    execute(m, "right_follow", leader=wrong)
    m.update(Q, wrong, 1.4, True, wrong, 0)
    assert m.right is not None and m.leader_target is None and not m.returning


def test_old_pose_requires_resave_and_packet_carries_targets(tmp_path):
    path = tmp_path / "old.json"; path.write_text('{"schema_version": 1}')
    assert CollectionMotion(path).saved is None
    from remote_teleop_protocol import ActionCommand, FollowerState, ControlState, FaultBits, encode_action, encode_state, decode_message
    state = FollowerState(1, 1, 1, 1, 1, 1, 1, ControlState.RUNNING,
                          FaultBits(0), tuple(Q), (0.,)*16, (0.,)*16, 7, tuple(Q[8:]))
    assert decode_message(encode_state(state)) == state
    for ack in (0, 1, 2):
        command = ActionCommand(1, 1, 1, tuple(Q), 100000000, ack)
        assert decode_message(encode_action(command)) == command


def test_flags_roundtrip_and_detached_leader_receives_zero_haptics():
    from types import SimpleNamespace
    from remote_teleop_protocol import FollowerState, ControlState, FaultBits, encode_state, decode_message
    from remote_teleop_runtime.leader import LeaderGateway
    gateway = object.__new__(LeaderGateway)
    gateway.collection_flags = 0; gateway.haptic_reference = None
    gateway.enable_left = True; gateway.action_history = {1: tuple(Q)}
    left, right = [], []
    gateway.left_force_pub = SimpleNamespace(publish=left.append)
    gateway.right_force_pub = SimpleNamespace(publish=right.append)
    for flags in (0, 1, 3, 2, 0):
        state = FollowerState(1, 1, 1, 1, 1, 1, 1, ControlState.RUNNING,
                              FaultBits(0), tuple(Q), (0.,)*16, (0.,)*16, flags)
        state = decode_message(encode_state(state))
        assert state.collection_flags == flags
        gateway.publish_force_feedback(state)
        if flags & 1:
            assert list(left[-1].effort) == [0.]*8
        if flags & 2:
            assert list(right[-1].effort) == [0.]*8

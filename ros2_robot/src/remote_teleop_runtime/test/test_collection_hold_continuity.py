"""Trajectory interruption keeps the last commanded load/gripper preload."""
import pytest

from remote_teleop_runtime.collection_motion import CollectionMotion


Q = [0., 0., 0., .6, 0., 0., 0., -.4] * 2


def moving_return(tmp_path):
    motion = CollectionMotion(tmp_path / "absent-pose.json")
    motion.left = tuple(Q[:8])
    motion.saved = {"schema_version": 2, "robot": "OpenArm-v10-right",
                    "saved_unix_s": 1., "target": Q[8:], "actual": Q[8:],
                    "leader": Q[8:]}
    applied = Q[:]
    applied[11] += .13
    applied[15] -= .20
    motion.command("right_return", {}, Q, applied, applied, [0.]*16, 1., True)
    motion.update(Q, applied, 1.01, True, applied, 0)
    published = motion.update(Q, applied, 1.02, True, applied, 1)
    assert motion.phase == "moving"
    assert tuple(published[8:]) == motion.right
    # The joint and closed gripper remain under load rather than reaching
    # their commanded positions exactly.
    measured = published[:]
    measured[11] -= .13
    measured[15] += .20
    leader = Q[:]
    leader[8:] = motion.leader_target
    return motion, published, measured, leader


@pytest.mark.parametrize("reason", ["transport", "leader_fault", "pause", "tracking_error"])
def test_return_failure_keeps_previous_target_in_same_cycle_and_after_recovery(tmp_path, reason):
    motion, previous, measured, leader = moving_return(tmp_path)
    if reason == "pause":
        motion.command("right_pause", {}, measured, previous, leader, [0.]*16, 1.03, False)
        result = motion.update(measured, Q, 1.03, True, leader, 1)
    elif reason == "tracking_error":
        leader[12] += .30
        result = motion.update(measured, Q, 1.03, True, leader, 1)
    else:
        result = motion.update(measured, Q, 1.03, reason != "transport", leader,
                               2 if reason == "leader_fault" else 1)
    assert not motion.returning
    assert motion.phase == "hold"
    assert tuple(result[8:]) == tuple(previous[8:])
    assert tuple(result[:8]) == tuple(previous[:8])
    assert motion.right == tuple(previous[8:])

    measured[11] -= .03
    measured[15] += .05
    motion.interrupt(measured, "second stop boundary")
    recovered = motion.update(measured, Q, 1.04, True, leader, 1)
    assert recovered == previous
    assert not motion.returning  # Fresh transport never restarts the path.


def test_interrupt_uses_actual_only_when_no_return_target_exists(tmp_path):
    motion = CollectionMotion(tmp_path / "absent-pose.json")
    motion.returning = True
    motion.right = None
    motion.interrupt(Q, "startup cancellation")
    assert motion.right == tuple(Q[8:])
    assert motion.phase == "hold" and not motion.returning


@pytest.mark.parametrize("alignment", ["left_follower", "left_master"])
def test_left_alignment_failure_preserves_follower_hold_and_opposite_arm(tmp_path, alignment):
    motion = CollectionMotion(tmp_path / "absent-pose.json")
    previous = Q[:]
    previous[3] += .13
    previous[7] -= .20
    motion.left = tuple(previous[:8])
    motion.right = tuple(previous[8:])
    if alignment == "left_follower":
        motion.left_align_active = True
        motion.left_align_phase = "aligning"
    else:
        motion.left_master_align_active = True
        motion.left_master_align_phase = "moving"
        motion.left_master_target = tuple(Q[:8])
    motion.leader_now = tuple(Q)
    interrupted = motion.update(Q, Q, 1., False, Q, 0x10)
    assert interrupted == previous
    assert motion.left == tuple(previous[:8])
    assert motion.right == tuple(previous[8:])

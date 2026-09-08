import pytest

from remote_teleop_runtime.common import haptic_desired_axes


def test_arm_joints_keep_relative_run_reference():
    leader = [0.0] * 16
    follower = [0.1] * 16
    applied = [0.2] * 16

    desired = haptic_desired_axes(leader, follower, applied)

    assert desired[0:7] == pytest.approx([0.3] * 7)
    assert desired[8:15] == pytest.approx([0.3] * 7)


def test_grippers_use_absolute_applied_opening():
    leader = [0.0] * 16
    follower = [0.0] * 16
    applied = [0.0] * 16
    # Deliberately different openings when RUN begins.
    leader[7], follower[7], applied[7] = -0.1, -0.8, -0.2
    leader[15], follower[15], applied[15] = -0.9, -0.1, -0.7

    desired = haptic_desired_axes(leader, follower, applied)

    assert desired[7] == pytest.approx(-0.2)
    assert desired[15] == pytest.approx(-0.7)


def test_haptic_vectors_must_cover_both_arms_and_grippers():
    with pytest.raises(ValueError):
        haptic_desired_axes([0.0] * 15, [0.0] * 16, [0.0] * 16)

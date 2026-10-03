import pytest

from remote_teleop_runtime.common import haptic_desired_axes
from remote_teleop_runtime.common import gripper_contact_reference
from types import SimpleNamespace


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


def test_gripper_reference_only_when_running_fresh_and_following():
    s=SimpleNamespace(control_state=SimpleNamespace(name='RUNNING'),fault_bits=0,
        collection_flags=0,sender_monotonic_ns=100_000_000,obs_timestamp_ns=90_000_000,
        positions=[0.]*16)
    s.positions[7]=-.7; s.positions[15]=-.4
    assert gripper_contact_reference(s,'left')==[-.7]
    assert gripper_contact_reference(s,'right')==[-.4]
    s.collection_flags=1
    assert gripper_contact_reference(s,'left')==[]
    assert gripper_contact_reference(s,'right')==[-.4]
    s.collection_flags=6
    assert gripper_contact_reference(s,'right')==[]
    s.collection_flags=0; s.fault_bits=1
    assert gripper_contact_reference(s,'right')==[]
    s.fault_bits=0; s.control_state.name='READY'
    assert gripper_contact_reference(s,'right')==[]
    s.control_state.name='RUNNING'; s.obs_timestamp_ns=1
    assert gripper_contact_reference(s,'right')==[]
    s.obs_timestamp_ns=101_000_000
    assert gripper_contact_reference(s,'right')==[]
    s.obs_timestamp_ns=90_000_000; s.positions[15]=float('nan')
    assert gripper_contact_reference(s,'right')==[]

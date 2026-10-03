"""Exercise the real publish gate without creating a ROS node or transport."""
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from remote_teleop_runtime.collection_motion import CollectionMotion
from remote_teleop_runtime.follower import FollowerGateway

Q = [0., 0., 0., .6, 0., 0., 0., -.4]*2


@pytest.mark.parametrize("phase", ["moving", "releasing"])
def test_running_state_with_fault_bits_cannot_progress_or_release_left_master(tmp_path, phase):
    collection = CollectionMotion(tmp_path / "pose.json")
    collection.command("left_lock", {}, Q, Q, Q, [0.]*16, 1., True, leader_ack=0x10)
    collection.command("left_master_align", {}, Q, Q, Q, [0.]*16, 1., True, leader_ack=0x10)
    if phase == "releasing":
        collection.command("left_master_pause", {}, Q, Q, Q, [0.]*16, 1., True, leader_ack=0x10)
    else:
        collection.left_master_align_phase = "moving"
        collection.left_master_target = tuple(Q[:8])
    gateway = object.__new__(FollowerGateway)
    gateway.enable_left = True
    gateway.have_right_feedback = gateway.have_left_feedback = True
    gateway.positions = Q[:]
    gateway.collection = collection
    gateway.hold_left = tuple(Q[:7]); gateway.hold_right = tuple(Q[8:15])
    gateway.hold_left_gripper = Q[7]; gateway.hold_right_gripper = Q[15]
    gateway.safety = {"state": "RUNNING", "fault_bits": 1}
    now_ns = 1_010_000_000
    gateway.last_feedback_ns = gateway.safety_rx_ns = gateway.last_action_rx_ns = now_ns
    gateway.command_was_running = True
    gateway.run_leader_left = tuple(Q[:7]); gateway.run_leader_right = tuple(Q[8:15])
    gateway.latest_action = SimpleNamespace(axes=tuple(Q), collection_ack=0x10,
        left_arm=tuple(Q[:7]), left_gripper=Q[7], right_arm=tuple(Q[8:15]), right_gripper=Q[15],
        session_id=1, sequence=1)
    left_messages = []
    right_messages = []
    gateway.left_publisher = SimpleNamespace(publish=left_messages.append)
    gateway.right_publisher = SimpleNamespace(publish=right_messages.append)
    gateway.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    gateway.publish_target(now_ns)
    assert collection.left_master_align_phase == "failed"
    assert collection.left_master_align_active
    assert collection.left == tuple(Q[:8])
    assert gateway.applied_axes == tuple(Q)
    assert len(left_messages) == len(right_messages) == 1

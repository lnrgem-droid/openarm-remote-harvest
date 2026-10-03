"""Exercise the real hold/publish/heartbeat paths without ROS nodes or devices."""
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from remote_teleop_runtime.collection_motion import CollectionMotion
from remote_teleop_runtime.follower import FollowerGateway


def gateway(tmp_path):
    node = object.__new__(FollowerGateway)
    node.enable_left = True
    node.have_left_feedback = node.have_right_feedback = True
    # A loaded arm can have a substantial steady error while its target stays
    # fixed. These J4 values reproduce the recorded failure's target/actual gap.
    node.positions = [0., 0., 0., 1.329633, 0., 0., 0., -.45,
                      0., 0., 0., .60, 0., 0., 0., -.35]
    desired = node.positions[:]
    desired[3] = 1.459716
    desired[7] = -.70
    desired[11] = .72
    desired[15] = -.62
    node.latest_action = SimpleNamespace(
        axes=tuple(desired), collection_ack=0x10,
        left_arm=tuple(desired[:7]), left_gripper=desired[7],
        right_arm=tuple(desired[8:15]), right_gripper=desired[15],
        session_id=1, sequence=1)
    node.run_leader_left = tuple(desired[:7])
    node.run_leader_right = tuple(desired[8:15])
    node.hold_left = node.hold_right = None
    node.hold_left_gripper = node.hold_right_gripper = None
    node.applied_axes = None
    node.command_was_running = False
    node.last_action_rx_ns = node.last_feedback_ns = node.safety_rx_ns = 1_000_000_000
    node.safety = {"state": "RUNNING", "fault_bits": 0}
    node.session = 2
    node.hb_sequence = 0
    node.collection = CollectionMotion(tmp_path / "absent-pose.json")
    node.left_messages = []
    node.right_messages = []
    node.left_publisher = SimpleNamespace(publish=node.left_messages.append)
    node.right_publisher = SimpleNamespace(publish=node.right_messages.append)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    return node


def publish(node, now_ns):
    node.last_feedback_ns = node.safety_rx_ns = now_ns
    node.publish_target(now_ns)


def test_loaded_target_survives_local_stale_then_watchdog_fault(tmp_path):
    node = gateway(tmp_path)
    publish(node, 1_000_000_000)
    previous = node.applied_axes
    previous_left_message = list(node.left_messages[-1].position)
    previous_right_message = list(node.right_messages[-1].position)
    assert previous[3] - node.positions[3] == pytest.approx(.130083)

    # The action gate closes before the independent 150 ms watchdog timeout.
    publish(node, 1_101_000_000)
    assert not node.command_was_running
    assert node.applied_axes == previous

    node.positions[3] -= .0313
    node.positions[7] += .08
    node.positions[11] -= .04
    node.positions[15] += .10
    action = node.latest_action
    node.io = SimpleNamespace(snapshot=lambda: {
        "action": SimpleNamespace(action=action, safe_rx_ns=node.last_action_rx_ns, peer_ip="offline"),
        "safety": {"state": "FAULT", "fault_bits": 1, "reason": "leader action receive timeout"},
        "safety_rx_ns": 1_160_000_000})
    node.heartbeat(1_161_000_000)
    publish(node, 1_161_000_000)
    assert node.applied_axes == previous
    assert list(node.left_messages[-1].position) == previous_left_message
    assert list(node.right_messages[-1].position) == previous_right_message

    # Repeated FAULT publishing and later fresh leader packets cannot restart
    # FOLLOW or follow a sagging follower/gripper into another lower hold.
    node.positions[3] -= .05
    node.latest_action.left_arm = (0., 0., 0., 1.6, 0., 0., 0.)
    node.latest_action.left_gripper = 0.
    for now_ns in (1_200_000_000, 1_300_000_000):
        node.last_action_rx_ns = now_ns
        publish(node, now_ns)
        assert node.applied_axes == previous
        assert node.safety["state"] == "FAULT"
        assert not node.command_was_running


@pytest.mark.parametrize("gate", ["action", "feedback", "watchdog_reply", "fault_bits"])
def test_each_control_gate_preserves_the_last_published_target(tmp_path, gate):
    node = gateway(tmp_path)
    publish(node, 1_000_000_000)
    previous = node.applied_axes
    now_ns = 1_200_000_000
    node.last_action_rx_ns = node.last_feedback_ns = node.safety_rx_ns = now_ns
    if gate == "action":
        node.last_action_rx_ns = 1_000_000_000
    elif gate == "feedback":
        node.last_feedback_ns = 1_000_000_000
    elif gate == "watchdog_reply":
        node.safety_rx_ns = 1_000_000_000
    else:
        node.safety["fault_bits"] = 1
    node.positions[3] -= .04
    node.positions[7] += .12
    node.publish_target(now_ns)
    assert node.applied_axes == previous
    assert not node.command_was_running


def test_explicit_hold_or_reset_preserves_published_targets_and_grippers(tmp_path):
    node = gateway(tmp_path)
    publish(node, 1_000_000_000)
    previous = node.applied_axes
    node.positions = [value + .03 for value in node.positions]
    node.clear_run_reference()
    assert node.hold_left + (node.hold_left_gripper,) == previous[:8]
    assert node.hold_right + (node.hold_right_gripper,) == previous[8:]
    assert node.run_leader_left is node.run_leader_right is None
    node.capture_hold_reference()
    assert node.hold_left + (node.hold_left_gripper,) == previous[:8]


def test_startup_without_published_target_uses_measured_snapshot(tmp_path):
    node = gateway(tmp_path)
    original = tuple(node.positions)
    node.capture_hold_reference()
    node.positions = [value + .02 for value in node.positions]
    assert node.hold_left + (node.hold_left_gripper,) == original[:8]
    assert node.hold_right + (node.hold_right_gripper,) == original[8:]


@pytest.mark.parametrize("invalid", [None, (), (0.,) * 15, (float("nan"),) * 16,
                                     (float("inf"),) * 16, ("invalid",) * 16])
def test_invalid_or_missing_applied_never_overwrites_an_existing_finite_hold(tmp_path, invalid):
    node = gateway(tmp_path)
    publish(node, 1_000_000_000)
    node.capture_hold_reference()
    previous_left = node.hold_left + (node.hold_left_gripper,)
    previous_right = node.hold_right + (node.hold_right_gripper,)
    node.applied_axes = invalid
    node.positions = [value + .03 for value in node.positions]
    node.capture_hold_reference()
    assert node.hold_left + (node.hold_left_gripper,) == previous_left
    assert node.hold_right + (node.hold_right_gripper,) == previous_right


@pytest.mark.parametrize("side", ["left", "right"])
def test_existing_collection_hold_and_other_arm_target_are_preserved(tmp_path, side):
    node = gateway(tmp_path)
    publish(node, 1_000_000_000)
    previous = node.applied_axes
    held = previous[:8] if side == "left" else previous[8:]
    setattr(node.collection, side, held)
    node.positions = [value + .04 for value in node.positions]
    publish(node, 1_101_000_000)
    assert getattr(node.collection, side) == held
    assert node.applied_axes == previous


def test_right_only_follower_does_not_require_a_left_hold(tmp_path):
    node = gateway(tmp_path)
    node.enable_left = False
    publish(node, 1_000_000_000)
    previous_right = node.applied_axes[8:]
    node.positions[11] -= .04
    publish(node, 1_101_000_000)
    assert node.applied_axes[8:] == previous_right
    assert node.hold_left is None
    assert not node.left_messages

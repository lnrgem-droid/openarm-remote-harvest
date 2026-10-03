"""Leader-side routing tests: no ROS node, socket, or hardware is created."""

import threading
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import JointState

from remote_teleop_protocol import decode_message
from remote_teleop_protocol.protocol import decode_collection_ack
from remote_teleop_runtime import leader
from remote_teleop_runtime.common import GRIPPER_MAX_RAD, GRIPPER_OPEN_M


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(list(message.position))


class Socket:
    def __init__(self):
        self.sent = []

    def sendto(self, payload, address):
        self.sent.append((payload, address))

    def recvfrom(self, _size):
        raise BlockingIOError


@pytest.fixture
def gateway(monkeypatch):
    monkeypatch.setattr(leader.time, "monotonic", lambda: 10.0)
    monkeypatch.setattr(leader.time, "monotonic_ns", lambda: 10_000_000_000)
    gateway = object.__new__(leader.LeaderGateway)
    gateway.lock = threading.Lock()
    gateway.enable_left = True
    gateway.axes = [0.0] * 16
    gateway.have_left = gateway.have_right = True
    gateway.left_rx = gateway.right_rx = 10.0
    gateway.collection_ack = 0
    gateway.left_collection_ack = 0
    gateway.left_collection_mode_invalid = False
    gateway.return_requested = gateway.left_return_requested = False
    gateway.return_pub = Publisher()
    gateway.left_return_pub = Publisher()
    gateway.peer = "unit-test-peer"
    gateway.period = .004
    gateway.session = 1
    gateway.sequence = 0
    gateway.sock = Socket()
    gateway.action_history = {}
    gateway.sent = gateway.received = gateway.invalid = 0
    gateway.last_log = 10.0
    return gateway


def state(flags=9, **updates):
    values = dict(
        collection_flags=flags, control_state=SimpleNamespace(name="RUNNING"),
        fault_bits=0, sender_monotonic_ns=1_000_000_000,
        obs_timestamp_ns=990_000_000,
        leader_return_target=(.1, -.2, .3, -.4, .5, -.6, .7, -.8),
    )
    values.update(updates)
    return SimpleNamespace(**values)


def feedback(side, *, mode=0, omit=(), nonfinite=None):
    fields = {f"openarm_{side}_joint{i}": .01 * i for i in range(1, 8)}
    fields[f"openarm_{side}_finger_joint1"] = GRIPPER_OPEN_M / 2
    if mode is not None:
        fields[f"openarm_{side}_collection_mode"] = float(mode)
    for name in omit:
        fields.pop(f"openarm_{side}_{name}")
    if nonfinite:
        fields[f"openarm_{side}_{nonfinite}"] = float("nan")
    message = JointState()
    message.name = list(fields)
    message.position = list(fields.values())
    return message


@pytest.mark.parametrize("flags", [9, 11])
def test_left_target_and_gripper_never_reach_right_publisher(gateway, flags):
    packet = state(flags)
    gateway.publish_return(packet)
    assert gateway.left_return_pub.messages == [list(packet.leader_return_target)]
    assert gateway.return_pub.messages == []
    assert gateway.left_return_requested and not gateway.return_requested


@pytest.mark.parametrize("flags", [6, 7])
def test_right_target_keeps_its_existing_side(gateway, flags):
    packet = state(flags)
    gateway.publish_return(packet)
    assert gateway.return_pub.messages == [list(packet.leader_return_target)]
    assert gateway.left_return_pub.messages == []
    assert gateway.return_requested and not gateway.left_return_requested


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("ack", [1, 2])
def test_explicit_withdrawal_releases_only_that_side(gateway, side, ack):
    if side == "left":
        gateway.left_collection_ack = ack
        gateway.left_return_requested = True
    else:
        gateway.collection_ack = ack
        gateway.return_requested = True
    gateway.publish_return(state(3))
    assert gateway.left_return_pub.messages == ([[]] if side == "left" else [])
    assert gateway.return_pub.messages == ([[]] if side == "right" else [])
    assert not gateway.left_return_requested and not gateway.return_requested
    # Repeated release is necessary while the explicit physical ACK is busy.
    gateway.publish_return(state(3))
    selected = gateway.left_return_pub if side == "left" else gateway.return_pub
    assert selected.messages == [[], []]


def test_withdrawal_before_servo_ack_still_releases_requested_side(gateway):
    gateway.publish_return(state(9))
    gateway.publish_return(state(1))
    assert gateway.left_return_pub.messages[-1] == []
    assert not gateway.left_return_requested
    assert gateway.return_pub.messages == []


@pytest.mark.parametrize("flags", [4, 5, 8, 10, 12, 13, 14, 15, 16])
def test_conflicting_or_unheld_servo_flags_publish_nothing(gateway, flags):
    gateway.left_return_requested = True
    gateway.publish_return(state(flags))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []
    assert gateway.left_return_requested  # invalid input cannot release a hold


@pytest.mark.parametrize("flags", [0, 9, 6])
@pytest.mark.parametrize("updates", [
    {"fault_bits": 1},
    {"control_state": SimpleNamespace(name="READY")},
    {"control_state": SimpleNamespace(name="FAULT")},
    {"obs_timestamp_ns": 899_000_000},
    {"obs_timestamp_ns": 1_000_000_001},
])
def test_fault_or_stale_state_neither_moves_nor_releases(gateway, flags, updates):
    gateway.left_return_requested = True
    gateway.publish_return(state(flags, **updates))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []
    assert gateway.left_return_requested


@pytest.mark.parametrize("target", [(0.0,) * 7, (0.0,) * 9,
                                     (0.0,) * 7 + (float("nan"),),
                                     (0.0,) * 7 + (float("inf"),)])
def test_invalid_vector_never_reaches_either_master(gateway, target):
    gateway.publish_return(state(9, leader_return_target=target))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


@pytest.mark.parametrize("left_ack", [None, 2])
def test_missing_capability_or_latched_left_fault_cannot_move_left(gateway, left_ack):
    gateway.left_collection_ack = left_ack
    gateway.publish_return(state(9))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


def test_left_disabled_cannot_move_left(gateway):
    gateway.enable_left = False
    gateway.left_return_pub = None
    gateway.publish_return(state(9))
    assert gateway.return_pub.messages == []


@pytest.mark.parametrize("ack", [1, 2])
def test_busy_right_ack_cannot_be_mistaken_for_left_capability(gateway, ack):
    gateway.collection_ack = ack
    gateway.publish_return(state(9))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


@pytest.mark.parametrize("side", ["left", "right"])
def test_opposite_busy_master_blocks_target_routing(gateway, side):
    if side == "left":
        gateway.left_collection_ack = 1
    else:
        gateway.collection_ack = 1
    gateway.publish_return(state(6 if side == "left" else 9))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


@pytest.mark.parametrize("side", ["left", "right"])
def test_pending_servo_without_ack_also_blocks_opposite_side(gateway, side):
    if side == "left":
        gateway.left_return_requested = True
    else:
        gateway.return_requested = True
    gateway.publish_return(state(6 if side == "left" else 9))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


@pytest.mark.parametrize("side", ["left", "right"])
def test_stale_physical_feedback_blocks_target_and_release(gateway, side):
    setattr(gateway, f"{side}_rx", 9.89)
    gateway.publish_return(state(9))
    gateway.left_return_requested = True
    gateway.publish_return(state(1))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


@pytest.mark.parametrize("right", [0, 1, 2])
@pytest.mark.parametrize("left", [None, 0, 1, 2])
def test_action_packet_reports_separate_physical_modes(gateway, right, left):
    gateway.collection_ack = right
    gateway.left_collection_ack = left
    gateway.tick()
    packet = decode_message(gateway.sock.sent[0][0])
    assert decode_collection_ack(packet.collection_ack) == (right, left)
    if left is None:
        assert packet.collection_ack == right  # old controllers remain explicit


def test_legacy_right_only_controller_still_routes_right(gateway):
    gateway.left_collection_ack = None
    gateway.publish_return(state(6))
    assert len(gateway.return_pub.messages) == 1
    assert gateway.left_return_pub.messages == []


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("omit", [("joint3",), ("finger_joint1",)])
def test_incomplete_physical_feedback_does_not_refresh_axes_or_ack(gateway, side, omit):
    setattr(gateway, f"{side}_rx", 9.0)
    setattr(gateway, f"have_{side}", False)
    before = list(gateway.axes)
    gateway.on_joint_state(feedback(side, mode=1, omit=omit))
    assert getattr(gateway, f"{side}_rx") == 9.0
    assert not getattr(gateway, f"have_{side}")
    assert gateway.axes == before
    assert gateway.left_collection_ack == gateway.collection_ack == 0
    gateway.tick()
    assert gateway.sock.sent == []


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("field", ["joint3", "finger_joint1"])
def test_nonfinite_physical_feedback_is_not_fresh(gateway, side, field):
    setattr(gateway, f"{side}_rx", 9.0)
    gateway.on_joint_state(feedback(side, nonfinite=field))
    assert getattr(gateway, f"{side}_rx") == 9.0
    assert gateway.axes == [0.0] * 16


def test_right_only_joint_state_does_not_refresh_left_feedback(gateway):
    gateway.have_left = False
    gateway.left_rx = 9.0
    gateway.on_joint_state(feedback("right", mode=1))
    assert gateway.right_rx == 10.0
    assert gateway.collection_ack == 1
    assert not gateway.have_left and gateway.left_rx == 9.0
    assert gateway.axes[:8] == [0.0] * 8
    gateway.tick()
    assert gateway.sock.sent == []


@pytest.mark.parametrize("side", ["left", "right"])
def test_complete_feedback_updates_seven_axes_gripper_and_own_mode(gateway, side):
    gateway.on_joint_state(feedback(side, mode=1))
    offset = 0 if side == "left" else 8
    assert gateway.axes[offset:offset + 7] == pytest.approx([.01 * i for i in range(1, 8)])
    assert gateway.axes[offset + 7] == pytest.approx(GRIPPER_MAX_RAD / 2)
    assert gateway.left_collection_ack == (1 if side == "left" else 0)
    assert gateway.collection_ack == (1 if side == "right" else 0)


@pytest.mark.parametrize("mode", [None, .5, 3, -1, float("nan"), float("inf")])
def test_invalid_or_missing_left_mode_removes_capability(gateway, mode):
    gateway.on_joint_state(feedback("left", mode=mode))
    assert gateway.left_collection_ack is None
    assert decode_collection_ack(gateway.collection_acknowledgement()) == (0, None)
    gateway.publish_return(state(9))
    assert gateway.left_return_pub.messages == []


@pytest.mark.parametrize("mode", [.5, 3, -1, float("nan"), float("inf")])
def test_explicit_invalid_left_mode_also_blocks_new_right_motion(gateway, mode):
    gateway.on_joint_state(feedback("left", mode=mode))
    gateway.publish_return(state(6))
    assert gateway.return_pub.messages == gateway.left_return_pub.messages == []


@pytest.mark.parametrize("mode", [None, .5, 3, -1, float("nan"), float("inf")])
def test_invalid_or_missing_right_mode_is_failure_not_free(gateway, mode):
    gateway.on_joint_state(feedback("right", mode=mode))
    assert gateway.collection_ack == 2
    gateway.publish_return(state(9))
    assert gateway.left_return_pub.messages == []

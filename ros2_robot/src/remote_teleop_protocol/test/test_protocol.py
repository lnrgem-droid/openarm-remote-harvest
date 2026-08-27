import math

import pytest

from remote_teleop_protocol.protocol import (
    AXIS_COUNT,
    ActionCommand,
    ControlState,
    FaultBits,
    FollowerState,
    PacketError,
    SequenceTracker,
    decode_message,
    encode_action,
    encode_state,
)


def test_action_round_trip_and_order():
    axes = tuple(float(index) / 10.0 for index in range(AXIS_COUNT))
    source = ActionCommand(7, 11, 123_456, axes, 100_000_000)
    decoded = decode_message(encode_action(source))
    assert decoded == source
    assert decoded.left_arm == axes[:7]
    assert decoded.left_gripper == axes[7]
    assert decoded.right_arm == axes[8:15]
    assert decoded.right_gripper == axes[15]


def test_state_round_trip():
    source = FollowerState(
        session_id=8,
        sequence=12,
        sender_monotonic_ns=500,
        obs_timestamp_ns=480,
        action_timestamp_ns=490,
        applied_action_session_id=7,
        applied_action_sequence=10,
        control_state=ControlState.ALIGNING,
        fault_bits=FaultBits.NETWORK_TIMEOUT | FaultBits.LOCAL_WATCHDOG,
        positions=(0.0,) * AXIS_COUNT,
        velocities=(0.1,) * AXIS_COUNT,
    )
    assert decode_message(encode_state(source)) == source


def test_crc_corruption_is_rejected():
    packet = bytearray(
        encode_action(ActionCommand(1, 1, 1, (0.0,) * AXIS_COUNT, 1_000_000))
    )
    packet[-1] ^= 0x01
    with pytest.raises(PacketError, match="CRC32"):
        decode_message(bytes(packet))


@pytest.mark.parametrize(
    "axes",
    [
        (0.0,) * (AXIS_COUNT - 1),
        (0.0,) * (AXIS_COUNT - 1) + (math.nan,),
        (0.0,) * (AXIS_COUNT - 1) + (math.inf,),
    ],
)
def test_invalid_axis_vectors_are_rejected(axes):
    with pytest.raises(PacketError):
        ActionCommand(1, 1, 1, axes, 1)


def test_sequence_tracker_handles_restart_and_reordering():
    tracker = SequenceTracker()
    assert tracker.accept(100, 1)
    assert tracker.accept(100, 2)
    assert not tracker.accept(100, 2)
    assert not tracker.accept(100, 1)
    assert not tracker.accept(200, 1)
    tracker.reset()
    assert tracker.accept(200, 1)


def test_applied_action_identity_is_consistent():
    with pytest.raises(PacketError, match="all be zero or nonzero"):
        FollowerState(
            session_id=8,
            sequence=1,
            sender_monotonic_ns=500,
            obs_timestamp_ns=480,
            action_timestamp_ns=490,
            applied_action_session_id=0,
            applied_action_sequence=10,
            control_state=ControlState.READY,
            fault_bits=FaultBits.NONE,
            positions=(0.0,) * AXIS_COUNT,
            velocities=(0.0,) * AXIS_COUNT,
        )


@pytest.mark.parametrize("session_id,sequence", [(0, 1), (1, 0)])
def test_action_identity_must_be_nonzero(session_id, sequence):
    with pytest.raises(PacketError, match="greater than zero"):
        ActionCommand(session_id, sequence, 1, (0.0,) * AXIS_COUNT, 1)


def test_fault_bit_assignments_are_wire_compatible():
    assert int(FaultBits.NETWORK_TIMEOUT) == 1
    assert int(FaultBits.CAN_ERROR) == 2
    assert int(FaultBits.CONTROL_OVERRUN) == 4
    assert int(FaultBits.INVALID_COMMAND) == 8
    assert int(FaultBits.LOCAL_WATCHDOG) == 16
    assert int(FaultBits.E_STOP_ACTIVE) == 32
    assert int(FaultBits.CONTROL_PROCESS_TIMEOUT) == 64
    assert int(FaultBits.CONTROL_CYCLE_TIMEOUT) == 128

import pytest

from remote_teleop_protocol.protocol import (
    ActionCommand, FollowerState, ControlState, FaultBits, PacketError,
    COLLECTION_ACK_VALUES, COLLECTION_ACK_LEFT_CAPABLE, decode_collection_ack,
    encode_action, encode_state, decode_message,
)


@pytest.mark.parametrize("ack", COLLECTION_ACK_VALUES)
def test_extended_ack_roundtrip_preserves_side_and_capability(ack):
    command = ActionCommand(1, 1, 1, (0.,)*16, 100_000_000, ack)
    assert decode_message(encode_action(command)) == command
    right, left = decode_collection_ack(ack)
    assert right == ack & 3
    assert left == ((ack >> 2) & 3 if ack & COLLECTION_ACK_LEFT_CAPABLE else None)


@pytest.mark.parametrize("ack", [3, 4, 8, 12, 15, 19, 23, 27, 28, 32, 256])
def test_unknown_reserved_or_missing_capability_modes_are_rejected(ack):
    with pytest.raises(PacketError):
        ActionCommand(1, 1, 1, (0.,)*16, 1, ack)


def state(flags):
    return FollowerState(1, 1, 10, 9, 0, 0, 0, ControlState.RUNNING,
                         FaultBits.NONE, (0.,)*16, (0.,)*16,
                         collection_flags=flags, leader_return_target=tuple(i/10 for i in range(8)))


@pytest.mark.parametrize("flags", [0, 1, 2, 3, 6, 7, 9, 11])
def test_servo_target_legal_flags_roundtrip(flags):
    source = state(flags)
    assert decode_message(encode_state(source)) == source


@pytest.mark.parametrize("flags", [4, 5, 8, 10, 12, 13, 14, 15, 16])
def test_target_without_own_follower_hold_or_two_servos_is_rejected(flags):
    with pytest.raises(PacketError):
        state(flags)

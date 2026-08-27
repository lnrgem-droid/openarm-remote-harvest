"""OpenArm remote teleoperation wire protocol v1.

This module deliberately has no ROS, CAN, OpenArm, NumPy, or clock-synchronization
dependency. It only validates and serializes packets. All integers use network byte
order and all timestamps are unsigned nanoseconds from the sender's monotonic clock.
Monotonic timestamps from different hosts must never be directly subtracted.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
import struct
from typing import Iterable, Union
import zlib


MAGIC = b"OARM"
PROTOCOL_VERSION = 1
JOINTS_PER_ARM = 7
AXES_PER_SIDE = JOINTS_PER_ARM + 1  # seven arm joints plus one gripper joint
AXIS_COUNT = AXES_PER_SIDE * 2
MAX_DATAGRAM_SIZE = 1200

# magic, version, type, flags, session, sequence, sender monotonic time,
# payload length, crc32. CRC is calculated with the crc field set to zero.
_HEADER = struct.Struct("!4sBBHQQQII")
_ACTION = struct.Struct("!16dQ")
# obs time, applied-action time, applied session, applied sequence, state,
# fault bits, padding,
# then 16 positions and 16 velocities.
_STATE_PREFIX = struct.Struct("!QQQQBI3x")
_STATE_AXES = struct.Struct("!32d")


class PacketError(ValueError):
    """Raised when a datagram violates protocol v1."""


class MessageType(IntEnum):
    ACTION = 1
    FOLLOWER_STATE = 2


class ControlState(IntEnum):
    INIT = 0
    ALIGNING = 1
    READY = 2
    RUNNING = 3
    FAULT = 4
    E_STOP = 5


class FaultBits(IntFlag):
    NONE = 0
    NETWORK_TIMEOUT = 1 << 0
    CAN_ERROR = 1 << 1
    CONTROL_OVERRUN = 1 << 2
    INVALID_COMMAND = 1 << 3
    LOCAL_WATCHDOG = 1 << 4
    E_STOP_ACTIVE = 1 << 5


def _axis_tuple(values: Iterable[float], field: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != AXIS_COUNT:
        raise PacketError(f"{field} must contain exactly {AXIS_COUNT} values")
    if not all(math.isfinite(value) for value in result):
        raise PacketError(f"{field} contains NaN or infinity")
    return result


def _uint64(value: int, field: str) -> int:
    value = int(value)
    if not 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        raise PacketError(f"{field} is outside uint64 range")
    return value


def _nonzero_uint64(value: int, field: str) -> int:
    value = _uint64(value, field)
    if value == 0:
        raise PacketError(f"{field} must be greater than zero")
    return value


@dataclass(frozen=True)
class ActionCommand:
    session_id: int
    sequence: int
    sender_monotonic_ns: int
    axes: tuple[float, ...]
    valid_for_ns: int

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "session_id", _nonzero_uint64(self.session_id, "session_id")
        )
        object.__setattr__(self, "sequence", _nonzero_uint64(self.sequence, "sequence"))
        object.__setattr__(
            self,
            "sender_monotonic_ns",
            _nonzero_uint64(self.sender_monotonic_ns, "sender_monotonic_ns"),
        )
        object.__setattr__(self, "axes", _axis_tuple(self.axes, "axes"))
        object.__setattr__(self, "valid_for_ns", _uint64(self.valid_for_ns, "valid_for_ns"))
        if self.valid_for_ns == 0:
            raise PacketError("valid_for_ns must be greater than zero")

    @property
    def left_arm(self) -> tuple[float, ...]:
        return self.axes[0:7]

    @property
    def left_gripper(self) -> float:
        return self.axes[7]

    @property
    def right_arm(self) -> tuple[float, ...]:
        return self.axes[8:15]

    @property
    def right_gripper(self) -> float:
        return self.axes[15]


@dataclass(frozen=True)
class FollowerState:
    session_id: int
    sequence: int
    sender_monotonic_ns: int
    obs_timestamp_ns: int
    action_timestamp_ns: int
    applied_action_session_id: int
    applied_action_sequence: int
    control_state: ControlState
    fault_bits: FaultBits
    positions: tuple[float, ...]
    velocities: tuple[float, ...]

    def __post_init__(self) -> None:
        for field in ("session_id", "sequence", "sender_monotonic_ns", "obs_timestamp_ns"):
            object.__setattr__(self, field, _nonzero_uint64(getattr(self, field), field))
        for field in (
            "action_timestamp_ns",
            "applied_action_session_id",
            "applied_action_sequence",
        ):
            object.__setattr__(self, field, _uint64(getattr(self, field), field))
        has_applied_action = self.action_timestamp_ns != 0
        has_applied_identity = (
            self.applied_action_session_id != 0 and self.applied_action_sequence != 0
        )
        if has_applied_action != has_applied_identity:
            raise PacketError(
                "action timestamp, applied session, and applied sequence must all be zero or nonzero"
            )
        object.__setattr__(self, "control_state", ControlState(self.control_state))
        object.__setattr__(self, "fault_bits", FaultBits(self.fault_bits))
        object.__setattr__(self, "positions", _axis_tuple(self.positions, "positions"))
        object.__setattr__(self, "velocities", _axis_tuple(self.velocities, "velocities"))


Message = Union[ActionCommand, FollowerState]


def _encode(message_type: MessageType, session_id: int, sequence: int,
            sender_monotonic_ns: int, payload: bytes) -> bytes:
    if len(payload) + _HEADER.size > MAX_DATAGRAM_SIZE:
        raise PacketError("datagram exceeds the protocol MTU budget")
    header_zero_crc = _HEADER.pack(
        MAGIC,
        PROTOCOL_VERSION,
        int(message_type),
        0,
        _uint64(session_id, "session_id"),
        _uint64(sequence, "sequence"),
        _uint64(sender_monotonic_ns, "sender_monotonic_ns"),
        len(payload),
        0,
    )
    checksum = zlib.crc32(header_zero_crc + payload) & 0xFFFFFFFF
    header = _HEADER.pack(
        MAGIC,
        PROTOCOL_VERSION,
        int(message_type),
        0,
        session_id,
        sequence,
        sender_monotonic_ns,
        len(payload),
        checksum,
    )
    return header + payload


def encode_action(command: ActionCommand) -> bytes:
    payload = _ACTION.pack(*command.axes, command.valid_for_ns)
    return _encode(
        MessageType.ACTION,
        command.session_id,
        command.sequence,
        command.sender_monotonic_ns,
        payload,
    )


def encode_state(state: FollowerState) -> bytes:
    prefix = _STATE_PREFIX.pack(
        state.obs_timestamp_ns,
        state.action_timestamp_ns,
        state.applied_action_session_id,
        state.applied_action_sequence,
        int(state.control_state),
        int(state.fault_bits),
    )
    payload = prefix + _STATE_AXES.pack(*state.positions, *state.velocities)
    return _encode(
        MessageType.FOLLOWER_STATE,
        state.session_id,
        state.sequence,
        state.sender_monotonic_ns,
        payload,
    )


def decode_message(datagram: bytes) -> Message:
    if len(datagram) < _HEADER.size:
        raise PacketError("datagram is shorter than the v1 header")
    if len(datagram) > MAX_DATAGRAM_SIZE:
        raise PacketError("datagram exceeds the protocol MTU budget")

    magic, version, raw_type, flags, session_id, sequence, sender_ns, length, checksum = (
        _HEADER.unpack_from(datagram)
    )
    if magic != MAGIC:
        raise PacketError("bad protocol magic")
    if version != PROTOCOL_VERSION:
        raise PacketError(f"unsupported protocol version {version}")
    if flags != 0:
        raise PacketError("v1 flags must be zero")
    if len(datagram) != _HEADER.size + length:
        raise PacketError("payload length does not match datagram length")

    header_zero_crc = _HEADER.pack(
        magic, version, raw_type, flags, session_id, sequence, sender_ns, length, 0
    )
    payload = datagram[_HEADER.size:]
    expected_checksum = zlib.crc32(header_zero_crc + payload) & 0xFFFFFFFF
    if checksum != expected_checksum:
        raise PacketError("CRC32 mismatch")

    try:
        message_type = MessageType(raw_type)
    except ValueError as exc:
        raise PacketError(f"unknown message type {raw_type}") from exc

    if message_type is MessageType.ACTION:
        if len(payload) != _ACTION.size:
            raise PacketError("invalid ACTION payload size")
        values = _ACTION.unpack(payload)
        return ActionCommand(session_id, sequence, sender_ns, values[:16], values[16])

    if len(payload) != _STATE_PREFIX.size + _STATE_AXES.size:
        raise PacketError("invalid FOLLOWER_STATE payload size")
    obs_ns, action_ns, applied_session, applied_seq, raw_state, raw_faults = (
        _STATE_PREFIX.unpack_from(payload)
    )
    axes = _STATE_AXES.unpack_from(payload, _STATE_PREFIX.size)
    try:
        control_state = ControlState(raw_state)
    except ValueError as exc:
        raise PacketError(f"unknown control state {raw_state}") from exc
    return FollowerState(
        session_id=session_id,
        sequence=sequence,
        sender_monotonic_ns=sender_ns,
        obs_timestamp_ns=obs_ns,
        action_timestamp_ns=action_ns,
        applied_action_session_id=applied_session,
        applied_action_sequence=applied_seq,
        control_state=control_state,
        fault_bits=FaultBits(raw_faults),
        positions=axes[:16],
        velocities=axes[16:],
    )


class SequenceTracker:
    """Reject duplicate, out-of-order, and unexpected-session packets.

    A caller must explicitly call reset only after its safety state machine has left
    RUNNING and returned to ALIGNING. This prevents a delayed packet from an old or
    restarted process from silently replacing the active sender session.
    """

    def __init__(self) -> None:
        self.session_id: int | None = None
        self.last_sequence: int | None = None

    def accept(self, session_id: int, sequence: int) -> bool:
        session_id = _nonzero_uint64(session_id, "session_id")
        sequence = _nonzero_uint64(sequence, "sequence")
        if self.session_id is None:
            self.session_id = session_id
            self.last_sequence = sequence
            return True
        if session_id != self.session_id:
            return False
        if self.last_sequence is not None and sequence <= self.last_sequence:
            return False
        self.last_sequence = sequence
        return True

    def reset(self) -> None:
        self.session_id = None
        self.last_sequence = None

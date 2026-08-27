"""Latched follower safety state machine with no hardware dependencies."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from remote_teleop_protocol import ControlState, FaultBits


class TransitionError(RuntimeError):
    """Raised when an unsafe or out-of-order transition is requested."""


class SafetyReaction(str, Enum):
    UNDECIDED = "undecided"
    POSITION_HOLD = "position_hold"
    CONTROLLED_UNLOAD = "controlled_unload"
    DRIVE_DISABLE = "drive_disable"


@dataclass(frozen=True)
class SafetySnapshot:
    state: ControlState
    fault_bits: FaultBits
    reason: str
    leader_session_id: int | None
    aligned: bool
    reaction: SafetyReaction
    reaction_verified: bool


class SafetyStateMachine:
    """Controls lifecycle transitions and latches every safety fault.

    Faults never recover automatically. `reset_fault` always returns to ALIGNING,
    clears alignment/session authorization, and therefore cannot resume motion.
    """

    def __init__(
        self,
        reaction: SafetyReaction = SafetyReaction.UNDECIDED,
        reaction_verified: bool = False,
    ) -> None:
        self._state = ControlState.INIT
        self._fault_bits = FaultBits.NONE
        self._reason = "process initialized"
        self._leader_session_id: int | None = None
        self._aligned = False
        self._reaction = SafetyReaction(reaction)
        self._reaction_verified = bool(reaction_verified)

    @property
    def state(self) -> ControlState:
        return self._state

    @property
    def leader_session_id(self) -> int | None:
        return self._leader_session_id

    def snapshot(self) -> SafetySnapshot:
        return SafetySnapshot(
            state=self._state,
            fault_bits=self._fault_bits,
            reason=self._reason,
            leader_session_id=self._leader_session_id,
            aligned=self._aligned,
            reaction=self._reaction,
            reaction_verified=self._reaction_verified,
        )

    def boot(self) -> None:
        if self._state is not ControlState.INIT:
            raise TransitionError("boot is valid only from INIT")
        self._state = ControlState.ALIGNING
        self._reason = "restart requires alignment"

    def observe_leader_session(self, session_id: int) -> bool:
        session_id = self._validate_session(session_id)
        if self._state in (ControlState.FAULT, ControlState.E_STOP):
            return False
        if self._leader_session_id is None:
            self._leader_session_id = session_id
            return True
        if session_id == self._leader_session_id:
            return True
        if self._state is ControlState.RUNNING:
            self.trip(FaultBits.INVALID_COMMAND, "leader session changed while RUNNING")
            return False
        self._leader_session_id = session_id
        self._aligned = False
        self._state = ControlState.ALIGNING
        self._reason = "leader session changed; alignment required"
        return True

    def alignment_complete(self, session_id: int) -> None:
        session_id = self._validate_session(session_id)
        if self._state is not ControlState.ALIGNING:
            raise TransitionError("alignment completion is valid only from ALIGNING")
        if self._leader_session_id != session_id:
            raise TransitionError("alignment session does not match active leader session")
        self._aligned = True
        self._state = ControlState.READY
        self._reason = "alignment complete; explicit RUN request required"

    def request_run(self, session_id: int) -> None:
        session_id = self._validate_session(session_id)
        if self._state is not ControlState.READY:
            raise TransitionError("RUN is valid only from READY")
        if not self._aligned or self._leader_session_id != session_id:
            raise TransitionError("RUN requires alignment with the active leader session")
        if self._reaction is SafetyReaction.UNDECIDED or not self._reaction_verified:
            raise TransitionError("RUN blocked until the physical safety reaction is verified")
        if self._fault_bits != FaultBits.NONE:
            raise TransitionError("RUN blocked while a fault is latched")
        self._state = ControlState.RUNNING
        self._reason = "explicit RUN accepted"

    def trip(self, fault: FaultBits, reason: str) -> None:
        fault = FaultBits(fault)
        if fault == FaultBits.NONE:
            raise ValueError("trip requires a nonzero fault bit")
        self._fault_bits |= fault
        self._aligned = False
        self._reason = str(reason)
        if fault & FaultBits.E_STOP_ACTIVE or self._state is ControlState.E_STOP:
            self._state = ControlState.E_STOP
        else:
            self._state = ControlState.FAULT

    def reset_fault(self, *, estop_released: bool) -> None:
        if self._state not in (ControlState.FAULT, ControlState.E_STOP):
            raise TransitionError("reset is valid only from FAULT or E_STOP")
        if self._state is ControlState.E_STOP and not estop_released:
            raise TransitionError("cannot reset while emergency stop remains active")
        self._fault_bits = FaultBits.NONE
        self._leader_session_id = None
        self._aligned = False
        self._state = ControlState.ALIGNING
        self._reason = "fault reset; fresh session and alignment required"

    @staticmethod
    def _validate_session(session_id: int) -> int:
        if type(session_id) is not int:
            raise ValueError("session_id must be an integer")
        if not 0 < session_id <= 0xFFFFFFFFFFFFFFFF:
            raise ValueError("session_id must be a nonzero uint64")
        return session_id

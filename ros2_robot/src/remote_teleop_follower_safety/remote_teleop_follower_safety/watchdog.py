"""Jetson-local watchdog core, designed to run outside the control process."""

from __future__ import annotations

from dataclasses import dataclass

from remote_teleop_protocol import ControlState, FaultBits, SequenceTracker

from .state_machine import SafetyStateMachine


@dataclass(frozen=True)
class WatchdogConfig:
    startup_grace_ns: int = 2_000_000_000
    # The network action timeout remains the fast 150 ms safety boundary.
    # Local Python scheduling gets a larger bound because the physical C++ CAN
    # controller independently keeps its last position target while the Jetson
    # finalizes an RGB-D episode.
    control_heartbeat_timeout_ns: int = 300_000_000
    control_cycle_timeout_ns: int = 250_000_000
    network_action_timeout_ns: int = 150_000_000
    max_consecutive_overruns: int = 5

    def __post_init__(self) -> None:
        for field in (
            "startup_grace_ns",
            "control_heartbeat_timeout_ns",
            "control_cycle_timeout_ns",
            "network_action_timeout_ns",
            "max_consecutive_overruns",
        ):
            value = getattr(self, field)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{field} must be greater than zero")


@dataclass(frozen=True)
class ControllerHeartbeat:
    controller_session_id: int
    sequence: int
    sent_monotonic_ns: int
    last_control_cycle_ns: int
    last_action_rx_ns: int
    leader_session_id: int
    can_ok: bool
    estop_active: bool
    consecutive_overruns: int = 0

    def __post_init__(self) -> None:
        for field in (
            "controller_session_id",
            "sequence",
            "sent_monotonic_ns",
            "last_control_cycle_ns",
        ):
            value = getattr(self, field)
            if type(value) is not int:
                raise ValueError(f"{field} must be an integer")
            if not 0 < value <= 0xFFFFFFFFFFFFFFFF:
                raise ValueError(f"{field} must be a nonzero uint64")
        for field in ("last_action_rx_ns", "leader_session_id"):
            value = getattr(self, field)
            if type(value) is not int:
                raise ValueError(f"{field} must be an integer")
            if not 0 <= value <= 0xFFFFFFFFFFFFFFFF:
                raise ValueError(f"{field} must be a uint64")
        if (self.last_action_rx_ns == 0) != (self.leader_session_id == 0):
            raise ValueError("last action time and leader session must both be zero or nonzero")
        if type(self.can_ok) is not bool or type(self.estop_active) is not bool:
            raise ValueError("can_ok and estop_active must be booleans")
        if type(self.consecutive_overruns) is not int or self.consecutive_overruns < 0:
            raise ValueError("consecutive_overruns must not be negative")
        if self.last_control_cycle_ns > self.sent_monotonic_ns:
            raise ValueError("last control cycle cannot be later than heartbeat send time")
        if self.last_action_rx_ns > self.sent_monotonic_ns:
            raise ValueError("last action receive cannot be later than heartbeat send time")


class WatchdogSupervisor:
    """Evaluates controller liveness and feeds a latched safety state machine."""

    def __init__(self, machine: SafetyStateMachine, config: WatchdogConfig) -> None:
        self.machine = machine
        self.config = config
        self._started_ns: int | None = None
        self._last_heartbeat_rx_ns: int | None = None
        self._latest: ControllerHeartbeat | None = None
        self._controller_sequences = SequenceTracker()

    def boot(self, now_ns: int) -> None:
        now_ns = self._time(now_ns)
        self.machine.boot()
        self._started_ns = now_ns

    def receive_heartbeat(self, heartbeat: ControllerHeartbeat, receive_ns: int) -> bool:
        receive_ns = self._time(receive_ns)
        if self._started_ns is None:
            raise RuntimeError("watchdog must boot before receiving heartbeats")
        if heartbeat.sent_monotonic_ns > receive_ns:
            self.machine.trip(FaultBits.INVALID_COMMAND, "controller heartbeat is from the future")
            return False
        if not self._controller_sequences.accept(
            heartbeat.controller_session_id, heartbeat.sequence
        ):
            if heartbeat.controller_session_id != self._controller_sequences.session_id:
                self.machine.trip(
                    FaultBits.LOCAL_WATCHDOG,
                    "controller process session changed; explicit reset required",
                )
            return False

        self._last_heartbeat_rx_ns = receive_ns
        self._latest = heartbeat

        if heartbeat.estop_active:
            self.machine.trip(FaultBits.E_STOP_ACTIVE, "local emergency stop active")
            return True
        if not heartbeat.can_ok:
            self.machine.trip(FaultBits.CAN_ERROR, "controller reports CAN failure")
            return True
        if heartbeat.consecutive_overruns >= self.config.max_consecutive_overruns:
            self.machine.trip(
                FaultBits.CONTROL_OVERRUN,
                f"controller reports {heartbeat.consecutive_overruns} consecutive overruns",
            )
            return True
        if receive_ns - heartbeat.last_control_cycle_ns > self.config.control_cycle_timeout_ns:
            self.machine.trip(
                FaultBits.LOCAL_WATCHDOG | FaultBits.CONTROL_CYCLE_TIMEOUT,
                "fresh process heartbeat reports a stale local control cycle",
            )
            return True
        if heartbeat.leader_session_id:
            self.machine.observe_leader_session(heartbeat.leader_session_id)
        return True

    def check(self, now_ns: int) -> None:
        now_ns = self._time(now_ns)
        if self._started_ns is None:
            raise RuntimeError("watchdog must boot before checks")
        if self.machine.state in (ControlState.FAULT, ControlState.E_STOP):
            return
        if self._last_heartbeat_rx_ns is None:
            if now_ns - self._started_ns > self.config.startup_grace_ns:
                self.machine.trip(
                    FaultBits.LOCAL_WATCHDOG,
                    "controller heartbeat missing after startup grace",
                )
            return
        if now_ns - self._last_heartbeat_rx_ns > self.config.control_heartbeat_timeout_ns:
            self.machine.trip(
                FaultBits.LOCAL_WATCHDOG | FaultBits.CONTROL_PROCESS_TIMEOUT,
                "controller process heartbeat timeout",
            )
            return
        assert self._latest is not None
        if self.machine.state is ControlState.RUNNING:
            if self._latest.last_action_rx_ns == 0:
                self.machine.trip(FaultBits.NETWORK_TIMEOUT, "no leader action received")
                return
            if now_ns - self._latest.last_action_rx_ns > self.config.network_action_timeout_ns:
                self.machine.trip(FaultBits.NETWORK_TIMEOUT, "leader action receive timeout")

    def reset_fault(self, now_ns: int, *, estop_released: bool) -> None:
        now_ns = self._time(now_ns)
        self.machine.reset_fault(estop_released=estop_released)
        self._controller_sequences.reset()
        self._last_heartbeat_rx_ns = None
        self._latest = None
        self._started_ns = now_ns

    @staticmethod
    def _time(value: int) -> int:
        value = int(value)
        if value <= 0:
            raise ValueError("monotonic time must be greater than zero")
        return value

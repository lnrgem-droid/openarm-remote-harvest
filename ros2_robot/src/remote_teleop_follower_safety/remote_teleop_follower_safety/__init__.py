"""Jetson-local follower safety primitives."""

from .state_machine import (
    SafetyReaction,
    SafetySnapshot,
    SafetyStateMachine,
    TransitionError,
)
from .watchdog import ControllerHeartbeat, WatchdogConfig, WatchdogSupervisor

__all__ = [
    "ControllerHeartbeat",
    "SafetyReaction",
    "SafetySnapshot",
    "SafetyStateMachine",
    "TransitionError",
    "WatchdogConfig",
    "WatchdogSupervisor",
]

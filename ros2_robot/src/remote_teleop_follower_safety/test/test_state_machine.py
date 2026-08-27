import pytest

from remote_teleop_protocol import ControlState, FaultBits
from remote_teleop_follower_safety.state_machine import (
    SafetyReaction,
    SafetyStateMachine,
    TransitionError,
)


def verified_machine():
    machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, reaction_verified=True)
    machine.boot()
    return machine


def test_restart_always_enters_aligning():
    machine = SafetyStateMachine()
    assert machine.state is ControlState.INIT
    machine.boot()
    assert machine.state is ControlState.ALIGNING


def test_unverified_reaction_blocks_running():
    machine = SafetyStateMachine()
    machine.boot()
    assert machine.observe_leader_session(10)
    machine.alignment_complete(10)
    with pytest.raises(TransitionError, match="physical safety reaction"):
        machine.request_run(10)
    assert machine.state is ControlState.READY


def test_explicit_happy_path():
    machine = verified_machine()
    assert machine.observe_leader_session(10)
    machine.alignment_complete(10)
    assert machine.state is ControlState.READY
    machine.request_run(10)
    assert machine.state is ControlState.RUNNING


def test_hold_requires_another_explicit_run():
    machine = verified_machine()
    machine.observe_leader_session(10)
    machine.alignment_complete(10)
    machine.request_run(10)
    machine.request_hold()
    snapshot = machine.snapshot()
    assert snapshot.state is ControlState.READY
    assert snapshot.aligned
    machine.request_run(10)
    assert machine.state is ControlState.RUNNING


def test_leader_session_change_while_running_latches_fault():
    machine = verified_machine()
    machine.observe_leader_session(10)
    machine.alignment_complete(10)
    machine.request_run(10)
    assert not machine.observe_leader_session(11)
    snapshot = machine.snapshot()
    assert snapshot.state is ControlState.FAULT
    assert snapshot.fault_bits & FaultBits.INVALID_COMMAND


def test_reset_never_resumes_and_requires_fresh_alignment():
    machine = verified_machine()
    machine.observe_leader_session(10)
    machine.alignment_complete(10)
    machine.request_run(10)
    machine.trip(FaultBits.NETWORK_TIMEOUT, "test")
    machine.reset_fault(estop_released=True)
    snapshot = machine.snapshot()
    assert snapshot.state is ControlState.ALIGNING
    assert snapshot.leader_session_id is None
    assert not snapshot.aligned
    with pytest.raises(TransitionError):
        machine.request_run(10)


def test_estop_requires_release_before_reset():
    machine = verified_machine()
    machine.trip(FaultBits.E_STOP_ACTIVE, "pressed")
    assert machine.state is ControlState.E_STOP
    with pytest.raises(TransitionError, match="remains active"):
        machine.reset_fault(estop_released=False)
    machine.reset_fault(estop_released=True)
    assert machine.state is ControlState.ALIGNING

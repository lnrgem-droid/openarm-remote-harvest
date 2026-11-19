"""A stop request racing a shutdown fault must preserve the fault evidence."""
import json

import pytest

from remote_teleop_protocol import ControlState, FaultBits
from remote_teleop_follower_safety import service
from remote_teleop_follower_safety.local_protocol import encode_command
from remote_teleop_follower_safety.state_machine import SafetyReaction, SafetyStateMachine, TransitionError
from remote_teleop_follower_safety.watchdog import WatchdogConfig, WatchdogSupervisor


@pytest.mark.parametrize('fault', [FaultBits.CAN_ERROR, FaultBits.NETWORK_TIMEOUT,
    FaultBits.CONTROL_OVERRUN, FaultBits.INVALID_COMMAND, FaultBits.E_STOP_ACTIVE])
def test_cleanup_hold_preserves_fault_and_first_cause(monkeypatch, fault):
    now = 10_000_000_000
    monkeypatch.setattr(service.time, 'monotonic_ns', lambda: now)
    machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
    supervisor = WatchdogSupervisor(machine, WatchdogConfig())
    supervisor.boot(now)
    machine.observe_leader_session(22)
    machine.alignment_complete(22)
    machine.request_run(22)
    # The shell can observe RUNNING, then disable can fault before HOLD arrives.
    supervisor.trip(fault, 'original fault before cleanup', now)
    snapshot, evidence = machine.snapshot(), supervisor.first_fault
    for _ in range(2):  # TERM and EXIT traps may both run.
        reply = json.loads(service._handle_datagram(supervisor, encode_command('hold')))
        assert machine.snapshot() == snapshot
        assert supervisor.first_fault == evidence
        assert reply['state'] == snapshot.state.name
        assert reply['fault_bits'] == int(fault)
    with pytest.raises(TransitionError):
        machine.request_run(22)


def test_hold_does_not_boot_uninitialized_controller():
    machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
    with pytest.raises(TransitionError):
        machine.request_hold()
    assert machine.state is ControlState.INIT

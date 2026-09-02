from remote_teleop_protocol import ControlState, FaultBits
from remote_teleop_follower_safety.state_machine import SafetyReaction, SafetyStateMachine
from remote_teleop_follower_safety.watchdog import (
    ControllerHeartbeat,
    WatchdogConfig,
    WatchdogSupervisor,
)
import pytest


BASE = 10_000_000_000


def test_production_defaults_keep_network_timeout_stricter_than_local_jitter_budget():
    config = WatchdogConfig()
    assert config.network_action_timeout_ns == 150_000_000
    assert config.control_cycle_timeout_ns == 250_000_000
    assert config.control_heartbeat_timeout_ns == 300_000_000
    assert config.network_action_timeout_ns < config.control_cycle_timeout_ns


def make_supervisor(running=False):
    machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
    config = WatchdogConfig(
        startup_grace_ns=200,
        control_heartbeat_timeout_ns=100,
        control_cycle_timeout_ns=50,
        network_action_timeout_ns=150,
        max_consecutive_overruns=5,
    )
    supervisor = WatchdogSupervisor(machine, config)
    supervisor.boot(BASE)
    if running:
        machine.observe_leader_session(22)
        machine.alignment_complete(22)
        machine.request_run(22)
    return supervisor


def heartbeat(
    *,
    sequence=1,
    now=BASE + 10,
    controller_session=11,
    last_cycle=None,
    last_action=None,
    leader_session=22,
    can_ok=True,
    estop=False,
    overruns=0,
):
    return ControllerHeartbeat(
        controller_session_id=controller_session,
        sequence=sequence,
        sent_monotonic_ns=now,
        last_control_cycle_ns=now if last_cycle is None else last_cycle,
        last_action_rx_ns=now if last_action is None else last_action,
        leader_session_id=leader_session,
        can_ok=can_ok,
        estop_active=estop,
        consecutive_overruns=overruns,
    )


def test_missing_controller_heartbeat_trips_after_grace():
    supervisor = make_supervisor()
    supervisor.check(BASE + 201)
    snapshot = supervisor.machine.snapshot()
    assert snapshot.state is ControlState.FAULT
    assert snapshot.fault_bits & FaultBits.LOCAL_WATCHDOG


def test_controller_process_timeout_is_independent_of_network():
    supervisor = make_supervisor()
    supervisor.receive_heartbeat(heartbeat(), BASE + 10)
    supervisor.check(BASE + 111)
    faults = supervisor.machine.snapshot().fault_bits
    assert faults & FaultBits.LOCAL_WATCHDOG
    assert faults & FaultBits.CONTROL_PROCESS_TIMEOUT
    assert not faults & FaultBits.CONTROL_CYCLE_TIMEOUT


def test_control_cycle_stall_is_detected_while_heartbeat_arrives():
    supervisor = make_supervisor()
    supervisor.receive_heartbeat(
        heartbeat(now=BASE + 100, last_cycle=BASE + 40), BASE + 100
    )
    snapshot = supervisor.machine.snapshot()
    assert "control cycle" in snapshot.reason
    assert snapshot.fault_bits & FaultBits.CONTROL_CYCLE_TIMEOUT
    assert not snapshot.fault_bits & FaultBits.CONTROL_PROCESS_TIMEOUT


def test_network_timeout_only_trips_while_running():
    supervisor = make_supervisor(running=False)
    supervisor.receive_heartbeat(
        heartbeat(now=BASE + 200, last_action=BASE + 1), BASE + 200
    )
    supervisor.check(BASE + 200)
    assert supervisor.machine.state is ControlState.ALIGNING

    supervisor = make_supervisor(running=True)
    supervisor.receive_heartbeat(
        heartbeat(now=BASE + 200, last_action=BASE + 1), BASE + 200
    )
    supervisor.check(BASE + 200)
    assert supervisor.machine.snapshot().fault_bits & FaultBits.NETWORK_TIMEOUT


def test_can_error_and_estop_are_distinct():
    supervisor = make_supervisor()
    supervisor.receive_heartbeat(heartbeat(can_ok=False), BASE + 10)
    assert supervisor.machine.snapshot().fault_bits & FaultBits.CAN_ERROR

    supervisor = make_supervisor()
    supervisor.receive_heartbeat(heartbeat(estop=True), BASE + 10)
    assert supervisor.machine.state is ControlState.E_STOP


def test_repeated_overruns_trip():
    supervisor = make_supervisor()
    supervisor.receive_heartbeat(heartbeat(overruns=5), BASE + 10)
    assert supervisor.machine.snapshot().fault_bits & FaultBits.CONTROL_OVERRUN


def test_controller_restart_requires_explicit_fault_reset():
    supervisor = make_supervisor()
    assert supervisor.receive_heartbeat(heartbeat(), BASE + 10)
    assert not supervisor.receive_heartbeat(
        heartbeat(controller_session=12, sequence=1, now=BASE + 20), BASE + 20
    )
    assert supervisor.machine.snapshot().fault_bits & FaultBits.LOCAL_WATCHDOG
    supervisor.reset_fault(BASE + 30, estop_released=True)
    assert supervisor.machine.state is ControlState.ALIGNING
    assert supervisor.receive_heartbeat(
        heartbeat(controller_session=12, sequence=1, now=BASE + 40), BASE + 40
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("last_cycle", BASE + 11),
        ("last_action", BASE + 11),
    ],
)
def test_component_timestamp_cannot_be_later_than_send_time(field, value):
    kwargs = {field: value, "now": BASE + 10}
    with pytest.raises(ValueError, match="cannot be later"):
        heartbeat(**kwargs)


def test_heartbeat_rejects_coerced_scalar_types():
    with pytest.raises(ValueError, match="integer"):
        ControllerHeartbeat(1.5, 1, 10, 10, 10, 2, True, False)

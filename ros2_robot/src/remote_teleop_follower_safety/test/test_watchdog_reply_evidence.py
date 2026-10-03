"""Offline reply ordering and first-fault evidence for independent follower I/O."""
import errno
import json
from unittest.mock import Mock

import pytest

from remote_teleop_protocol import ControlState, FaultBits
from remote_teleop_follower_safety import service, watchdog
from remote_teleop_follower_safety.local_protocol import encode_command, encode_heartbeat
from remote_teleop_follower_safety.state_machine import SafetyReaction, SafetyStateMachine, TransitionError
from remote_teleop_follower_safety.watchdog import ControllerHeartbeat, WatchdogConfig, WatchdogSupervisor


BASE = 10_000_000_000
MS = 1_000_000


def supervisor(running=True):
    machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
    value = WatchdogSupervisor(machine, WatchdogConfig())
    value.boot(BASE)
    if running:
        machine.observe_leader_session(22)
        machine.alignment_complete(22)
        machine.request_run(22)
    return value


def heartbeat(at=10, *, sequence=1, control=None, action=None, leader=22, **extra):
    return ControllerHeartbeat(11, sequence, BASE+at*MS,
        BASE+(at if control is None else control)*MS,
        0 if leader == 0 else BASE+(at if action is None else action)*MS,
        leader, extra.get('can_ok', True), extra.get('estop', False))


def clock(monkeypatch, at):
    monkeypatch.setattr(service.time, 'monotonic_ns', lambda: BASE+at*MS)


def test_network_first_fault_contains_frozen_real_component_ages(monkeypatch):
    value = supervisor()
    monkeypatch.setattr(watchdog.time, 'time_ns', lambda: 123_456_789)
    value.receive_heartbeat(heartbeat(action=1), BASE+12*MS)
    value.check(BASE+152*MS)
    evidence = value.first_fault
    assert evidence['detected_monotonic_ns'] == BASE+152*MS
    assert evidence['detected_unix_ns'] == 123_456_789
    assert evidence['action_age_ms'] == 151
    assert evidence['heartbeat_age_ms'] == 140
    assert evidence['heartbeat_sent_age_ms'] == 142
    assert evidence['control_age_ms'] == 142
    assert evidence['heartbeat']['controller_session_id'] == 11
    assert evidence['heartbeat']['leader_session_id'] == 22
    assert evidence['fault_bits'] == int(FaultBits.NETWORK_TIMEOUT)
    value.receive_heartbeat(heartbeat(200, sequence=2, can_ok=False), BASE+200*MS)
    value.check(BASE+1000*MS)
    assert value.first_fault == evidence
    assert value.machine.state is ControlState.FAULT
    # The live ages keep advancing, but the evidence does not.
    assert value.diagnostics(BASE+1000*MS)['action_age_ms'] == 800


def test_alive_io_does_not_hide_stopped_control_cycle():
    value = supervisor()
    value.receive_heartbeat(heartbeat(300, control=10), BASE+300*MS)
    evidence = value.first_fault
    assert evidence['control_age_ms'] == 290
    assert evidence['action_age_ms'] == 0
    assert evidence['heartbeat_age_ms'] == 0
    assert evidence['fault_bits'] & int(FaultBits.CONTROL_CYCLE_TIMEOUT)
    assert not evidence['fault_bits'] & int(FaultBits.CONTROL_PROCESS_TIMEOUT)


def test_first_fault_defensive_copy_and_only_successful_explicit_reset_clears():
    value = supervisor()
    session = value.watchdog_session_id
    value.receive_heartbeat(heartbeat(estop=True), BASE+10*MS)
    evidence = value.first_fault
    modified = value.first_fault
    modified['heartbeat']['sequence'] = 999
    assert value.first_fault == evidence
    with pytest.raises(TransitionError):
        value.reset_fault(BASE+20*MS, estop_released=False)
    assert value.first_fault == evidence
    value.reset_fault(BASE+30*MS, estop_released=True)
    assert value.first_fault is None
    assert value.watchdog_session_id == session
    assert value.machine.state is ControlState.ALIGNING


def test_missing_heartbeat_evidence_keeps_unknown_ages_unknown():
    value = supervisor(False)
    value.check(BASE+2_000*MS+1)
    assert value.first_fault['heartbeat'] is None
    for key in ('heartbeat_age_ms', 'heartbeat_sent_age_ms', 'action_age_ms', 'control_age_ms'):
        assert value.first_fault[key] is None


def test_implicit_leader_session_fault_also_records_first_cause():
    value = supervisor()
    value.receive_heartbeat(heartbeat(10, leader=33), BASE+10*MS)
    assert value.first_fault['reason'] == 'leader session changed while RUNNING'
    assert value.first_fault['detected_monotonic_ns'] == BASE+10*MS


def test_reply_echoes_exact_heartbeat_and_checks_after_accepting_fresh_input(monkeypatch):
    value = supervisor()
    value.receive_heartbeat(heartbeat(10), BASE+10*MS)
    clock(monkeypatch, 200)
    incoming = heartbeat(200, sequence=7)
    reply = json.loads(service._handle_datagram(value, encode_heartbeat(incoming)))
    # Checking the OLD heartbeat first would have incorrectly latched FAULT.
    assert reply['state'] == 'RUNNING'
    assert reply['first_fault'] is None
    assert reply['reply_to'] == {'type': 'heartbeat', 'controller_session_id': 11,
                                 'sequence': 7, 'sent_monotonic_ns': BASE+200*MS}
    assert reply['snapshot_monotonic_ns'] == BASE+200*MS
    assert reply['watchdog_session_id'] == value.watchdog_session_id
    assert reply['watchdog_io_protocol_version'] == service.WATCHDOG_IO_PROTOCOL_VERSION == 1


def test_heartbeat_response_never_reports_running_with_already_stale_action(monkeypatch):
    value = supervisor()
    clock(monkeypatch, 200)
    reply = json.loads(service._handle_datagram(value, encode_heartbeat(heartbeat(200, action=1))))
    assert reply['state'] == 'FAULT'
    assert reply['fault_bits'] & int(FaultBits.NETWORK_TIMEOUT)
    assert reply['first_fault']['action_age_ms'] == 199


@pytest.mark.parametrize('command', ['status', 'request_run'])
def test_command_reply_checks_current_safety_and_is_not_an_hb_reply(monkeypatch, command):
    value = supervisor()
    value.receive_heartbeat(heartbeat(10), BASE+10*MS)
    if command == 'request_run':
        value.machine.request_hold()
    clock(monkeypatch, 161)
    request = encode_command(command, **({'leader_session_id': 22} if command == 'request_run' else {}))
    reply = json.loads(service._handle_datagram(value, request))
    assert reply['state'] == 'FAULT'
    assert reply['reply_to'] == {'type': 'command', 'command': command}
    assert reply['first_fault']['reason'] == 'leader action receive timeout'


def test_common_snapshot_version_orders_hb_before_hold_and_fault(monkeypatch):
    value = supervisor()
    clock(monkeypatch, 10)
    previous = json.loads(service._handle_datagram(value, encode_heartbeat(heartbeat(10))))
    clock(monkeypatch, 20)
    held = json.loads(service._handle_datagram(value, encode_command('hold')))
    clock(monkeypatch, 30)
    stopped = json.loads(service._handle_datagram(value, encode_command('estop')))
    assert previous['state'] == 'RUNNING'
    assert held['state'] == 'READY'
    assert stopped['state'] == 'E_STOP'
    assert previous['snapshot_monotonic_ns'] < held['snapshot_monotonic_ns'] < stopped['snapshot_monotonic_ns']
    assert previous['watchdog_session_id'] == held['watchdog_session_id'] == stopped['watchdog_session_id']
    clock(monkeypatch, 40)
    late_heartbeat = json.loads(service._handle_datagram(value, encode_heartbeat(heartbeat(40, sequence=2))))
    assert late_heartbeat['state'] == 'E_STOP'
    assert late_heartbeat['first_fault'] == stopped['first_fault']


def test_restart_has_new_instance_and_reset_clears_evidence_without_running(monkeypatch):
    value = supervisor()
    second = supervisor(False)
    assert 0 < value.watchdog_session_id <= 0xffffffffffffffff
    assert second.watchdog_session_id != value.watchdog_session_id
    clock(monkeypatch, 10)
    stopped = json.loads(service._handle_datagram(value, encode_command('estop')))
    clock(monkeypatch, 20)
    reset = json.loads(service._handle_datagram(value, encode_command('reset', estop_released=True)))
    assert stopped['first_fault'] is not None
    assert reset['first_fault'] is None
    assert reset['state'] == 'ALIGNING'
    assert reset['watchdog_session_id'] == stopped['watchdog_session_id']


def test_bad_packet_has_no_borrowed_heartbeat_identity(monkeypatch):
    value = supervisor()
    clock(monkeypatch, 10)
    reply = json.loads(service._handle_datagram(value, b'not json'))
    assert reply['state'] == 'FAULT'
    assert reply['reply_to'] is None
    assert reply['first_fault']['fault_bits'] & int(FaultBits.INVALID_COMMAND)


@pytest.mark.parametrize('code', [errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN])
def test_expired_or_full_client_reply_does_not_interrupt_watchdog(code):
    sock = Mock()
    sock.sendto.side_effect = OSError(code, 'expired peer')
    assert service._send_reply(sock, '{}', '/tmp/test-client-not-created') is False
    assert sock.sendto.call_args.args[1] == service.socket.MSG_DONTWAIT


def test_unexpected_socket_failure_is_not_hidden():
    sock = Mock()
    sock.sendto.side_effect = OSError(errno.EBADF, 'invalid server socket')
    with pytest.raises(OSError):
        service._send_reply(sock, '{}', '/tmp/test-client-not-created')


def test_fault_reply_stays_within_existing_4096_byte_client_buffer(monkeypatch):
    value = supervisor()
    clock(monkeypatch, 200)
    payload = service._handle_datagram(value, encode_heartbeat(heartbeat(200, action=1)))
    assert len(payload.encode()) < 4096

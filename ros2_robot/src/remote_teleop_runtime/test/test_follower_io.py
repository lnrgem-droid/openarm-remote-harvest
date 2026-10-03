"""Deterministic I/O and ROS-commit regressions; all sockets are fakes."""
from collections import deque
import errno
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from remote_teleop_protocol import ActionCommand, encode_action
from remote_teleop_runtime.follower_io import (
    FollowerIOWorker, ReceivedAction, DRAIN_PACKETS, HEARTBEAT_REPLY_NS,
)


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def advance(self, milliseconds): self.now += int(milliseconds*1_000_000)


class Socket:
    def __init__(self): self.incoming, self.sent = deque(), []
    def recvfrom(self, size):
        if not self.incoming: raise BlockingIOError
        return self.incoming.popleft(), ("leader", 50010)
    def recv(self, size):
        return self.recvfrom(size)[0]
    def sendto(self, value, peer): self.sent.append((value, peer))
    def close(self): pass


@pytest.fixture
def rig():
    clock, udp, unix = Clock(), Socket(), Socket()
    watchdog = SimpleNamespace(sock=unix, server="offline-watchdog", close=Mock())
    worker = FollowerIOWorker(udp, watchdog, 77, clock=clock, queue_empty_probe=lambda: not udp.incoming)
    return worker, clock, udp, unix


def action(sequence, session=88):
    return encode_action(ActionCommand(session, sequence, 20_000_000_000+sequence,
                                       (0.,)*16, 100_000_000))


def reply(worker, clock, *, state="RUNNING", bits=0, service=99, echo=None):
    return {"state": state, "fault_bits": bits,
            "watchdog_session_id": service, "snapshot_monotonic_ns": clock(),
            "reply_to": worker.pending_heartbeat["reply_to"] if echo is None else echo}


def start_heartbeat(worker, clock):
    worker.control_completed(clock(), clock(), clock(), True)
    worker.step()
    assert worker.pending_heartbeat


def test_startup_queue_is_discarded_and_new_packet_uses_empty_boundary(rig):
    worker, clock, udp, _ = rig
    udp.incoming.append(action(1))
    worker.step()
    assert worker.snapshot()["action"] is None
    boundary = clock()
    clock.advance(4)
    udp.incoming.append(action(2))
    worker.step()
    sample = worker.snapshot()["action"]
    assert sample.action.sequence == 2
    assert sample.safe_rx_ns == boundary
    assert sample.processed_ns == clock()


def test_exhausted_udp_budget_never_publishes_partial_backlog(rig):
    worker, clock, udp, _ = rig
    worker.step()  # establish empty
    clock.advance(4)
    udp.incoming.extend(action(i) for i in range(1, DRAIN_PACKETS+2))
    worker.step()
    assert worker.snapshot()["action"] is None
    assert len(udp.incoming) == 1
    worker.step()  # Only the fully drained, still-fresh batch may now publish.
    assert worker.snapshot()["action"].action.sequence == DRAIN_PACKETS+1
    assert worker.snapshot()["action"].safe_rx_ns == clock()-4_000_000
    clock.advance(4)
    udp.incoming.append(action(DRAIN_PACKETS+2))
    worker.step()
    assert worker.snapshot()["action"].action.sequence == DRAIN_PACKETS+2


def test_worker_pause_discards_queued_old_packets_without_washing_freshness(rig):
    worker, clock, udp, _ = rig
    worker.step()
    clock.advance(4); udp.incoming.append(action(1)); worker.step()
    original = worker.snapshot()["action"]
    clock.advance(160); udp.incoming.append(action(2)); worker.step()
    assert worker.snapshot()["action"] is original
    assert clock()-original.safe_rx_ns > 150_000_000
    clock.advance(4); udp.incoming.append(action(3)); worker.step()
    assert worker.snapshot()["action"].action.sequence == 3


def test_wrong_or_timed_out_heartbeat_never_refreshes_safety(rig):
    worker, clock, _, unix = rig
    start_heartbeat(worker, clock)
    wrong = dict(worker.pending_heartbeat["reply_to"], sequence=999)
    clock.advance(1)
    unix.incoming.append(json.dumps(reply(worker, clock, echo=wrong)).encode())
    worker.step()
    assert worker.snapshot()["safety_rx_ns"] == 0
    late = reply(worker, clock)
    clock.advance(HEARTBEAT_REPLY_NS/1e6+1)
    unix.incoming.append(json.dumps(late).encode())
    worker.step()
    assert worker.snapshot()["safety_rx_ns"] == 0
    assert worker.snapshot()["diagnostics"]["heartbeat_timeouts"] == 1


def test_safety_age_is_bounded_by_request_send_not_reply_arrival(rig):
    worker, clock, _, unix = rig
    start_heartbeat(worker, clock)
    sent = clock()
    clock.advance(3)
    unix.incoming.append(json.dumps(reply(worker, clock)).encode())
    worker.step()
    assert worker.snapshot()["safety"]["state"] == "RUNNING"
    assert worker.snapshot()["safety_rx_ns"] == sent


def test_heartbeat_reports_real_last_completed_control_even_when_io_keeps_running(rig):
    worker, clock, udp, unix = rig
    completed = clock()
    start_heartbeat(worker, clock)
    for sequence in range(1, 30):
        clock.advance(10)
        udp.incoming.append(action(sequence))
        worker.step()
    heartbeat = json.loads(unix.sent[-1][0])
    assert heartbeat["last_control_cycle_ns"] == completed
    assert heartbeat["sent_monotonic_ns"]-completed >= 250_000_000
    assert heartbeat["sent_monotonic_ns"]-heartbeat["last_action_rx_ns"] < 150_000_000


def test_pre_hold_heartbeat_cannot_overwrite_new_command_state(rig):
    worker, clock, _, unix = rig
    start_heartbeat(worker, clock)
    old = reply(worker, clock)
    worker.begin_command()
    command_sent = clock()
    clock.advance(1)
    hold = reply(worker, clock, state="READY", echo={"type": "command", "command": "hold"})
    assert worker.finish_command("hold", hold, command_sent)
    unix.incoming.append(json.dumps(old).encode())
    worker.step()
    assert worker.snapshot()["safety"]["state"] == "READY"


def test_fault_latches_until_explicit_confirmed_reset_and_resets_action_stream(rig):
    worker, clock, udp, unix = rig
    start_heartbeat(worker, clock)
    clock.advance(1)
    unix.incoming.append(json.dumps(reply(worker, clock, state="FAULT", bits=1)).encode())
    worker.step()
    clock.advance(10); worker.step()
    clock.advance(1)
    unix.incoming.append(json.dumps(reply(worker, clock)).encode())
    worker.step()
    assert worker.snapshot()["safety"]["state"] == "FAULT"
    worker.begin_command()
    sent = clock(); clock.advance(1)
    reset = reply(worker, clock, state="ALIGNING", echo={"type": "command", "command": "reset"})
    assert worker.finish_command("reset", reset, sent)
    assert worker.snapshot()["safety"]["state"] == "ALIGNING"
    udp.incoming.append(action(50, session=123))
    worker.step()  # reset and discard any old queue before accepting new session
    assert worker.snapshot()["action"] is None
    clock.advance(4); udp.incoming.append(action(51, session=123)); worker.step()
    assert worker.snapshot()["action"].action.session_id == 123


def test_watchdog_restart_never_silently_replaces_live_safety(rig):
    worker, clock, _, unix = rig
    start_heartbeat(worker, clock)
    clock.advance(1); unix.incoming.append(json.dumps(reply(worker, clock)).encode()); worker.step()
    clock.advance(10); worker.step()
    clock.advance(1); unix.incoming.append(json.dumps(reply(worker, clock, service=100)).encode()); worker.step()
    assert worker.snapshot()["safety"]["state"] == "FAULT"
    assert worker.snapshot()["safety_rx_ns"] == 0


def test_no_completed_ros_cycle_means_no_synthetic_heartbeat(rig):
    worker, clock, _, unix = rig
    for _ in range(5):
        worker.step(); clock.advance(10)
    assert not unix.sent


def test_failed_second_arm_publish_does_not_commit_applied_target(tmp_path):
    from test_follower_hold_continuity import gateway
    node = gateway(tmp_path)
    node.publish_target(1_000_000_000)
    previous = (node.applied_axes, node.last_target_left, node.last_target_right, node.applied_sequence)
    node.latest_action.sequence = 2
    node.latest_action.left_arm = (0., 0., 0., 1.49, 0., 0., 0.)
    node.left_publisher = SimpleNamespace(publish=Mock(side_effect=RuntimeError("synthetic publish failure")))
    with pytest.raises(RuntimeError, match="synthetic"):
        node.publish_target(1_004_000_000)
    assert (node.applied_axes, node.last_target_left, node.last_target_right, node.applied_sequence) == previous


def test_tick_does_not_commit_control_liveness_after_partial_publish(tmp_path, monkeypatch):
    from test_follower_hold_continuity import gateway
    from remote_teleop_runtime import follower
    node = gateway(tmp_path)
    monkeypatch.setattr(follower.time, "monotonic_ns", lambda: 1_000_000_000)
    sample = ReceivedAction(node.latest_action, 1_000_000_000, 1_000_000_000, "offline")
    node.io = SimpleNamespace(snapshot=lambda: {"action": sample, "safety": node.safety,
        "safety_rx_ns": 1_000_000_000}, control_completed=Mock())
    node.align_since_ns = 0
    node.send_state = Mock()
    node.left_publisher = SimpleNamespace(publish=Mock(side_effect=RuntimeError("synthetic publish failure")))
    with pytest.raises(RuntimeError): node.tick()
    node.io.control_completed.assert_not_called()


def test_operator_socket_drain_is_one_request_per_callback():
    from remote_teleop_runtime.follower import FollowerGateway
    node = object.__new__(FollowerGateway)
    node.command = Socket()
    node.command.incoming.extend([b'{"command":"status"}']*3)
    node.runtime_status = lambda: {"state": "READY"}
    node.handle_commands(1)
    assert len(node.command.incoming) == 2
    assert len(node.command.sent) == 1


def test_watchdog_ack_failure_cannot_block_explicit_hardware_disable():
    from remote_teleop_runtime.follower import FollowerGateway
    node = object.__new__(FollowerGateway)
    node.command = Socket()
    node.command.incoming.append(b'{"command":"disable"}')
    node.runtime_status = lambda: {"state": "FAULT"}
    node.safety = {"state": "FAULT", "fault_bits": 1}
    node.watchdog_command = Mock(side_effect=RuntimeError("missing watchdog reply"))
    node.disable_client = SimpleNamespace(wait_for_service=lambda **kwargs: True, call_async=Mock())
    node.handle_commands(1)
    node.disable_client.call_async.assert_called_once()
    result = json.loads(node.command.sent[-1][0])
    assert result["hardware_disable_requested"]
    assert result["watchdog_estop_confirmed"] is False
    assert "missing watchdog" in result["watchdog_confirmation_error"]


@pytest.mark.parametrize("safety_rx,expected", [(0, "READY"), (900_000_000, "READY"),
                                              (900_000_001, "RUNNING")])
def test_runtime_and_udp_state_never_grant_expired_safety_authority(
        tmp_path, monkeypatch, safety_rx, expected):
    from test_follower_hold_continuity import gateway
    from remote_teleop_protocol import decode_message
    from remote_teleop_runtime import follower
    node = gateway(tmp_path)
    monkeypatch.setattr(follower.time, "monotonic_ns", lambda: 1_000_000_000)
    node.publish_target(1_000_000_000)
    node.safety_rx_ns = safety_rx
    node.sequence, node.peer_ip = 0, "offline"
    node.velocities = node.efforts = (0.,) * 16
    node.udp = Socket()
    status = node.runtime_status()
    node.send_state(1_000_000_000)
    packet = decode_message(node.udp.sent[-1][0])
    assert status["state"] == expected
    assert packet.control_state.name == expected
    assert node.safety["state"] == "RUNNING"  # No fabricated permanent fault.
    if expected == "READY":
        assert status["watchdog_state"] == "RUNNING"
        assert "holding last published target" in status["reason"]


@pytest.mark.parametrize("code", [errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN])
def test_disappeared_operator_client_does_not_abort_command_callback(code):
    from remote_teleop_runtime.follower import FollowerGateway
    node = object.__new__(FollowerGateway)
    node.command = Socket()
    node.command.incoming.append(b'{"command":"status"}')
    node.command.sendto = Mock(side_effect=OSError(code, "offline client gone"))
    node.runtime_status = lambda: {"state": "READY"}
    node.handle_commands(1)
    node.command.sendto.assert_called_once()
    assert node.command_reply_dropped == 1


def test_unexpected_operator_reply_error_is_not_silently_hidden():
    from remote_teleop_runtime.follower import FollowerGateway
    node = object.__new__(FollowerGateway)
    node.command = SimpleNamespace(sendto=Mock(side_effect=OSError(errno.EIO, "broken socket")))
    with pytest.raises(OSError, match="broken socket"):
        node.send_command_reply({"state": "READY"}, "offline")

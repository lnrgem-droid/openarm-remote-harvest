"""Independent adversarial checks; every transport and clock is in memory."""
from collections import deque
from types import SimpleNamespace

import pytest

from remote_teleop_follower_safety.local_protocol import decode_local
from remote_teleop_follower_safety.state_machine import SafetyReaction, SafetyStateMachine
from remote_teleop_follower_safety.watchdog import WatchdogConfig, WatchdogSupervisor
from remote_teleop_protocol import ActionCommand, FaultBits, encode_action
from remote_teleop_runtime.follower_io import FollowerIOWorker, ReceivedAction, HEARTBEAT_REPLY_NS

from test_follower_hold_continuity import gateway, publish


class Clock:
    def __init__(self):
        self.now = 10_000_000_000

    def __call__(self):
        return self.now


class Inbox:
    def __init__(self):
        self.queue = deque()
        self.sent = []
        self.empty_hook = None

    def recv(self, _size):
        if self.queue:
            return self.queue.popleft()
        raise BlockingIOError

    def recvfrom(self, _size):
        if self.queue:
            return self.queue.popleft(), ("127.0.0.1", 10000)
        if self.empty_hook:
            hook, self.empty_hook = self.empty_hook, None
            hook()
        raise BlockingIOError

    def sendto(self, payload, _peer):
        self.sent.append(payload)


def make_worker():
    clock, udp, watchdog_socket = Clock(), Inbox(), Inbox()
    worker = FollowerIOWorker(udp, SimpleNamespace(sock=watchdog_socket, server="in-memory"),
                              12, clock=clock)
    worker.control_completed(clock.now, clock.now, clock.now, True)
    return worker, clock, udp, watchdog_socket


def reply(clock, *, state="RUNNING", bits=0, service_id=7, reply_to=None, timestamp=None):
    return {"state": state, "fault_bits": bits, "watchdog_session_id": service_id,
            "snapshot_monotonic_ns": clock.now if timestamp is None else timestamp,
            "reply_to": reply_to}


def packet(sequence=1):
    return encode_action(ActionCommand(55, sequence, 1, (0.,) * 16, 100_000_000))


def test_heartbeat_send_clock_is_after_concurrently_completed_control_cycle():
    worker, clock, _, socket = make_worker()

    class ConcurrentCompletion:
        def __enter__(self):
            clock.now += 10
            worker.control_ns = clock.now

        def __exit__(self, *_args):
            pass

    # The ROS callback completes while the worker is acquiring its snapshot.
    worker.lock = ConcurrentCompletion()
    worker._heartbeat_io()
    kind, heartbeat = decode_local(socket.sent[-1])
    assert kind == "heartbeat"
    assert heartbeat.last_control_cycle_ns <= heartbeat.sent_monotonic_ns


def test_reset_between_decode_and_queue_empty_cannot_republish_old_action():
    worker, clock, udp, _ = make_worker()
    worker._receive_actions()  # Establish the empty receive-queue boundary.
    clock.now += 1_000_000
    udp.queue.append(packet())

    def reset_before_commit():
        worker.begin_command()
        clock.now += 10
        acknowledged = reply(clock, state="ALIGNING",
                             reply_to={"type": "command", "command": "reset"})
        assert worker.finish_command("reset", acknowledged, clock.now)

    udp.empty_hook = reset_before_commit
    worker._receive_actions()
    assert worker.snapshot()["action"] is None
    worker.step()
    assert worker.snapshot()["action"] is None
    clock.now += 1_000_000
    udp.queue.append(packet(sequence=2))
    worker.step()
    assert worker.snapshot()["action"].action.sequence == 2


def test_late_running_heartbeat_cannot_undo_acknowledged_hold():
    import json

    worker, clock, _, socket = make_worker()
    worker._heartbeat_io()
    pending = dict(worker.pending_heartbeat)
    clock.now += 10
    old_running = reply(clock, reply_to=pending["reply_to"])
    worker.begin_command()
    clock.now += 10
    held = reply(clock, state="READY", reply_to={"type": "command", "command": "hold"})
    assert worker.finish_command("hold", held, clock.now)
    accepted_at = worker.snapshot()["safety_rx_ns"]
    socket.queue.append(json.dumps(old_running).encode())
    worker._heartbeat_io()
    assert worker.snapshot()["safety"]["state"] == "READY"
    assert worker.snapshot()["safety_rx_ns"] == accepted_at


def test_fault_cannot_be_cleared_by_a_newer_running_reply_without_reset():
    worker, clock, _, _ = make_worker()
    assert worker._accept_safety_locked(reply(clock, state="FAULT", bits=1), clock.now)
    clock.now += 10
    assert not worker._accept_safety_locked(reply(clock), clock.now)
    assert worker.snapshot()["safety"]["state"] == "FAULT"
    worker.begin_command()
    clock.now += 10
    reset = reply(clock, state="ALIGNING", reply_to={"type": "command", "command": "reset"})
    assert worker.finish_command("reset", reset, clock.now)
    assert worker.snapshot()["safety"]["state"] == "ALIGNING"


def test_watchdog_process_change_cannot_grant_fresh_running_authority():
    worker, clock, _, _ = make_worker()
    assert worker._accept_safety_locked(reply(clock), clock.now)
    clock.now += 10
    assert not worker._accept_safety_locked(reply(clock, service_id=8), clock.now)
    assert worker.snapshot()["safety"]["state"] == "FAULT"
    assert worker.snapshot()["safety_rx_ns"] == 0


def test_matching_but_timed_out_reply_does_not_refresh_safety():
    import json

    worker, clock, _, socket = make_worker()
    worker._heartbeat_io()
    old = worker.pending_heartbeat
    clock.now += HEARTBEAT_REPLY_NS + 1_000_000
    socket.queue.append(json.dumps(reply(clock, reply_to=old["reply_to"])).encode())
    worker._heartbeat_io()
    assert worker.snapshot()["safety_rx_ns"] == 0
    assert worker.snapshot()["diagnostics"]["heartbeat_timeouts"] == 1


def test_live_io_does_not_relabel_stalled_control_as_fresh():
    worker, clock, _, socket = make_worker()
    original_control = clock.now
    clock.now += 260_000_000
    action = ActionCommand(55, 1, 1, (0.,) * 16, 100_000_000)
    worker.action = ReceivedAction(action, clock.now, clock.now, "127.0.0.1")
    worker._heartbeat_io()
    _, heartbeat = decode_local(socket.sent[-1])
    assert heartbeat.last_control_cycle_ns == original_control
    assert heartbeat.last_action_rx_ns == clock.now

    machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
    supervisor = WatchdogSupervisor(machine, WatchdogConfig())
    supervisor.boot(original_control)
    machine.observe_leader_session(55)
    machine.alignment_complete(55)
    machine.request_run(55)
    supervisor.receive_heartbeat(heartbeat, clock.now)
    faults = supervisor.machine.snapshot().fault_bits
    assert faults & FaultBits.CONTROL_CYCLE_TIMEOUT
    assert not faults & FaultBits.NETWORK_TIMEOUT


def test_cached_running_snapshot_expires_while_ros_keeps_publishing(tmp_path):
    node = gateway(tmp_path)
    publish(node, 1_000_000_000)
    previous = node.applied_axes
    cached_action = ReceivedAction(node.latest_action, 1_000_000_000, 1_000_000_000, "127.0.0.1")
    node.io = SimpleNamespace(snapshot=lambda: {
        "action": cached_action, "safety": {"state": "RUNNING", "fault_bits": 0},
        "safety_rx_ns": 1_000_000_000})
    node.positions[3] -= .03
    node.last_feedback_ns = 1_200_000_000
    node.heartbeat(1_200_000_000)
    node.publish_target(1_200_000_000)
    assert node.applied_axes == previous
    assert not node.command_was_running
    assert node.safety_rx_ns == 1_000_000_000


def test_preemption_after_empty_poll_cannot_make_queued_packet_fresh():
    worker, clock, udp, _ = make_worker()

    def preempt_after_empty():
        clock.now += 200_000_000

    udp.empty_hook = preempt_after_empty
    worker._receive_actions()
    udp.queue.append(packet())
    worker._receive_actions()
    assert worker.snapshot()["action"] is None
    assert worker.snapshot()["diagnostics"]["rx_discarded"] == 1
    clock.now += 1_000_000
    udp.queue.append(packet(sequence=2))
    worker._receive_actions()
    assert worker.snapshot()["action"].action.sequence == 2


def test_continuous_udp_flood_cannot_prevent_a_heartbeat_step():
    worker, clock, udp, socket = make_worker()
    udp.recvfrom = lambda _size: (packet(), ("127.0.0.1", 10000))
    worker.queue_empty_probe = lambda: False  # Endless synthetic readability.
    worker.step()
    assert worker.snapshot()["diagnostics"]["rx_packets"] == 32
    assert socket.sent
    assert worker.snapshot()["action"] is None


def test_missing_watchdog_ack_cannot_block_explicit_hardware_disable(tmp_path):
    import json

    node = gateway(tmp_path)
    requests = []
    node.command = Inbox()
    node.command.queue.append(b'{"command":"disable"}')
    node.runtime_status = lambda: {}

    def no_watchdog_ack(*_args, **_kwargs):
        raise RuntimeError("watchdog unavailable")

    node.watchdog_command = no_watchdog_ack
    node.disable_client = SimpleNamespace(
        wait_for_service=lambda **_kwargs: True, call_async=requests.append)
    node.handle_commands(1_000_000_000)
    assert len(requests) == 1
    response = json.loads(node.command.sent[-1])
    assert response["hardware_disable_requested"] is True
    assert response["watchdog_estop_confirmed"] is False
    assert "watchdog unavailable" in response["watchdog_confirmation_error"]


@pytest.mark.parametrize("stop_command", ["hold", "estop"])
def test_unconfirmed_stop_cannot_be_undone_by_later_running_heartbeat(stop_command):
    worker, clock, _, _ = make_worker()
    assert worker._accept_safety_locked(reply(clock), clock.now)
    worker.begin_command()
    assert not worker.finish_command(stop_command, None, clock.now)
    clock.now += 10
    worker._accept_safety_locked(reply(clock), clock.now)
    snapshot = worker.snapshot()
    assert (snapshot["safety"]["state"] != "RUNNING"
            or snapshot["safety"]["fault_bits"]
            or snapshot["safety_rx_ns"] == 0)


def test_terminal_worker_error_cannot_be_reauthorized_by_command_ack():
    worker, clock, _, _ = make_worker()
    assert worker._accept_safety_locked(reply(clock), clock.now)
    worker.diagnostics["worker_error"] = "OSError: test worker exited"
    worker.safety_rx_ns = 0
    worker.begin_command()
    clock.now += 10
    running = reply(clock, reply_to={"type": "command", "command": "request_run"})
    worker.finish_command("request_run", running, clock.now)
    snapshot = worker.snapshot()
    assert (snapshot["safety"]["state"] != "RUNNING"
            or snapshot["safety"]["fault_bits"]
            or snapshot["safety_rx_ns"] == 0)

"""Bounded UDP/heartbeat I/O, independent of the ROS executor and CAN.

Only completed ROS control cycles supply control liveness. Receiving packets or
running this thread never advances that evidence. Action freshness uses a
kernel arrival timestamp in production, never the time a buffered packet is
read. The conservative queue-empty fallback remains for simulated transports.
"""
from dataclasses import dataclass
import json
import select
import socket
import threading
import time

from .kernel_rx import KernelReceiveClock

from remote_teleop_protocol import ActionCommand, PacketError, SequenceTracker, decode_message
from remote_teleop_follower_safety.local_protocol import encode_heartbeat
from remote_teleop_follower_safety.watchdog import ControllerHeartbeat


IO_PROTOCOL_VERSION = 1
ACTION_FRESH_NS = 100_000_000
DRAIN_PACKETS = 32
DRAIN_BUDGET_NS = 1_000_000
# This is the correlated local request/reply budget, not motion authority.
# Four milliseconds spuriously discards healthy replies after ordinary Python
# scheduling delays. Accepted safety still ages from the ORIGINAL send time,
# and the unchanged 100ms control gate / 150ms action watchdog apply separately.
HEARTBEAT_REPLY_NS = 20_000_000
FEEDBACK_CAN_FAULT_NS = 1_000_000_000


@dataclass(frozen=True)
class ReceivedAction:
    action: ActionCommand
    safe_rx_ns: int
    processed_ns: int
    peer_ip: str


class FollowerIOWorker:
    def __init__(self, udp, watchdog, controller_session_id, *, rate=100., clock=time.monotonic_ns,
                 queue_empty_probe=None, kernel_timestamps=False):
        self.udp, self.watchdog = udp, watchdog
        self.controller_session_id = controller_session_id
        self.clock = clock
        self.kernel_clock = KernelReceiveClock(udp, monotonic=clock) if kernel_timestamps else None
        self.kernel_min_rx_ns = clock()
        # A nonblocking readiness probe can prove the queue empty even if the
        # final recv/decode was preempted past the work budget. Its timestamp
        # is sampled BEFORE probing, exactly like the EAGAIN boundary below.
        self.queue_empty_probe = (queue_empty_probe if queue_empty_probe is not None
                                  else lambda: not select.select([self.udp], [], [], 0)[0])
        self.period_ns = int(1e9 / rate)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name="follower-network-watchdog", daemon=True)
        self.tracker = SequenceTracker()  # Worker is the only owner, including reset.
        self.action = None
        self.safety = {"state": "ALIGNING", "fault_bits": 0, "reason": "waiting for correlated watchdog heartbeat"}
        self.safety_rx_ns = 0
        self.control_ns = self.feedback_ns = 0
        self.have_feedback = False
        self.last_control_started_ns = 0
        self.epoch = 0
        self.command_pending = False
        self.stop_intent = None
        self.closed = False
        self.reset_requested = False
        self.watchdog_session_id = None
        self.watchdog_snapshot_ns = 0
        self.fault_latched = None
        self.sequence = 0
        self.pending_heartbeat = None
        self.next_heartbeat_ns = 0
        self.empty_boundary_ns = None
        self.resync = True
        self.pending_action = None
        self.pending_action_epoch = None
        self.last_step_ns = 0
        self.last_processed_ns = 0
        self.diagnostics = dict(rx_packets=0, rx_invalid=0, rx_rejected=0, rx_discarded=0,
            rx_resyncs=0, rx_budget_yields=0, heartbeat_timeouts=0, heartbeat_unmatched=0,
            max_worker_gap_ns=0, max_packet_processing_gap_ns=0, max_valid_action_gap_ns=0,
            max_control_tick_gap_ns=0, max_control_duration_ns=0, worker_error=None,
            kernel_timestamps=bool(self.kernel_clock), rx_kernel_timestamp_rejected=0,
            rx_kernel_stale=0, rx_kernel_committed=0)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return {"action": self.action, "safety": dict(self.safety),
                    "safety_rx_ns": 0 if self.command_pending or self.stop_intent or self.closed
                        or self.diagnostics["worker_error"] else self.safety_rx_ns,
                    "stop_intent": self.stop_intent,
                    "diagnostics": {**self.diagnostics, "last_control_cycle_ns": self.control_ns,
                        "last_feedback_ns": self.feedback_ns,
                        "last_action_safe_rx_ns": self.action.safe_rx_ns if self.action else 0,
                        "last_action_processed_ns": self.action.processed_ns if self.action else 0}}

    def control_completed(self, started_ns, completed_ns, feedback_ns, have_feedback):
        """Called only after the ROS callback publishes every enabled side."""
        with self.lock:
            if self.last_control_started_ns:
                self.diagnostics["max_control_tick_gap_ns"] = max(
                    self.diagnostics["max_control_tick_gap_ns"], started_ns-self.last_control_started_ns)
            self.diagnostics["max_control_duration_ns"] = max(
                self.diagnostics["max_control_duration_ns"], completed_ns-started_ns)
            self.last_control_started_ns = started_ns
            self.control_ns, self.feedback_ns = completed_ns, feedback_ns
            self.have_feedback = have_feedback

    def begin_command(self):
        # A command uses a separate reply socket. Epoch fencing also prevents
        # an in-flight, pre-HOLD RUNNING heartbeat from undoing its reply.
        with self.lock:
            self.epoch += 1
            self.command_pending = True
            self.safety_rx_ns = 0

    def finish_command(self, command, reply, sent_ns):
        with self.lock:
            self.epoch += 1
            self.command_pending = False
            self.safety_rx_ns = 0
            if command in {"hold", "estop", "reset"}:
                self.stop_intent = command
            if not isinstance(reply, dict) or reply.get("reply_to") != {"type": "command", "command": command}:
                return False
            resetting = command == "reset" and reply.get("state") == "ALIGNING" and reply.get("fault_bits") == 0
            resuming = command == "request_run" and reply.get("state") == "RUNNING" and reply.get("fault_bits") == 0
            accepted = self._accept_safety_locked(reply, sent_ns, allow_reset=resetting, allow_run=resuming)
            if accepted and resetting:
                self.reset_requested = True
                self.kernel_min_rx_ns = self.clock()
                self.action = None  # Immediately invalidate the ROS-visible old session.
            return accepted

    def _accept_safety_locked(self, reply, sent_ns, *, allow_reset=False, allow_run=False):
        if self.closed or self.diagnostics["worker_error"]:
            return False  # A command ACK cannot revive a dead I/O worker.
        service_id, snapshot_ns = reply.get("watchdog_session_id"), reply.get("snapshot_monotonic_ns")
        now = self.clock()
        if (type(service_id) is not int or service_id <= 0 or type(snapshot_ns) is not int
                or not sent_ns <= snapshot_ns <= now
                or reply.get("state") not in {"INIT", "ALIGNING", "READY", "RUNNING", "FAULT", "E_STOP"}
                or type(reply.get("fault_bits")) is not int):
            return False
        if self.watchdog_session_id not in (None, service_id) and not allow_reset:
            self.safety_rx_ns = 0
            self.fault_latched = {"state": "FAULT", "fault_bits": 16,
                "reason": "watchdog process changed; explicit reset required"}
            self.safety = dict(self.fault_latched)
            return False
        if service_id == self.watchdog_session_id and snapshot_ns <= self.watchdog_snapshot_ns:
            return False
        if allow_reset:
            self.fault_latched = None
        elif self.fault_latched is not None and reply["state"] not in {"FAULT", "E_STOP"}:
            return False
        if self.stop_intent:
            if allow_reset or (allow_run and self.stop_intent != "reset"):
                self.stop_intent = None  # Explicit, acknowledged operator recovery.
            elif self.stop_intent != "reset" and reply["state"] in {"ALIGNING", "READY", "FAULT", "E_STOP"}:
                self.stop_intent = None  # Fresh evidence confirms the stop.
            else:
                return False
        self.watchdog_session_id, self.watchdog_snapshot_ns = service_id, snapshot_ns
        self.safety = dict(reply)
        # The request send time is the oldest bound on this exchange, so a
        # delayed response cannot turn an old watchdog sample into fresh safety.
        self.safety_rx_ns = sent_ns
        if reply["state"] in {"FAULT", "E_STOP"} or reply["fault_bits"]:
            self.fault_latched = dict(reply)
        return True

    def _queue_drained(self, candidate, batch_epoch, before_probe):
        """Only a proven empty queue permits committing a private candidate."""
        self.empty_boundary_ns = before_probe
        was_resync, self.resync = self.resync, False
        self.pending_action = None
        self.pending_action_epoch = None
        if candidate is not None and not was_resync and self.clock()-candidate.safe_rx_ns <= ACTION_FRESH_NS:
            with self.lock:
                if batch_epoch == self.epoch and not self.reset_requested and not self.command_pending:
                    if self.action is not None:
                        self.diagnostics["max_valid_action_gap_ns"] = max(
                            self.diagnostics["max_valid_action_gap_ns"], candidate.safe_rx_ns-self.action.safe_rx_ns)
                    self.action = candidate

    def _receive_timestamped_actions(self):
        # Each packet has an arrival timestamp, so a finite budget can publish
        # a fresh candidate even if the queue never becomes completely empty.
        # Neither backlog nor scheduler delay is converted into fresh evidence.
        with self.lock:
            batch_epoch = self.epoch
        candidate = None
        started = self.clock()
        for _ in range(DRAIN_PACKETS):
            try:
                data, ancillary, flags, peer = self.udp.recvmsg(2048, socket.CMSG_SPACE(16))
            except BlockingIOError:
                break
            processed = self.clock()
            self.diagnostics["rx_packets"] += 1
            if self.last_processed_ns:
                self.diagnostics["max_packet_processing_gap_ns"] = max(
                    self.diagnostics["max_packet_processing_gap_ns"], processed-self.last_processed_ns)
            self.last_processed_ns = processed
            stamp = self.kernel_clock.timestamp(ancillary, flags)
            if stamp is None:
                self.diagnostics["rx_kernel_timestamp_rejected"] += 1
            elif stamp < self.kernel_min_rx_ns or not 0 <= self.clock()-stamp <= ACTION_FRESH_NS:
                self.diagnostics["rx_kernel_stale"] += 1
            else:
                try:
                    action = decode_message(data)
                    if isinstance(action, ActionCommand) and self.tracker.accept(action.session_id, action.sequence):
                        candidate = ReceivedAction(action, stamp, processed, peer[0])
                    else:
                        self.diagnostics["rx_rejected"] += 1
                except PacketError:
                    self.diagnostics["rx_invalid"] += 1
            if self.clock()-started >= DRAIN_BUDGET_NS:
                self.diagnostics["rx_budget_yields"] += 1
                break
        if candidate is not None:
            with self.lock:
                if (batch_epoch == self.epoch and not self.reset_requested and not self.command_pending
                        and 0 <= self.clock()-candidate.safe_rx_ns <= ACTION_FRESH_NS
                        and (self.action is None or candidate.safe_rx_ns >= self.action.safe_rx_ns)):
                    if self.action is not None:
                        self.diagnostics["max_valid_action_gap_ns"] = max(
                            self.diagnostics["max_valid_action_gap_ns"], candidate.safe_rx_ns-self.action.safe_rx_ns)
                    self.action = candidate
                    self.diagnostics["rx_kernel_committed"] += 1

    def _receive_actions(self):
        if self.kernel_clock is not None:
            return self._receive_timestamped_actions()
        with self.lock:
            batch_epoch = self.epoch
        batch_start = self.clock()
        boundary = self.empty_boundary_ns
        if boundary is None or batch_start-boundary > ACTION_FRESH_NS:
            self.resync = True
        candidate = (self.pending_action if not self.resync and self.pending_action_epoch == batch_epoch else None)
        self.pending_action = None
        self.pending_action_epoch = None
        for _ in range(DRAIN_PACKETS):
            # Sample BEFORE recv: preemption after an EAGAIN must not give
            # packets arriving during that preemption a falsely recent bound.
            before_recv = self.clock()
            try:
                data, peer = self.udp.recvfrom(2048)
            except BlockingIOError:
                self._queue_drained(candidate, batch_epoch, before_recv)
                return
            processed_ns = self.clock()
            self.diagnostics["rx_packets"] += 1
            if self.last_processed_ns:
                self.diagnostics["max_packet_processing_gap_ns"] = max(
                    self.diagnostics["max_packet_processing_gap_ns"], processed_ns-self.last_processed_ns)
            self.last_processed_ns = processed_ns
            if self.resync:
                self.diagnostics["rx_discarded"] += 1
            else:
                try:
                    action = decode_message(data)
                    if isinstance(action, ActionCommand) and self.tracker.accept(action.session_id, action.sequence):
                        candidate = ReceivedAction(action, boundary, processed_ns, peer[0])
                    else:
                        self.diagnostics["rx_rejected"] += 1
                except PacketError:
                    self.diagnostics["rx_invalid"] += 1
            if self.clock()-batch_start >= DRAIN_BUDGET_NS:
                break
        # Budget exhaustion is NOT proof of an old queue: recv/decode can be
        # preempted even for one freshly arrived packet. Probe once, without
        # waiting. If backlog remains, keep at most one PRIVATE candidate for
        # the next bounded drain; do not publish it or refresh its timestamp.
        self.diagnostics["rx_budget_yields"] += 1
        before_probe = self.clock()
        if self.queue_empty_probe():
            self._queue_drained(candidate, batch_epoch, before_probe)
        elif not self.resync and candidate is not None:
            self.pending_action = candidate
            self.pending_action_epoch = batch_epoch
        else:
            self.diagnostics["rx_resyncs"] += 1

    def _heartbeat_io(self):
        now = self.clock()
        pending = self.pending_heartbeat
        if pending and now > pending["deadline_ns"]:
            self.pending_heartbeat = pending = None
            self.diagnostics["heartbeat_timeouts"] += 1
        # Replies are independently bounded, including unrelated/late replies.
        reply_batch_start = self.clock()
        for _ in range(DRAIN_PACKETS):
            try:
                raw = self.watchdog.sock.recv(4096)
            except BlockingIOError:
                break
            now = self.clock()
            try:
                reply = json.loads(raw.decode())
            except (UnicodeDecodeError, ValueError):
                reply = None
            pending = self.pending_heartbeat
            if (pending and now <= pending["deadline_ns"] and isinstance(reply, dict)
                    and reply.get("reply_to") == pending["reply_to"]):
                with self.lock:
                    if pending["epoch"] == self.epoch and not self.command_pending:
                        self._accept_safety_locked(reply, pending["sent_ns"])
                self.pending_heartbeat = None
            else:
                self.diagnostics["heartbeat_unmatched"] += 1
            if self.clock()-reply_batch_start >= DRAIN_BUDGET_NS:
                break
        now = self.clock()
        if self.pending_heartbeat is not None or now < self.next_heartbeat_ns:
            return
        with self.lock:
            action, control_ns, feedback_ns = self.action, self.control_ns, self.feedback_ns
            have_feedback, epoch = self.have_feedback, self.epoch
        if not control_ns or not have_feedback:
            return  # Preserve the watchdog's existing startup grace.
        # A ROS completion can race the snapshot above. Sample send time AFTER
        # copying its evidence so it can never appear to be from the future.
        now = self.clock()
        self.sequence += 1
        heartbeat = ControllerHeartbeat(self.controller_session_id, self.sequence, now,
            control_ns, action.safe_rx_ns if action else 0,
            action.action.session_id if action else 0,
            0 <= now-feedback_ns < FEEDBACK_CAN_FAULT_NS, False, 0)
        self.watchdog.sock.sendto(encode_heartbeat(heartbeat), self.watchdog.server)
        self.pending_heartbeat = {"reply_to": {"type": "heartbeat",
            "controller_session_id": self.controller_session_id, "sequence": self.sequence,
            "sent_monotonic_ns": now}, "sent_ns": now,
            "deadline_ns": now+HEARTBEAT_REPLY_NS, "epoch": epoch}
        self.next_heartbeat_ns = now+self.period_ns

    def step(self):
        now = self.clock()
        if self.last_step_ns:
            gap = now-self.last_step_ns
            self.diagnostics["max_worker_gap_ns"] = max(self.diagnostics["max_worker_gap_ns"], gap)
            if gap > ACTION_FRESH_NS:
                self.resync = True
        self.last_step_ns = now
        with self.lock:
            reset, self.reset_requested = self.reset_requested, False
            if reset:
                self.action = None
        if reset:
            self.tracker.reset()
            self.empty_boundary_ns = None
            self.resync = True
        self._receive_actions()
        self._heartbeat_io()

    def _run(self):
        try:
            while not self.stop.is_set():
                self.step()
                now = self.clock()
                due = self.pending_heartbeat["deadline_ns"] if self.pending_heartbeat else self.next_heartbeat_ns
                timeout = min(.004, max(.0001, (due-now)/1e9)) if due else .004
                select.select([self.udp, self.watchdog.sock], [], [], timeout)
        except Exception as exc:
            # Do not restart a failed worker or synthesize a heartbeat. The
            # independent watchdog observes the real process/control gap.
            with self.lock:
                self.diagnostics["worker_error"] = f"{type(exc).__name__}: {exc}"
                self.safety_rx_ns = 0
                self.safety = {"state": "FAULT", "fault_bits": 16,
                    "reason": "follower I/O worker stopped: " + self.diagnostics["worker_error"]}
                self.fault_latched = dict(self.safety)

    def close(self):
        with self.lock:
            self.closed = True
            self.safety_rx_ns = 0
        self.stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1.)
        self.udp.close()
        self.watchdog.close()

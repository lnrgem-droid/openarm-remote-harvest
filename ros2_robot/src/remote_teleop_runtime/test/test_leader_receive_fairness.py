"""A busy/invalid feedback stream must not starve the next action timer."""
from test_leader_collection_routing import gateway
from remote_teleop_runtime import leader
from types import SimpleNamespace


def test_continuous_foreign_stream_has_finite_packet_budget(gateway):
    calls = []
    def receive(_size):
        calls.append(1)
        assert len(calls) <= 8, 'feedback receive monopolized the action timer'
        return b'unrelated', ('other-peer', 50011)
    gateway.sock.recvfrom = receive
    gateway.tick()
    assert len(calls) == 8
    assert gateway.sent == 1
    calls.clear()
    gateway.tick()
    assert gateway.sent == 2


def test_receive_time_budget_also_applies_to_rejected_peers(gateway, monkeypatch):
    clock = [10_000_000_000]
    monkeypatch.setattr(leader.time, 'monotonic_ns', lambda: clock[0])
    calls = []
    def receive(_size):
        calls.append(1)
        assert len(calls) == 1, 'receive continued after its time budget'
        clock[0] += 2_000_000
        return b'unrelated', ('other-peer', 50011)
    gateway.sock.recvfrom = receive
    gateway.tick()
    assert gateway.sent == 1


def test_malformed_reply_does_not_skip_remaining_bounded_batch(gateway):
    packets = [b'bad-one', b'bad-two']
    def receive(_size):
        if not packets:
            raise BlockingIOError
        return packets.pop(0), (gateway.peer, 50011)
    gateway.sock.recvfrom = receive
    gateway.tick()
    assert gateway.invalid == 2
    assert not packets


def test_valid_feedback_publish_work_yields_to_next_action(gateway, monkeypatch):
    clock = [10_000_000_000]
    monkeypatch.setattr(leader.time, 'monotonic_ns', lambda: clock[0])
    monkeypatch.setattr(leader, 'FollowerState', SimpleNamespace)
    gateway.state_session = None
    gateway.state_sequence = 0
    sequence = [0]
    def receive(_size):
        sequence[0] += 1
        assert sequence[0] <= gateway.sent, 'valid feedback starved action sending'
        return b'valid', (gateway.peer, 50011)
    monkeypatch.setattr(leader, 'decode_message', lambda _: SimpleNamespace(
        session_id=2, sequence=sequence[0], applied_action_session_id=gateway.session,
        applied_action_sequence=gateway.sequence))
    gateway.sock.recvfrom = receive
    gateway.publish_return = lambda _: clock.__setitem__(0, clock[0]+2_000_000)
    gateway.publish_force_feedback = lambda _: None
    gateway.tick()
    gateway.tick()
    assert gateway.sent == gateway.received == 2

from test_kernel_rx import busy


def test_stale_queue_is_measured_without_renewing_action_permission():
    worker, clock, sock = busy()
    worker.step()
    good = worker.action
    sock.age = 180_000_000
    clock.advance(200)
    worker.step()
    diag = worker.snapshot()['diagnostics']
    assert worker.action is good
    assert diag['max_queue_delay_ns'] == 180_000_000
    assert diag['last_slow_receive']['queue_delay_ns'] == 180_000_000
    assert diag['rx_kernel_stale'] == 1
    assert diag['accepted_sequence_gaps'] == 0


def test_missing_sequences_are_acceptance_gaps_not_claimed_network_loss():
    worker, clock, sock = busy()
    worker.step()
    sock.seq += 3
    clock.advance(10)
    worker.step()
    diag = worker.snapshot()['diagnostics']
    assert diag['accepted_sequence_gaps'] == 3
    assert diag['max_accepted_arrival_gap_ns'] > 0
    # Sender and receiver clocks in this fixture have different origins.
    # Telemetry compares times only within their respective clock domains.
    assert diag['max_accepted_sender_gap_ns'] == 16_000_000


def test_reset_does_not_compare_different_sessions():
    worker, clock, sock = busy()
    worker.step()
    worker.reset_requested = True
    sock.seq += 20
    clock.advance(10)
    worker.step()
    assert worker.snapshot()['diagnostics']['accepted_sequence_gaps'] == 0

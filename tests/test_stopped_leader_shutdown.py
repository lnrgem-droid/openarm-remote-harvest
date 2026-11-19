"""Read-only recovery cannot infer disabled state from silence or old frames."""
import copy
import io
import struct
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import power_down_arms as stop
import inspect_teleop_recovery as recovery
from test_disconnect_recovery import normal_power_down


def fake_can(monkeypatch, mode='disabled'):
    sent = []
    class Socket:
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def setsockopt(self, *_): pass
        def bind(self, address): self.bus = address[0]
        def setblocking(self, _): pass
        def send(self, frame):
            cid, length, flags, _, _, data = struct.unpack('=IBBBB64s', frame)
            assert (cid, length, flags, data[1:8]) == (0x7ff, 8, 1, bytes([0, 0xcc, 0, 0, 0, 0, 0]))
            self.motor = data[0]; sent.append((self.bus, self.motor))
        def recvmsg(self, *_):
            from check_motor_enable import SO_TIMESTAMPNS
            stamp = time.time_ns() - (1000000000 if mode == 'stale' else 0)
            state = 1 if mode == 'enabled' else 0
            mid = self.motor if mode != 'wrong_motor' else 8
            cid = 0x20000088 if mode == 'error' else 0x10 + mid
            frame = struct.pack('=IBBBB8s', cid, 8, 0, 0, 0, bytes([mid | state << 4, 0, 0, 0, 0, 0, 20, 20]))
            return frame, [(stop.socket.SOL_SOCKET, SO_TIMESTAMPNS,
                           struct.pack('=qq', stamp // 1000000000, stamp % 1000000000))], 0, None
    monkeypatch.setattr(stop.socket, 'socket', lambda *_: Socket())
    monkeypatch.setattr(stop.select, 'select', lambda read, *_: ([] if mode == 'missing' else read, [], []))
    monkeypatch.setattr(stop, 'stopped_leader_identity', lambda _: ('unit', 'usb'))
    return sent


def test_all_16_motors_require_three_distinct_refresh_replies(monkeypatch):
    sent = fake_can(monkeypatch)
    report = stop.query_disabled_leader(['can0', 'can1'])
    assert report['verified'] and len(sent) == 48
    assert len(report['motors']) == 16
    assert all(v['states'] == [0, 0, 0] for v in report['motors'].values())


@pytest.mark.parametrize('mode', ['missing', 'enabled', 'error', 'stale', 'wrong_motor'])
def test_missing_or_unsafe_feedback_never_arms_follower(monkeypatch, mode):
    fake_can(monkeypatch, mode)
    emitted = []
    monkeypatch.setattr(stop, 'emit', emitted.append)
    with pytest.raises(stop.ShutdownError):
        stop.verify_stopped_leader_worker(['can0', 'can1'])
    assert emitted == []


def test_device_identity_change_rejects_query_proof(monkeypatch):
    fake_can(monkeypatch)
    identities = iter(['old', 'new'])
    monkeypatch.setattr(stop, 'stopped_leader_identity', lambda _: next(identities))
    with pytest.raises(stop.ShutdownError, match='改变'):
        stop.query_disabled_leader(['can0', 'can1'])


def test_worker_queries_again_only_after_explicit_go(monkeypatch):
    fake_can(monkeypatch)
    calls = []
    monkeypatch.setattr(stop, 'query_disabled_leader', lambda _: calls.append('query') or {'verified': True})
    monkeypatch.setattr(stop.sys, 'stdin', io.StringIO('disable\n'))
    monkeypatch.setattr(stop, 'emit', lambda event: calls.append(event['event']))
    stop.verify_stopped_leader_worker(['can0', 'can1'])
    assert calls == ['query', 'armed', 'query', 'device_result']


def test_worker_without_go_never_completes(monkeypatch):
    fake_can(monkeypatch)
    calls = []
    monkeypatch.setattr(stop, 'query_disabled_leader', lambda _: calls.append('query') or {'verified': True})
    monkeypatch.setattr(stop.sys, 'stdin', io.StringIO(''))
    monkeypatch.setattr(stop, 'emit', lambda event: calls.append(event['event']))
    with pytest.raises(stop.ShutdownError): stop.verify_stopped_leader_worker(['can0', 'can1'])
    assert calls == ['query', 'armed']


def recovered_alignment(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', tmp_path / 'receipt.json')
    e = normal_power_down()
    e['local_control_processes'] = []
    e['remote']['status'].update(state='ALIGNING', aligned=False, fault_bits=0,
        reason='explicit HOLD accepted; fresh RUN request required')
    devices = copy.deepcopy(e['power_down_receipt']['devices'])
    devices['host']['verification_method'] = 'stopped_leader_refresh'
    e['power_down_receipt'] = recovery.save_disable_checkpoint(e, devices)
    return e


def test_verified_failed_start_stop_receipt_allows_explicit_restart(tmp_path, monkeypatch):
    e = recovered_alignment(tmp_path, monkeypatch)
    assert recovery.known_power_down_eligible(e)
    assert recovery.classify(e)['code'] == 'OPERATOR_POWER_DOWN_CONFIRM'
    assert recovery.complete_disable_checkpoint(e['power_down_receipt']['shutdown_id'], e)['phase'] == 'complete'


@pytest.mark.parametrize('change', ['no_method', 'active_before', 'orphan_before', 'not_hold',
                                  'fault', 'aligned', 'missing_motor', 'enabled', 'changed_session'])
def test_alignment_stop_receipt_is_narrow(tmp_path, monkeypatch, change):
    e = recovered_alignment(tmp_path, monkeypatch)
    receipt = e['power_down_receipt']; status = e['remote']['status']
    if change == 'no_method': receipt['devices']['host'].pop('verification_method')
    if change == 'active_before': receipt['host_unit_before_stop'] = {'ActiveState': 'active'}
    if change == 'orphan_before': receipt['host_control_processes'] = [123]
    if change == 'not_hold': status['reason'] = 'restart requires alignment'
    if change == 'fault': status['fault_bits'] = 1
    if change == 'aligned': status['aligned'] = True
    if change == 'missing_motor': receipt['devices']['host']['motors'].pop('can0/1')
    if change == 'enabled': e['motor_report']['follower']['motors']['can1/1']['states'] = [1]
    if change == 'changed_session': status['leader_session_id'] += 1
    assert not recovery.known_power_down_eligible(e)

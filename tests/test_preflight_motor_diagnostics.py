"""Mocked CAN only: diagnostic wording must not relax startup admission."""
import struct
from unittest.mock import Mock

import pytest

from test_start_preflight_recovery import (
    fake_can, good_reports, mock_links, pre, rebooted, recovery, start, stop,
)


def motor_feedback(monkeypatch, state, *, fd=False):
    sent = fake_can(monkeypatch)
    mock_links(monkeypatch)
    make_socket = pre.socket.socket

    def socket(*args):
        sock = make_socket(*args)
        receive = sock.recvmsg

        def recvmsg(*args):
            packet, ancillary, flags, address = receive(*args)
            data = bytearray(packet[8:16])
            data[0] = (state << 4) | (data[0] & 0xF)
            can_id = struct.unpack_from('=I', packet)[0]
            packet = (struct.pack('=IBBBB64s', can_id, 8, 5, 0, 0, data) if fd else
                      struct.pack('=IB3x8s', can_id, 8, data))
            return packet, ancillary, flags, address

        sock.recvmsg = recvmsg
        return sock

    monkeypatch.setattr(pre.socket, 'socket', socket)
    return sent


@pytest.mark.parametrize('fd', [False, True])
@pytest.mark.parametrize('state', range(2, 16))
def test_abnormal_states_preserve_raw_evidence_and_block(monkeypatch, state, fd):
    sent = motor_feedback(monkeypatch, state, fd=fd)
    report = pre.endpoint('follower', ['can1', 'can2'])
    assert report['ok'] is False
    assert report['rejected_feedback'] == {
        'interface': 'can1', 'motor': 1, 'state': state, 'can_id': 0x11,
        'data_hex': f'{(state << 4) | 1:02X}00000000001414',
    }
    assert f'0x{state:X}' in report['error']
    if 8 <= state <= 14:
        assert '故障状态' in report['error']
        assert pre.MOTOR_FAULT_NAMES[state] in report['error']
    else:
        assert f'未识别状态 {state}' in report['error']
        assert '需核对固件含义' in report['error']
        assert '故障状态' not in report['error']
    # fake_can asserts every transmitted packet is exactly a refresh query.
    assert sent == [('can1', 1)]


@pytest.mark.parametrize('fd', [False, True])
@pytest.mark.parametrize('state', [0, 1])
def test_normal_states_keep_existing_admission(monkeypatch, state, fd):
    sent = motor_feedback(monkeypatch, state, fd=fd)
    report = pre.endpoint('follower', ['can1', 'can2'])
    assert report['ok'] and len(sent) == 48
    assert 'rejected_feedback' not in report
    assert all(states == [state] * 3 for states in report['motors'].values())
    reports = good_reports()
    reports['follower'] = report
    monkeypatch.setattr(pre, 'both_endpoints', lambda *args, **kwargs: reports)
    assert pre.verify(recovery.connection_config(), require_disabled=False) == reports
    if state == 1:
        with pytest.raises(RuntimeError, match='仍有电机使能'):
            pre.verify(recovery.connection_config())
    else:
        assert pre.verify(recovery.connection_config()) == reports


def test_raw_41_reboot_recovery_never_disables_or_changes_service(monkeypatch):
    sent = motor_feedback(monkeypatch, 4, fd=True)
    reports = good_reports()
    reports['follower'] = pre.endpoint('follower', ['can1', 'can2'])
    assert reports['follower']['rejected_feedback']['data_hex'].startswith('41')
    monkeypatch.setattr(pre, 'both_endpoints', lambda *args, **kwargs: reports)
    monkeypatch.setattr(start, 'verify_motor_preflight', pre.verify)
    monkeypatch.setattr(start, 'emit', lambda _: None)
    device, stop_service, run = Mock(), Mock(), Mock()
    monkeypatch.setattr(stop, 'DeviceProcess', device)
    monkeypatch.setattr(stop, 'stop_service', stop_service)
    monkeypatch.setattr(start.subprocess, 'run', run)
    before = rebooted()
    with pytest.raises(RuntimeError, match='未识别状态 4'):
        start.recover_rebooted_peer(recovery.connection_config(), before, lambda _: before)
    device.assert_not_called()
    stop_service.assert_not_called()
    run.assert_not_called()
    assert sent == [('can1', 1)]

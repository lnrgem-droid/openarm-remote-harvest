#!/usr/bin/env python3
"""Read-only left-arm preflight. No enable, disable, reset, or motion commands."""
import argparse
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time

# Raw per-motor calibration fingerprints captured from the known healthy LEFT
# follower J1 on 2026-11-13, before/after the operator swapped the CAN cables.
# These values are read/compared only. NEVER write or copy calibration values.
FOLLOWER_LEFT_J1 = {17: '5975cc3d', 18: '404d1f38', 19: '0dd6873b', 54: 'e00f9ebf'}


def controller_owners():
    found = []
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try: args = p.read_bytes().decode().split('\0')
        except (FileNotFoundError, PermissionError, UnicodeDecodeError): continue
        names = {Path(a).name for a in args if a}
        if names & {'openarm_gravity_pd_node', 'remote-teleop-leader',
                    'remote-teleop-follower', 'remote-teleop-follower-watchdog'}:
            found.append({'pid': int(p.parent.name), 'names': sorted(names)})
    return found


def validate_bus(role, bus):
    allowed = {'leader': {'can1'}, 'follower': {'can1', 'can2'}}
    if role not in allowed or bus not in allowed[role]:
        raise ValueError('左臂接口不在本设备支持范围；禁止自动切换到右主臂接口')
    usb = (Path('/sys/class/net')/bus/'device').resolve().parent
    if (usb/'idVendor').read_text().strip() != '0c72' or (usb/'idProduct').read_text().strip() != '0011':
        raise RuntimeError('接口不是预期的双通道机械臂 CAN 盒')
    link = json.loads(subprocess.run(['ip', '-j', '-d', 'link', 'show', bus],
        check=True, capture_output=True, text=True, timeout=3).stdout)[0]
    data = link['linkinfo']['info_data']
    if not ('UP' in link['flags'] and data['state'] == 'ERROR-ACTIVE'
            and 'FD' in data.get('ctrlmode', [])
            and data.get('bittiming', {}).get('bitrate') == 1_000_000
            and data.get('data_bittiming', {}).get('bitrate') == 5_000_000):
        raise RuntimeError(f'{bus} 未就绪，要求 CAN FD 1M/5M、ERROR-ACTIVE；未自动重置总线')
    return str(usb), link['ifindex']


def query(bus, motor, command, rid=0):
    # A fresh socket per query avoids accepting a previous queued reply.
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as s:
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', motor+16, 0xC00007FF))
        s.bind((bus,)); s.settimeout(.2)
        s.send(struct.pack('=IBBBB64s', 0x7ff, 8, 1, 0, 0,
                           bytes([motor, 0, command, rid, 0, 0, 0, 0])))
        end = time.monotonic()+.2
        while time.monotonic() < end:
            p = s.recv(72)
            cid, length = struct.unpack_from('=IB', p)
            d = p[8:16]
            if cid != motor+16 or length != 8: continue
            if command == 0x33 and d[:4] == bytes([motor, 0, 0x33, rid]): return d
            if command == 0xcc and d[0] & 15 == motor: return d
    raise RuntimeError(f'{bus}/{motor} 没有本次查询反馈')


def check(role, bus, *, expected=0, require_idle=True):
    if require_idle and controller_owners():
        raise RuntimeError('仍有旧遥操/控制进程；请在机械臂可靠支撑且电机断电后停止旧服务，再运行检查')
    identity = validate_bus(role, bus)
    if role == 'follower':
        actual = {rid: query(bus, 1, 0x33, rid)[4:].hex() for rid in FOLLOWER_LEFT_J1}
        if actual != FOLLOWER_LEFT_J1:
            raise RuntimeError('这路CAN未匹配已记录的健康左从臂J1；请核对左右接线，不会自动改映射或放行')
    motors = {}
    for _ in range(3):
        for motor in range(1, 9):
            data = query(bus, motor, 0xcc)
            state = data[0] >> 4
            motors.setdefault(str(motor), []).append(state)
            if state != expected:
                raise RuntimeError(f'{bus}/{motor} 状态{state}，要求{expected}；原始帧{data.hex()}；未清错或使能')
    if validate_bus(role, bus) != identity:
        raise RuntimeError('查询期间CAN设备身份改变')
    if require_idle and controller_owners():
        raise RuntimeError('查询期间出现控制进程，取消启动')
    return {'ok': True, 'role': role, 'bus': bus, 'motors': motors, 'right_arm': 'excluded'}


def drive_states(role, bus):
    """Query drive status only while a session exists; never read registers.

    Register replies share the feedback CAN IDs and must not be injected into
    the live controller's passive status monitor. Identity is checked again by
    the ordinary preflight once both old controllers have stopped.
    """
    identity = validate_bus(role, bus)
    motors = {str(i): [] for i in range(1, 9)}
    for _ in range(3):
        for motor in range(1, 9):
            state = query(bus, motor, 0xcc)[0] >> 4
            if state not in (0, 1):
                raise RuntimeError(f'{bus}/{motor} 电机状态{state}异常；不自动清错或重启')
            motors[str(motor)].append(state)
    if validate_bus(role, bus) != identity:
        raise RuntimeError('查询期间CAN设备身份改变')
    return {'ok': True, 'role': role, 'bus': bus, 'motors': motors,
            'all_disabled': all(v == [0, 0, 0] for v in motors.values())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role', choices=['leader', 'follower'], required=True)
    p.add_argument('--bus', required=True)
    a = p.parse_args()
    try: result = check(a.role, a.bus)
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=False)); return 1
    print(json.dumps(result, ensure_ascii=False)); return 0

if __name__ == '__main__': raise SystemExit(main())

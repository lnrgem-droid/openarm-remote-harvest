#!/usr/bin/env python3
"""Bounded pre-start refresh queries; never enable motors or command motion."""
import concurrent.futures
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time


# DM-J8009P-2EC manual V1.0, feedback ERR table (pp. 4, 6).
# Unknown firmware states remain blocked; names are diagnostics only.
MOTOR_FAULT_NAMES = {
    0x8: '过压', 0x9: '欠压', 0xA: '过电流', 0xB: 'MOS 过温',
    0xC: '电机线圈过温', 0xD: '通讯丢失', 0xE: '过载',
}


def endpoint(side, interfaces, *, repair=False):
    from check_motor_enable import SO_TIMESTAMPNS, decode_feedback, feedback_receive_time
    label = '主臂（笔记本）' if side == 'host' else '从臂（Jetson）'
    result = {'side': side, 'ok': False, 'motors': {}, 'repaired': []}
    try:
        if list(interfaces) not in (['can0', 'can1'], ['can1', 'can2']):
            raise RuntimeError('接口配置不受支持')
        # Validate BOTH endpoints before changing either. Never select a device
        # by enumeration alone or touch the native CAN / four-channel adapter.
        links = {}
        devices = {}
        for bus in interfaces:
            usb = (Path('/sys/class/net') / bus / 'device').resolve().parent
            if ((usb / 'idVendor').read_text().strip() != '0c72'
                    or (usb / 'idProduct').read_text().strip() != '0011'):
                raise RuntimeError(f'{bus} 不是已配置的双通道机械臂 CAN 盒，未操作')
            link = json.loads(subprocess.run(['ip', '-j', '-d', 'link', 'show', bus],
                check=True, capture_output=True, text=True, timeout=3).stdout)[0]
            links[bus] = link
            devices[bus] = (str(usb), link['ifindex'])
        if len({p for p, _ in devices.values()}) != 1:
            raise RuntimeError('两路接口不属于同一只双通道盒')
        for bus, link in links.items():
            data = link['linkinfo']['info_data']
            ready = ('UP' in link['flags'] and data['state'] == 'ERROR-ACTIVE'
                     and 'FD' in data.get('ctrlmode', [])
                     and data.get('bittiming', {}).get('bitrate') == 1000000
                     and data.get('data_bittiming', {}).get('bitrate') == 5000000)
            if ready:
                continue
            if not repair or 'UP' in link['flags'] or data['state'] != 'STOPPED':
                raise RuntimeError(f'{bus} 未就绪（{data["state"]}），不会重置正在使用或故障的总线')
            for proc in Path('/proc').glob('[0-9]*/cmdline'):
                try:
                    args = proc.read_bytes().decode().split('\0')
                except FileNotFoundError:
                    continue
                if any(Path(arg).name in {'openarm_gravity_pd_node', 'remote-teleop-leader',
                        'remote-teleop-follower', 'bimanual_follower.launch.py', 'bimanual_leader.launch.py'} for arg in args):
                    raise RuntimeError('仍有机械臂控制进程，不能自动配置 CAN')
            for args in ([bus, 'type', 'can', 'bitrate', '1000000', 'dbitrate', '5000000', 'fd', 'on'],
                         [bus, 'up']):
                p = subprocess.run(['sudo', '-n', 'ip', 'link', 'set', *args],
                    capture_output=True, text=True, timeout=4)
                if p.returncode:
                    raise RuntimeError(f'{bus} 自动配置未获权限或失败；请在桌面启动器完成 CAN 配置，无需重启或断电')
            result['repaired'].append(bus)
        if repair:
            result['ok'] = True
            return result
        for bus in interfaces:
            with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                    b''.join(struct.pack('=II', i, 0xC00007FF) for i in range(0x11, 0x19)))
                sock.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack('=I', 0x1fffffff))
                sock.bind((bus,)); sock.setblocking(False)
                for _ in range(3):
                    for motor in range(1, 9):
                        sent = time.monotonic()
                        sock.send(struct.pack('=IBBBB64s', 0x7ff, 8, 1, 0, 0,
                                             bytes([motor, 0, 0xcc, 0, 0, 0, 0, 0])))
                        deadline = sent + .08
                        answered = False
                        while time.monotonic() < deadline:
                            if not select.select([sock], [], [], max(0., deadline-time.monotonic()))[0]:
                                break
                            packet, ancillary, flags, _ = sock.recvmsg(72, socket.CMSG_SPACE(16))
                            if struct.unpack_from('=I', packet)[0] & 0x20000000:
                                raise RuntimeError(f'{bus} 查询出现总线错误；检查电机电源、急停和 CAN 接头')
                            stamp = feedback_receive_time(ancillary, flags, time.time_ns(), time.monotonic())
                            decoded = decode_feedback(packet) if stamp is not None and stamp >= sent else None
                            if decoded is None:
                                continue
                            if decoded[1] not in (0, 1):
                                motor_id, state = decoded
                                result['rejected_feedback'] = {
                                    'interface': bus, 'motor': motor_id, 'state': state,
                                    'can_id': struct.unpack_from('=I', packet)[0],
                                    'data_hex': packet[8:16].hex().upper(),
                                }
                                name = MOTOR_FAULT_NAMES.get(state)
                                description = (f'故障状态 {state}（0x{state:X}：{name}）' if name else
                                    f'未识别状态 {state}（0x{state:X}，需核对固件含义）')
                                raise RuntimeError(f'{bus} 电机 {motor_id} {description}，不自动清故障')
                            if decoded[0] == motor:
                                result['motors'].setdefault(f'{bus}/{motor}', []).append(decoded[1])
                                answered = True
                                break
                        if not answered:
                            raise RuntimeError(f'{bus} 电机 {motor} 没有本次反馈；请确认电机电源已接通、急停已释放、CAN 插头牢固。USB 亮灯不能证明电机供电')
            current = json.loads(subprocess.run(['ip', '-j', '-d', 'link', 'show', bus],
                check=True, capture_output=True, text=True, timeout=3).stdout)[0]
            usb = str((Path('/sys/class/net') / bus / 'device').resolve().parent)
            if ((usb, current['ifindex']) != devices[bus]
                    or current['linkinfo']['info_data']['state'] != 'ERROR-ACTIVE'):
                raise RuntimeError(f'{bus} 查询期间设备或总线状态改变')
        result['ok'] = len(result['motors']) == 16 and all(
            len(v) == 3 and len(set(v)) == 1 for v in result['motors'].values())
        if not result['ok']:
            raise RuntimeError('电机反馈不完整或状态变化，未通过启动预检')
    except Exception as exc:
        result['error'] = f'{label}：{exc}'
    return result


def both_endpoints(config, *, repair=False):
    """Stream this exact implementation over SSH; remote installs may be stale."""
    def remote():
        checker = Path(__file__).with_name('check_motor_enable.py').read_text()
        source = Path(__file__).read_text()
        code = ('import types,sys,json\nm=types.ModuleType("check_motor_enable")\n'
                + 'exec(' + repr(checker) + ',m.__dict__);sys.modules[m.__name__]=m\n'
                + 'ns={"__name__":"motor_preflight"}\nexec(' + repr(source) + ',ns)\n'
                + 'print(json.dumps(ns["endpoint"]("follower", '
                + repr(config['follower_interfaces']) + ', repair=' + repr(repair) + ')))\n')
        try:
            p = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
                config['peer'], '/usr/bin/python3 -'], input=code, capture_output=True,
                text=True, timeout=20)
            if p.returncode:
                raise RuntimeError('SSH 查询失败：' + p.stderr[-300:])
            return json.loads(p.stdout)
        except Exception as exc:
            return {'ok': False, 'error': '从臂（Jetson）：' + str(exc)}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        a = pool.submit(endpoint, 'host', config['host_interfaces'], repair=repair)
        b = pool.submit(remote)
        return {'host': a.result(), 'follower': b.result()}


def verify(config, *, require_disabled=True, repair=False):
    reports = both_endpoints(config, repair=repair)
    errors = []
    if not isinstance(reports, dict) or set(reports) != {'host', 'follower'}:
        raise RuntimeError('主从两端电机查询结果不完整，未通过启动预检')
    for side, report in reports.items():
        if not isinstance(report, dict):
            errors.append(side + ' 电机查询结果格式错误')
            continue
        if report.get('ok') is not True:
            errors.append(report.get('error', side + ' 未通过检查'))
        elif not repair:
            expected = {f'{bus}/{j}' for bus in config[side + '_interfaces'] for j in range(1, 9)}
            values = report.get('motors', {})
            if (not isinstance(values, dict) or set(values) != expected
                    or any(not isinstance(v, list) or len(v) != 3
                    or any(type(s) is not int or s not in (0, 1) for s in v)
                    or len(set(v)) != 1 for v in values.values())):
                errors.append(side + ' 电机反馈未完整核验')
            elif require_disabled and any(v != [0, 0, 0] for v in values.values()):
                errors.append(('主臂' if side == 'host' else '从臂') + '仍有电机使能，不能直接重启归位')
    if errors:
        raise RuntimeError('启动前检查未通过：' + '；'.join(errors) + '。未启动新的控制器或归位')
    return reports


if __name__ == '__main__':
    import argparse
    import os
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jetson', required=True)
    parser.add_argument('--follower-interfaces', nargs=2, required=True)
    args = parser.parse_args()
    profile = 'host-test' if args.jetson == 'openarm-follower-test' else 'jetson'
    os.environ['OPENARM_TELEOP_PROFILE'] = profile
    from inspect_teleop_recovery import connection_config
    config = connection_config()
    if args.jetson != config['peer'] or args.follower_interfaces != config['follower_interfaces']:
        parser.error('目标或接口与已配置的机械臂不一致')
    try:
        verify(config)
    except Exception as exc:
        print('ERROR: ' + str(exc), flush=True)
        raise SystemExit(1)
    print('启动前供电与反馈检查通过：32/32 电机均有连续新反馈且已失能。', flush=True)

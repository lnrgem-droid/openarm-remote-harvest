#!/usr/bin/env python3
"""Explicit motor-power-isolated cleanup after peer reboot or failed startup.

Does not start controllers, remove fault evidence, or certify motor disable.
The operator must physically support the arms and isolate both motor supplies.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

from inspect_teleop_recovery import (
    connection_config, inspect as inspect_recovery, peer_reboot_cleanup_available,
    failed_start_cleanup_available, empty_power_off_stack,
)
from power_down_arms import RecorderClient, emit, require_reply, service_stopped
from start_teleop_from_console import identity, start_decision, unit_status


def cleanup_kind(result):
    evidence = result.get('evidence') or {}
    if peer_reboot_cleanup_available(evidence):
        return 'peer_reboot'
    if failed_start_cleanup_available(evidence):
        return 'failed_start'
    # Cleanup completion is independent of CAN recovery. Never claim that an
    # empty stack on a BUS-OFF interface is ready to start.
    try:
        if empty_power_off_stack(evidence):
            return 'already_empty'
    except (RuntimeError, KeyError, TypeError, ValueError, AttributeError):
        pass
    return None


def stop_isolated_follower(peer, evidence):
    source = Path(__file__).with_name('terminate_isolated_follower.py').read_text()
    result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', peer,
        '/usr/bin/python3 -c ' + shlex.quote(source)], input=json.dumps(evidence['remote']),
        capture_output=True, text=True, timeout=23, check=True)
    reply = json.loads(result.stdout)
    if reply.get('stopped_pids') != sorted(evidence['remote']['process_identities']):
        raise RuntimeError('从端清理回执不匹配；保持电机电源关闭')


def cleanup(config, *, power_off_confirmed=False, inspect_fn=inspect_recovery):
    if power_off_confirmed is not True:
        raise RuntimeError("请先支撑四臂并确认两端机械臂电机电源已关闭；电脑保持开机")
    peer, service = config['peer'], config['service']
    emit({'event': 'progress', 'message': '核对遥操旧会话、两端电机反馈和录制状态；请保持电机电源关闭…'})
    before = inspect_fn(peer)
    kind = cleanup_kind(before)
    if kind is None:
        raise RuntimeError("未满足断电清理条件：可能仍有电机反馈、正常控制或录制任务。未清理任何进程")
    again = inspect_fn(peer)
    if (cleanup_kind(again) != kind
            or identity(before) != identity(again)
            or sorted(before['evidence']['local_control_processes']) !=
               sorted(again['evidence']['local_control_processes'])):
        raise RuntimeError("检查期间会话或电机状态改变，取消清理")
    recorder = require_reply(RecorderClient(f"tcp://{config['peer_ip']}:5557")('status'))
    if (recorder.get('running') is not False or 'active_episode' not in recorder
            or recorder['active_episode'] is not None or recorder.get('phase') not in ('idle', 'error')):
        raise RuntimeError("录制器不再空闲，取消清理")
    # Check the systemd instance again immediately before stopping this one unit.
    unit = unit_status(service)
    expected = again['evidence']['host_unit']
    if any(unit.get(k) != expected.get(k) for k in ('InvocationID', 'MainPID', 'ActiveState')):
        raise RuntimeError("旧服务已被替换，取消清理")
    if kind == 'peer_reboot':
        emit({'event': 'progress', 'message': '正在停止笔记本旧遥操进程；保持电机电源关闭…'})
        subprocess.run(['systemctl', '--user', 'stop', service], check=True,
                       capture_output=True, text=True, timeout=25)
    elif kind == 'failed_start':
        audit_root = Path(__file__).resolve().parents[1] / 'work' / 'power-off-recovery'
        audit_root.mkdir(parents=True, exist_ok=True)
        audit = Path(tempfile.mkdtemp(prefix='failed-start-', dir=audit_root))
        (audit / 'before.json').write_text(json.dumps(again, ensure_ascii=False, indent=2))
        emit({'event': 'progress', 'message': '仅清理已核验的从臂遥操残留进程；不会操作其他程序…'})
        stop_isolated_follower(peer, again['evidence'])
    after = inspect_fn(peer)
    if kind == 'failed_start':
        (audit / 'after.json').write_text(json.dumps(after, ensure_ascii=False, indent=2))
    e = after['evidence']
    stopped = '\n'.join(f'{k}={v}' for k, v in e.get('host_unit', {}).items())
    if (not service_stopped(stopped)
            or e.get('local_control_processes') != []
            or e.get('host_boot_id') != again['evidence']['host_boot_id']
            or e.get('remote', {}).get('boot_id') != again['evidence']['remote']['boot_id']
            or not empty_power_off_stack(e)):
        raise RuntimeError("旧会话清理结果未完全确认，请保持电机电源关闭并检查；不会自动启动")
    can_pending = e['remote'].get('can_ok') is not True
    next_step = ('保持电机电源关闭，重新打开桌面采集启动器恢复从臂 CAN 接口；'
                 'CAN 未恢复前不要开始遥操。' if can_pending else
                 '可恢复两端电机电源，再单独点击“开始主从遥操…”并确认归位。')
    return {'event': 'complete', 'ok': True, 'operation': 'cleanup', 'can_recovery_required': can_pending,
            'message': ('两端已无遥操残留进程，无需重复清理。' if kind == 'already_empty'
                        else '旧会话已清理。') + '未启动遥操，也未生成电机失能回执。' + next_step}


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--confirm-power-off', action='store_true')
    parser.add_argument('--profile', choices=('jetson', 'host-test'), default='jetson')
    parser.add_argument('--jetson', required=True)
    parser.add_argument('--recorder', required=True)
    args = parser.parse_args()
    if not args.confirm_power_off:
        parser.error('需要单独确认：四臂已支撑，两端机械臂电机电源已物理关闭')
    os.environ['OPENARM_TELEOP_PROFILE'] = args.profile
    config = connection_config()
    if args.jetson != config['peer'] or args.recorder != f"tcp://{config['peer_ip']}:5557":
        parser.error('连接目标与遥操配置不一致')
    try:
        with open('/tmp/openarm-teleop-lifecycle.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with open('/tmp/openarm-daily-teleop.lock', 'a') as daily:
                fcntl.flock(daily, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = cleanup(config, power_off_confirmed=True)
    except Exception as exc:
        result = {'event': 'complete', 'ok': False, 'operation': 'cleanup',
                  'message': '清理未确认：' + str(exc) + '。保持电机电源关闭；不会自动重试或启动。'}
    emit(result)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

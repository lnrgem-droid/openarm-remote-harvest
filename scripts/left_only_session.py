#!/usr/bin/env python3
"""Explicit left-only session control. No startup on import or status checks."""
import argparse
import concurrent.futures
from contextlib import contextmanager, nullcontext
import fcntl
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import tempfile
import time
from left_only_device import check, drive_states
from verify_left_release import verify

ROOT = Path(__file__).resolve().parents[1]
SOCKET = '/tmp/openarm_left_runtime.sock'
UNITS = {'leader': 'openarm-left-leader.service', 'follower': 'openarm-left-follower.service'}


@contextmanager
def operation_lock():
    path = Path.home()/'.local/state/openarm-left-only/session-operation.lock'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('另一个单左臂启动/停止操作正在执行，请等待完成')
        try: yield
        finally: fcntl.flock(lock, fcntl.LOCK_UN)


def run(args, timeout=10):
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if p.returncode: raise RuntimeError(p.stderr.strip() or p.stdout.strip() or str(args))
    return p.stdout


def config():
    c = json.loads((ROOT/'left_only_config.json').read_text())
    if c.get('role') != 'leader': raise RuntimeError('请在笔记本主臂端执行会话命令')
    if c['peer'].startswith('-'): raise RuntimeError('invalid SSH peer')
    return c


def remote(c, args, timeout=10):
    return run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3', c['peer'], shlex.join(args)], timeout)


def rpc_source(command):
    return '''import json,socket,tempfile
with tempfile.TemporaryDirectory(prefix='oa_left_rpc_') as d:
 with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
  s.bind(d+'/reply');s.settimeout(2.)
  s.sendto(json.dumps({'command':COMMAND}).encode(), '/tmp/openarm_left_runtime.sock')
  print(s.recv(65536).decode())
'''.replace('COMMAND', repr(command))


def control(c, command):
    result = json.loads(remote(c, ['python3', '-c', rpc_source(command)], 5))
    if 'error' in result: raise RuntimeError(result['error'])
    return result


def left_status(s, *, running=False):
    if s.get('mode') != 'left_only' or s.get('enabled_arms') != ['left']:
        raise RuntimeError('返回的不是独立单左臂会话')
    if s.get('state') in ('FAULT', 'E_STOP') or s.get('fault_bits'):
        raise RuntimeError('左臂保护已锁存：' + str(s.get('reason')))
    if running and (s.get('state') != 'RUNNING' or not s.get('feedback_fresh_for_control')
                    or (s.get('motor_health') or {}).get('healthy') is not True
                    or not 0 <= s.get('action_age_ms', 9999) <= 100):
        raise RuntimeError('左臂跟随/反馈尚未通过验证')
    return s


class RecoveryNeedsSupport(RuntimeError):
    """Exit 3 lets either UI request support before a torque-changing restart."""


def unit_state(c, role):
    args = ['systemctl', '--user', 'show', UNITS[role], '-p', 'ActiveState', '-p', 'MainPID']
    text = run(args) if role == 'leader' else remote(c, args)
    return dict(line.split('=',1) for line in text.splitlines() if '=' in line)


def is_idle(fields):
    return fields.get('ActiveState') in ('inactive', 'failed') and fields.get('MainPID') == '0'


def idle_unit(c, role):
    fields = unit_state(c, role)
    if fields.get('ActiveState') not in ('inactive', 'failed') or fields.get('MainPID') != '0':
        raise RuntimeError(f'{role} 单左臂服务不处于停止状态；不覆盖现有会话')


def verify_versions(c):
    # Verify the exact release before either endpoint can start moving.
    verify(ROOT)
    remote(c, ['python3', c['peer_root']+'/scripts/verify_left_release.py'], 30)
    local_manifest = (ROOT/'release.sha256').read_text().strip()
    remote_manifest = remote(c, ['cat', c['peer_root']+'/release.sha256']).strip()
    if local_manifest != remote_manifest:
        raise RuntimeError('主从更新包版本不一致，未启动')


def preflight(c):
    for role in UNITS: idle_unit(c, role)
    verify_versions(c)
    command = ['python3', c['peer_root']+'/scripts/left_only_device.py',
               '--role', 'follower', '--bus', c['follower_bus']]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        a = pool.submit(check, 'leader', c['leader_bus'])
        b = pool.submit(remote, c, command, 10)
        host, peer = a.result(), json.loads(b.result())
    if not host.get('ok') or not peer.get('ok'): raise RuntimeError('左臂反馈预检失败')
    return {'host': host, 'follower': peer}


def disabled_pair(c):
    code = ('import sys,json;sys.path.insert(0,'+repr(c['peer_root']+'/scripts')+');'
            'from left_only_device import drive_states;print(json.dumps(drive_states("follower",'
            +repr(c['follower_bus'])+')))')
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        host = pool.submit(drive_states, 'leader', c['leader_bus'])
        peer = pool.submit(remote, c, ['python3', '-c', code], 12)
        reports = [host.result(), json.loads(peer.result())]
    return all(r.get('ok') is True and r.get('all_disabled') is True
               and set(r.get('motors', {})) == {str(i) for i in range(1, 9)}
               and all(v == [0, 0, 0] for v in r['motors'].values()) for r in reports)


def prepare_start(c, arms_supported=False):
    """Reuse healthy RUNNING; recover disabled stale sessions, never clear faults online."""
    states = {role: unit_state(c, role) for role in UNITS}
    if all(is_idle(s) for s in states.values()): return None
    if any(s.get('ActiveState') not in ('active', 'inactive', 'failed') for s in states.values()):
        raise RuntimeError('单左臂服务正在启动或停止，请稍候再试')
    try: status = control(c, 'status')
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired): status = {}
    try:
        left_status(status, running=True)
        if all(s.get('ActiveState') == 'active' for s in states.values()):
            print('单左臂遥操已经正常运行，无需重复启动。', flush=True)
            return status
    except RuntimeError: pass
    # Only a latched, identified left-only session is eligible for unattended
    # cleanup. A controller still initializing must not be stopped in a gap
    # before it enables its motors.
    faulted_left = (status.get('mode') == 'left_only' and status.get('enabled_arms') == ['left']
                    and status.get('state') in ('FAULT', 'E_STOP'))
    disabled = disabled_pair(c)
    if not (faulted_left and disabled):
        if not arms_supported:
            raise RecoveryNeedsSupport('旧会话尚未结束，可能仍带力矩。请可靠支撑左主臂和左从臂，再确认“支撑后重新启动”；将先失能停止旧会话，再对齐到已保存的垂落初始位。')
        print('已确认两臂支撑：先失能并停止旧会话。', flush=True)
        disable(c)
    else:
        if states != {role: unit_state(c, role) for role in UNITS}:
            raise RuntimeError('检查期间服务发生变化，取消清理，请重试')
        # Recheck immediately before stopping; no missing/stale/error reply
        # can be treated as evidence that a drive is disabled.
        if not disabled_pair(c):
            raise RecoveryNeedsSupport('电机状态已变化，请支撑两臂后再确认重新启动')
        print('两端16颗电机已确认失能，自动清理故障旧会话。', flush=True)
        run(['systemctl','--user','stop',UNITS['leader']], 20)
        remote(c, ['systemctl','--user','stop',UNITS['follower']], 20)
    for role in UNITS: idle_unit(c, role)
    return None


def home_log(c, role):
    rel = '.local/state/openarm-left-only/'+role+'.log'
    if role == 'leader':
        path = Path.home()/rel
        return path.read_text() if path.exists() else ''
    return remote(c, ['python3', '-c', 'from pathlib import Path; p=Path.home()/'+repr(rel)+'; print(p.read_text() if p.exists() else "")'])


def await_startup_hold(c):
    end = time.monotonic()+60
    last = '等待左主从臂初始化'
    while time.monotonic() < end:
        logs = [home_log(c, role) for role in UNITS]
        if any('Startup homing timed out' in log or 'process has died' in log or 'Traceback' in log for log in logs):
            raise RuntimeError('左臂初始化失败或进程退出；查看两端左臂日志')
        try:
            s = control(c, 'status')
        except (RuntimeError, OSError, ValueError, subprocess.TimeoutExpired) as exc:
            last = str(exc); time.sleep(.3); continue
        left_status(s)
        if all('Startup homing reached upstream initial pose' in log for log in logs) and s.get('leader_session_id'):
            return
        time.sleep(.3)
    raise RuntimeError('原位保持等待超时：'+last)


def verify_alignment(c, phase):
    try:
        result = json.loads(remote(c, ['python3', c['peer_root']+'/scripts/verify_startup_alignment.py',
                                     '--left-only', '--policy', c['peer_root']+'/scripts/left_startup_alignment_policy.json', '--phase', phase], 22))
    except RuntimeError as exc:
        try: result = json.loads(str(exc))
        except ValueError: raise
    if result.get('accepted') is not True:
        raise RuntimeError('启动检查未通过：'+'；'.join(result.get('issues') or ['未收到有效验收结果']))


def ros_call(c, role, service, service_type, request):
    root = str(ROOT) if role == 'leader' else c['peer_root']
    script = 'source '+shlex.quote(root+'/scripts/left_only_env.sh')+'; exec timeout 12 ros2 service call '+shlex.join([service,service_type,request])
    args = ['bash', '-c', script]
    text = run(args, 15) if role == 'leader' else remote(c, args, 17)
    if 'success=True' not in text and 'success=true' not in text:
        raise RuntimeError(role+'服务未确认成功：'+text)


def start(c, arms_supported=False):
    print('预检左主/左从16颗电机及两端版本；右臂不参与。', flush=True)
    verify_versions(c)
    existing = prepare_start(c, arms_supported)
    if existing is not None: return existing
    preflight(c)
    print('启动左主从臂：缓慢对齐到以左主臂为基准保存的共同垂落初始位，检查通过后进入绝对位置跟随。', flush=True)
    started = False
    try:
        started = True  # A timed-out SSH call may still have started the service.
        remote(c, ['systemctl', '--user', 'start', UNITS['follower']])
        run(['systemctl', '--user', 'start', UNITS['leader']])
        await_startup_hold(c)
        verify_alignment(c, 'aligning')
        reply = control(c, 'align')
        if reply.get('state') != 'READY': raise RuntimeError('ALIGN未获确认')
        reply = control(c, 'run')
        if reply.get('state') != 'RUNNING': raise RuntimeError('RUN未获确认')
        for role in ('leader', 'follower'):
            ros_call(c, role, f'/{role}/openarm_gravity_pd/startup_hold', 'std_srvs/srv/SetBool', '{data: false}')
        verify_alignment(c, 'following')
        s = left_status(control(c, 'status'), running=True)
        print('左臂启动检查通过：仅左主臂→左从臂跟随，右臂未启动。首次现场验收请小幅低速检查方向及断联保持。', flush=True)
        print('暂停：left_only_session.py hold；支撑左主从臂后停止：left_only_session.py disable --arms-supported', flush=True)
        return s
    except BaseException:
        if started:
            try: control(c, 'hold')
            except Exception: pass  # Watchdog independently closes stale motion permission.
            print('启动未完成；已请求从臂保持，控制服务可能仍带力矩。再次选择启动可进入恢复流程；需要停止旧会话时会要求确认两臂已支撑。', flush=True)
        raise


def disable(c):
    # Operator explicitly confirmed support. Disable only the two left-only
    # controller instances, then require fresh disabled drive evidence.
    s = control(c, 'status')
    if s.get('mode') != 'left_only' or s.get('enabled_arms') != ['left']:
        raise RuntimeError('无法确认左臂会话身份；不调用共享ROS失能服务')
    control(c, 'disable')
    ros_call(c, 'leader', '/leader/openarm_gravity_pd/disable', 'std_srvs/srv/Trigger', '{}')
    code = ('import sys,json;sys.path.insert(0,'+repr(c['peer_root']+'/scripts')+');'
            'from left_only_device import check;print(json.dumps(check("follower",'+repr(c['follower_bus'])+',require_idle=False)))')
    deadline = time.monotonic()+5
    while True:
        try:
            check('leader', c['leader_bus'], require_idle=False)
            remote(c, ['python3','-c',code])
            break
        except Exception:
            if time.monotonic() >= deadline: raise
            time.sleep(.2)
    run(['systemctl','--user','stop',UNITS['leader']], 20)
    remote(c, ['systemctl','--user','stop',UNITS['follower']], 20)
    return {'ok': True, 'message': '左主从16颗电机已确认失能，单左臂服务已停止；右臂未操作'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['check', 'start', 'status', 'hold', 'disable'])
    p.add_argument('--workspace-clear', action='store_true', help='现场确认左主从臂垂落姿态对应，活动范围无人，右臂已隔离且接线正确')
    p.add_argument('--arms-supported', action='store_true', help='左主从臂已可靠支撑，可以失能')
    a = p.parse_args()
    if a.command == 'start' and not a.workspace_clear: p.error('启动会使能并对齐左主从臂到共同垂落初始位；现场确认后加 --workspace-clear')
    if a.command == 'disable' and not a.arms_supported: p.error('失能前先支撑左主从臂，再加 --arms-supported')
    try:
        c = config()
        with operation_lock() if a.command in ('start', 'disable', 'hold') else nullcontext():
            if a.command == 'check': r = preflight(c)
            elif a.command == 'start': r = start(c, a.arms_supported)
            elif a.command == 'disable': r = disable(c)
            else: r = control(c, a.command)
        print(json.dumps(r, ensure_ascii=False, indent=2))
        return 0
    except RecoveryNeedsSupport as exc:
        print('NEEDS_ARM_SUPPORT: '+str(exc)); return 3
    except Exception as exc:
        print('ERROR: '+str(exc)); return 1

if __name__ == '__main__': raise SystemExit(main())

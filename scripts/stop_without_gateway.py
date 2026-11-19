"""Explicit supported-arm stop when the follower gateway has exited.

Never used to authorize start. A separate journal permits cleanup retries only
with unchanged process/boot identities AND newly queried disabled motor states.
No hardware access on import; the caller owns both lifecycle locks.
"""
import copy
import json
import os
from pathlib import Path
import shlex
import subprocess
import time

JOURNAL = Path.home() / '.local/state/openarm/gateway-loss-stop.json'
ROLES = {'bimanual_follower.launch.py', 'remote-teleop-follower-watchdog',
         'openarm_gravity_pd_node', 'remote-teleop-follower'}


def host_stopped(e):
    unit = e.get('host_unit', {})
    return (unit.get('MainPID') == '0' and unit.get('ControlGroup') == ''
            and unit.get('ActiveState') in ('inactive', 'failed')
            and e.get('local_control_processes') == [])


def gateway_absent(e):
    r = e.get('remote', {})
    return (e.get('ssh_ok') is True and r.get('status', 'missing') is None
            and r.get('runtime_socket_exists') is False
            and isinstance(r.get('process_identities'), dict)
            and not any('remote-teleop-follower' in {Path(a).name for a in p.get('argv', [])}
                        for p in r['process_identities'].values()))


def require_snapshot(e, config, *, initial=False):
    r = e.get('remote', {})
    unit = e.get('host_unit', {})
    if (not gateway_absent(e) or e.get('profile') != config['profile']
            or e.get('peer') != config['peer'] or not e.get('host_boot_id')
            or not r.get('boot_id') or e.get('host_can_ok') is not True
            or r.get('can_ok') is not True or r.get('recorder_idle') is not True
            or unit.get('LoadState') != 'loaded' or not unit.get('InvocationID')
            or not isinstance(e.get('local_control_processes'), list)):
        raise RuntimeError('网关退出后的 SSH、CAN、录制或会话身份未完整核验')
    identities = r['process_identities']
    if set(map(str, r.get('processes', []))) != set(identities):
        raise RuntimeError('从端进程清单不完整')
    seen = set()
    for pid, identity in identities.items():
        roles = {Path(a).name for a in identity.get('argv', [])} & ROLES
        if (not str(pid).isdigit() or int(pid) <= 1
                or type(identity.get('start_ticks')) is not int or identity['start_ticks'] <= 0
                or len(roles) != 1 or roles & seen or 'remote-teleop-follower' in roles):
            raise RuntimeError('从端残留控制进程不符合已知网关退出场景')
        if 'openarm_gravity_pd_node' in roles and '__node:=follower_gravity_pd' not in identity['argv']:
            raise RuntimeError('从臂控制器身份不明')
        seen.update(roles)
    if initial and 'openarm_gravity_pd_node' not in seen:
        raise RuntimeError('从臂失能服务的所属控制器不存在')
    if not host_stopped(e) and not (unit.get('ActiveState') == 'active'
            and str(unit.get('MainPID', '')).isdigit() and int(unit['MainPID']) > 1
            and unit.get('ControlGroup') and e['local_control_processes']):
        raise RuntimeError('主臂控制服务状态未确认')


def require_same(before, now, config, *, cleanup=False):
    require_snapshot(now, config)
    for key in ('host_boot_id', 'profile', 'peer'):
        if now.get(key) != before.get(key):
            raise RuntimeError('停止期间主机身份已改变')
    if (now['remote']['boot_id'] != before['remote']['boot_id']
            or now['host_unit']['InvocationID'] != before['host_unit']['InvocationID']):
        raise RuntimeError('停止期间控制会话已改变')
    if not (cleanup and host_stopped(now)) and (
            now['host_unit'] != before['host_unit']
            or now['local_control_processes'] != before['local_control_processes']):
        raise RuntimeError('停止期间主臂进程已改变')
    old, new = before['remote']['process_identities'], now['remote']['process_identities']
    if (not cleanup and old != new) or any(old.get(pid) != value for pid, value in new.items()):
        raise RuntimeError('停止期间从端进程已改变，未清理新进程')


def write_journal(value):
    JOURNAL.parent.mkdir(parents=True, exist_ok=True)
    temporary = JOURNAL.with_suffix('.pending')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush(); os.fsync(stream.fileno())
    temporary.replace(JOURNAL)
    fd = os.open(JOURNAL.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def read_journal():
    try:
        value = json.loads(JOURNAL.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def refresh_disabled(config):
    from preflight_motor_feedback import verify
    reports = verify(config, require_disabled=True, repair=False)
    # Validate the boundary too: never turn missing evidence or booleans into 0.
    if set(reports) != {'host', 'follower'}:
        raise RuntimeError('两端新反馈不完整')
    for side, report in reports.items():
        expected = {f'{bus}/{i}' for bus in config[side + '_interfaces'] for i in range(1, 9)}
        motors = report.get('motors', {})
        if (report.get('ok') is not True or set(motors) != expected
                or any(states != [0, 0, 0] or any(type(s) is not int for s in states)
                       for states in motors.values())):
            raise RuntimeError('32个电机未全部返回连续的新失能反馈')
    return reports


def verify_disable_reports(config, reports):
    """Allow an idempotent disable with silence only after fresh read-only proof."""
    if set(reports) != {'host', 'follower'}:
        raise RuntimeError('两端失能回执不完整')
    for side, report in reports.items():
        expected = {f'{bus}/{i}' for bus in config[side + '_interfaces'] for i in range(1, 9)}
        motors = report.get('motors', {})
        normal = report.get('ok') is True and report.get('verified') is True
        quiet = (report.get('ok') is False and report.get('verified') is False
                 and str(report.get('error', '')).startswith('缺少连续失能反馈：'))
        if (report.get('acknowledged') is not True or not (normal or quiet)
                or set(motors) != expected):
            raise RuntimeError(side + ' 失能服务未确认，保留控制器等待处理')
        for value in motors.values():
            states = value.get('states')
            if (not isinstance(states, list) or len(states) > 3
                    or any(type(s) is not int or s != 0 for s in states)):
                raise RuntimeError(side + ' 失能回执包含使能、故障或无效反馈')
    fresh = refresh_disabled(config)
    verified = copy.deepcopy(reports)
    for side, report in verified.items():
        report.update(ok=True, verified=True, error=None, verification='fresh_read_only_query',
                      motors={key: {'states': states, 'disabled': True}
                              for key, states in fresh[side]['motors'].items()})
    return verified


def terminate_follower(config, evidence):
    from power_down_arms import ssh_args
    source = Path(__file__).with_name('terminate_isolated_follower.py').read_text()
    reply = subprocess.run(ssh_args(config['peer']) + ['/usr/bin/python3 -c ' + shlex.quote(source)],
                           input=json.dumps(evidence['remote']), capture_output=True,
                           text=True, check=True, timeout=23)
    if json.loads(reply.stdout).get('stopped_pids') != sorted(evidence['remote']['process_identities']):
        raise RuntimeError('从端进程退出回执不匹配')


def finish_cleanup(config, inspect_fn, receipt, progress):
    from power_down_arms import stop_service
    before = receipt['before']
    def checked():
        now = inspect_fn(config['peer'])['evidence']
        require_same(before, now, config, cleanup=True)
        return now
    checked()
    refresh_disabled(config)
    now = checked()
    if not host_stopped(now):
        stop_service(config['service'])
    checked()
    refresh_disabled(config)
    now = checked()
    if now['remote']['process_identities']:
        terminate_follower(config, now)
    after = checked()
    if not host_stopped(after) or after['remote']['process_identities']:
        raise RuntimeError('电机失能已核验，但控制进程退出未完成，可再次点击停止重试')
    refresh_disabled(config)
    checked()
    receipt.update(phase='complete', completed_unix_s=time.time())
    write_journal(receipt)


def try_stop_without_gateway(config, inspect_fn, progress):
    import power_down_arms as stop
    from inspect_teleop_recovery import verified_disable_reports
    initial = inspect_fn(config['peer'])['evidence']
    if not gateway_absent(initial):
        return None  # An unresponsive/live gateway is not an absent gateway.
    receipt = read_journal()
    if (receipt.get('schema_version') == 1 and receipt.get('phase') in ('drives_disabled', 'complete')
            and isinstance(receipt.get('before'), dict)
            and verified_disable_reports(receipt.get('devices'))):
        try:
            require_same(receipt['before'], initial, config, cleanup=True)
        except (RuntimeError, KeyError, TypeError):
            pass  # An older boot/session is never reusable evidence.
        else:
            stop.seal_recording(stop.RecorderClient(f"tcp://{config['peer_ip']}:5557"))
            progress({'event': 'progress', 'stage': 'verifying', 'message': '重新查询32个电机，续做网关退出后的停止清理'})
            finish_cleanup(config, inspect_fn, receipt, progress)
            result = {'event': 'complete', 'ok': True, 'stage': 'complete', 'devices': receipt['devices'],
                      'message': '停止流程已完成：32个电机已重新核验失能，两端控制进程已退出。'}
            progress(result)
            return result
    if not initial['remote']['process_identities']:
        return None
    progress({'event': 'progress', 'stage': 'holding',
              'message': '从臂网关已退出，改用两端 ROS 失能服务；保持四臂稳固支撑'})
    def capture():
        e = inspect_fn(config['peer'])['evidence']
        require_snapshot(e, config, initial=True)
        require_same(initial, e, config)
        return e
    def recheck(before):
        require_same(before, inspect_fn(config['peer'])['evidence'], config)
    def verify(before, reports):
        recheck(before)
        result = verify_disable_reports(config, reports)
        recheck(before)
        return result
    def checkpoint(before, reports):
        nonlocal receipt
        if not verified_disable_reports(reports):
            raise RuntimeError('禁止保存未完整核验的失能回执')
        # Do not reuse the normal session receipt: no live gateway session exists.
        identity = {key: before[key] for key in ('ssh_ok', 'profile', 'peer', 'host_can_ok',
                    'host_boot_id', 'host_unit', 'local_control_processes')}
        identity['remote'] = {key: before['remote'][key] for key in ('boot_id', 'processes',
            'process_identities', 'status', 'runtime_socket_exists', 'recorder_idle', 'can_ok')}
        receipt = {'schema_version': 1, 'phase': 'drives_disabled', 'before': identity,
                   'devices': reports, 'disabled_unix_s': time.time()}
        write_journal(receipt)
    return stop.coordinate(stop.RecorderClient(f"tcp://{config['peer_ip']}:5557"),
        lambda: None,
        # Full identity inspection can outlast the normal 12-second worker wait.
        lambda side: stop.DeviceProcess(side, config['peer'], config[side + '_interfaces'], 45.),
        lambda: finish_cleanup(config, inspect_fn, receipt, progress), progress=progress,
        capture=capture, before_disable=recheck, verify_reports=verify, checkpoint=checkpoint)

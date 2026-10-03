#!/usr/bin/env python3
"""Read-only recovery evidence. Never enables motors, resets faults or restarts.

A rebooted empty follower, or an all-drives-disabled old action-timeout session,
can be reinitialized ONLY after operator confirmation and a second observation.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import uuid
import sys


POWER_DOWN_RECEIPT = Path.home() / '.local/state/openarm/last-power-down.json'


def read_power_down_receipt():
    try:
        value = json.loads(POWER_DOWN_RECEIPT.read_text())
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def consume_power_down_receipt(expected_id):
    receipt = read_power_down_receipt()
    if not receipt or receipt.get('shutdown_id') != expected_id or receipt.get('consumed') is not False:
        raise RuntimeError('正常下电记录已改变，取消恢复')
    receipt['consumed'] = True
    receipt['consumed_unix_s'] = time.time()
    atomic_receipt(receipt)


def atomic_receipt(receipt):
    import os
    POWER_DOWN_RECEIPT.parent.mkdir(parents=True, exist_ok=True)
    temporary = POWER_DOWN_RECEIPT.with_suffix('.pending')
    with temporary.open('w') as stream:
        json.dump(receipt, stream, ensure_ascii=False, indent=2)
        stream.flush(); os.fsync(stream.fileno())
    temporary.replace(POWER_DOWN_RECEIPT)


def verified_disable_reports(devices):
    if not isinstance(devices, dict) or set(devices) != {'host', 'follower'}:
        return False
    for side, buses in (('host', ('can0', 'can1')), ('follower', ('can1', 'can2'))):
        report = devices.get(side) or {}
        if not isinstance(report, dict): return False
        if any(report.get(key) is not True for key in ('ok', 'verified', 'acknowledged')):
            return False
        motors = report.get('motors')
        expected = {f'{bus}/{motor}' for bus in buses for motor in range(1, 9)}
        if not isinstance(motors, dict) or set(motors) != expected:
            return False
        for sample in motors.values():
            if (not isinstance(sample, dict) or sample.get('disabled') is not True
                    or sample.get('states') != [0, 0, 0]
                    or any(type(value) is not int for value in sample['states'])):
                return False
    return True


def known_power_down_eligible(e):
    """Historical disable evidence is valid only for the exact stopped session.

    This grants an operator-confirmed restart, never a current motor-health claim.
    Disabled motors may stop broadcasting; UNKNOWN alone grants no exception.
    """
    try:
        receipt = e.get('power_down_receipt') or {}
        remote = e['remote']; status = remote['status']; collection = status['collection']
        unit = e['host_unit']
        if (receipt.get('schema_version') != 1 or receipt.get('consumed') is not False
                or not receipt.get('shutdown_id') or not verified_disable_reports(receipt.get('devices'))):
            return False
        if (e.get('host_boot_id') != receipt.get('host_boot_id')
                or remote['boot_id'] != receipt.get('remote_boot_id')
                or not receipt.get('service_invocation_id')
                or unit.get('InvocationID') != receipt['service_invocation_id']
                or unit.get('ActiveState') not in ('inactive', 'failed')
                or unit.get('MainPID') != '0' or unit.get('ControlGroup') != ''):
            return False
        identities = receipt.get('remote_process_identities')
        if (not identities or remote.get('process_identities') != identities
                or not receipt.get('leader_session_id')
                or status.get('leader_session_id') != receipt['leader_session_id']):
            return False
        if (e.get('ssh_ok') is not True or e.get('host_can_ok') is not True
                or remote.get('can_ok') is not True or remote.get('recorder_idle') is not True):
            return False
        if status.get('state') == 'FAULT':
            if (type(status.get('fault_bits')) is not int or status['fault_bits'] not in (1, 2, 3)
                    or status.get('reason') not in ('controller reports CAN failure', 'leader action receive timeout')):
                return False
        elif status.get('state') != 'READY' or status.get('fault_bits') != 0:
            return False
        if (collection.get('recording', 'unknown') is not None
                or collection.get('return_phase') not in ('idle', 'hold')
                or collection.get('transitioning_arms') != []
                or collection.get('right_mode') not in ('FOLLOW', 'HOLD')
                or collection.get('left_mode') not in ('FOLLOW', 'HOLD')):
            return False
        # Missing feedback is expected after intentional disable. Positive
        # evidence of enabled/faulted motors invalidates the historical receipt.
        motors = e.get('motor_report') or {}
        if motors.get('result') not in ('UNKNOWN', 'DISABLED'):
            return False
        for side, buses in (('host', ('can0', 'can1')), ('follower', ('can1', 'can2'))):
            part = motors.get(side) or {}; values = part.get('motors') or {}
            if part.get('result') not in ('UNKNOWN', 'DISABLED'):
                return False
            if set(values) != {f'{bus}/{motor}' for bus in buses for motor in range(1, 9)}:
                return False
            if any(any(state != 0 for state in value.get('states', [])) for value in values.values()):
                return False
        return True
    except (KeyError, TypeError, AttributeError, ValueError):
        return False


def save_power_down_receipt(before, after, devices):
    if not verified_disable_reports(devices):
        raise RuntimeError('32 个电机失能反馈未全部核验，不能记录正常下电')
    for key in ('host_boot_id',):
        if not before.get(key) or before[key] != after.get(key):
            raise RuntimeError('下电期间主机身份改变')
    for key in ('boot_id', 'process_identities'):
        if not before['remote'].get(key) or before['remote'][key] != after['remote'].get(key):
            raise RuntimeError('下电期间从端控制进程身份改变')
    invocation = before['host_unit'].get('InvocationID')
    if not invocation or invocation != after['host_unit'].get('InvocationID'):
        raise RuntimeError('下电期间遥操服务实例改变')
    session = before['remote']['status'].get('leader_session_id')
    if not session or session != after['remote']['status'].get('leader_session_id'):
        raise RuntimeError('下电期间控制会话改变')
    receipt = {'schema_version': 1, 'shutdown_id': uuid.uuid4().hex, 'consumed': False,
               'completed_unix_s': time.time(), 'host_boot_id': before['host_boot_id'],
               'remote_boot_id': before['remote']['boot_id'], 'service_invocation_id': invocation,
               'leader_session_id': session,
               'remote_process_identities': before['remote']['process_identities'], 'devices': devices}
    if not known_power_down_eligible({**after, 'power_down_receipt': receipt}):
        raise RuntimeError('下电后状态未满足可确认恢复条件，未写入正常下电记录')
    atomic_receipt(receipt)
    return receipt


def command(args, **kwargs):
    return subprocess.run(args, text=True, capture_output=True, timeout=8,
                          check=True, **kwargs).stdout


def can_links(interfaces):
    links=[json.loads(command(['ip','-j','-d','link','show',bus]))[0] for bus in interfaces]
    def valid(link):
        data=link['linkinfo']['info_data']
        return ('UP' in link['flags'] and data['state']=='ERROR-ACTIVE' and
                'FD' in data['ctrlmode'] and data['bittiming']['bitrate']==1000000 and
                data['data_bittiming']['bitrate']==5000000)
    return links, all(valid(link) for link in links)


def recorder_is_idle(reply):
    # The recorder's running flag includes active writers AND finalization.
    # A completed failed episode retains phase=error until the next recording;
    # it is idle for robot recovery, not evidence that recording succeeded.
    return (isinstance(reply, dict) and reply.get('ok') is True
            and reply.get('running') is False and reply.get('phase') in ('idle', 'error')
            and 'active_episode' in reply and reply['active_episode'] is None)


def remote_probe():
    result = {'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              'uptime_s': float(Path('/proc/uptime').read_text().split()[0]),
              'processes': [], 'process_identities': {}, 'status': None, 'can_ok': False, 'recorder_idle': False}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv=proc.joinpath('cmdline').read_bytes().decode().split('\0')
            names={Path(a).name for a in argv if a}
            if names & {'openarm_gravity_pd_node', 'remote-teleop-follower',
                        'remote-teleop-follower-watchdog', 'bimanual_follower.launch.py'}:
                result['processes'].append(int(proc.name))
                result['process_identities'][proc.name] = {
                    'start_ticks': int(proc.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19]),
                    'argv': argv,
                }
        except (OSError, UnicodeError):
            continue
    result['runtime_socket_exists']=Path('/tmp/openarm_remote_runtime.sock').exists()
    try:
        with tempfile.TemporaryDirectory(prefix='oa_recovery_') as tmp:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
                s.bind(tmp+'/reply'); s.settimeout(.5)
                s.sendto(b'{"command":"status"}', '/tmp/openarm_remote_runtime.sock')
                result['status']=json.loads(s.recv(65536))
    except (OSError, ValueError):
        pass
    try:
        links, result['can_ok']=can_links(('can1','can2'))
        result['can']=links
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError):
        pass
    try:
        code = '''import json,zmq
c=zmq.Context();s=c.socket(zmq.REQ);s.setsockopt(zmq.LINGER,0)
s.setsockopt(zmq.RCVTIMEO,2000);s.setsockopt(zmq.SNDTIMEO,2000)
s.connect("tcp://127.0.0.1:5557");s.send_json({"command":"status"})
print(json.dumps(s.recv_json()));s.close();c.term()
'''
        r=json.loads(command(['/home/nvidia/miniconda3/envs/lerobot/bin/python','-c',code]))
        result['recorder_idle'] = recorder_is_idle(r)
        result['recorder_status'] = {key: r.get(key) for key in
                                     ('ok', 'running', 'phase', 'active_episode', 'stop_reason')}
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    return result


def all_disabled(report):
    """Check every physical motor, not just a summary string or ROS state."""
    if not isinstance(report, dict) or report.get('result') != 'DISABLED':
        return False
    for role,buses in (('host',('can0','can1')),('follower',('can1','can2'))):
        part=report.get(role) or {}
        if not isinstance(part,dict): return False
        motors=part.get('motors') or {}
        if not isinstance(motors,dict): return False
        expected={f'{bus}/{j}' for bus in buses for j in range(1,9)}
        if part.get('result')!='DISABLED' or set(motors)!=expected:
            return False
        for value in motors.values():
            if not isinstance(value,dict): return False
            samples=value.get('samples')
            if (value.get('state')!=0 or value.get('states')!=[0] or
                value.get('fresh') is not True or type(samples) is not int or samples<3):
                return False
    return True


def disabled_timeout_eligible(e):
    """Narrow confirmed recovery, NOT automatic clearing of an arbitrary fault."""
    r=e.get('remote') or {}; s=r.get('status') or {}; c=s.get('collection') or {}
    if not isinstance(c,dict): return False
    if (s.get('state')!='FAULT' or type(s.get('fault_bits')) is not int or s['fault_bits']!=1
        or s.get('reason')!='leader action receive timeout'
        or s.get('feedback_fresh_for_control') is not True
        or set(s.get('enabled_arms') or [])!={'left','right'}):
        return False
    if (r.get('can_ok') is not True or e.get('host_can_ok') is not True
        or r.get('recorder_idle') is not True or not all_disabled(e.get('motor_report'))):
        return False
    if (not isinstance(r.get('processes'),list) or not r['processes']
        or r.get('runtime_socket_exists') is not True):
        return False
    for key in ('action_age_ms','feedback_age_ms'):
        age=s.get(key)
        if type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<100:
            return False
    session=s.get('leader_session_id')
    if type(session) is not int or session<=0: return False
    try:
        uuid.UUID(r['boot_id'])
    except (ValueError,KeyError,TypeError,AttributeError):
        return False
    # HOLD is only permitted here because ALL motors are proved disabled, and
    # the prompt explicitly asks to clear objects and discard the old HOLD.
    return (c.get('left_mode') in ('FOLLOW','HOLD') and c.get('right_mode') in ('FOLLOW','HOLD')
            and 'recording' in c and c['recording'] is None
            and c.get('return_phase') in ('idle','hold') and c.get('transitioning_arms')==[])


def health_latch_recovery_eligible(e):
    """Only offer a confirmed new initialization, never clear the old latch."""
    from teleop_readiness import evaluate, session_id
    try:
        remote = e['remote']; status = remote['status']; health = e['health_report']
        collection = status['collection']; unit = e['host_unit']
        sid = session_id(status)
        if (e.get('ssh_ok') is not True or e.get('host_can_ok') is not True
                or remote.get('can_ok') is not True or remote.get('recorder_idle') is not True
                or e.get('health_report_fresh') is not True or sid is None
                or health.get('invalidated_session') != sid
                or session_id(health.get('status')) != sid
                or unit.get('ActiveState') != 'active' or int(unit.get('MainPID', 0)) <= 0):
            return False
        # A current motor fault, disable, stale action or changed controller is
        # not eligible for this narrow historical-health recovery path.
        if not evaluate(status, e.get('motor_report'))['teleop_ready']:
            return False
        return (collection.get('left_mode') == 'FOLLOW' and collection.get('right_mode') == 'FOLLOW'
                and 'recording' in collection and collection['recording'] is None
                and collection.get('return_phase') == 'idle'
                and collection.get('transitioning_arms') == [])
    except (KeyError, TypeError, ValueError, AttributeError):
        return False


def classify(e):
    def decision(code, reason, allow=False):
        return {'code':code,'reason':reason,'confirmation_required':allow,
                'automatic_restart_allowed':False}
    if not e.get('ssh_ok'):
        return decision('NETWORK_OR_SSH', '网络或SSH不可用；不能确认从端状态，不重启、不归位')
    r=e.get('remote') or {}; s=r.get('status') or {}
    if not isinstance(s, dict):
        return decision('INVALID_STATUS', '从端状态格式异常，禁止恢复')
    if known_power_down_eligible(e):
        return decision('OPERATOR_POWER_DOWN_CONFIRM',
            '检测到本会话正常下电记录，32个电机的失能回执已核验；当前旧进程缺少反馈是下电后的遗留状态。确认后可重新初始化、归位和对齐', True)
    if disabled_timeout_eligible(e):
        return decision('DISABLED_TIMEOUT_CONFIRM',
            '旧会话仅锁存动作超时；当前通信与CAN正常、32个电机均有新反馈且未使能、录制空闲。可确认后重新初始化，不需要断电', True)
    if health_latch_recovery_eligible(e):
        return decision('HEALTH_LATCH_CONFIRM',
            '当前32个电机使能、通信正常，但旧会话有历史健康锁定；不能直接复用。可确认后重新初始化、归位、对齐', True)
    if s.get('state') in ('FAULT','E_STOP') or s.get('fault_bits'):
        return decision('LATCHED_FAULT', '从端故障/急停已锁存；重连不解除故障，需排查后明确恢复')
    c=s.get('collection') or {}
    if (c.get('recording') is not None or c.get('left_mode')=='HOLD' or
        c.get('right_mode') in ('HOLD','RETURNING') or c.get('transitioning_arms') or
        c.get('return_phase','idle')!='idle'):
        return decision('ACTIVE_TASK', '存在录制、夹持保持或回位任务；禁止自动归位')
    if r.get('recorder_idle') is not True:
        return decision('RECORDER_NOT_IDLE', '录制器未确认空闲或仍在保存；禁止重启遥操')
    if r.get('can_ok') is not True:
        return decision('CAN_NOT_READY', '从端CAN缺失、BUS-OFF或参数不符；先检查连接，禁止恢复')
    if r.get('processes') != [] or r.get('runtime_socket_exists') is not False or r.get('status') is not None:
        return decision('EXISTING_REMOTE_STACK', '从端仍有控制栈或状态残留；不能按重启后的空栈处理')
    if e.get('host_motors')!='DISABLED':
        return decision('HOST_MOTORS_NOT_DISABLED', '主端电机并非全部确认失能；不能自动替换旧控制栈')
    try:
        boot=str(uuid.UUID(r['boot_id']))
        previous=str(uuid.UUID(e['previous_boot_id'])) if e.get('previous_boot_id') else None
        age=e.get('host_service_age_s'); uptime=r.get('uptime_s')
        age_proof=(isinstance(age,(int,float)) and isinstance(uptime,(int,float)) and
                   math.isfinite(age) and math.isfinite(uptime) and uptime>=0 and age>uptime+5)
        reboot_proven=(previous is not None and previous!=boot) or (previous is None and age_proof)
    except (ValueError, KeyError, TypeError):
        reboot_proven=False
    if not reboot_proven:
        return decision('UNEXPLAINED_STACK_LOSS', '未证实Jetson重启；可能是进程崩溃，不自动重启掩盖原因')
    return decision('PEER_REBOOT_CONFIRM',
        '已证实Jetson重启、从端控制栈不存在、主端全部失能且录制空闲；需确认无夹持物和归位路径安全', True)


def inspect(jetson):
    e={'ssh_ok':False}
    e['power_down_receipt'] = read_power_down_receipt()
    try:
        e['host_boot_id'] = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        output = command(['systemctl', '--user', 'show', 'openarm-remote-teleop.service',
                          '-p', 'MainPID', '-p', 'InvocationID', '-p', 'ActiveState', '-p', 'ControlGroup'])
        e['host_unit'] = dict(line.split('=', 1) for line in output.splitlines() if '=' in line)
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    try:
        probe_result=subprocess.run([sys.executable,str(Path(__file__).with_name('check_motor_enable.py')),
            '--jetson',jetson],text=True,capture_output=True,timeout=12,check=False)
        if probe_result.returncode in (0,10,11,12):
            e['motor_report']=json.loads(probe_result.stdout)
            e['host_motors']=e['motor_report']['host']['result']
        _,e['host_can_ok']=can_links(('can0','can1'))
    except (OSError,ValueError,KeyError,TypeError,IndexError,subprocess.SubprocessError):
        pass
    try:
        if jetson.startswith('-'): raise ValueError('invalid SSH target')
        e['remote']=json.loads(command(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=3',
            jetson,'/usr/bin/python3 - --remote'], input=Path(__file__).read_text()))
        e['ssh_ok']=True
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        e['error']=str(exc)
    try:
        f=Path('/tmp/openarm-remote-teleop/peer-boot-id')
        e['previous_boot_id']=f.read_text().strip() if f.exists() else None
        pid=int(command(['systemctl','--user','show','openarm-remote-teleop.service','-p','MainPID','--value']))
        e['host_service_pid']=pid
        import os
        stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        e['host_service_age_s']=float(Path('/proc/uptime').read_text().split()[0])-int(stat[19])/os.sysconf('SC_CLK_TCK')
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    try:
        from teleop_readiness import read_report
        e['health_report'] = read_report()
        e['health_report_fresh'] = True
    except (OSError, ValueError, TypeError):
        e['health_report_fresh'] = False
    return dict(classify(e), evidence=e)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--remote',action='store_true');p.add_argument('--jetson',default='openarm-jetson')
    p.add_argument('--output')
    args=p.parse_args()
    result=remote_probe() if args.remote else inspect(args.jetson)
    payload=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if args.output: Path(args.output).write_text(payload)
    print(payload)


if __name__=='__main__': main()

#!/usr/bin/env python3
"""Operator-confirmed lifecycle start. No hardware actions occur on import."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import uuid

from inspect_teleop_recovery import (connection_config, inspect as inspect_recovery,
                                    consume_power_down_receipt)
from teleop_readiness import apply_report, apply_current_report
from power_down_arms import emit, ssh_args, RecorderClient, require_reply
from preflight_motor_feedback import verify as verify_motor_preflight


def start_decision(result):
    """An explicit click permits cold start or a narrowly verified recovery."""
    e = result.get("evidence") or {}
    remote = e.get("remote") or {}
    unit = e.get("host_unit") or {}
    status = remote.get("status") or {}
    if not (e.get("ssh_ok") and e.get("host_can_ok") and remote.get("can_ok")):
        raise RuntimeError("网络或 CAN 检查未通过，未启动遥操")
    if not remote.get("recorder_idle"):
        raise RuntimeError("请先保存当前条目并等待录制器空闲")
    if (unit.get("ActiveState") == "active" and e.get("health_report_fresh")
            and apply_report(status, e.get("health_report") or {}).get("teleop_ready")):
        return "reuse"
    if result.get("confirmation_required") is True and result.get("code") in {
            "OPERATOR_POWER_DOWN_CONFIRM", "DISABLED_TIMEOUT_CONFIRM", "PEER_REBOOT_CONFIRM"}:
        return "restart"
    # A genuinely empty stack is the first-start path. Missing/malformed
    # evidence and any live/faulted old controller are not an empty stack.
    motors = e.get("motor_report") or {}
    cold = (unit.get("LoadState") == "loaded"
            and unit.get("ActiveState") in ("inactive", "failed")
            and unit.get("MainPID") == "0" and unit.get("ControlGroup") == ""
            and remote.get("processes") == []
            and remote.get("runtime_socket_exists") is False
            and remote.get("status") is None
            and e.get("local_control_processes") == []
            and motors.get("result") in ("DISABLED", "UNKNOWN"))
    for side in ("host", "follower"):
        part = motors.get(side) or {}
        values = part.get("motors") or {}
        expected = {f"{bus}/{motor}" for bus in connection_config()[side + "_interfaces"]
                    for motor in range(1, 9)}
        cold = cold and part.get("result") in ("DISABLED", "UNKNOWN") and set(values) == expected
        cold = cold and all(all(state == 0 for state in v.get("states", [])) for v in values.values())
    if cold:
        return "cold"
    if result.get("code") == "PEER_REBOOT_POWER_OFF_REQUIRED":
        raise RuntimeError(str(result["reason"]))
    if result.get("code") == "FAILED_START_POWER_OFF_REQUIRED":
        raise RuntimeError(str(result["reason"]))
    if (unit.get('MainPID') == '0' and unit.get('ActiveState') in ('failed', 'inactive')
            and e.get('local_control_processes') == [] and remote.get('processes')
            and status.get('state') == 'FAULT'):
        raise RuntimeError('主臂控制已退出，从端仍有故障旧会话。'
                           '本次未满足自动恢复条件；先查看具体故障，并使用“停止遥操并失能”核验。'
                           '不需要立即关闭 Jetson 或整机电源')
    raise RuntimeError("现有遥操状态不允许重新启动：" + str(result.get("reason", "状态未确认")) +
                       "。请先停止遥操并确认失能；故障需排查后恢复。")


def identity(result):
    e = result["evidence"]
    return [e.get("profile"), e.get("peer"), e.get("host_boot_id"), e.get("host_unit"),
            e["remote"].get("boot_id"), e["remote"].get("process_identities"),
            (e["remote"].get("status") or {}).get("leader_session_id"),
            (e.get("power_down_receipt") or {}).get("shutdown_id")]


def unit_status(service):
    result = subprocess.run(["systemctl", "--user", "show", service, "-p", "ActiveState",
                             "-p", "MainPID", "-p", "InvocationID", "-p", "Environment",
                             "-p", "StandardOutput"],
                            capture_output=True, text=True, check=True, timeout=5)
    return dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)


def startup_log_cursor(unit, service=None):
    """Remember the append-log boundary before this particular start request."""
    output = unit.get("StandardOutput", "")
    # systemd 249 reports only "append" in `show`, omitting the file path.
    # Read the effective unit and its ordered drop-ins once, before restart.
    if output == "append" and service:
        try:
            result = subprocess.run(["systemctl", "--user", "cat", service],
                                    capture_output=True, text=True, check=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            return None
        section = None
        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith("["):
                section = line
            elif section == "[Service]" and "=" in line and not line.startswith(("#", ";")):
                key, value = line.split("=", 1)
                if key.strip() == "StandardOutput":
                    output = value.strip()
        # Unexpanded specifiers and escaped paths require systemd's parser.
        # Prefer a generic failure over reading a different invocation's log.
        if "%" in output or "\\" in output:
            return None
    if not output.startswith("append:/"):
        return None
    path = Path(output[len("append:"):])
    try:
        stat = path.stat()
        return path, (stat.st_dev, stat.st_ino), stat.st_size
    except FileNotFoundError:
        return path, None, 0
    except OSError:
        return None


def startup_exit_reason(cursor):
    """Explain a failed invocation without reusing an earlier attempt's error."""
    fallback = "启动服务已退出；未确认可遥操，请查看启动日志"
    if cursor is None:
        return fallback
    path, identity, offset = cursor
    try:
        with path.open("rb") as stream:
            stat = os.fstat(stream.fileno())
            if (identity is not None and identity != (stat.st_dev, stat.st_ino)) or stat.st_size < offset:
                return fallback
            stream.seek(max(offset, stat.st_size - 65536))
            log = stream.read(65536).decode(errors="replace")
    except OSError:
        return fallback
    if "ERROR: 从臂未实际到达初始位" in log:
        if "feedback_enabled_fresh=0" in log:
            # A zero/stale cached angle is not a verified physical position.
            # Explain the evidence failure before suggesting mechanical fixes.
            message = ("从臂归位超时：电机使能或反馈未通过核验；"
                       "日志角度不能作为已核验的实测位置。请检查电机实际供电、急停与 CAN 通信，"
                       "不要据此调整关节零点或反复归位")
            if "[4/6]" not in log:
                message += "。主臂尚未启动"
            return message + "；当前不能跟随"
        details = []
        pattern = r"follower_gravity_pd\.(can\d+)\].*Startup homing timed out: J(\d+) target=([-\d.]+) actual=([-\d.]+)"
        for bus, joint, target, actual in re.findall(pattern, log):
            details.append(f"{bus} 的 J{joint} 目标 {math.degrees(float(target)):.1f}°，实测 {math.degrees(float(actual)):.1f}°")
        message = "从臂归位超时" + ("：" + "；".join(dict.fromkeys(details)) if details else "")
        if "[4/6]" not in log:
            message += "。主臂尚未启动"
        return message + "；请检查安装朝向、零点与机械阻挡，当前不能跟随"
    if "ERROR: 主臂未实际到达初始位" in log:
        failed = re.findall(r"leader_gravity_pd\.(can\d+)\].*Startup homing timed out: ([^\n]+)", log)
        unverified = [bus for bus, detail in failed if 'feedback_enabled_fresh=0' in detail]
        if unverified:
            sides = {'can0': '右主臂', 'can1': '左主臂'}
            names = '、'.join(sides.get(bus, bus) + f'（笔记本 {bus}）'
                             for bus in dict.fromkeys(unverified))
            return (f'主臂归位超时：{names}电机使能或反馈未通过核验。'
                    '请检查主臂电源是否实际接通、急停与主臂 CAN 接头；USB 盒亮灯不代表电机已通电。'
                    '日志角度不能作为已核验的实测位置。补上电源后不会自动恢复；'
                    '先支撑四臂，点击“停止遥操并失能”完成恢复，再单独开始。'
                    '从臂可能仍使能保持，当前不能跟随')
        return ('主臂归位超时：未确认到达初始位，当前不能跟随。'
                '请检查主臂反馈、机械阻挡与姿态；不要反复强推归位。'
                '先支撑四臂，点击“停止遥操并失能”，核验完成后再处理和重新启动')
    errors = [line.strip() for line in log.splitlines() if line.startswith("ERROR:")]
    if not errors and '自动对齐验收：未通过，不允许开始遥操' in log:
        # Older verifier output still identifies a failed acceptance stage.
        # Never read a shared report from another startup attempt here.
        tail = log.rsplit('自动对齐验收：未通过，不允许开始遥操', 1)[1]
        issues = [line.strip()[2:] for line in tail.splitlines() if line.strip().startswith('- ')]
        return '启动验收失败：' + ('；'.join(issues[:3]) or '未通过归位/跟随检查')
    return errors[-1][:600] if errors else fallback


def follower_status(peer):
    code = '''
import json,socket,tempfile
with tempfile.TemporaryDirectory(prefix="oa_start_status_") as d:
 with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
  s.bind(d+"/reply");s.settimeout(2)
  s.sendto(b'{"command":"status"}',"/tmp/openarm_remote_runtime.sock")
  print(s.recv(65536).decode())
'''
    p = subprocess.run(ssh_args(peer) + ["/usr/bin/python3 -"], input=code,
                       capture_output=True, text=True, check=True, timeout=6)
    return json.loads(p.stdout)


def powered_recovery_eligible(result, config):
    """A known abandoned startup only; never a running/held collection task."""
    try:
        e = result['evidence']; r = e['remote']; u = e['host_unit']; s = r['status']
        if (not e.get('ssh_ok') or not e.get('host_can_ok') or not r.get('can_ok')
                or r.get('recorder_idle') is not True or not e.get('host_boot_id') or not r.get('boot_id')
                or u.get('LoadState') != 'loaded' or u.get('ActiveState') not in ('failed', 'inactive')
                or u.get('MainPID') != '0' or u.get('ControlGroup') != '' or not u.get('InvocationID')
                or e.get('local_control_processes') != [] or r.get('runtime_socket_exists') is not True):
            return False
        hold = (s.get('state') == 'ALIGNING' and s.get('aligned') is False
                and type(s.get('fault_bits')) is int and s['fault_bits'] == 0
                and s.get('reason') == 'explicit HOLD accepted; fresh RUN request required'
                and 'leader_session_id' in s)
        fault = (s.get('state') == 'FAULT' and type(s.get('fault_bits')) is int
                 and s['fault_bits'] in (1, 2, 3) and bool(s.get('leader_session_id'))
                 and s.get('reason') in ('leader action receive timeout', 'controller reports CAN failure'))
        if not (hold or fault):
            return False
        c = s['collection']
        if (c.get('recording', 'unknown') is not None or c.get('transitioning_arms') != []
                or c.get('return_phase') != 'idle' or c.get('left_static_hold_phase') != 'idle'
                or c.get('left_mode') != 'FOLLOW' or c.get('right_mode') != 'FOLLOW'
                or any(c.get(k) is not False for k in
                       ('left_align_active', 'right_master_align_active', 'left_master_align_active'))):
            return False
        ids = r['process_identities']
        if not ids or {int(p) for p in ids} != set(r['processes']):
            return False
        root = ('/home/openarm' if config['profile'] == 'host-test' else '/home/nvidia') + '/dev/openarm-remote-harvest/ros2_robot/'
        roles = []
        for pid, item in ids.items():
            if int(pid) <= 1 or type(item['start_ticks']) is not int or item['start_ticks'] <= 0:
                return False
            a = item['argv']; a = a[:-1] if a and a[-1] == '' else a
            if not a or not all(isinstance(v, str) for v in a): return False
            if a[:5] == ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'remote_teleop_runtime', 'bimanual_follower.launch.py']:
                roles.append('launch')
            elif a[0].startswith(root) and a[0].endswith('/openarm_gravity_pd_control/openarm_gravity_pd_node') and '__node:=follower_gravity_pd' in a:
                roles.append('controller')
            elif len(a) >= 2 and a[0] == '/usr/bin/python3' and a[1].startswith(root) and a[1].endswith('/remote_teleop_runtime/remote-teleop-follower'):
                roles.append('gateway')
            elif len(a) >= 2 and a[0] == '/usr/bin/python3' and a[1].startswith(root) and a[1].endswith('/remote_teleop_follower_safety/remote-teleop-follower-watchdog'):
                roles.append('watchdog')
            else: return False
        return sorted(roles) == ['controller', 'gateway', 'launch', 'watchdog']
    except (KeyError, TypeError, ValueError, AttributeError):
        return False


def rebooted_peer_recovery_eligible(result):
    """Prove a reboot, not merely a dropped connection or absent heartbeat."""
    try:
        e = result['evidence']; r = e['remote']; u = e['host_unit']
        previous = uuid.UUID(e['previous_boot_id']); current = uuid.UUID(r['boot_id'])
        age, uptime = e['host_service_age_s'], r['uptime_s']
        return (previous != current and all(type(v) in (int, float) and math.isfinite(v) for v in (age, uptime))
                and 0 <= uptime < age - 5 and e.get('ssh_ok') is True
                and e.get('host_can_ok') is True and r.get('can_ok') is True
                and r.get('recorder_idle') is True and r.get('processes') == []
                and r.get('process_identities') == {} and r.get('runtime_socket_exists') is False
                and r.get('status') is None and bool(e.get('host_boot_id'))
                and u.get('LoadState') == 'loaded' and u.get('ActiveState') == 'active'
                and int(u['MainPID']) > 0 and int(u['MainPID']) == e.get('host_service_pid')
                and bool(u.get('InvocationID')) and bool(u.get('ControlGroup'))
                and isinstance(e.get('local_control_processes'), list) and bool(e['local_control_processes']))
    except (KeyError, TypeError, ValueError, AttributeError):
        return False


def recover_rebooted_peer(config, result, inspect_fn):
    from power_down_arms import DeviceProcess, stop_service
    emit({'event': 'progress', 'message': '确认 Jetson 已重启且无从臂控制；查询电机，准备收尾旧主臂会话…'})
    reports = verify_motor_preflight(config, require_disabled=False)
    if any(v != [0, 0, 0] for v in reports['follower']['motors'].values()):
        raise RuntimeError('Jetson 已重启但从臂仍使能，不能按空会话自动恢复；未启动归位')
    again = inspect_fn(config['peer'])
    if identity(again) != identity(result) or not rebooted_peer_recovery_eligible(again):
        raise RuntimeError('重启恢复期间会话改变，未执行失能')
    # The follower has no controller and all 16 drives are already disabled.
    # Only the existing local ROS service may disable the still-live leader.
    device = DeviceProcess('host', config['peer'], config['host_interfaces'])
    try:
        device.arm()
        latest = inspect_fn(config['peer'])
        if identity(latest) != identity(again) or not rebooted_peer_recovery_eligible(latest):
            raise RuntimeError('等待失能服务期间会话改变，未执行失能')
        device.go()
        proof = device.result()
        from inspect_teleop_recovery import verified_disable_reports
        follower = {'ok': True, 'verified': True, 'acknowledged': True,
                    'motors': {k: {'states': v, 'disabled': v == [0, 0, 0]}
                               for k, v in reports['follower']['motors'].items()}}
        if not verified_disable_reports({'host': proof, 'follower': follower}):
            # disableArms() is idempotent: after its first call, ROS still
            # acknowledges success but sends no more CAN disable frames.
            # Only missing passive receipts may use a fresh read-only query;
            # never substitute it for a failed ROS acknowledgement or a fault.
            expected = {f'{bus}/{j}' for bus in config['host_interfaces'] for j in range(1, 9)}
            samples = proof.get('motors')
            missing_only = (proof.get('acknowledged') is True
                and proof.get('verified') is False and proof.get('ok') is False
                and isinstance(proof.get('error'), str)
                and proof['error'].startswith('缺少连续失能反馈：')
                and isinstance(samples, dict) and set(samples) == expected
                and all(isinstance(v, dict) and isinstance(v.get('states'), list)
                        and len(v['states']) <= 3
                        and all(type(state) is int and state == 0 for state in v['states'])
                        for v in samples.values()))
            if not missing_only:
                raise RuntimeError('旧主臂会话失能未完整核验：' +
                    str(proof.get('error') or '失能回执不完整或状态异常') +
                    '；未停止或重新启动控制器')
            emit({'event': 'progress', 'message':
                  '旧主臂已确认停止驱动，但重复失能未产生足够回执；正在只读复核全部电机…'})
        latest = inspect_fn(config['peer'])
        if identity(latest) != identity(again) or not rebooted_peer_recovery_eligible(latest):
            raise RuntimeError('失能后会话改变，未继续重启')
        verify_motor_preflight(config)  # Re-query both sides before killing an owner.
        latest = inspect_fn(config['peer'])
        if identity(latest) != identity(again) or not rebooted_peer_recovery_eligible(latest):
            raise RuntimeError('电机复核期间会话改变，未停止旧控制器')
        stop_service(config['service'])
    finally:
        device.close()
    after = inspect_fn(config['peer'])
    if (after['evidence'].get('host_boot_id') != again['evidence'].get('host_boot_id')
            or after['evidence']['remote'].get('boot_id') != again['evidence']['remote'].get('boot_id')
            or start_decision(after) != 'cold'):
        raise RuntimeError('旧主臂会话收尾后状态未确认，未启动')
    return after


def prepare_start(config, result, inspect_fn):
    """Runs inside the start locks, after support/re-home confirmation."""
    e = result.get('evidence') or {}; remote = e.get('remote') or {}
    if e.get('ssh_ok') and (not e.get('host_can_ok') or not remote.get('can_ok')):
        if remote.get('recorder_idle') is not True:
            raise RuntimeError('录制器未空闲，未配置 CAN 或启动遥操')
        emit({'event': 'progress', 'message': '检查原双通道 CAN 盒；仅在无控制进程时自动启用停用接口…'})
        verify_motor_preflight(config, repair=True)
        result = inspect_fn(config['peer'])
    # Reuse healthy sessions before considering any recovery or querying.
    try:
        if start_decision(result) in ('reuse', 'cold', 'restart'):
            return result
    except RuntimeError:
        pass
    if rebooted_peer_recovery_eligible(result):
        return recover_rebooted_peer(config, result, inspect_fn)
    from recover_powered_session import eligible as powered_session_eligible, recover as recover_powered_session
    if powered_session_eligible(result, config):
        return recover_powered_session(config, result, inspect_fn)
    if result.get('code') != 'STOP_INCOMPLETE' and not powered_recovery_eligible(result, config):
        return result
    again = inspect_fn(config['peer'])
    if identity(result) != identity(again) or (
            result.get('code') == 'STOP_INCOMPLETE' and again.get('code') != 'STOP_INCOMPLETE') or (
            result.get('code') != 'STOP_INCOMPLETE' and not powered_recovery_eligible(again, config)):
        raise RuntimeError('恢复检查期间会话改变，未执行失能或启动')
    if result.get('code') != 'STOP_INCOMPLETE':
        emit({'event': 'progress', 'message': '发现上次启动留下的保持会话；先查询两端供电与电机反馈…'})
        reports = verify_motor_preflight(config, require_disabled=False)
        if any(v != [0, 0, 0] for v in reports['host']['motors'].values()):
            raise RuntimeError('主臂控制已退出但电机仍使能，不能自动恢复；请使用停止并失能处理')
        latest = inspect_fn(config['peer'])
        if identity(latest) != identity(again) or not powered_recovery_eligible(latest, config):
            raise RuntimeError('电机查询期间会话改变，未执行失能或启动')
    from power_down_arms import perform_stop
    emit({'event': 'progress', 'message': '保持电源接通，正在恢复旧会话；四臂将解除驱动力，请保持支撑。'})
    def progress(event):
        # The UI must not treat the internal stop's completion as START success.
        if event.get('event') == 'complete' and event.get('ok') is True:
            emit({'event': 'progress', 'message': '32/32 电机已确认失能，准备重新核验并启动；请保持供电和现场监护。'})
        elif event.get('message'):
            emit({'event': 'progress', 'message': '启动前恢复：' + event['message']})
    stopped = perform_stop(config, inspect_fn=inspect_fn, progress=progress)
    if not stopped.get('ok'):
        details = '；'.join(str(v.get('error')) for v in stopped.get('devices', {}).values() if v.get('error'))
        raise RuntimeError('保持供电恢复未完成：' + stopped.get('message', '') + '；' + details)
    result = inspect_fn(config['peer'])
    if start_decision(result) != 'restart':
        raise RuntimeError('失能已执行，但恢复凭据未核验，未重新使能')
    return result


def start(config, *, inspect_fn=inspect_recovery):
    peer, service = config["peer"], config["service"]
    emit({"event": "progress", "message": "检查录制、连接、CAN 与现有控制会话…"})
    before = prepare_start(config, inspect_fn(peer), inspect_fn)
    mode = start_decision(before)
    if mode == "reuse":
        return {"event": "complete", "ok": True, "message": "主从遥操已运行，已复用当前会话；未重新归位。"}
    # A second independent read immediately before the mutation rejects stale
    # approval/evidence (another UI, recorder, reboot or service replacement).
    again = inspect_fn(peer)
    if start_decision(again) != mode or identity(before) != identity(again):
        raise RuntimeError("检查期间控制会话发生变化，未启动；请重新确认")
    emit({'event': 'progress', 'message': '启动前逐个查询 32 个电机；确认电源、通信和失能状态，不发送运动指令…'})
    verify_motor_preflight(config)
    latest = inspect_fn(peer)
    if start_decision(latest) != mode or identity(again) != identity(latest):
        raise RuntimeError('启动前查询期间控制会话改变，未启动')
    state = require_reply(RecorderClient(f"tcp://{config['peer_ip']}:5557")("status"))
    if state.get("running") is not False or state.get("active_episode") is not None:
        raise RuntimeError("录制器状态已改变，未启动")
    unit_before = unit_status(service)
    if config["profile"] == "host-test":
        import shlex
        environment = dict(item.split("=", 1) for item in shlex.split(unit_before["Environment"]) if "=" in item)
        if any(environment.get(key) != value for key, value in {
                "JETSON_HOST": peer, "PEER_IP": config["peer_ip"],
                "FOLLOWER_RIGHT_CAN": "can0", "FOLLOWER_LEFT_CAN": "can1"}.items()):
            raise RuntimeError("测试遥操服务的连接配置不匹配，未启动")
    previous_session = (again["evidence"]["remote"].get("status") or {}).get("leader_session_id")
    emit({"event": "progress", "message": "正在启动：四臂将使能、归位并对齐，请勿触碰。"})
    started_ns = time.monotonic_ns()
    log_cursor = startup_log_cursor(unit_before, service)
    subprocess.run(["systemctl", "--user", "restart", service], check=True, timeout=25)
    invocation = unit_status(service).get("InvocationID")
    if not invocation or invocation == unit_before.get("InvocationID"):
        raise RuntimeError("未确认新的遥操服务实例")
    # Preserve a valid stop receipt if systemd fails before creating a new
    # invocation. A changed invocation already invalidates the old receipt.
    if again.get("code") == "OPERATOR_POWER_DOWN_CONFIRM":
        consume_power_down_receipt(again["evidence"]["power_down_receipt"]["shutdown_id"])
    deadline = time.monotonic() + 180
    last_message = None
    while time.monotonic() < deadline:
        unit = unit_status(service)
        if unit.get("InvocationID") != invocation:
            raise RuntimeError("启动服务实例已被替换；未确认可遥操，请检查是否有其他启动操作")
        if unit.get("ActiveState") != "active":
            raise RuntimeError(startup_exit_reason(log_cursor))
        try:
            status = follower_status(peer)
            ready = apply_current_report(status)
            from teleop_readiness import read_report
            health = read_report()
            acceptance = json.loads(Path("/tmp/openarm-remote-teleop/alignment-following.json").read_text())
            if (ready.get("teleop_ready") is True
                    and status.get("leader_session_id") != previous_session
                    and health.get("probe_started_monotonic_ns", 0) >= started_ns
                    and acceptance.get("accepted") is True
                    and acceptance.get("leader_session_id") == status.get("leader_session_id")):
                return {"event": "complete", "ok": True,
                        "message": "主从遥操已运行：归位和对齐通过，32/32 电机使能已核验。"}
            message = "正在归位、对齐和核验电机，当前状态：" + str(status.get("state", "未知"))
        except (OSError, ValueError, subprocess.SubprocessError):
            message = "等待两端控制器启动及状态反馈…"
        if message != last_message:
            emit({"event": "progress", "message": message})
            last_message = message
        time.sleep(.5)
    raise RuntimeError("启动超时，尚未确认可遥操；控制器可能仍在运行。请使用停止遥操并失能处理，不要重复启动。")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--confirm", action="store_true")
    p.add_argument("--status", action="store_true", help="read-only stopped-session verification")
    p.add_argument("--profile", choices=("jetson", "host-test"), default="jetson")
    p.add_argument("--jetson", required=True)
    p.add_argument("--recorder", required=True)
    args = p.parse_args()
    if not args.confirm and not args.status:
        p.error("请先确认四臂已安置支撑、无夹持物、归位路径无人且无障碍、急停可用并有人监护；恢复时可能先失能再归位")
    os.environ["OPENARM_TELEOP_PROFILE"] = args.profile
    config = connection_config()
    if args.jetson != config["peer"] or args.recorder != f"tcp://{config['peer_ip']}:5557":
        p.error("连接目标与遥操配置不一致")
    if args.status:
        result = inspect_recovery(config["peer"])
        stopped = result.get("code") == "OPERATOR_POWER_DOWN_CONFIRM"
        emit({"event": "complete", "ok": True, "stopped": stopped,
              "leader_session_id": ((result.get("evidence") or {}).get("power_down_receipt") or {}).get("leader_session_id"),
              "message": "遥操已停止 · 32/32 电机失能回执已核验" if stopped else "未确认正常停止记录"})
        return 0
    try:
        with open("/tmp/openarm-teleop-lifecycle.lock", "a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Cooperate with the existing desktop initializer too.
            with open("/tmp/openarm-daily-teleop.lock", "a") as daily:
                fcntl.flock(daily, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = start(config)
    except Exception as exc:
        result = {"event": "complete", "ok": False,
                  "message": "启动未确认：" + str(exc) + "；不会自动重试。"}
    emit(result)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

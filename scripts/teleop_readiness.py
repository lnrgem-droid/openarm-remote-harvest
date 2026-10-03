#!/usr/bin/env python3
"""Shared, fail-closed operator readiness. No CAN writes or recovery commands.

This slow diagnostic contract is NOT a real-time watchdog or an emergency stop.
Raw controller state is retained separately from permission to operate/record.
"""
import json
import math
from pathlib import Path
import time

HEALTH_PATH = Path('/tmp/openarm-remote-teleop/live-health.json')
BOOT_PATH = Path('/proc/sys/kernel/random/boot_id')
MAX_REPORT_AGE_S = 4.0


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def session_id(status):
    value = status.get('leader_session_id') if isinstance(status, dict) else None
    return value if type(value) is int and value > 0 else None


def motor_health(report):
    """Validate all 32 entries; never trust only the summary ENABLED string."""
    faults, disabled, unknown = [], [], []
    for role, buses in (('host', ('can0', 'can1')), ('follower', ('can1', 'can2'))):
        part = report.get(role, {}) if isinstance(report, dict) else {}
        motors = part.get('motors', {}) if isinstance(part, dict) else {}
        for bus in buses:
            for motor in range(1, 9):
                key = f'{bus}/{motor}'; label = f'{role}:{key}'
                entry = motors.get(key, {}) if isinstance(motors, dict) else {}
                if not isinstance(entry, dict): entry = {}
                state = entry.get('state')
                states = entry.get('states')
                # A mixed window can contain a real disable/fault even when its
                # summary state is None. Never hide this evidence behind a
                # different motor's missing samples or an ENABLED summary.
                observed = list(states) if isinstance(states, list) else []
                observed.append(state)
                bad = {v for v in observed if type(v) is int and v != 1}
                if any(v != 0 for v in bad):
                    faults.extend(f'{label}=0x{v:X}' for v in sorted(bad) if v != 0)
                elif 0 in bad:
                    disabled.append(label)
                if (entry.get('fresh') is not True or type(state) is not int or
                    states != [state] or type(entry.get('samples')) is not int or
                    entry['samples'] < 3):
                    unknown.append(label)
    if faults: return 'MOTOR_FAULT', '电机故障：' + ', '.join(faults)
    if disabled: return 'MOTORS_DISABLED', f'{len(disabled)}/32 个电机失能：' + ', '.join(disabled)
    if unknown: return 'MOTOR_UNKNOWN', '电机反馈缺失/过期：' + ', '.join(unknown)
    return 'ENABLED', '32/32 电机已使能'


def evaluate(status, motors, latched_session=None):
    code, message = motor_health(motors)
    sid = session_id(status)
    # A drive loss on a RUNNING session cannot silently become healthy again
    # just because a motor is re-enabled. Require a new initialized session.
    # Unknown acquisition is immediately not ready, but only the monitor can
    # judge its persistence across samples. Confirmed drive loss is immediate.
    if isinstance(status, dict) and status.get('state') == 'RUNNING' and sid and code in ('MOTOR_FAULT', 'MOTORS_DISABLED'):
        latched_session = sid
    result = dict(teleop_ready=False, follow_enabled={'left':False, 'right':False},
                  invalidated_session=latched_session, diagnostic=code, message=message)
    if not isinstance(status, dict):
        return dict(result, diagnostic='UNAVAILABLE', message='控制状态不可达；禁止依据旧 RUNNING 操作')
    if status.get('state') in ('FAULT', 'E_STOP') or status.get('fault_bits') not in (0, None):
        return dict(result, diagnostic='FAULT', message='控制故障：'+str(status.get('reason', '未知')))
    if code != 'ENABLED': return result
    if sid and sid == latched_session:
        return dict(result, diagnostic='REINITIALIZE_REQUIRED', message='本会话曾失去电机使能/有效反馈；需确认安全后重新初始化，禁止自动续接')
    ages = [status.get(k) for k in ('action_age_ms','feedback_age_ms')]
    collection = status.get('collection') or {}
    if not isinstance(collection, dict): collection = {}
    arms = status.get('enabled_arms')
    valid = (status.get('state') == 'RUNNING' and type(status.get('fault_bits')) is int
             and status['fault_bits'] == 0 and sid is not None
             and status.get('feedback_fresh_for_control') is True
             and status.get('relative_follow_reference_captured') is True
             and isinstance(arms,list) and all(isinstance(arm,str) for arm in arms)
             and sorted(arms) == ['left','right']
             and all(number(x) and 0 <= x < 100 for x in ages)
             and all(collection.get(side+'_mode') in ('FOLLOW','HOLD','RETURNING') for side in ('left','right')))
    if not valid:
        return dict(result, diagnostic='NOT_READY', message='控制状态、会话或动作/反馈新鲜度未满足遥操条件')
    transition = collection.get('transitioning_arms')
    if not isinstance(transition,list):
        return dict(result, diagnostic='NOT_READY', message='各臂衔接状态不可用')
    follow = {side:collection.get(side+'_mode') == 'FOLLOW' and side not in transition
              for side in ('left','right')}
    modes = '；'.join(('左' if side=='left' else '右')+'臂 '+collection[side+'_mode'] for side in ('left','right'))
    return dict(result, teleop_ready=True, diagnostic='READY', follow_enabled=follow,
                message='32/32 电机使能、通信有效；'+modes)


def read_report(path=HEALTH_PATH, now_ns=None, boot_id=None):
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    boot_id = BOOT_PATH.read_text().strip() if boot_id is None else boot_id
    report = json.loads(Path(path).read_text())
    if not isinstance(report,dict) or report.get('schema_version') != 1:
        raise ValueError('invalid health report')
    # Age is measured from acquisition START, not when slow SSH finally ends.
    started = report.get('probe_started_monotonic_ns')
    if (type(started) is not int or not 0 <= now_ns-started <= MAX_REPORT_AGE_S*1e9
        or report.get('host_boot_id') != boot_id):
        raise ValueError('health report expired or belongs to another boot')
    return report


def invalidation_summary(evidence):
    """Short operator explanation; full motor/status evidence stays in JSON."""
    code = evidence.get('diagnostic') if isinstance(evidence, dict) else None
    if code not in ('MOTOR_UNKNOWN', 'MOTORS_DISABLED', 'MOTOR_FAULT',
                    'UNAVAILABLE', 'NOT_READY', 'FAULT'):
        return '历史首次原因未知（旧版本未保留证据）'
    stamp = evidence.get('observed_unix_s')
    when = '时间未知'
    if number(stamp):
        try:
            when = time.strftime('%Y-%m-%d %H:%M:%S %Z', time.localtime(stamp))
        except (ValueError, OverflowError, OSError):
            pass
    return f'首次记录：{code}，{when}'


def apply_report(status, report):
    """UI-facing state; preserve raw state, never turn FAULT into READY."""
    value = dict(status) if isinstance(status,dict) else {}
    value['control_state'] = value.get('state', 'UNKNOWN')
    same = session_id(value) is not None and session_id(value) == session_id(report.get('status'))
    if not same:
        check = dict(teleop_ready=False, diagnostic='CHECKING', message='电机检查与当前控制会话不一致', follow_enabled={})
    else:
        latch = report.get('invalidated_session')
        history = report.get('invalidation_history')
        evidence = report.get('invalidation_evidence')
        if isinstance(history, list) and any(isinstance(item, dict) and
                item.get('leader_session_id') == session_id(value) for item in history):
            latch = session_id(value)
            evidence = next(item for item in history if isinstance(item, dict) and
                            item.get('leader_session_id') == latch)
        check = evaluate(value, report.get('motors'), latch)
        if check['diagnostic'] == 'REINITIALIZE_REQUIRED':
            check['message'] += '；' + invalidation_summary(evidence)
        if check['teleop_ready'] and 'recovery_pending' in report and not (
                report['recovery_pending'] is False and
                type(report.get('recovery_good_samples')) is int and
                report['recovery_good_samples'] >= 3 and
                report.get('monitor_session_id') == session_id(value)):
            check = dict(teleop_ready=False, diagnostic='HEALTH_RECOVERING',
                         message='健康采样恢复核验中，需连续 3 次同会话完整检查通过',
                         follow_enabled={'left':False, 'right':False})
    value.update(teleop_ready=check['teleop_ready'], readiness_reason=check['message'],
                 follow_enabled=check['follow_enabled'])
    if not check['teleop_ready'] and value.get('state') not in ('FAULT','E_STOP','DISCONNECTED'):
        value['state'] = check['diagnostic']
    return value


def apply_current_report(status, path=HEALTH_PATH):
    try:
        return apply_report(status, read_report(path))
    except (OSError, ValueError, TypeError):
        value = dict(status) if isinstance(status,dict) else {}
        value.update(control_state=value.get('state','UNKNOWN'), teleop_ready=False,
                     readiness_reason='电机健康监测未就绪或已过期，不能确认可遥操', follow_enabled={})
        if value.get('state') not in ('FAULT','E_STOP','DISCONNECTED'): value['state']='HEALTH_STALE'
        return value

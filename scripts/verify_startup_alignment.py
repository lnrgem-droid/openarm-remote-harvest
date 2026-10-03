#!/usr/bin/env python3
"""Read-only, bounded startup pose acceptance. Never commands any robot motion.

Checks physical home/correspondence before ALIGN, then correspondence/targets
after releasing startup hold. A continuous stable window is mandatory.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import shlex
import socket
import subprocess
import tempfile
import time


def validate_policy(p):
    if p.get('schema_version') != 1:
        raise ValueError('unsupported alignment policy')
    for key in ('home_rad', 'pair_tolerance_rad'):
        values = p[key]
        if len(values) != 7 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(key + ' must contain seven finite radians')
    if not all(0 < v <= .07 for v in p['pair_tolerance_rad']):
        raise ValueError('pair tolerance must be positive and no larger than 0.07 rad')
    for key, maximum in (('home_tolerance_rad', .07), ('target_tolerance_rad', .01),
                         ('stable_range_rad', .02), ('max_age_ms', 100.),
                         ('timeout_s', 20.)):
        if not math.isfinite(p[key]) or not 0 < p[key] <= maximum:
            raise ValueError('invalid ' + key)
    if not 1 <= p['stable_duration_s'] <= min(3., p['timeout_s']):
        raise ValueError('invalid stable duration')
    return p


def vector(s, key):
    values = s.get(key)
    if not isinstance(values, list) or len(values) != 7:
        raise ValueError('缺少七关节反馈：' + key)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError('关节反馈非有限数值：' + key)
    return values


def assess(s, phase, p):
    issues, rows, positions = [], [], []
    expected = 'ALIGNING' if phase == 'aligning' else 'RUNNING'
    if s.get('state') != expected or s.get('fault_bits') != 0:
        issues.append('控制状态必须为 ' + expected + ' 且无故障')
    if s.get('feedback_fresh_for_control') is not True:
        issues.append('从臂反馈不新鲜')
    for key in ('action_age_ms', 'feedback_age_ms'):
        age = s.get(key)
        if not isinstance(age, (int, float)) or not math.isfinite(age) or not 0 <= age <= p['max_age_ms']:
            issues.append(key + ' 缺失或过期')
    if not isinstance(s.get('leader_session_id'), int) or s['leader_session_id'] <= 0:
        issues.append('主臂会话无效')
    if set(s.get('enabled_arms', [])) != {'left', 'right'}:
        issues.append('必须同时具有左右臂反馈')
    if phase == 'following':
        c = s.get('collection') or {}
        if s.get('relative_follow_reference_captured') is not True:
            issues.append('跟随参考未建立')
        if c.get('left_mode') != 'FOLLOW' or c.get('right_mode') != 'FOLLOW':
            issues.append('左右臂未同时处于 FOLLOW')
        if c.get('recording') is not None or c.get('transitioning_arms') or c.get('return_phase') != 'idle':
            issues.append('存在录制/姿态过渡任务')
    try:
        for side in ('left', 'right'):
            leader = vector(s, 'leader_' + side + '_rad')
            actual = vector(s, side + '_actual_rad')
            target = vector(s, side + '_target_rad') if phase == 'following' else None
            positions.extend(leader + actual)
            for j, (a, b, tolerance) in enumerate(zip(leader, actual, p['pair_tolerance_rad'])):
                name = ('左' if side == 'left' else '右') + ' J' + str(j + 1)
                error = a - b
                row = {'joint': name, 'leader_rad': a, 'actual_rad': b,
                       'pair_error_rad': error, 'tolerance_rad': tolerance}
                if abs(error) > tolerance:
                    issues.append(f'{name} 主从差 {math.degrees(error):+.2f}°，允许 ±{math.degrees(tolerance):.2f}°')
                if phase == 'aligning':
                    for role, value in (('主臂', a), ('从臂', b)):
                        if abs(value - p['home_rad'][j]) > p['home_tolerance_rad']:
                            issues.append(f'{name} {role}未到初始位')
                else:
                    row.update(target_rad=target[j], tracking_error_rad=target[j] - b)
                    if abs(target[j] - a) > p['target_tolerance_rad']:
                        issues.append(f'{name} 下发目标未与主臂一一对应')
                    if abs(target[j] - b) > tolerance:
                        issues.append(f'{name} 实际跟踪误差超限')
                rows.append(row)
    except ValueError as exc:
        issues.append(str(exc))
    return {'ok': not issues, 'issues': issues, 'joints': rows,
            'positions': positions, 'leader_session_id': s.get('leader_session_id')}


class StableWindow:
    def __init__(self, policy):
        self.policy = policy
        self.samples = deque()

    def add(self, now, result):
        if not result['ok']:
            self.samples.clear()
            return False
        if self.samples and now - self.samples[-1][0] > .4:
            self.samples.clear()
        self.samples.append((now, result['positions']))
        while len(self.samples) > 1 and now - self.samples[1][0] >= self.policy['stable_duration_s']:
            self.samples.popleft()
        if len(self.samples) < 6 or now - self.samples[0][0] < self.policy['stable_duration_s']:
            return False
        return all(max(axis) - min(axis) <= self.policy['stable_range_rad']
                   for axis in zip(*(values for _, values in self.samples)))


def status_request():
    with tempfile.TemporaryDirectory(prefix='oa_align_') as tmp:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
            client.bind(tmp + '/reply')
            client.settimeout(.5)
            client.sendto(b'{"command":"status"}', '/tmp/openarm_remote_runtime.sock')
            return json.loads(client.recv(65536))


def verify(read_status, phase, p, clock=time.monotonic, sleep=time.sleep):
    window = StableWindow(p)
    deadline = clock() + p['timeout_s']
    session = None
    last = {'ok': False, 'issues': ['尚未收到反馈']}
    samples = 0
    while clock() < deadline:
        try:
            s = read_status()
            last = assess(s, phase, p)
            # Never continue acceptance across a restart or latched fault.
            if s.get('state') in ('FAULT', 'E_STOP') or s.get('fault_bits', 0):
                return dict(last, ok=False, accepted=False, phase=phase, samples=samples)
            current_session = s.get('leader_session_id')
            if session is not None and current_session != session:
                return dict(last, ok=False, accepted=False, phase=phase,
                            issues=['检查中主臂会话变化，禁止继续启动'])
            if last['ok']:
                session = current_session
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            last = {'ok': False, 'issues': ['读取控制状态失败：' + str(exc)]}
        samples += 1
        if window.add(clock(), last):
            return dict(last, accepted=True, phase=phase, samples=samples,
                        verified_unix_s=time.time())
        sleep(.1)
    return dict(last, ok=False, accepted=False, phase=phase, samples=samples,
                issues=last['issues'] or ['未获得连续静止、误差合格的 1 秒窗口'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jetson')
    parser.add_argument('--phase', choices=('aligning', 'following'), required=True)
    parser.add_argument('--policy', default=str(Path(__file__).with_name('startup_alignment_policy.json')))
    parser.add_argument('--policy-json')
    parser.add_argument('--report')
    args = parser.parse_args()
    try:
        p = validate_policy(json.loads(args.policy_json or Path(args.policy).read_text()))
        if args.jetson:
            if args.jetson.startswith('-'):
                raise ValueError('invalid SSH destination')
            command = '/usr/bin/python3 - --phase ' + args.phase + ' --policy-json ' + shlex.quote(json.dumps(p))
            remote = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                                     args.jetson, command], text=True,
                                    input=Path(__file__).read_text(), capture_output=True,
                                    timeout=p['timeout_s'] + 7)
            result = json.loads(remote.stdout)
            if remote.returncode != 0:
                result['accepted'] = False
        else:
            result = verify(status_request, args.phase, p)
    except (OSError, ValueError, TypeError, KeyError, AttributeError, subprocess.TimeoutExpired) as exc:
        result = {'accepted': False, 'issues': [str(exc)], 'phase': args.phase}
    result.pop('positions', None)
    if args.report:
        Path(args.report).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    if args.jetson:
        print('  自动对齐验收：' + ('通过' if result.get('accepted') else '未通过，不允许开始遥操'))
        for issue in result.get('issues', []):
            print('  - ' + issue)
    else:
        print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get('accepted') is True else 1


if __name__ == '__main__':
    raise SystemExit(main())

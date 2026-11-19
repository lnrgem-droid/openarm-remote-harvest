"""Read-only single-left recording contract, shared by manager, recorder and UI."""
import json
import math
import socket
import tempfile
from pathlib import Path

ROLES = ('left_wrist', 'chest')
TASKS = {'LEFT_PULL_LOG': 'left_pull', 'LEFT_PUSH_LOG': 'left_push'}
TITLES = {'LEFT_PULL_LOG': '拉出菌棒', 'LEFT_PUSH_LOG': '推回菌棒'}
SOCKET = '/tmp/openarm_left_runtime.sock'
ACTIVE = Path('/tmp/openarm-left-log-recording.active')
ERROR = Path('/tmp/openarm-left-log-recording.error')
CAMERA_STATUS = Path('/tmp/openarm-left-log-camera-status.json')


def runtime_status(timeout=.15):
    with tempfile.TemporaryDirectory(prefix='oa_left_read_') as directory:
        with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as client:
            client.bind(directory+'/reply'); client.settimeout(timeout)
            client.sendto(b'{"command":"status"}', SOCKET)
            value=json.loads(client.recv(65536))
    if not isinstance(value,dict) or value.get('error'):
        raise ValueError(str(value))
    return value


def finite_vector(value,size):
    return isinstance(value,(list,tuple)) and len(value)==size and all(
        type(x) in (int,float) and math.isfinite(x) for x in value)


def ready_reason(s):
    if s.get('mode')!='left_only' or s.get('enabled_arms')!=['left']:
        return '未连接单左臂遥操'
    if s.get('state')!='RUNNING' or type(s.get('fault_bits')) is not int or s['fault_bits']!=0:
        return '左臂未处于正常跟随状态：'+str(s.get('reason') or s.get('state'))
    if (s.get('motor_health') or {}).get('healthy') is not True:
        return '左臂电机健康检查未通过'
    if s.get('feedback_fresh_for_control') is not True:
        return '左臂反馈已过期'
    for name in ('action_age_ms','feedback_age_ms'):
        value=s.get(name)
        if type(value) not in (int,float) or not math.isfinite(value) or not 0<=value<100:
            return '主从反馈已过期或缺失'
    if type(s.get('leader_session_id')) is not int or s['leader_session_id']<=0:
        return '主臂会话无效'
    if not finite_vector(s.get('applied_axes'),16) or not finite_vector(s.get('left_actual_rad'),7):
        return '左臂状态或已下发动作不完整'
    return ''


def action_vector(status):
    reason=ready_reason(status)
    if reason: raise ValueError(reason)
    result=list(status['applied_axes'][:8])
    result[7]=max(0.,min(1.,result[7]/-1.0472))
    return result

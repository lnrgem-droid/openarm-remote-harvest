"""Recovery on an explicit START click after drive power/feedback was lost.

Admission is not proof of a physical power cycle. It recognizes only known,
idle control owners. Fresh drive queries and a verified stop remain mandatory.
The caller holds both lifecycle locks and obtained support/re-home confirmation.
"""


def known_owners(remote, config, *, gateway):
    identities = remote.get('process_identities')
    if not isinstance(identities, dict) or not identities:
        return False
    if {int(pid) for pid in identities} != set(remote.get('processes', [])):
        return False
    root = ('/home/openarm' if config['profile'] == 'host-test' else '/home/nvidia') + '/dev/openarm-remote-harvest/ros2_robot/'
    roles = []
    for pid, item in identities.items():
        if int(pid) <= 1 or type(item.get('start_ticks')) is not int or item['start_ticks'] <= 0:
            return False
        argv = item['argv']
        argv = argv[:-1] if argv and argv[-1] == '' else argv
        if not argv or not all(isinstance(a, str) for a in argv):
            return False
        if argv[:5] == ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
                        'remote_teleop_runtime', 'bimanual_follower.launch.py']:
            roles.append('launch')
        elif (argv[0].startswith(root)
              and argv[0].endswith('/openarm_gravity_pd_control/openarm_gravity_pd_node')
              and '__node:=follower_gravity_pd' in argv):
            roles.append('controller')
        elif len(argv) >= 2 and argv[0] == '/usr/bin/python3' and argv[1].startswith(root):
            if argv[1].endswith('/remote_teleop_runtime/remote-teleop-follower'):
                roles.append('gateway')
            elif argv[1].endswith('/remote_teleop_follower_safety/remote-teleop-follower-watchdog'):
                roles.append('watchdog')
            else:
                return False
        else:
            return False
    expected = ['controller', 'launch', 'watchdog'] + (['gateway'] if gateway else [])
    return sorted(roles) == sorted(expected)


def eligible(result, config):
    """Do not reset E-stop, motor faults, active collection or unknown owners."""
    try:
        e = result['evidence']; r = e['remote']; unit = e['host_unit']
        if (e.get('profile') != config['profile'] or e.get('peer') != config['peer']
                or e.get('ssh_ok') is not True or e.get('host_can_ok') is not True
                or r.get('can_ok') is not True or r.get('recorder_idle') is not True
                or not e.get('host_boot_id') or not r.get('boot_id')
                or unit.get('LoadState') != 'loaded' or not unit.get('InvocationID')
                or not isinstance(e.get('local_control_processes'), list)):
            return False
        active = (unit.get('ActiveState') == 'active' and int(unit['MainPID']) > 1
                  and bool(unit.get('ControlGroup')) and bool(e['local_control_processes']))
        stopped = (unit.get('ActiveState') in ('inactive', 'failed') and unit.get('MainPID') == '0'
                   and unit.get('ControlGroup') == '' and e['local_control_processes'] == [])
        if not (active or stopped):
            return False
        status = r.get('status')
        if status is None:
            # The dedicated stop path independently proves gateway absence again.
            return r.get('runtime_socket_exists') is False and known_owners(r, config, gateway=False)
        if r.get('runtime_socket_exists') is not True or not known_owners(r, config, gateway=True):
            return False
        if type(status.get('leader_session_id')) is not int or status['leader_session_id'] <= 0:
            return False
        bits = status.get('fault_bits')
        if type(bits) is not int:
            return False
        if status.get('state') == 'FAULT':
            if (bits not in (1, 2, 3) or status.get('reason') not in
                    ('leader action receive timeout', 'controller reports CAN failure')):
                return False
        elif status.get('state') not in ('RUNNING', 'READY') or bits != 0:
            return False
        c = status['collection']
        return (c.get('recording', 'unknown') is None and c.get('transitioning_arms') == []
                and c.get('return_phase') in ('idle', 'hold')
                and c.get('left_static_hold_phase') == 'idle'
                and c.get('left_mode') in ('FOLLOW', 'HOLD') and c.get('right_mode') in ('FOLLOW', 'HOLD')
                and all(c.get(key) is False for key in
                    ('left_align_active', 'right_master_align_active', 'left_master_align_active')))
    except (KeyError, ValueError, TypeError, AttributeError):
        return False


def recover(config, result, inspect_fn):
    from start_teleop_from_console import identity, start_decision, verify_motor_preflight, emit
    from power_down_arms import perform_stop
    from stop_without_gateway import refresh_disabled
    if not eligible(result, config):
        raise RuntimeError('旧会话不符合供电恢复条件，未执行失能或启动')
    emit({'event': 'progress', 'message':
          '检测到需重新初始化的旧会话；先确认主从电源已恢复并逐个查询32个电机…'})
    # Healthy 0/1 feedback is required from EVERY drive. A missing supply, motor
    # alarm, stale response or BUS-OFF cannot be repaired by restarting owners.
    verify_motor_preflight(config, require_disabled=False)
    again = inspect_fn(config['peer'])
    if identity(again) != identity(result) or not eligible(again, config):
        raise RuntimeError('复电查询期间会话或任务状态改变，未执行失能或启动')
    emit({'event': 'progress', 'message':
          '供电和反馈已恢复；先停止旧会话并核验全部失能，请保持四臂支撑。'})
    def progress(event):
        if event.get('message'):
            emit({'event': 'progress', 'message': '复电恢复：' + event['message']})
    stopped = perform_stop(config, inspect_fn=inspect_fn, progress=progress)
    if stopped.get('ok') is not True:
        raise RuntimeError('复电后旧会话失能未完成：' + str(stopped.get('message', '未知错误')))
    after = inspect_fn(config['peer'])
    old, new = result['evidence'], after['evidence']
    if (new.get('host_boot_id') != old['host_boot_id']
            or new['remote'].get('boot_id') != old['remote']['boot_id']
            or new['host_unit'].get('InvocationID') != old['host_unit']['InvocationID']
            or start_decision(after) not in ('restart', 'cold')):
        raise RuntimeError('失能后旧会话退出未核验，未重新使能')
    refresh_disabled(config)
    latest = inspect_fn(config['peer'])
    if identity(latest) != identity(after) or start_decision(latest) != start_decision(after):
        raise RuntimeError('复电恢复结束前状态改变，未重新使能')
    emit({'event': 'progress', 'message': '复电恢复完成，32/32 电机已确认失能；继续重新初始化、归位和对齐。'})
    return latest

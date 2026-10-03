"""Read-only guidance for the controller's existing manual left-follow gate."""
import math

TOLERANCE_RAD = 0.06


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _axes(value):
    return isinstance(value, (list, tuple)) and len(value) == 16 and all(_number(v) for v in value)


def manual_left_alignment(status):
    """Describe how to move the master to the held command; never send motion.

    HOLD retains the applied command, including gripper preload. Measured
    follower angles can differ under load and are not the resume reference.
    The controller independently rechecks all eight axes when left_follow is
    explicitly requested, so UI readiness is only an operator aid.
    """
    result = dict(rows=[], available=False, aligned=False, can_adjust=False, can_resume=False,
                  reason="等待完整的左主臂和从臂保持目标读数，请暂缓调整。",
                  max_error_rad=None, max_error_deg=None, worst_axis=None,
                  tolerance_rad=TOLERANCE_RAD, tolerance_deg=math.degrees(TOLERANCE_RAD))
    if not isinstance(status, dict):
        return result
    collection = status.get('collection')
    if not isinstance(collection, dict):
        return result
    leader, target = status.get('leader_axes'), status.get('applied_axes')
    if not _axes(leader) or not _axes(target):
        return result
    rows = []
    for index, (current, goal) in enumerate(zip(leader[:8], target[:8])):
        delta = goal-current
        aligned = abs(delta) <= TOLERANCE_RAD
        if aligned:
            instruction = "已对齐，保持此位置"
        elif index == 7:
            # The calibrated motor angle is 0 when closed and -1.0472 fully open.
            instruction = ("张开" if delta < 0 else "合拢") + f"左主臂夹爪，使差值减小（约 {abs(math.degrees(delta)):.1f}°）"
        else:
            instruction = ("增大" if delta > 0 else "减小") + f"该关节读数，朝目标调整约 {abs(math.degrees(delta)):.1f}°"
        rows.append(dict(axis=f'J{index+1}' if index < 7 else '夹爪',
                         leader_deg=math.degrees(current), target_deg=math.degrees(goal),
                         delta_deg=math.degrees(delta), delta_rad=delta,
                         aligned=aligned, instruction=instruction))
    worst = max(rows, key=lambda row: abs(row['delta_rad']))
    max_error = abs(worst['delta_rad'])
    reported = collection.get('left_alignment_error_rad')
    reported_ok = _number(reported) and reported >= 0
    aligned = max_error <= TOLERANCE_RAD and reported_ok and reported <= TOLERANCE_RAD
    result.update(rows=rows, available=True, aligned=bool(aligned),
                  max_error_rad=max_error, max_error_deg=math.degrees(max_error),
                  worst_axis=worst['axis'])
    reason = None
    if (status.get('teleop_ready') is not True or status.get('state') != 'RUNNING'
            or type(status.get('fault_bits')) is not int or status['fault_bits'] != 0
            or status.get('connected') is False):
        reason = '遥操状态不可确认，请暂停调整：' + str(status.get('readiness_reason') or status.get('error') or '等待健康检查')
    elif not all(_number(status.get(key)) and 0 <= status[key] < 100
                 for key in ('action_age_ms', 'feedback_age_ms')):
        reason = '主从读数已过期或缺失，请暂停调整，等待实时反馈恢复。'
    elif collection.get('left_align_active') or status.get('left_align_may_be_active') or status.get('left_align_stop_requested'):
        reason = '自动对齐或停止请求尚未结束；先停止并确认左从臂保持，再手动对齐。'
    elif collection.get('left_mode') != 'HOLD':
        reason = '左臂当前未处于保持状态；请先点击“保持左臂及夹爪”。'
    elif collection.get('recording'):
        reason = '请先结束并保存本条录制，再恢复跟随。'
    elif collection.get('right_mode') == 'RETURNING':
        reason = '右臂正在回位，请等待或停止右臂回位后再恢复左臂跟随。'
    elif not isinstance(collection.get('transitioning_arms'), list) or collection['transitioning_arms']:
        reason = '运动衔接尚未完成或状态不可确认，请稍候。'
    elif not reported_ok or abs(reported-max_error) > 1e-5:
        reason = '保持目标与控制器对齐检查尚未一致，请等待下一次状态更新。'
    elif not aligned:
        result['can_adjust'] = True
        reason = f"优先调整 {worst['axis']}：{worst['instruction']}。其余关节及夹爪也须全部对齐。"
    else:
        result['can_adjust'] = True
        result['can_resume'] = True
        reason = '全部关节及夹爪已对齐；保持主臂位置，点击确认后才恢复跟随。'
    result['reason'] = reason
    # Do not declare overall alignment on stale or otherwise unsafe readings.
    # Individual rows remain a readout of the last sample, not permission.
    result['aligned'] = result['can_resume']
    return result

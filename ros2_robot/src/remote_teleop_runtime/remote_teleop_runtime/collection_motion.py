"""Per-arm collection control. No ROS/CAN access; all motion requires a live gate.

Holds preserve applied targets (including contact preload). Saved poses contain
both the applied target and the measured equilibrium. Return is a slow joint
trajectory, not a collision-aware Cartesian planner.
"""
import json
import math
import os
from pathlib import Path
import time
from remote_teleop_protocol.protocol import decode_collection_ack, PacketError


# Match the right-arm xacro/controller J2 +pi/2 offset.
LIMITS = [(-1.396263, 3.490659), (-1.745329+math.pi/2, 1.745329+math.pi/2),
          (-1.570796, 1.570796), (0.0, 2.443461), (-1.570796, 1.570796),
          (-0.785398, 0.785398), (-1.570796, 1.570796), (-1.0472, 0.0)]
COMMANDS = {"left_lock", "left_follow", "left_align_follow", "left_align_pause",
            "left_master_align", "left_master_pause", "right_save", "right_return",
            "right_pause", "right_follow", "right_master_align", "right_master_pause", "collection_begin", "collection_end"}
# Saved-pose return only. Startup homing and normal FOLLOW limits are separate.
# Quintic smoothstep has peak derivative 1.875; both arms share a duration
# calculated from these peak velocity bounds (gripper speed is unchanged).
RETURN_SPEED_RAD_S = (0.20,) * 7 + (0.25,)
# Left v10 URDF offsets differ from the right arm: J1 -2.094396,
# J2 -pi/2. Remaining arm limits and the motor-space gripper range match.
LEFT_LIMITS = [(-1.396263-2.094396, 3.490659-2.094396),
               (-1.745329-math.pi/2, 1.745329-math.pi/2), *LIMITS[2:]]
LEFT_ALIGN_SPEED_RAD_S = (0.10,) * 7 + (0.15,)
RIGHT_MASTER_ALIGN_SPEED_RAD_S = (0.10,) * 7 + (0.15,)
LEFT_ALIGN_MAX_LEADER_DRIFT_RAD = 0.10
LEFT_ALIGN_MAX_TRACKING_ERROR_RAD = 0.20
LEFT_ALIGN_TOLERANCE_RAD = 0.06


def pose(values):
    result = tuple(float(v) for v in values)
    if len(result) != 8 or not all(math.isfinite(v) for v in result):
        raise ValueError("姿态必须包含 8 个有限数值")
    return result


def distance(a, b):
    return max(abs(x-y) for x, y in zip(a, b))


class CollectionMotion:
    def __init__(self, path):
        self.path = Path(path)
        self.left = self.right = None
        self.returning = False
        self.leader_target = None
        self.phase = "idle"
        self.resume_on_release = True
        self.right_servo_release_pending = False
        self.right_servo_release_detail = {}
        self.right_servo_free_since = None
        self.leader_now = None
        self.leader_speed = 0.0
        self.saved = None
        self.note = ""
        self.recording = None
        self.reached_since = None
        self.return_detail = {}
        self.transition = {"left": None, "right": None}
        self.last_update = None
        self.left_align_active = False
        self.left_align_phase = "idle"
        self.left_align_detail = {}
        self.collection_ack = 0
        self.left_master_target = None
        self.left_master_align_active = False
        self.left_master_align_phase = "idle"
        self.left_master_align_detail = {}
        self.left_master_pause_requested = False
        self.right_master_align_active = False
        self.right_master_align_phase = "idle"
        self.right_master_align_detail = {}
        self.right_master_pause_requested = False
        try:
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict):
                raise ValueError("起始位必须是 JSON 对象")
            if data.get("schema_version") != 2:
                raise ValueError("起始位版本不匹配")
            self.validate_saved(data)
            self.saved = data
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.note = f"起始位文件不可用，请重新保存：{exc}"

    @staticmethod
    def validate_saved(data):
        if data.get("robot") != "OpenArm-v10-right":
            raise ValueError("起始位机械臂型号/左右角色不匹配")
        for field in ("target", "actual", "leader"):
            q = pose(data[field])
            if any(v < low - 0.01 or v > high + 0.01 for v, (low, high) in zip(q, LIMITS)):
                raise ValueError("右臂起始位超出 v10 关节限位")
        if distance(data["target"][:7], data["actual"][:7]) > 0.20:
            raise ValueError("起始位跟踪误差过大")
        if distance(data["leader"], data["target"]) > 0.06:
            raise ValueError("保存时主从目标未对齐，请静止后重新保存")

    @property
    def flags(self):
        return ((1 if self.left is not None else 0) | (2 if self.right is not None else 0)
                | (4 if self.leader_target is not None else 0)
                | (8 if self.left_master_target is not None else 0))

    @property
    def leader_servo_target(self):
        return self.left_master_target if self.left_master_target is not None else self.leader_target

    def _collection_modes(self):
        try:
            return decode_collection_ack(self.collection_ack)
        except (PacketError, TypeError):
            return 3, None  # Never turn an unknown peer into an idle servo.

    def interrupt(self, actual, reason):
        if self.right_master_align_active and self.right_master_align_phase != "failed":
            self._fail_right_master(reason + "；右主臂轨迹已停止，从臂继续保持", self.leader_now)
        if self.left_master_align_active and self.left_master_align_phase != "failed":
            self._fail_left_master(reason + "；左主臂轨迹已停止，从臂继续保持", self.leader_now)
        if self.left_align_active:
            self._stop_left_alignment("failed", reason + "；左从臂停止对齐并保持，不会自动继续")
        if self.right_servo_release_pending:
            # Withdrawal was explicitly requested. Never turn a failed release
            # into a fresh servo target, or relatch either follower's pose.
            self.right_servo_release_pending = False
            self.returning = False
            self.phase = "hold"
            self.right_servo_free_since = None
            self.note = reason + "；右从臂保持，右主臂解除尚未确认，请显式重试"
            self.right_servo_release_detail["message"] = self.note
        elif self.returning:
            # right_return seeds this from the last applied target; each
            # successful trajectory step replaces it with the target returned
            # for publication. Preserve that preload on interruption, including
            # a healthy transport reporting a failed leader servo ACK.
            if self.right is None:
                self.right = pose(actual[8:16])
            if self.leader_now is not None:
                self.leader_target = pose(self.leader_now[8:16])
            self.returning = False
            self.phase = "hold"
            self.note = reason + "；右主从臂停止回位并保持，不会自动继续"
        self.transition = {"left": None, "right": None}

    def status(self, actual, leader):
        right_ack, left_ack = self._collection_modes()
        return {
            "left_mode": "HOLD" if self.left is not None else "FOLLOW",
            "right_mode": "RETURNING" if self.returning else "HOLD" if self.right is not None else "FOLLOW",
            "saved": self.saved, "note": self.note, "recording": self.recording,
            "return_phase": self.phase,
            "return_detail": self.return_detail,
            "right_pause_release_supported": True,
            "right_servo_release_pending": self.right_servo_release_pending,
            "right_servo_release_required": (self.right is not None and not self.returning
                and not self.right_master_align_active
                and (self.leader_target is not None or right_ack in (1, 2))),
            "right_servo_release_detail": dict(self.right_servo_release_detail),
            "transitioning_arms": [side for side, value in self.transition.items() if value is not None],
            "left_alignment_error_rad": distance(leader[:8], self.left) if self.left else 0.0,
            "right_alignment_error_rad": distance(leader[8:16], self.right) if self.right else 0.0,
            "right_ready_error_rad": distance(actual[8:15], self.saved["actual"][:7]) if self.saved else None,
            "left_align_supported": True,
            "left_align_active": self.left_align_active,
            "left_align_phase": self.left_align_phase,
            "left_align_detail": dict(self.left_align_detail),
            "right_master_align_supported": left_ack is not None and right_ack in (0, 1, 2),
            "right_master_align_active": self.right_master_align_active,
            "right_master_align_phase": self.right_master_align_phase,
            "right_master_align_detail": {**self.right_master_align_detail,
                "leader_ack": self.collection_ack, "leader_left_mode": left_ack,
                "leader_right_mode": right_ack,
                "servo_released": right_ack == 0 and self.leader_target is None and not self.right_master_align_active},
            "left_master_align_supported": left_ack is not None,
            "left_master_align_active": self.left_master_align_active,
            "left_master_align_phase": self.left_master_align_phase,
            "left_master_align_detail": {**self.left_master_align_detail,
                "leader_ack": self.collection_ack, "leader_left_mode": left_ack,
                "leader_right_mode": right_ack,
                "servo_released": left_ack == 0 and self.left_master_target is None and not self.left_master_align_active},
        }

    def _stop_left_alignment(self, phase, message):
        # self.left is the last commanded target, including any contact preload.
        # Never relatch a drifting measurement or resume an interrupted path.
        self.left_align_active = False
        self.left_align_phase = phase
        self.transition["left"] = None
        self.left_align_detail = {**self.left_align_detail, "message": message}

    def _start_left_alignment(self, actual, applied, leader, now):
        if self.left is None:
            raise ValueError("请先保持左臂及夹爪，再开始低速对齐")
        if self.returning or any(self.transition.values()):
            raise ValueError("运动切换未完成，不能开始左臂对齐")
        start, goal, measured = pose(applied[:8]), pose(leader[:8]), pose(actual[:8])
        for label, values in (("当前目标", start), ("主臂目标", goal)):
            for index, (value, (low, high)) in enumerate(zip(values, LEFT_LIMITS)):
                if value < low-1e-6 or value > high+1e-6:
                    axis = f"J{index+1}" if index < 7 else "夹爪"
                    raise ValueError(f"左臂{label}{axis}超出 v10 左臂限位")
        if distance(start, measured) > LEFT_ALIGN_MAX_TRACKING_ERROR_RAD:
            raise ValueError("左从臂当前目标与实测偏差超过 0.20 rad，不能开始对齐")
        self.left_align_start, self.left_align_goal = start, goal
        self.left_align_duration = max(2.0, 1.875 * max(
            abs(a-b)/speed for a, b, speed in zip(start, goal, LEFT_ALIGN_SPEED_RAD_S)))
        self.left_align_started = self.left_align_last_tick = now
        self.left_align_elapsed = 0.0
        self.left_align_reached_since = None
        self.left_align_stable_s = 0.0
        self.left = start
        self.left_align_active = True
        self.left_align_phase = "aligning"
        self.left_align_detail = {
            "message": "左从臂及夹爪正低速对齐固定主臂目标；请保持左主臂和夹爪静止",
            "goal_rad": list(goal), "start_rad": list(start),
            "elapsed_s": 0.0, "duration_s": self.left_align_duration, "progress": 0.0,
            "leader_error_rad": 0.0, "follower_error_rad": distance(start, measured),
            "speed_limits_rad_s": list(LEFT_ALIGN_SPEED_RAD_S),
        }

    def _update_left_alignment(self, actual, leader, now):
        try:
            measured, master = pose(actual[:8]), pose(leader[:8])
        except (TypeError, ValueError):
            self._stop_left_alignment("failed", "左臂反馈包含无效数值；已停止对齐并保持")
            return
        dt = min(0.02, max(0.0, now-self.left_align_last_tick))
        self.left_align_last_tick = now
        elapsed = self.left_align_elapsed + dt
        fraction = min(1.0, elapsed/self.left_align_duration)
        blend = 10*fraction**3-15*fraction**4+6*fraction**5
        candidate = tuple(a+blend*(b-a) for a, b in zip(self.left_align_start, self.left_align_goal))
        errors = [abs(a-b) for a, b in zip(candidate, measured)]
        leader_error = distance(master, self.left_align_goal)
        worst = max(range(8), key=errors.__getitem__)
        axis = f"J{worst+1}" if worst < 7 else "夹爪"
        self.left_align_detail.update(
            elapsed_s=elapsed, duration_s=self.left_align_duration, progress=fraction,
            leader_error_rad=leader_error, follower_error_rad=errors[worst],
            follower_goal_error_rad=distance(measured, self.left_align_goal), worst_axis=axis)
        failure = None
        if leader_error > LEFT_ALIGN_MAX_LEADER_DRIFT_RAD:
            failure = f"左主臂或夹爪已移离固定目标 {leader_error:.3f} rad（上限 0.10）"
        elif errors[worst] > LEFT_ALIGN_MAX_TRACKING_ERROR_RAD:
            failure = f"左从臂{axis}跟踪偏差 {errors[worst]:.3f} rad（上限 0.20）"
        elif now-self.left_align_started > self.left_align_duration+8.0:
            failure = "左臂对齐到位超时，未满足稳定对齐"
        if failure:
            self._stop_left_alignment("failed", failure + "；已停止并保持，不会自动继续")
            return
        self.left_align_elapsed = elapsed
        self.left = candidate
        self.left_align_phase = "settling" if fraction >= 1.0 else "aligning"
        reached = (fraction >= 1.0 and leader_error <= LEFT_ALIGN_TOLERANCE_RAD
                   and distance(measured, self.left_align_goal) <= LEFT_ALIGN_TOLERANCE_RAD)
        if reached:
            if self.left_align_reached_since is None:
                self.left_align_reached_since = now
            self.left_align_stable_s += dt
            self.left_align_detail["message"] = "左臂及夹爪已到达固定目标，正在确认稳定对齐"
            if self.left_align_stable_s >= 0.5:
                self.left_align_active = False
                self.left_align_phase = "completed"
                self.transition["left"] = list(candidate)
                self.left = None
                self.left_align_detail["message"] = "左臂及夹爪已稳定对齐，正在平滑恢复跟随"
        else:
            self.left_align_reached_since = None
            self.left_align_stable_s = 0.0

    def _fail_left_master(self, message, leader):
        # Once a path fails it can only hold or release, never resume motion.
        if self.left_master_target is not None:
            try:
                self.left_master_target = pose(leader[:8])
            except (TypeError, ValueError):
                pass  # Preserve the last finite target; local watchdog holds.
        self.left_master_align_phase = "failed"
        # Even a cached ACK=0 is insufficient after an interrupted handshake.
        # An explicit pause must withdraw the target and observe fresh free ACKs.
        self.left_master_align_active = True
        self.left_master_align_detail.update(message=message, failure_reason=message)

    def _left_master_release(self, now, result):
        self.left_master_target = None  # Explicit withdrawal; receiver sends empty release.
        self.left_master_align_active = True
        self.left_master_align_phase = "releasing"
        self.left_master_release_result = result
        self.left_master_phase_started = self.left_master_last_tick = now
        self.left_master_free_s = 0.0
        self.left_master_align_detail["message"] = "等待左主臂解除伺服；左从臂和夹爪继续保持"

    def _pause_left_master(self, applied, leader, now, healthy):
        if self.left_master_pause_requested and self.left_master_align_phase in {"stopping", "releasing"}:
            return
        if (not self.left_master_align_active and self.left_master_align_phase != "completed"
                and self._collection_modes()[1] not in (1, 2)):
            return
        if self.left is None:
            # A queued stop may arrive after follow has resumed. Latch exactly
            # the newest applied command rather than the old alignment goal.
            self.left = pose(applied[:8])
            self.transition["left"] = None
        self.left_master_pause_requested = True
        if self.left_master_align_phase == "releasing":
            self.left_master_release_result = "paused"
            return  # Keep the withdrawal/free-ACK timer already in progress.
        if not healthy:
            self._fail_left_master("停止已请求；等待新鲜控制状态以确认左主臂安全释放", leader)
            self.left_master_align_active = True
            return
        right_ack, left_ack = self._collection_modes()
        if left_ack is None:
            self._fail_left_master("左主臂伺服能力/状态未知；从臂保持，不能确认释放", leader)
            self.left_master_align_active = True
        elif left_ack in (0, 2) or self.left_master_target is None:
            self._left_master_release(now, "paused")
        else:
            self.left_master_target = pose(leader[:8])
            self.left_master_align_active = True
            self.left_master_align_phase = "stopping"
            self.left_master_phase_started = self.left_master_last_tick = now
            self.left_master_stable_s = 0.0
            self.left_master_align_detail["message"] = "左主臂停止在当前姿态，正在解除伺服；从臂继续保持"

    def _start_left_master(self, actual, leader, now):
        right_ack, left_ack = self._collection_modes()
        if left_ack is None:
            raise ValueError("当前左主臂控制器未提供左伺服能力及模式反馈")
        if left_ack != 0:
            raise ValueError("左主臂伺服或故障尚未释放，请先明确停止左主臂对齐")
        if self.left is None:
            raise ValueError("请先保持左从臂及夹爪")
        if (self.returning or self.leader_target is not None or self.left_align_active
                or any(self.transition.values()) or right_ack != 0):
            raise ValueError("已有回位、伺服或跟随切换；请先完成或停止")
        goal, master, measured = pose(self.left), pose(leader[:8]), pose(actual[:8])
        for label, values in (("从臂保持目标", goal), ("主臂当前位置", master)):
            if any(v < lo-1e-6 or v > hi+1e-6 for v, (lo, hi) in zip(values, LEFT_LIMITS)):
                raise ValueError(f"左{label}超出左臂及夹爪限位")
        self.left_master_goal = goal
        self.left_master_follower_start = measured
        self.left_master_target = None
        self.left_master_align_active = True
        self.left_master_align_phase = "resetting"
        self.left_master_phase_started = self.left_master_last_tick = now
        self.left_master_free_s = 0.0
        self.left_master_pause_requested = False
        self.left_master_align_detail = {"message": "确认左主臂伺服已释放；左从臂及夹爪保持目标不变",
            "goal_rad": list(goal), "progress": 0.0}

    def _update_left_master(self, actual, requested, leader, now):
        right_ack, left_ack = self._collection_modes()
        phase = self.left_master_align_phase
        if phase == "failed":
            if self.left_master_pause_requested:
                self._pause_left_master(requested, leader, now, True)
            return
        if left_ack is None:
            self._fail_left_master("左主臂模式反馈/能力丢失；轨迹停止，从臂保持", leader)
            return
        try:
            master, measured = pose(leader[:8]), pose(actual[:8])
        except (TypeError, ValueError):
            self._fail_left_master("左主从反馈包含非有限值；轨迹停止，从臂保持", leader)
            return
        dt = min(.02, max(0., now-self.left_master_last_tick))
        self.left_master_last_tick = now
        follower_drift = distance(measured, getattr(self, "left_master_follower_start", measured))
        self.left_master_align_detail["follower_drift_rad"] = follower_drift
        if phase == "releasing":
            if self.left_master_release_result == "completed" and follower_drift > .10:
                self.left_master_release_result = "failed"
                self.left_master_align_detail["failure_reason"] = "释放期间左从臂或夹爪发生显著位移"
            self.left_master_free_s = self.left_master_free_s+dt if left_ack == 0 else 0.0
            if self.left_master_free_s >= .1:
                result = self.left_master_release_result
                self.left_master_align_active = False
                self.left_master_pause_requested = False
                if result == "completed" and right_ack == 0 and distance(master, self.left) <= .06:
                    self.transition["left"] = list(self.left)
                    self.left = None
                    self.left_align_phase = "idle"; self.left_align_detail = {}
                    self.left_master_align_phase = "completed"
                    self.left_master_align_detail["message"] = "左主臂及夹爪已对齐并释放伺服，正在恢复跟随"
                elif result == "paused":
                    self.left_master_align_phase = "paused"
                    self.left_master_align_detail["message"] = "左主臂伺服已释放；左从臂及夹爪继续保持"
                else:
                    self.left_master_align_phase = "failed"
                    reason = self.left_master_align_detail.get("failure_reason", "未满足最终对齐门槛")
                    self.left_master_align_detail["message"] = f"左主臂已释放；{reason}；从臂保持，请重新对齐"
            elif now-self.left_master_phase_started > 2.0:
                # Keep explicit release pending; never reacquire an old target.
                self.left_master_align_detail["message"] = "尚未确认左主臂伺服释放；从臂保持，继续等待或检查控制状态"
            return
        if right_ack != 0:
            self._fail_left_master("右主臂伺服状态冲突；左轨迹停止，从臂保持", leader)
            return
        if phase not in {"stopping"} and follower_drift > .10:
            self._fail_left_master("左从臂或夹爪在保持期间发生显著位移；轨迹停止，从臂保持", leader)
            return
        if phase == "resetting":
            self.left_master_free_s = self.left_master_free_s+dt if left_ack == 0 else 0.0
            if self.left_master_free_s >= .1:
                self.left_master_target = master
                self.left_master_align_phase = "preparing"
                self.left_master_phase_started = now
                self.left_master_align_detail["message"] = "等待左主臂伺服确认；请松开左主臂和夹爪"
            elif now-self.left_master_phase_started > 2.0:
                self._fail_left_master("左主臂未确认退出旧伺服状态；从臂保持", leader)
            return
        if phase == "preparing":
            if left_ack == 1:
                self.left_master_start = master
                self.left_master_target = master
                self.left_master_duration = max(2., 1.875*max(abs(a-b)/v for a, b, v in zip(
                    master, self.left_master_goal, LEFT_ALIGN_SPEED_RAD_S)))
                self.left_master_started = now
                self.left_master_elapsed = self.left_master_stable_s = 0.0
                self.left_master_align_phase = "moving"
                self.left_master_align_detail.update(message="左主臂及夹爪低速对齐；左从臂保持不动",
                    duration_s=self.left_master_duration, elapsed_s=0., start_rad=list(master))
            elif left_ack == 2 or now-self.left_master_phase_started > 2.0:
                self._fail_left_master("左主臂未确认伺服或控制器故障；从臂保持", leader)
            return
        if phase == "stopping":
            if left_ack in (0, 2):
                self._left_master_release(now, "paused")
            elif distance(master, self.left_master_target) <= .06:
                self.left_master_stable_s += dt
                if self.left_master_stable_s >= .1:
                    self._left_master_release(now, "paused")
            else:
                self.left_master_stable_s = 0.0
            if self.left_master_align_phase == "stopping" and now-self.left_master_phase_started > 2.0:
                self._fail_left_master("左主臂停止姿态未稳定，保持伺服；请再次停止或检查状态", leader)
                self.left_master_pause_requested = False
            return
        if phase not in {"moving", "settling"}:
            return
        elapsed = self.left_master_elapsed + dt
        fraction = min(1., elapsed/self.left_master_duration)
        blend = 10*fraction**3-15*fraction**4+6*fraction**5
        candidate = tuple(a+blend*(b-a) for a, b in zip(self.left_master_start, self.left_master_goal))
        errors = [abs(a-b) for a, b in zip(candidate, master)]
        worst = max(range(8), key=errors.__getitem__)
        axis = f"J{worst+1}" if worst < 7 else "夹爪"
        follower_drift = distance(measured, self.left_master_follower_start)
        self.left_master_align_detail.update(elapsed_s=elapsed, progress=fraction,
            leader_error_rad=errors[worst], leader_goal_error_rad=distance(master, self.left_master_goal),
            follower_drift_rad=follower_drift, worst_axis=axis)
        failure = None
        if left_ack != 1:
            failure = "左主臂伺服确认丢失或控制器故障"
        elif errors[worst] > .20:
            failure = f"左主臂{axis}跟踪偏差 {errors[worst]:.3f} rad 超过 0.20"
        elif follower_drift > .10:
            failure = "左从臂或夹爪在保持期间发生显著位移"
        elif now-self.left_master_started > self.left_master_duration+8.0:
            failure = "左主臂对齐超时，尚未稳定到达目标"
        if failure:
            self._fail_left_master(failure+"；轨迹停止，从臂继续保持", leader)
            return
        self.left_master_target = candidate
        self.left_master_elapsed = elapsed
        self.left_master_align_phase = "settling" if fraction >= 1. else "moving"
        reached = fraction >= 1. and distance(master, self.left_master_goal) <= .06
        self.left_master_stable_s = self.left_master_stable_s+dt if reached else 0.0
        if self.left_master_stable_s >= .5:
            self._left_master_release(now, "completed")

    def _fail_right_master(self, message, leader):
        # Once a path fails it can only hold or release, never resume motion.
        if self.leader_target is not None:
            try:
                self.leader_target = pose(leader[8:16])
            except (TypeError, ValueError):
                pass  # Preserve the last finite target; local watchdog holds.
        self.right_master_align_phase = "failed"
        # Even a cached ACK=0 is insufficient after an interrupted handshake.
        # An explicit pause must withdraw the target and observe fresh free ACKs.
        self.right_master_align_active = True
        self.right_master_align_detail.update(message=message, failure_reason=message)

    def _right_master_release(self, now, result):
        self.leader_target = None  # Explicit withdrawal; receiver sends empty release.
        self.right_master_align_active = True
        self.right_master_align_phase = "releasing"
        self.right_master_release_result = result
        self.right_master_phase_started = self.right_master_last_tick = now
        self.right_master_free_s = 0.0
        self.right_master_align_detail["message"] = "等待右主臂解除伺服；右从臂和夹爪继续保持"

    def _pause_right_master(self, applied, leader, now, healthy):
        if self.right_master_pause_requested and self.right_master_align_phase in {"stopping", "releasing"}:
            return
        if (not self.right_master_align_active and self.right_master_align_phase != "completed"
                and self._collection_modes()[0] not in (1, 2)):
            return
        if self.right is None:
            # A queued stop may arrive after follow has resumed. Latch exactly
            # the newest applied command rather than the old alignment goal.
            self.right = pose(applied[8:16])
            self.transition["right"] = None
        self.right_master_pause_requested = True
        if self.right_master_align_phase == "releasing":
            self.right_master_release_result = "paused"
            return  # Keep the withdrawal/free-ACK timer already in progress.
        if not healthy:
            self._fail_right_master("停止已请求；等待新鲜控制状态以确认右主臂安全释放", leader)
            self.right_master_align_active = True
            return
        right_ack, left_ack = self._collection_modes()
        if left_ack is None or right_ack not in (0, 1, 2):
            self._fail_right_master("右主臂伺服能力/状态未知；从臂保持，不能确认释放", leader)
            self.right_master_align_active = True
        elif right_ack in (0, 2) or self.leader_target is None:
            self._right_master_release(now, "paused")
        else:
            self.leader_target = pose(leader[8:16])
            self.right_master_align_active = True
            self.right_master_align_phase = "stopping"
            self.right_master_phase_started = self.right_master_last_tick = now
            self.right_master_stable_s = 0.0
            self.right_master_align_detail["message"] = "右主臂停止在当前姿态，正在解除伺服；从臂继续保持"

    def _start_right_master(self, actual, leader, now):
        right_ack, left_ack = self._collection_modes()
        if left_ack is None or right_ack not in (0, 1, 2):
            raise ValueError("当前右主臂控制器未提供右伺服能力及模式反馈")
        if right_ack != 0:
            raise ValueError("右主臂伺服或故障尚未释放，请先明确停止右主臂对齐")
        if self.right is None or self.left is None:
            raise ValueError("请先保持左右从臂及夹爪")
        if (self.returning or self.leader_target is not None or self.left_master_target is not None
                or self.left_master_align_active or self.left_align_active
                or any(self.transition.values()) or left_ack != 0):
            raise ValueError("已有回位、伺服或跟随切换；请先完成或停止")
        goal, master, measured = pose(self.right), pose(leader[8:16]), pose(actual[8:16])
        for label, values in (("从臂保持目标", goal), ("主臂当前位置", master)):
            if any(v < lo-1e-6 or v > hi+1e-6 for v, (lo, hi) in zip(values, LIMITS)):
                raise ValueError(f"右{label}超出右臂及夹爪限位")
        self.right_master_goal = goal
        self.right_master_follower_start = measured
        self.leader_target = None
        self.right_master_align_active = True
        self.right_master_align_phase = "resetting"
        self.right_master_phase_started = self.right_master_last_tick = now
        self.right_master_free_s = 0.0
        self.right_master_pause_requested = False
        self.right_master_align_detail = {"message": "确认右主臂伺服已释放；右从臂及夹爪保持目标不变",
            "goal_rad": list(goal), "progress": 0.0}

    def _update_right_master(self, actual, requested, leader, now):
        right_ack, left_ack = self._collection_modes()
        phase = self.right_master_align_phase
        if phase == "failed":
            if self.right_master_pause_requested:
                self._pause_right_master(requested, leader, now, True)
            return
        if left_ack is None or right_ack not in (0, 1, 2):
            self._fail_right_master("右主臂模式反馈/能力丢失；轨迹停止，从臂保持", leader)
            return
        try:
            master, measured = pose(leader[8:16]), pose(actual[8:16])
        except (TypeError, ValueError):
            self._fail_right_master("右主从反馈包含非有限值；轨迹停止，从臂保持", leader)
            return
        dt = min(.02, max(0., now-self.right_master_last_tick))
        self.right_master_last_tick = now
        follower_drift = distance(measured, getattr(self, "right_master_follower_start", measured))
        self.right_master_align_detail["follower_drift_rad"] = follower_drift
        if phase == "releasing":
            if self.right_master_release_result == "completed" and follower_drift > .10:
                self.right_master_release_result = "failed"
                self.right_master_align_detail["failure_reason"] = "释放期间右从臂或夹爪发生显著位移"
            self.right_master_free_s = self.right_master_free_s+dt if right_ack == 0 else 0.0
            if self.right_master_free_s >= .1:
                result = self.right_master_release_result
                self.right_master_align_active = False
                self.right_master_pause_requested = False
                if result == "completed" and left_ack == 0 and distance(master, self.right) <= .06:
                    self.transition["right"] = list(self.right)
                    self.right = None
                    self.phase = "idle"
                    self.right_servo_release_detail = {}
                    self.right_master_align_phase = "completed"
                    self.right_master_align_detail["message"] = "右主臂及夹爪已对齐并释放伺服，正在恢复跟随"
                elif result == "paused":
                    self.right_master_align_phase = "paused"
                    self.right_master_align_detail["message"] = "右主臂伺服已释放；右从臂及夹爪继续保持"
                else:
                    self.right_master_align_phase = "failed"
                    reason = self.right_master_align_detail.get("failure_reason", "未满足最终对齐门槛")
                    self.right_master_align_detail["message"] = f"右主臂已释放；{reason}；从臂保持，请重新对齐"
            elif now-self.right_master_phase_started > 2.0:
                # Keep explicit release pending; never reacquire an old target.
                self.right_master_align_detail["message"] = "尚未确认右主臂伺服释放；从臂保持，继续等待或检查控制状态"
            return
        if left_ack != 0:
            self._fail_right_master("左主臂伺服状态冲突；右轨迹停止，从臂保持", leader)
            return
        if phase not in {"stopping"} and follower_drift > .10:
            self._fail_right_master("右从臂或夹爪在保持期间发生显著位移；轨迹停止，从臂保持", leader)
            return
        if phase == "resetting":
            self.right_master_free_s = self.right_master_free_s+dt if right_ack == 0 else 0.0
            if self.right_master_free_s >= .1:
                self.leader_target = master
                self.right_master_align_phase = "preparing"
                self.right_master_phase_started = now
                self.right_master_align_detail["message"] = "等待右主臂伺服确认；请松开右主臂和夹爪"
            elif now-self.right_master_phase_started > 2.0:
                self._fail_right_master("右主臂未确认退出旧伺服状态；从臂保持", leader)
            return
        if phase == "preparing":
            if right_ack == 1:
                self.right_master_start = master
                self.leader_target = master
                self.right_master_duration = max(2., 1.875*max(abs(a-b)/v for a, b, v in zip(
                    master, self.right_master_goal, RIGHT_MASTER_ALIGN_SPEED_RAD_S)))
                self.right_master_started = now
                self.right_master_elapsed = self.right_master_stable_s = 0.0
                self.right_master_align_phase = "moving"
                self.right_master_align_detail.update(message="右主臂及夹爪低速对齐；右从臂保持不动",
                    duration_s=self.right_master_duration, elapsed_s=0., start_rad=list(master))
            elif right_ack == 2 or now-self.right_master_phase_started > 2.0:
                self._fail_right_master("右主臂未确认伺服或控制器故障；从臂保持", leader)
            return
        if phase == "stopping":
            if right_ack in (0, 2):
                self._right_master_release(now, "paused")
            elif distance(master, self.leader_target) <= .06:
                self.right_master_stable_s += dt
                if self.right_master_stable_s >= .1:
                    self._right_master_release(now, "paused")
            else:
                self.right_master_stable_s = 0.0
            if self.right_master_align_phase == "stopping" and now-self.right_master_phase_started > 2.0:
                self._fail_right_master("右主臂停止姿态未稳定，保持伺服；请再次停止或检查状态", leader)
                self.right_master_pause_requested = False
            return
        if phase not in {"moving", "settling"}:
            return
        elapsed = self.right_master_elapsed + dt
        fraction = min(1., elapsed/self.right_master_duration)
        blend = 10*fraction**3-15*fraction**4+6*fraction**5
        candidate = tuple(a+blend*(b-a) for a, b in zip(self.right_master_start, self.right_master_goal))
        errors = [abs(a-b) for a, b in zip(candidate, master)]
        worst = max(range(8), key=errors.__getitem__)
        axis = f"J{worst+1}" if worst < 7 else "夹爪"
        follower_drift = distance(measured, self.right_master_follower_start)
        self.right_master_align_detail.update(elapsed_s=elapsed, progress=fraction,
            leader_error_rad=errors[worst], leader_goal_error_rad=distance(master, self.right_master_goal),
            follower_drift_rad=follower_drift, worst_axis=axis)
        failure = None
        if right_ack != 1:
            failure = "右主臂伺服确认丢失或控制器故障"
        elif errors[worst] > .20:
            failure = f"右主臂{axis}跟踪偏差 {errors[worst]:.3f} rad 超过 0.20"
        elif follower_drift > .10:
            failure = "右从臂或夹爪在保持期间发生显著位移"
        elif now-self.right_master_started > self.right_master_duration+8.0:
            failure = "右主臂对齐超时，尚未稳定到达目标"
        if failure:
            self._fail_right_master(failure+"；轨迹停止，从臂继续保持", leader)
            return
        self.leader_target = candidate
        self.right_master_elapsed = elapsed
        self.right_master_align_phase = "settling" if fraction >= 1. else "moving"
        reached = fraction >= 1. and distance(master, self.right_master_goal) <= .06
        self.right_master_stable_s = self.right_master_stable_s+dt if reached else 0.0
        if self.right_master_stable_s >= .5:
            self._right_master_release(now, "completed")

    def command(self, name, request, actual, applied, leader, velocities, now, healthy, leader_ack=None):
        self.leader_now = tuple(leader)
        if leader_ack is not None:
            self.collection_ack = leader_ack
        if name == "collection_end":
            if self.recording and self.recording["token"] != request.get("token"):
                raise ValueError("录制锁不属于本条 episode")
            self.recording = None
            return
        if name == "right_master_pause" or (name == "right_pause" and self.right_master_align_active):
            if self.returning:
                self.interrupt(actual, "已点击停止右臂运动")
                return  # Never run two release/trajectory coordinators together.
            self._pause_right_master(applied, leader, now, healthy)
            return
        if name == "right_pause":
            if self.right_servo_release_pending:
                return  # A duplicate cannot extend the release deadline.
            if self.returning:
                self.interrupt(actual, "已点击停止回位")
            elif request.get("release_servo") is True and self.right is not None:
                # A delayed/repeated ordinary STOP must never relax the master.
                right_ack, left_ack = self._collection_modes()
                if not healthy:
                    raise ValueError("需要 RUNNING、无故障、主从反馈及动作均新鲜")
                if self.recording:
                    raise ValueError("正在录制或封装，请先结束本条数据")
                if (self.left_master_align_active or self.left_master_target is not None
                        or self.left_align_active or any(self.transition.values())
                        or left_ack not in (None, 0) or right_ack not in (0, 1, 2)):
                    raise ValueError("运动切换或主臂伺服状态不明，不能解除右主臂回位伺服")
                if self.leader_target is None and right_ack == 0:
                    return  # Already free; never relatch or start another wait.
                self.right_servo_release_detail = {
                    "source_phase": self.phase, "source_leader_mode": right_ack,
                    "source_target_present": self.leader_target is not None,
                    "message": "等待右主臂确认解除回位伺服；右从臂继续保持，不会自动跟随",
                }
                self.leader_target = None
                self.resume_on_release = False
                self.phase = "releasing"
                self.returning = True
                self.right_servo_release_pending = True
                self.right_servo_free_since = None
                self.started = now
                self.note = self.right_servo_release_detail["message"]
            return
        if name == "left_master_pause" or (name in {"left_lock", "left_align_pause"} and self.left_master_align_active):
            self._pause_left_master(applied, leader, now, healthy)
            return
        if name == "left_align_pause" or (name == "left_lock" and self.left_align_active):
            if self.left_align_active:
                self._stop_left_alignment("paused", "已停止左臂对齐并保持当前目标；不会自动继续")
            elif name == "left_align_pause" and self.left_align_phase == "completed":
                # A pause click may arrive after completion through SSH. Hold
                # the latest applied target, never the old alignment goal.
                self.left = pose(applied[:8])
                self._stop_left_alignment("paused", "已停止左臂衔接跟随并保持当前目标；不会自动继续")
            return
        if not healthy:
            raise ValueError("需要 RUNNING、无故障、主从反馈及动作均新鲜")
        if self.recording:
            if name == "collection_begin" and request.get("token") == self.recording["token"]:
                return
            raise ValueError("正在录制或封装，请先结束本条数据")
        if self.right_master_align_active:
            if name == "right_master_align" and self.right_master_align_phase != "failed":
                return
            raise ValueError("右主臂正在对齐或尚未释放伺服，请先停止右主臂对齐")
        if self.left_master_align_active:
            if name == "left_master_align" and self.left_master_align_phase != "failed":
                return
            raise ValueError("左主臂正在对齐或尚未释放伺服，请先停止左主臂对齐")
        if name == "right_master_align":
            self._start_right_master(actual, leader, now)
            return
        if name == "left_master_align":
            self._start_left_master(actual, leader, now)
            return
        if (name in {"right_return", "right_follow", "right_save", "left_follow", "left_align_follow", "collection_begin"}
                and (self._collection_modes()[1] not in (None, 0) or self._collection_modes()[0] == 3)):
            raise ValueError("左主臂伺服尚未释放，禁止切换其他运动")
        if self.left_align_active:
            if name == "left_align_follow":
                return  # Duplicate clicks cannot recapture a moving master.
            raise ValueError("左臂正在低速对齐，请先完成或停止对齐")
        if name == "left_align_follow":
            self._start_left_alignment(actual, applied, leader, now)
            return
        if name == "collection_begin":
            side = request.get("side")
            if side not in {"left", "right"} or not request.get("token"):
                raise ValueError("录制任务/标识无效")
            if self.returning or any(self.transition.values()):
                raise ValueError("运动切换未完成，不能录制")
            if side == "left" and self.left is not None:
                raise ValueError("左臂仍在保持，请先恢复左臂跟随")
            if side == "right":
                if self.left is None or self.right is not None or self.saved is None:
                    raise ValueError("右臂采集需要左臂保持、已保存起始位、右臂恢复跟随")
                if distance(actual[8:15], self.saved["actual"][:7]) > 0.07:
                    raise ValueError("右臂尚未到达保存的起始位（允许误差 0.07 rad）")
            self.recording = {"side": side, "token": request["token"]}
            return
        if name == "left_lock":
            if self.left is None:
                self.left = pose(applied[:8])
                self.transition["left"] = None
            self.left_align_phase = "idle"
            self.left_align_detail = {}
            self.left_master_align_phase = "idle"
            self.left_master_align_detail = {}
            self.note = "左臂姿态与夹爪目标已保持，可松开左主臂"
        elif name in {"left_follow", "right_follow"}:
            side = name.split("_")[0]; offset = 0 if side == "left" else 8
            target = getattr(self, side)
            if target is None:
                return
            if self.returning:
                raise ValueError("右臂仍在回位")
            error = distance(leader[offset:offset+8], target)
            if side == "right" and self.leader_target is not None:
                self.leader_target = None
                self.phase = "releasing"
                self.returning = True
                self.resume_on_release = error <= 0.06
                self.started = now
                self.note = "等待右主臂退出回位伺服；未对齐时从臂将继续保持"
                return
            if side == "right" and self._collection_modes()[0] != 0:
                # A failed explicit release leaves no target but can still
                # have a physically latched master servo. Geometry alone is
                # never permission to resume follower motion.
                raise ValueError("右主臂伺服尚未确认解除，请先明确解除右主臂回位伺服")
            if error > 0.06:
                arm = "左" if side == "left" else "右"
                raise ValueError(f"请将{arm}主臂和夹爪对齐，当前最大差 {error:.3f} rad，要求 ≤0.06")
            self.transition[side] = list(target)
            setattr(self, side, None)
            if side == "left":
                self.left_align_phase = "idle"
                self.left_align_detail = {}
                self.left_master_align_phase = "idle"
                self.left_master_align_detail = {}
            self.note = "已对齐，正在平滑恢复跟随"
        elif name == "right_save":
            if self.returning or self.right is not None:
                raise ValueError("请在右臂跟随模式下保存起始位")
            if max(abs(v) for v in velocities[8:15]) > 0.08 or self.leader_speed > 0.08:
                raise ValueError("请先让右主从臂都静止再保存")
            data = {"schema_version": 2, "robot": "OpenArm-v10-right",
                    "saved_unix_s": time.time(), "target": list(pose(applied[8:16])),
                    "actual": list(pose(actual[8:16])), "leader": list(pose(leader[8:16]))}
            self.validate_saved(data)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(".tmp")
            with temporary.open("w") as file:
                json.dump(data, file, ensure_ascii=False, indent=2); file.flush(); os.fsync(file.fileno())
            os.replace(temporary, self.path)
            self.saved = data; self.note = "已保存右主从臂起始位及夹爪目标"
        elif name == "right_return":
            if self.returning:
                return
            if self.left is None:
                raise ValueError("请先保持左臂")
            if not self.saved:
                raise ValueError("请先保存右臂起始位")
            self.validate_saved(self.saved)
            self.right_master_align_phase = "idle"
            self.right_master_align_detail = {}
            self.start_pose = pose(applied[8:16])
            self.leader_start = pose(leader[8:16])
            # An explicit new return first releases any previously latched
            # master servo failure. The follower stays held throughout.
            self.leader_target = None
            self.right = self.start_pose
            self.duration = max(2.0, 1.875 * max(abs(a-b)/v for a,b,v in zip(
                self.start_pose, self.saved["target"], RETURN_SPEED_RAD_S)))
            self.duration = max(self.duration, 1.875 * max(abs(a-b)/v for a,b,v in zip(
                self.leader_start, self.saved["leader"], RETURN_SPEED_RAD_S)))
            self.started = now; self.returning = True; self.reached_since = None
            self.return_detail = {}
            self.phase = "resetting"
            self.transition["right"] = None
            self.note = "准备右主从回位；请松开右主臂和夹爪"

    def update(self, actual, requested, now, healthy, leader=None, leader_ack=0):
        self.collection_ack = leader_ack
        right_ack, left_ack = self._collection_modes()
        if leader is not None:
            if self.leader_now is not None and self.last_update is not None and now > self.last_update:
                self.leader_speed = distance(leader[8:16], self.leader_now[8:16]) / (now-self.last_update)
            self.leader_now = tuple(leader)
        dt = min(0.02, max(0.0, now-self.last_update)) if self.last_update is not None else 0.0
        self.last_update = now
        if not healthy:
            self.interrupt(actual, "控制许可或反馈中断")
            result = list(requested)
            if self.left is not None:
                result[:8] = self.left
            if self.right is not None:
                result[8:16] = self.right
            return result
        result = list(requested)
        if self.right_master_align_active:
            self._update_right_master(actual, requested, self.leader_now, now)
        if self.left_master_align_active:
            self._update_left_master(actual, requested, self.leader_now, now)
        if self.left_align_active:
            self._update_left_alignment(actual, self.leader_now, now)
        if self.returning and left_ack not in (None, 0):
            self.interrupt(actual, "左主臂伺服状态冲突，右回位已停止")
        leader_ack = right_ack  # Old right-return state machine retains its 0/1/2 meaning.
        if self.returning and self.phase == "resetting":
            if leader_ack == 0:
                self.phase = "preparing"; self.started = now
                self.leader_start = pose(self.leader_now[8:16])
                self.leader_target = self.leader_start
                self.duration = max(self.duration, 1.875 * max(abs(a-b)/v for a,b,v in zip(
                    self.leader_start, self.saved["leader"], RETURN_SPEED_RAD_S)))
            elif now-self.started > 2.0:
                self.interrupt(actual, "右主臂无法解除旧回位状态")
        if self.returning and leader_ack == 2 and self.phase not in {"resetting", "releasing"}:
            self.interrupt(actual, "右主臂控制器故障，请恢复跟随后重试")
        if self.returning and self.phase == "preparing":
            if leader_ack == 1:
                self.phase = "moving"; self.started = now
                self.note = "右主从臂一起低速回位中；不要触碰右主臂及夹爪"
            elif now-self.started > 2.0:
                self.interrupt(actual, "右主臂未确认回位控制")
        if self.returning and self.phase == "releasing":
            if self.right_servo_release_pending:
                if now-self.started > 2.0:
                    self.interrupt(actual, "右主臂解除伺服确认超时")
                elif leader_ack not in (0, 1, 2):
                    self.interrupt(actual, "右主臂伺服确认无效")
                elif leader_ack == 0:
                    if self.right_servo_free_since is None:
                        self.right_servo_free_since = now
                    # Healthy means all inputs are <100 ms old. Requiring a
                    # continuous 100 ms free ACK prevents a pre-withdrawal
                    # sample from completing this explicit release handshake.
                    if now-self.right_servo_free_since >= 0.10:
                        self.right_servo_release_pending = False
                        self.returning = False
                        self.phase = "hold"
                        self.note = "右主臂已确认解除回位伺服；右从臂继续保持，不会自动跟随"
                        self.right_servo_release_detail["message"] = self.note
                else:
                    self.right_servo_free_since = None
            elif leader_ack == 0:
                if self.resume_on_release and distance(self.leader_now[8:16], self.right) <= 0.06:
                    self.transition["right"] = list(self.right)
                    self.right = None; self.phase = "idle"
                    self.note = "右主从已自动恢复跟随；可以准备录制"
                else:
                    self.phase = "hold"
                    self.note = "右主臂已恢复重力补偿，从臂保持；可重新回位，或手动对齐后恢复跟随"
                self.returning = False
            elif now-self.started > 2.0:
                self.interrupt(actual, "右主臂未确认恢复跟随")
        if self.returning and self.phase == "moving":
            s = min(1.0, max(0.0, (now-self.started)/self.duration))
            blend = 10*s**3-15*s**4+6*s**5
            candidate = tuple(a+blend*(b-a) for a,b in zip(self.start_pose, self.saved["target"]))
            master = tuple(a+blend*(b-a) for a,b in zip(self.leader_start, self.saved["leader"]))
            follower_errors = [abs(a-b) for a,b in zip(candidate[:7], actual[8:15])]
            leader_errors = [abs(a-b) for a,b in zip(master, self.leader_now[8:16])]
            fj = max(range(7), key=follower_errors.__getitem__)
            lj = max(range(8), key=leader_errors.__getitem__)
            leader_axis = f"J{lj+1}" if lj < 7 else "夹爪"
            self.return_detail = {
                "elapsed_s": now-self.started, "duration_s": self.duration,
                "leader_ack": leader_ack, "leader_worst_axis": leader_axis,
                "leader_error_rad": leader_errors[lj],
                "follower_worst_axis": f"J{fj+1}", "follower_error_rad": follower_errors[fj],
                "leader_goal_error_rad": distance(self.leader_now[8:16], self.saved["leader"]),
                "follower_goal_error_rad": distance(actual[8:15], self.saved["actual"][:7]),
                "pair_target_error_rad": distance(self.leader_now[8:16], candidate),
            }
            failure = None
            if leader_ack != 1:
                failure = "右主臂回位控制确认丢失"
            elif max(follower_errors) > 0.20 or max(leader_errors) > 0.20:
                failure = (f"回位跟踪误差超限：主臂{leader_axis} {leader_errors[lj]:.3f} rad，"
                           f"从臂J{fj+1} {follower_errors[fj]:.3f} rad（上限0.20）")
            elif now-self.started > self.duration+8.0:
                failure = (f"回位到位超时：主臂{leader_axis}仍差{leader_errors[lj]:.3f} rad，"
                           f"从臂J{fj+1}仍差{follower_errors[fj]:.3f} rad；未满足稳定对齐")
            if failure:
                self.interrupt(actual, failure)
            else:
                self.right = candidate
                self.leader_target = master
                reached = (s >= 1.0 and distance(actual[8:15], self.saved["actual"][:7]) <= 0.06
                           and abs(actual[15]-self.saved["actual"][7]) <= 0.1
                           and distance(self.leader_now[8:16], self.right) <= 0.06)
                self.reached_since = (self.reached_since if self.reached_since is not None else now) if reached else None
                if reached and now-self.reached_since >= 0.5:
                    self.leader_target = None; self.phase = "releasing"; self.started = now
                    self.resume_on_release = True
                    self.note = "右主从已到位，等待主臂解除回位伺服"
        for side, offset in (("left", 0), ("right", 8)):
            held = getattr(self, side)
            if held is not None:
                result[offset:offset+8] = held
            elif self.transition[side] is not None:
                old = self.transition[side]; goal = result[offset:offset+8]
                stepped = [a+max(-0.15*dt, min(0.15*dt, b-a)) for a,b in zip(old,goal)]
                result[offset:offset+8] = stepped
                self.transition[side] = None if distance(stepped, goal) < 0.001 else stepped
        return result

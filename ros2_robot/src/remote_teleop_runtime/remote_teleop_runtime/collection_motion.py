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


# Match the right-arm xacro/controller J2 +pi/2 offset.
LIMITS = [(-1.396263, 3.490659), (-1.745329+math.pi/2, 1.745329+math.pi/2),
          (-1.570796, 1.570796), (0.0, 2.443461), (-1.570796, 1.570796),
          (-0.785398, 0.785398), (-1.570796, 1.570796), (-1.0472, 0.0)]
COMMANDS = {"left_lock", "left_follow", "right_save", "right_return",
            "right_pause", "right_follow", "collection_begin", "collection_end"}


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
        self.leader_now = None
        self.leader_speed = 0.0
        self.saved = None
        self.note = ""
        self.recording = None
        self.reached_since = None
        self.transition = {"left": None, "right": None}
        self.last_update = None
        try:
            data = json.loads(self.path.read_text())
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
                | (4 if self.leader_target is not None else 0))

    def interrupt(self, actual, reason):
        if self.returning:
            self.right = pose(actual[8:16])
            if self.leader_now is not None:
                self.leader_target = pose(self.leader_now[8:16])
            self.returning = False
            self.phase = "hold"
            self.note = reason + "；右主从臂停止回位并保持，不会自动继续"
        self.transition = {"left": None, "right": None}

    def status(self, actual, leader):
        return {
            "left_mode": "HOLD" if self.left is not None else "FOLLOW",
            "right_mode": "RETURNING" if self.returning else "HOLD" if self.right is not None else "FOLLOW",
            "saved": self.saved, "note": self.note, "recording": self.recording,
            "return_phase": self.phase,
            "left_alignment_error_rad": distance(leader[:8], self.left) if self.left else 0.0,
            "right_alignment_error_rad": distance(leader[8:16], self.right) if self.right else 0.0,
            "right_ready_error_rad": distance(actual[8:15], self.saved["actual"][:7]) if self.saved else None,
        }

    def command(self, name, request, actual, applied, leader, velocities, now, healthy):
        self.leader_now = tuple(leader)
        if name == "collection_end":
            if self.recording and self.recording["token"] != request.get("token"):
                raise ValueError("录制锁不属于本条 episode")
            self.recording = None
            return
        if name == "right_pause":
            if self.returning:
                self.interrupt(actual, "已点击停止回位")
            return
        if not healthy:
            raise ValueError("需要 RUNNING、无故障、主从反馈及动作均新鲜")
        if self.recording:
            if name == "collection_begin" and request.get("token") == self.recording["token"]:
                return
            raise ValueError("正在录制或封装，请先结束本条数据")
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
            if error > 0.06:
                arm = "左" if side == "left" else "右"
                raise ValueError(f"请将{arm}主臂和夹爪对齐，当前最大差 {error:.3f} rad，要求 ≤0.06")
            self.transition[side] = list(target)
            setattr(self, side, None)
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
            self.start_pose = pose(applied[8:16])
            self.leader_start = pose(leader[8:16])
            # An explicit new return first releases any previously latched
            # master servo failure. The follower stays held throughout.
            self.leader_target = None
            self.right = self.start_pose
            self.duration = max(2.0, 1.875 * max(abs(a-b)/v for a,b,v in zip(
                self.start_pose, self.saved["target"], [0.15]*7+[0.25])))
            self.duration = max(self.duration, 1.875 * max(abs(a-b)/v for a,b,v in zip(
                self.leader_start, self.saved["leader"], [0.15]*7+[0.25])))
            self.started = now; self.returning = True; self.reached_since = None
            self.phase = "resetting"
            self.transition["right"] = None
            self.note = "准备右主从回位；请松开右主臂和夹爪"

    def update(self, actual, requested, now, healthy, leader=None, leader_ack=0):
        if leader is not None:
            if self.leader_now is not None and self.last_update is not None and now > self.last_update:
                self.leader_speed = distance(leader[8:15], self.leader_now[8:15]) / (now-self.last_update)
            self.leader_now = tuple(leader)
        dt = min(0.02, max(0.0, now-self.last_update)) if self.last_update is not None else 0.0
        self.last_update = now
        if not healthy:
            self.interrupt(actual, "控制许可或反馈中断")
            return list(requested)
        result = list(requested)
        if self.returning and self.phase == "resetting":
            if leader_ack == 0:
                self.phase = "preparing"; self.started = now
                self.leader_start = pose(self.leader_now[8:16])
                self.leader_target = self.leader_start
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
            if leader_ack == 0:
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
            if (leader_ack != 1 or distance(candidate[:7], actual[8:15]) > 0.20
                or distance(master, self.leader_now[8:16]) > 0.20
                or now-self.started > self.duration+8.0):
                self.interrupt(actual, "回位误差超限或超时")
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

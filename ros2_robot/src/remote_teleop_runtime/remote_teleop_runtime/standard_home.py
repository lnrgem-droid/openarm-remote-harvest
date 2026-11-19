"""Slow per-side bilateral zero return, using the existing collection servo.

The follower gripper keeps its applied target; the master gripper slowly aligns
to it. Completion resumes following after master release and eight-axis checks.
"""
import math

# Zero is the commanded pose, not a promise of zero PD steady-state error.
# Use 0.05 rad arrival tolerance (stricter than startup's 0.07 rad), with
# continuous dwell and the separate 0.06 rad handoff / 0.20 rad tracking gates.
ZERO_ARRIVAL_TOLERANCE_RAD = .05


class StandardHome:
    def __init__(self, owner):
        self.owner = owner
        self.side = None
        self.phase = "idle"
        self.detail = {}

    @property
    def busy(self):
        return self.phase in {"preparing", "moving", "settling", "releasing", "handover", "failed"}

    @property
    def offset(self):
        return 0 if self.side == "left" else 8

    @property
    def target_name(self):
        return "left_master_target" if self.side == "left" else "leader_target"

    def start(self, side, actual, applied, leader, now, limits):
        o = self.owner
        if (self.busy or o.returning or o.left_align_active or o.left_master_align_active
                or o.right_master_align_active or any(o.transition.values())
                or o.left_static_phase != "idle" or o._collection_modes() != (0, 0)
                or o.left_master_target is not None or o.leader_target is not None):
            raise ValueError("请先完成或停止其他运动，并解除主臂保持/伺服，再恢复标准启动位置")
        offset = 0 if side == "left" else 8
        start, master, measured = [tuple(v[offset:offset+8]) for v in (applied, leader, actual)]
        for values in (start, master, measured):
            if len(values) != 8 or not all(math.isfinite(v) for v in values):
                raise ValueError("标准回位反馈无效")
            if any(v < low-.01 or v > high+.01 for v, (low, high) in zip(values, limits)):
                raise ValueError("标准回位姿态超出关节限位")
        if max(abs(a-b) for a, b in zip(start, measured)) > .20:
            raise ValueError("当前从臂跟踪误差超过 0.20 rad，不能开始标准回位")
        if not limits[7][0] <= start[7] <= limits[7][1]:
            raise ValueError("从臂夹爪保持目标超出限位，不能开始标准回位")
        # Encoder quantization can put J4 just below its zero lower limit.
        # The physical master rejects the entire servo packet if any target
        # is outside the exact limits, even within the feedback tolerance above.
        # Saturate the command, never the measured feedback used for safety.
        master = tuple(max(low, min(high, v)) for v, (low, high) in zip(master, limits))
        self.side, self.phase = side, "preparing"
        self.limits = limits
        self.handover_allowed = False
        self.start_pose, self.master_start = start, master
        self.goal = self.master_goal = (0.,)*7+(start[7],)
        self.duration = max(2., 1.875*max(abs(v) for v in (*start[:7], *master[:7]))/.10,
                            1.875*abs(master[7]-start[7])/.15)
        self.started = self.last_tick = now
        self.elapsed = self.stable = 0.
        self.settling_started = None
        setattr(o, side, start)
        setattr(o, self.target_name, master)
        self.detail = {"message": "等待主臂确认标准回位伺服", "progress": 0.,
                       "goal_rad": [0.]*7, "gripper_target_rad": start[7], "duration_s": self.duration,
                       "zero_arrival_tolerance_rad": ZERO_ARRIVAL_TOLERANCE_RAD}

    def fail(self, reason):
        if not self.busy or self.phase == "failed":
            return
        # Preserve last published targets, including follower contact preload.
        self.phase = "failed"
        self.detail["message"] = reason + "；已停止轨迹并保持，请点击停止标准回位解除主臂伺服"

    def stop(self, now):
        if not self.busy:
            return
        if self.phase == "releasing":
            # A stop during release must cancel the pending automatic follow.
            self.outcome = "paused"
            return
        self.release(now, "paused")

    def release(self, now, outcome):
        setattr(self.owner, self.target_name, None)
        self.phase, self.outcome = "releasing", outcome
        self.started, self.last_tick, self.stable = now, now, 0.
        self.detail["message"] = "从臂保持，等待主臂解除标准回位伺服"

    def finish_follow(self, held):
        self.owner.transition[self.side] = list(held)
        setattr(self.owner, self.side, None)
        self.phase = "completed"
        self.detail["message"] = "标准回位及交接完成；主臂伺服已解除，已恢复遥操跟随"

    def hold_for_alignment(self, reason):
        self.phase = "alignment_required"
        self.detail["message"] = reason + "；从臂保持，未恢复遥操；请对齐后恢复跟随"

    def release_feedback(self, actual, leader):
        measured = tuple(actual[self.offset:self.offset+8])
        master = tuple(leader[self.offset:self.offset+8])
        held = getattr(self.owner, self.side)
        if (held is None or len(held) != 8 or len(measured) != 8 or len(master) != 8
                or not all(math.isfinite(v) for v in (*held, *measured, *master))):
            self.hold_for_alignment("回位交接反馈无效")
            return None
        errors = [max(abs(h-a), abs(h-b)) for h, a, b in zip(held, measured, master)]
        axis = max(range(8), key=errors.__getitem__)
        self.detail.update(handover_error_rad=errors[axis],
                           handover_worst_axis=f"J{axis+1}" if axis < 7 else "夹爪",
                           handover_follower_rad=list(measured), handover_leader_rad=list(master),
                           handover_target_rad=list(held))
        return measured, master, held, max(errors)

    def handover_safe(self, measured, master, held):
        # Only recover small ARM drift after a verified zero return. Never
        # chase an arbitrary master pose, move a mismatched gripper, or extend
        # the original 0.20 rad tracking envelope / exact joint limits.
        within_limits = all(low-.01 <= v <= high+.01
                            for values in (measured, master) for v, (low, high) in zip(values, self.limits))
        if not within_limits:
            self.hold_for_alignment("回位交接姿态超出关节限位")
        elif max(abs(v-g) for v, g in zip(master[:7], self.goal[:7])) > .20:
            self.hold_for_alignment("主臂释放后偏移超过 0.20 rad")
        elif max(abs(a-b) for a, b in zip(held, measured)) > .20:
            self.hold_for_alignment("从臂交接跟踪误差超过 0.20 rad")
        elif max(abs(master[7]-held[7]), abs(measured[7]-held[7])) > .06:
            self.hold_for_alignment("回位后夹爪未对齐")
        else:
            return True
        return False

    def update(self, actual, leader, now):
        if not self.busy or self.phase == "failed":
            return
        o = self.owner
        right_ack, left_ack = o._collection_modes()
        ack = left_ack if self.side == "left" else right_ack
        dt = min(.02, max(0., now-self.last_tick))
        self.last_tick = now
        if self.phase == "handover":
            if ack != 0:
                self.fail("交接期间主臂伺服释放确认丢失")
                return
            feedback = self.release_feedback(actual, leader)
            if feedback is None:
                return
            measured, master, held, error = feedback
            if not self.handover_safe(measured, master, held):
                return
            if now-self.started > 4.:
                self.hold_for_alignment("低速交接超时，请保持主臂静止后重新对齐")
                return
            if error <= .06:
                self.finish_follow(held)
                return
            target = tuple(max(low, min(high, value)) for value, (low, high) in zip(master, self.limits))
            candidate = tuple(a+max(-.10*dt, min(.10*dt, b-a))
                              for a, b in zip(held[:7], target[:7])) + (held[7],)
            setattr(o, self.side, candidate)
            self.detail["message"] = "主臂已释放；从臂正低速接回跟随，夹爪保持；请保持主臂静止"
            return
        if self.phase == "releasing":
            self.stable = self.stable+dt if ack == 0 else 0.
            if self.stable >= .10:
                if self.outcome == "completed":
                    feedback = self.release_feedback(actual, leader)
                    if feedback is None:
                        return
                    measured, master, held, error = feedback
                    if error <= .06:
                        self.finish_follow(held)
                    elif not self.handover_allowed:
                        self.hold_for_alignment("释放前主从臂或夹爪未对齐")
                    elif self.handover_safe(measured, master, held):
                        self.phase, self.started = "handover", now
                        self.detail["message"] = "主臂释放后有小幅偏移；正在低速接回跟随，请保持主臂静止"
                else:
                    self.phase = self.outcome
                    self.detail["message"] = "已停止标准回位；从臂保持，主臂伺服已解除"
            elif now-self.started > 2.:
                self.fail("主臂解除伺服确认超时")
            return
        if self.phase == "preparing":
            if ack == 1:
                self.phase, self.started = "moving", now
            elif ack not in (0, 1) or now-self.started > 2.:
                self.fail("主臂未确认标准回位伺服")
            return
        measured = tuple(actual[self.offset:self.offset+8])
        master = tuple(leader[self.offset:self.offset+8])
        if len(measured) != 8 or len(master) != 8 or not all(math.isfinite(v) for v in (*measured, *master)):
            self.fail("标准回位反馈无效")
            return
        elapsed = self.elapsed+dt
        fraction = min(1., elapsed/self.duration)
        blend = 10*fraction**3-15*fraction**4+6*fraction**5
        candidate = tuple(a+blend*(b-a) for a, b in zip(self.start_pose, self.goal))
        master_target = tuple(a+blend*(b-a) for a, b in zip(self.master_start, self.master_goal))
        error = max(abs(a-b) for target, feedback in ((candidate, measured), (master_target, master))
                    for a, b in zip(target, feedback))
        if fraction >= 1. and self.settling_started is None:
            self.settling_started = now
        follower_axis = max(range(7), key=lambda j: abs(measured[j]))
        master_axis = max(range(7), key=lambda j: abs(master[j]))
        settling_s = 0. if self.settling_started is None else now-self.settling_started
        self.detail.update(trajectory_elapsed_s=elapsed, wall_elapsed_s=now-self.started,
                           settling_elapsed_s=settling_s, stable_s=self.stable,
                           follower_goal_error_rad=abs(measured[follower_axis]),
                           follower_worst_axis=f"J{follower_axis+1}",
                           leader_goal_error_rad=abs(master[master_axis]),
                           leader_worst_axis=f"J{master_axis+1}",
                           leader_gripper_error_rad=abs(master[7]-self.goal[7]),
                           follower_gripper_error_rad=abs(measured[7]-self.goal[7]),
                           tracking_error_rad=error)
        if ack != 1:
            self.fail("标准回位主臂伺服确认丢失")
        elif error > .20:
            self.fail("标准回位跟踪误差超过 0.20 rad")
        elif fraction < 1. and now-self.started > 2*self.duration+8.:
            self.fail("标准回位轨迹执行超时")
        elif settling_s > 8.:
            self.fail(f"标准回位到位后等待 8 秒仍未稳定：主臂 J{master_axis+1} "
                      f"{abs(master[master_axis]):.3f} rad，从臂 J{follower_axis+1} "
                      f"{abs(measured[follower_axis]):.3f} rad（关节要求 ≤{ZERO_ARRIVAL_TOLERANCE_RAD:.3f} rad）；"
                      f"主/从夹爪误差 {abs(master[7]-self.goal[7]):.3f}/"
                      f"{abs(measured[7]-self.goal[7]):.3f} rad（夹爪要求 ≤0.060 rad）")
        else:
            self.elapsed = elapsed
            setattr(o, self.side, candidate)
            setattr(o, self.target_name, master_target)
            self.phase = "settling" if fraction >= 1. else "moving"
            self.detail.update(message="主从臂正低速回零；从臂夹爪保持，主臂夹爪低速对齐，请勿握住主臂夹爪",
                               progress=fraction, tracking_error_rad=error)
            reached = (fraction >= 1. and max(abs(v) for v in (*measured[:7], *master[:7])) <= ZERO_ARRIVAL_TOLERANCE_RAD
                       and max(abs(master[7]-self.goal[7]), abs(measured[7]-self.goal[7])) <= .06)
            self.stable = self.stable+dt if reached else 0.
            self.detail['stable_s'] = self.stable
            if self.stable >= .5:
                self.handover_allowed = max(abs(a-b) for feedback in (measured, master)
                                           for a, b in zip(candidate, feedback)) <= .06
                self.release(now, "completed")

    def status(self):
        return {"side": self.side, "phase": self.phase, "busy": self.busy, **self.detail}

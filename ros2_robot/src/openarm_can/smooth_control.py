#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM4310电机平滑控制 - 五次多项式轨迹 + 重力补偿
适用于轮式机器人腰部俯仰控制
"""

import openarm_can as oa
import numpy as np
import time
import sys


class SmoothController:
    """
    平滑控制器

    整合：
    1. 五次多项式轨迹规划（保证丝滑）
    2. MIT控制（位置+速度前馈）
    3. 重力补偿（保证精度和安全）
    """

    def __init__(self, arm, kp=5.0, kd=1.0, mass=0.0, length=0.0,
                 max_velocity=5.0, max_acceleration=10.0):
        """
        参数:
            arm: OpenArm实例
            kp: 位置增益（建议3-8）
            kd: 速度增益（建议0.5-2）
            mass: 上体质量 (kg)，0表示不补偿重力
            length: 重心到关节距离 (m)
            max_velocity: 最大速度限制 (rad/s)，建议3-5
            max_acceleration: 最大加速度限制 (rad/s²)，建议5-15
        """
        self.arm = arm
        self.kp = kp
        self.kd = kd
        self.mass = mass
        self.length = length
        self.g = 9.8  # 重力加速度

        # 控制频率
        self.dt = 0.001  # 1ms = 1kHz

        # 安全限制
        self.max_velocity = max_velocity      # 最大速度
        self.max_acceleration = max_acceleration  # 最大加速度
        self.max_torque = 8.0                 # 最大力矩

    def generate_trajectory(self, q_start, q_end, duration):
        """
        生成五次多项式轨迹

        公式：
        τ = t/T ∈ [0,1]
        s(τ) = 10τ³ - 15τ⁴ + 6τ⁵

        返回: (位置数组, 速度数组, 加速度数组)
        """
        t = np.arange(0, duration, self.dt)
        tau = t / duration  # 归一化时间

        # 五次多项式
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / duration
        s_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / (duration**2)

        # 位置、速度、加速度
        q = q_start + (q_end - q_start) * s
        dq = (q_end - q_start) * s_dot
        ddq = (q_end - q_start) * s_ddot

        return q, dq, ddq

    def compute_gravity_torque(self, q):
        """
        计算重力补偿力矩

        公式: G(q) = m × L × g × cos(q)
        """
        if self.mass == 0 or self.length == 0:
            return 0.0

        return self.mass * self.length * self.g * np.cos(q)

    def move_to(self, q_target, duration, q_start=None):
        """
        平滑移动到目标位置

        参数:
            q_target: 目标位置 (rad)
            duration: 运动时间 (s)
            q_start: 起始位置 (rad)，None则自动读取
        """
        # 读取起始位置
        if q_start is None:
            self.arm.refresh_all()
            self.arm.recv_all(300)
            motor = self.arm.get_arm().get_motors()[0]
            q_start = motor.get_position()

        print(f"\n开始运动: {q_start:.3f} → {q_target:.3f} rad ({duration:.1f}s)")

        # 生成轨迹
        q_traj, dq_traj, ddq_traj = self.generate_trajectory(q_start, q_target, duration)

        # 设置状态反馈模式
        self.arm.set_callback_mode_all(oa.CallbackMode.STATE)

        # 跟踪轨迹
        for i, (q_ref, dq_ref, ddq_ref) in enumerate(zip(q_traj, dq_traj, ddq_traj)):
            # 重力补偿
            # tau_gravity = self.compute_gravity_torque(q_ref)
            tau_gravity = 0.0

            # 安全限制：速度和加速度
            dq_ref = np.clip(dq_ref, -self.max_velocity, self.max_velocity)
            ddq_ref = np.clip(ddq_ref, -self.max_acceleration, self.max_acceleration)
            tau_gravity = np.clip(tau_gravity, -self.max_torque, self.max_torque)

            # MIT控制命令
            # tau = Kp*(q_target - q_actual) + Kd*(dq_target - dq_actual) + G(q)
            mit_param = oa.MITParam(
                kp=self.kp,
                kd=self.kd,
                q=q_ref,        # 目标位置
                dq=dq_ref,      # 目标速度（前馈，已限速）
                tau=tau_gravity # 重力补偿（前馈）
            )

            # 发送命令
            self.arm.get_arm().mit_control_all([mit_param])
            self.arm.recv_all(300)

            # 每100ms打印一次状态
            if i % 100 == 0:
                motor = self.arm.get_arm().get_motors()[0]
                q_actual = motor.get_position()
                dq_actual = motor.get_velocity()
                error = q_ref - q_actual
                print(f"  t={i*self.dt:.2f}s: 目标={q_ref:.3f}, 实际={q_actual:.3f}, "
                      f"速度={dq_actual:.3f}, 误差={error:.4f} rad")

            time.sleep(self.dt)

        print("运动完成\n")


def main():
    print("=== DM4310平滑控制示例 ===\n")

    try:
        # 1. 初始化CAN接口
        print("初始化CAN接口...")
        arm = oa.OpenArm("can0", False)

        # 2. 配置电机
        motor_types = [oa.MotorType.DM4310]
        send_ids = [0x01]
        recv_ids = [0x11]
        arm.init_arm_motors(motor_types, send_ids, recv_ids)

        # 3. 使能电机
        print("使能电机...")
        arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
        arm.enable_all()
        arm.recv_all(2000)
        print("电机已使能\n")

        # 4. 创建控制器
        # 参数说明：
        # - kp, kd: 需要根据实际负载调试
        # - mass, length: 需要测量，设为0则不补偿重力
        # - max_velocity: 最大速度限制 (rad/s)，越小越慢越安全
        # - max_acceleration: 最大加速度限制 (rad/s²)，越小启动越平缓
        controller = SmoothController(
            arm=arm,
            kp=2.0,               # 位置增益（降低到3，更平缓）
            kd=0.4,               # 速度增益
            mass=0.0,             # 上体质量 (kg)，改为0禁用重力补偿
            length=0.0,           # 重心距离 (m)
            max_velocity=1.0,     # 最大速度 3 rad/s（约170°/s）
            max_acceleration=0.3  # 最大加速度 8 rad/s²
        )
        controller.move_to(q_target=0.0, duration=2.0)

        time.sleep(1)

        # 5. 平滑运动测试
        print("=== 测试1: 移动到90度 ===")
        controller.move_to(q_target=3.14, duration=10.0)

        time.sleep(1)

        # print("=== 测试2: 返回零位 ===")
        # controller.move_to(q_target=0.0, duration=2.0)

        # time.sleep(1)

        # print("=== 测试3: 移动到-45度 ===")
        # controller.move_to(q_target=-0.785, duration=1.5)

        # time.sleep(1)

        print("=== 测试4: 返回零位 ===")
        controller.move_to(q_target=0.0, duration=1.5)

        # # 6. 失能电机
        # print("\n失能电机...")
        # arm.disable_all()
        # arm.recv_all(1000)
        # print("电机已失能")

        print("\n=== 控制完成 ===")

    except KeyboardInterrupt:
        print("\n\n用户中断")
        arm.disable_all()
        arm.recv_all(1000)
    except Exception as e:
        print(f"\n错误: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

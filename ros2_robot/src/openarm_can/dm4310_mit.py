#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
达妙DM4310电机MIT模式控制示例
使用openarm_can库通过CAN总线控制达妙电机
"""

import openarm_can as oa
import time
import sys


def main():
    print("=== 达妙DM4310电机MIT控制示例 ===\n")

    try:
        # 1. 初始化CAN接口
        print("初始化CAN接口 (can0, CAN-FD模式)...")
        arm = oa.OpenArm("can0", False)  # True表示启用CAN-FD

        # 2. 配置电机参数
        # 单个DM4310电机配置
        motor_types = [oa.MotorType.DM4310]
        send_ids = [0x01]  # 电机发送CAN ID
        recv_ids = [0x11]  # 电机接收CAN ID

        print(f"配置电机: DM4310")
        print(f"  发送ID: 0x{send_ids[0]:02X}")
        print(f"  接收ID: 0x{recv_ids[0]:02X}\n")

        arm.init_arm_motors(motor_types, send_ids, recv_ids)

        # 3. 使能电机
        print("使能电机...")
        arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
        arm.enable_all()
        arm.recv_all(2000)  # 等待2ms接收使能响应
        print("电机已使能\n")

        # 4. 切换到状态反馈模式
        arm.set_callback_mode_all(oa.CallbackMode.STATE)

        # 5. MIT模式位置控制 - 回到零位
        print("MIT控制: 回到零位 (kp=2.0, kd=1.0)...")
        mit_param_zero = oa.MITParam(
            kp=20.0,   # 位置增益
            kd=1.0,   # 速度增益
            q=0.0,    # 目标位置 (rad)
            dq=0.0,   # 目标速度 (rad/s)
            tau=0.0   # 前馈力矩 (Nm)
        )
        arm.get_arm().mit_control_all([mit_param_zero])
        arm.recv_all(500)
        time.sleep(2)

        # 读取当前状态
        arm.refresh_all()
        arm.recv_all(300)
        motor = arm.get_arm().get_motors()[0]
        print(f"当前位置: {motor.get_position():.3f} rad\n")

        # 6. MIT模式位置控制 - 移动到目标位置
        target_position = 3.14  # 约90度
        print(f"MIT控制: 移动到 {target_position:.2f} rad...")
        mit_param_move = oa.MITParam(
            kp=50.0,
            kd=1.0,
            q=target_position,
            dq=0.0,
            tau=0.0
        )
        arm.get_arm().mit_control_all([mit_param_move])
        arm.recv_all(500)
        time.sleep(2)

        # 8. 实时监控电机状态 (5秒)
        print("\n实时监控电机状态 (5秒)...")
        print("时间(s) | 位置(rad) | 速度(rad/s) | 力矩(Nm) | MOS温度(°C) | 转子温度(°C)")
        print("-" * 80)

        start_time = time.time()
        while time.time() - start_time < 2.0:
            arm.refresh_all()
            arm.recv_all(300)

            motor = arm.get_arm().get_motors()[0]
            elapsed = time.time() - start_time

            print(f"{elapsed:6.2f}  | {motor.get_position():9.3f} | "
                  f"{motor.get_velocity():11.3f} | {motor.get_torque():8.3f} | "
                  f"{motor.get_state_tmos():11d} | {motor.get_state_trotor():13d}")

            time.sleep(0.1)

        # 9. 回到零位
        print("\n回到零位...")
        arm.get_arm().mit_control_all([mit_param_zero])
        arm.recv_all(500)
        time.sleep(2)

        # 10. 失能电机
        print("\n失能电机...")
        arm.disable_all()
        arm.recv_all(1000)
        print("电机已失能")

        print("\n=== 控制完成 ===")

    except Exception as e:
        print(f"\n错误: {e}", file=sys.stderr)
        print("请确保:")
        print("  1. CAN接口已配置: openarm-can-configure-socketcan can0 -fd")
        print("  2. 电机已正确连接并上电")
        print("  3. CAN ID配置正确")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())


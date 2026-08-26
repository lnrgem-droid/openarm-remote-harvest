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

        # # 7. MIT模式力矩控制
        # print("\nMIT控制: 力矩控制 (0.2 Nm)...")
        # mit_param_torque = oa.MITParam(
        #     kp=0.0,   # 关闭位置控制
        #     kd=0.0,   # 关闭速度控制
        #     q=0.0,
        #     dq=0.0,
        #     tau=0.2   # 施加0.2 Nm力矩
        # )
        # arm.get_arm().mit_control_all([mit_param_torque])
        # arm.recv_all(500)

        # 8. 实时监控电机状态 (5秒)
        print("\n实时监控电机状态 (5秒)...")
        print("时间(s) | 位置(rad) | 速度(rad/s) | 力矩(Nm) | MOS温度(°C) | 转子温度(°C)")
        print("-" * 80)

        start_time = time.time()
        while time.time() - start_time < 5.0:
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


"""
================================================================================
openarm_can 库完整接口参考
================================================================================

一、主类 OpenArm
----------------
class OpenArm(can_interface: str, enable_fd: bool = False)
    初始化OpenArm实例

    参数:
        can_interface: CAN接口名称，如 "can0"
        enable_fd: 是否启用CAN-FD模式，默认False

    方法:
        init_arm_motors(motor_types, send_can_ids, recv_can_ids, control_modes=[])
            初始化机械臂电机
            - motor_types: 电机型号列表 [MotorType, ...]
            - send_can_ids: 发送CAN ID列表 [int, ...]
            - recv_can_ids: 接收CAN ID列表 [int, ...]
            - control_modes: 控制模式列表（可选）[ControlMode, ...]

        init_gripper_motor(motor_type, send_can_id, recv_can_id, control_mode=MIT)
            初始化夹爪电机
            - motor_type: 电机型号
            - send_can_id: 发送CAN ID
            - recv_can_id: 接收CAN ID
            - control_mode: 控制模式，默认MIT

        enable_all()
            使能所有电机

        disable_all()
            失能所有电机

        set_zero_all()
            设置所有电机当前位置为零点

        set_callback_mode_all(callback_mode)
            设置所有电机的回调模式
            - callback_mode: CallbackMode.STATE / PARAM / IGNORE

        refresh_all()
            刷新所有电机状态（发送刷新命令）

        recv_all(first_timeout_us=500)
            接收所有电机的CAN响应
            - first_timeout_us: 超时时间（微秒），默认500us

        query_param_all(rid)
            查询所有电机的参数
            - rid: 寄存器ID (int)

        get_arm() -> ArmComponent
            获取机械臂组件对象

        get_gripper() -> GripperComponent
            获取夹爪组件对象

        get_master_can_device_collection() -> CANDeviceCollection
            获取主CAN设备集合


二、组件类 ArmComponent / GripperComponent
------------------------------------------
    方法:
        get_motors() -> list[Motor]
            获取所有电机对象列表

        mit_control_all(mit_params: list[MITParam])
            MIT模式控制所有电机
            - mit_params: MIT参数列表

        posvel_control_all(posvel_params: list[PosVelParam])
            位置速度模式控制所有电机
            - posvel_params: 位置速度参数列表

        posforce_control_all(posforce_params: list[PosForceParam])
            位置力控模式控制所有电机
            - posforce_params: 位置力控参数列表


三、电机类 Motor
----------------
class Motor(motor_type, send_can_id, recv_can_id)

    状态读取方法:
        get_position() -> float
            获取位置 (rad)

        get_velocity() -> float
            获取速度 (rad/s)

        get_torque() -> float
            获取力矩 (Nm)

        get_state_tmos() -> int
            获取MOS管温度 (°C)

        get_state_trotor() -> int
            获取转子温度 (°C)

        is_enabled() -> bool
            获取使能状态

    属性读取方法:
        get_motor_type() -> MotorType
            获取电机型号

        get_send_can_id() -> int
            获取发送CAN ID

        get_recv_can_id() -> int
            获取接收CAN ID

        get_param(rid: int) -> float
            获取参数值（需先query_param查询）
            - rid: 寄存器ID


四、控制参数结构
----------------
class MITParam(kp, kd, q, dq, tau)
    MIT模式控制参数
    - kp: 位置增益 (float)
    - kd: 速度增益 (float)
    - q: 目标位置 (rad)
    - dq: 目标速度 (rad/s)
    - tau: 前馈力矩 (Nm)

class PosVelParam(q, dq)
    位置速度模式参数
    - q: 目标位置 (rad)
    - dq: 目标速度 (rad/s)

class PosForceParam(q, dq, i)
    位置力控模式参数
    - q: 目标位置 (rad)
    - dq: 速度限制 (rad/s)，打包时乘以100
    - i: 力矩电流限制 (0-1)，打包时乘以10000


五、枚举类型
------------
MotorType (电机型号):
    DM3507, DM4310, DM4310_48V, DM4340, DM4340_48V,
    DM6006, DM8006, DM8009, DM10010L, DM10010,
    DMH3510, DMH6215, DMG6220

ControlMode (控制模式):
    MIT = 1         # MIT模式（位置+速度+力矩混合）
    POS_VEL = 2     # 位置速度模式
    VEL = 3         # 速度模式
    POS_FORCE = 4   # 位置力控模式

CallbackMode (回调模式):
    STATE           # 接收并解析电机状态反馈
    PARAM           # 接收并解析参数查询结果
    IGNORE          # 忽略反馈（用于使能/失能等命令）

RID (寄存器ID - 用于参数查询):
    UV_Value = 0    # 欠压保护值
    KT_Value = 1    # 力矩常数
    OT_Value = 2    # 过温保护值
    OC_Value = 3    # 过流保护值
    ACC = 4         # 加速度
    DEC = 5         # 减速度
    MAX_SPD = 6     # 最大速度
    MST_ID = 7      # 主站ID
    ESC_ID = 8      # 从站ID
    TIMEOUT = 9     # 超时时间
    CTRL_MODE = 10  # 控制模式
    Damp = 11       # 阻尼
    Inertia = 12    # 惯量
    hw_ver = 13     # 硬件版本
    sw_ver = 14     # 软件版本
    SN = 15         # 序列号
    NPP = 16        # 极对数
    Rs = 17         # 定子电阻
    LS = 18         # 定子电感
    Flux = 19       # 磁链
    Gr = 20         # 减速比
    PMAX = 21       # 最大位置
    VMAX = 22       # 最大速度
    TMAX = 23       # 最大力矩
    I_BW = 24       # 电流环带宽
    KP_ASR = 25     # 速度环P增益
    KI_ASR = 26     # 速度环I增益
    KP_APR = 27     # 位置环P增益
    KI_APR = 28     # 位置环I增益
    OV_Value = 29   # 过压保护值
    GREF = 30       # 参考增益
    Deta = 31       # Delta
    V_BW = 32       # 速度环带宽
    IQ_c1 = 33      # IQ校准1
    VL_c1 = 34      # VL校准1
    can_br = 35     # CAN波特率
    sub_ver = 36    # 子版本
    u_off = 50      # U相偏移
    v_off = 51      # V相偏移
    k1 = 52         # 校准系数1
    k2 = 53         # 校准系数2
    m_off = 54      # 机械偏移
    dir = 55        # 方向
    p_m = 80        # 位置模式
    xout = 81       # 输出


六、电机限制参数 LimitParam
---------------------------
每种电机型号的限制参数 (pMax, vMax, tMax):

DM3507:      位置±12.5 rad, 速度50 rad/s,  力矩5 Nm
DM4310:      位置±12.5 rad, 速度30 rad/s,  力矩10 Nm
DM4310_48V:  位置±12.5 rad, 速度50 rad/s,  力矩10 Nm
DM4340:      位置±12.5 rad, 速度8 rad/s,   力矩28 Nm
DM4340_48V:  位置±12.5 rad, 速度10 rad/s,  力矩28 Nm
DM6006:      位置±12.5 rad, 速度45 rad/s,  力矩20 Nm
DM8006:      位置±12.5 rad, 速度45 rad/s,  力矩40 Nm
DM8009:      位置±12.5 rad, 速度45 rad/s,  力矩54 Nm
DM10010L:    位置±12.5 rad, 速度25 rad/s,  力矩200 Nm
DM10010:     位置±12.5 rad, 速度20 rad/s,  力矩200 Nm
DMH3510:     位置±12.5 rad, 速度280 rad/s, 力矩1 Nm
DMH6215:     位置±12.5 rad, 速度45 rad/s,  力矩10 Nm
DMG6220:     位置±12.5 rad, 速度45 rad/s,  力矩10 Nm


七、底层CAN接口（高级用户）
---------------------------
CANSocket:
    CAN套接字底层接口

CANDevice:
    CAN设备基类

CANDeviceCollection:
    CAN设备集合管理

CanPacketEncoder:
    CAN数据包编码器（创建控制命令）

CanPacketDecoder:
    CAN数据包解码器（解析电机反馈）

CANPacket:
    CAN数据包结构

CanFrame / CanFdFrame:
    CAN 2.0 / CAN-FD 帧结构


八、使用流程总结
----------------
1. 初始化: OpenArm(interface, enable_fd)
2. 配置电机: init_arm_motors() / init_gripper_motor()
3. 使能: enable_all() + recv_all()
4. 设置模式: set_callback_mode_all(STATE)
5. 控制: mit_control_all() / posvel_control_all() / posforce_control_all()
6. 接收反馈: recv_all()
7. 读取状态: get_motors()[i].get_position/velocity/torque()
8. 失能: disable_all() + recv_all()


九、注意事项
------------
1. 使用前需配置CAN接口:
   openarm-can-configure-socketcan can0 -fd

2. recv_all() 超时时间建议:
   - 使能/失能: 1000-2000 us
   - 控制命令: 300-500 us
   - 参数查询: 2000 us

3. 控制频率建议: ≥ 1ms (1000Hz)

4. Python API目前为实验性质，未来可能变化

5. 需要root权限或将用户加入can组:
   sudo usermod -aG can $USER

================================================================================
"""

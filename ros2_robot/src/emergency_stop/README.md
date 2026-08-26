# Emergency Stop — 底层 CAN 旁路急停节点

## 概述

独立 C++ ROS2 节点：对机械臂、腰部和升降机构通过底层 SocketCAN 旁路发送失能帧；对底盘发布 `chassis/state_cmd=disable`，由 `chassis_control` 持续下发失能控制帧。

### 工作原理

Linux SocketCAN 允许多个进程同时打开同一 CAN 接口。急停节点独立打开 can0~can3，在触发时直接向总线发送失能帧。原控制节点继续运行，但其控制命令对已失能的电机无效。

```
┌──────────────────────────────────────────────────┐
│          emergency_stop_node (独立进程)            │
│                                                  │
│  键盘 [e] ──┐                                    │
│             ├──→ can0 ──→ 右臂失能帧 ×5           │
│  ROS2 服务 ─┤    ├ can1 ──→ 左臂失能帧 ×5           │
│             │    ├ can2 ──→ 腰部失能帧 ×5           │
│             │    ├ can3 ──→ 升降失能帧 ×3           │
│             └──→ chassis/state_cmd ──→ disable     │
└──────────────────────────────────────────────────┘
```

## CAN 接口分配

| CAN 接口 | 子系统 | 电机类型 | 失能帧 CAN ID | 失能帧数据 |
|----------|--------|----------|---------------|-----------|
| can0 | 右臂 7关节 + 夹爪 | 达妙 (MIT) | 0x01~0x08 | `FF FF FF FF FF FF FF FD` |
| can1 | 左臂 7关节 + 夹爪 | 达妙 (MIT) | 0x01~0x08 | `FF FF FF FF FF FF FF FD` |
| can2 | 腰部 | 达妙 DMJ10422P | 0x01 | `FF FF FF FF FF FF FF FD` |
| can3 | 升降 | CANopen (UM电机) | 0x602 | `2B 40 60 00 07 00 00 00` |
| ROS 话题 | 底盘 | `std_msgs/msg/String` | `chassis/state_cmd` | `disable` |

## 特性

- **旁路失能**：机械臂、腰部和升降机构独立发送失能帧
- **底盘失能**：发布 `chassis/state_cmd=disable`，避免与 `chassis_control` 的周期控制帧竞争
- **一次性触发**：每次触发发送一波失能帧，节点持续运行不退出
- **自动恢复**：其他控制节点重启后自动使能电机，急停节点不干预
- **多路触发**：支持键盘 `e` 键、ROS2 服务，以及底盘急停状态联动三种触发方式
- **底盘联动**：订阅 `chassis/status`，当底盘状态进入急停(`estop`)时（上升沿）自动触发一次失能
- **输入可替换**：当前为 termios 键盘监听，后续可替换为 USB-HID 设备

## 使用方法

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select emergency_stop
source install/setup.bash
```

### 方式1：直接运行（键盘可用）

```bash
ros2 run emergency_stop emergency_stop_node
```

按 `e` 键触发急停。

### 方式2：launch 启动

```bash
ros2 launch emergency_stop emergency_stop.launch.py
```

### 方式3：ROS2 服务触发

```bash
ros2 service call /emergency_stop_node/trigger std_srvs/srv/Trigger
```

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `can_interfaces` | `string[]` | `["can0","can1","can2","can3"]` | 机械臂、腰部和升降机构的 CAN 接口列表 |
| `chassis_status_topic` | `string` | `"chassis/status"` | 底盘状态话题，检测到 `estop` 状态时自动触发急停 |
| `chassis_state_cmd_topic` | `string` | `"chassis/state_cmd"` | 底盘状态控制话题；急停时发布 `disable` |

## 文件结构

```
emergency_stop/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── emergency_stop_node.cpp    # 核心节点
├── launch/
│   └── emergency_stop.launch.py
└── config/
```

## 失能帧说明

### 达妙电机 (MIT 模式)

- **失能命令**：`0xFD`，数据为 `[FF FF FF FF FF FF FF FD]`
- **发送策略**：每个电机连续发送 5 次，间隔 2ms
- 来源：`openarm_can` 库的 `CanPacketEncoder::create_disable_command()`

### CANopen 升降电机 (DS402)

- **失能命令**：写 ControlWord (0x6040) = 0x0007 (DISABLE_OPERATION)
- **CAN ID**：`0x600 + node_id` = `0x602`（node_id=2）
- **SDO 帧数据**：`[2B 40 60 00 07 00 00 00]`
- **发送策略**：连续发送 3 次，间隔 10ms
- 来源：`lift_motor_canopen` 的 `LiftMotorDriver::disable()`

### 底盘（ROS 话题）

- **失能命令**：发布 `std_msgs/msg/String` 到 `chassis/state_cmd`，数据为 `disable`
- **发送策略**：`chassis_control` 收到命令后，以自身控制周期持续下发零速度 + 失能控制帧

## 后续扩展

- 替换键盘输入为 USB-HID 设备监听
- 增加 ROS2 话题 `/emergency_stop/status` 发布急停状态
- 将失能帧配置抽取为 YAML 文件，支持动态加载

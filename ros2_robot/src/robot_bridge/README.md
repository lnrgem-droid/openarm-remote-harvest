# robot_bridge

WebSocket + JSON 桥接层：把 OpenArm 双臂 / 升降 / 腰 / 急停的 ROS2 接口暴露给外部算法。

## 架构

```
外部算法 ──WebSocket/JSON──► WsServer ──入队──► BridgeNode
                                              │
                         CommandCache ◄───────┤  arm_pd（100Hz ZOH 重发）
                         pending 队列 ◄───────┤  其余指令（50Hz 执行）
                                              ▼
                                         RosInterface / ProcessManager
                                              ▼
                                           ROS2
```


| 模块                | 职责                            |
| ----------------- | ----------------------------- |
| `bridge_node`     | 组装各模块；WS 只入队，ROS 定时器唯一执行收发    |
| `ws_server`       | 独立线程 asyncio WebSocket，线程安全广播 |
| `protocol`        | 帧编解码与构造（无 ROS 依赖）             |
| `command_cache`   | `arm_pd` 最新值缓存 + TTL          |
| `ros_interface`   | 话题 / 服务 / Action 封装与状态快照      |
| `process_manager` | 按组件拉起底层 launch                |


**线程模型**：WS 线程只解析并写缓存/队列；ROS 线程负责全部 ROS 调用（控制环默认 100Hz、辅助环 50Hz）。

## 依赖

- ROS2：`rclpy`、`std_msgs`、`std_srvs`、`sensor_msgs`、`trajectory_msgs`、`control_msgs`
- Python：`websockets`
- 底层包（startup 时拉起）：`openarm_bringup` / `openarm_gravity_pd_control`、`lift_motor_canopen`、`openarm_waist_control`、`emergency_stop`



## 编译与启动

```bash
cd ~/ros2_arms
colcon build --packages-select robot_bridge
source install/setup.bash

ros2 launch robot_bridge bridge.launch.py

```



### 参数


| 参数               | 默认        | 说明                    |
| ---------------- | --------- | --------------------- |
| `host`           | `0.0.0.0` | WebSocket 监听地址        |
| `port`           | `9000`    | WebSocket 端口          |
| `control_rate`   | `100.0`   | `arm_pd` 重发频率 (Hz)    |
| `state_rate`     | `50.0`    | 状态广播 / 一次性指令执行频率 (Hz) |
| `command_ttl_ms` | `100.0`   | `arm_pd` 命令有效期；超时停止重发 |


```bash
ros2 launch robot_bridge bridge.launch.py port:=9001 command_ttl_ms:=50.0
```

连接：`ws://<host>:9000`，每条消息一个 JSON 对象。

---



## 协议

公共字段：`type`、`seq`（可选）、`time`（秒）。


| type       | 方向  | 用途           |
| ---------- | --- | ------------ |
| `command`  | 外→桥 | 运动控制         |
| `request`  | 外→桥 | 启动 / 急停 / 复位 |
| `response` | 桥→外 | request 受理应答 |
| `state`    | 桥→外 | 周期状态（~50Hz）  |
| `event`    | 桥→外 | 异步事件         |
| `error`    | 桥→外 | 错误           |




### 双臂模式 `arm_mode`


| 值            | 说明                             | 底层 launch                                                                                                                                                               |
| ------------ | ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `trajectory` | 默认；FollowJointTrajectory       | `openarm_bringup`                                                                                                                                                       |
| `gravity_pd` | PD + 重力补偿直驱                    | `openarm_gravity_pd_control`                                                                                                                                            |
| `cartesian`  | MoveIt 笛卡尔：`/move_pose` action | 与 `trajectory` 共享 `openarm_bringup`（进程名同为 `arms`，`_start` 自动去重不重启）；额外拉起 `openarm_bimanual_moveit_config/move_group.launch.py` + `openarm_move_pose/move_pose.launch.py` |


---



## Command

```json
{
    "type": "command",
    "seq": 1,
    "target": "<name>",
    "ttl_ms": 100,
    "data": {}
}
```


| target          | 条件           | data                                                                                                                                           |
| --------------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `arm_pd`        | `gravity_pd` | `left`/`right`: 7 关节；可选 `left_gripper`/`right_gripper` (0~1)。写入缓存，100Hz ZOH 重发，超 TTL 停发                                                        |
| `arm_traj`      | `trajectory` | `left`/`right`: 长度 7；`duration` 秒（默认 2）；可选 `left_velocity`/`right_velocity`                                                                    |
| `gripper`       | `trajectory` | `left`/`right`: 0~~1（映射 0~~0.044 m）                                                                                                            |
| `lift`          | —            | `position` 和/或 `velocity`                                                                                                                      |
| `waist`         | —            | 仅 `position` → 位置指令；含 `torque`/`velocity` → `[pos,vel,torque]`                                                                                 |
| `arm_cartesian` | `cartesian`  | `arm`(`left`/`right`)、`position`{x,y,z}、`orientation`{x,y,z,w}、`vel_scale`、`acc_scale`、`plan_only`、`frame_id`(空=`world`) → `/move_pose` action |


未知 `target` 静默忽略。

**示例 — PD 直驱**

```json
{
  "type": "command", "seq": 10, "target": "arm_pd", "ttl_ms": 100,
  "data": {
    "left": [0,0,0,0,0,0,0],
    "right": [0,0,0,0,0,0,0],
    "left_gripper": 0.5
  }
}
```

**示例 — 轨迹**

```json
{
  "type": "command", "seq": 11, "target": "arm_traj",
  "data": {"left": [0,0.5,0,0,0,0,0], "duration": 2.0}
}
```

**示例 — 笛卡尔位姿（cartesian 模式）**

```json
{
  "type": "command", "seq": 12, "target": "arm_cartesian",
  "data": {
    "arm": "left",
    "position": {"x": 0.15, "y": 0.15, "z": 0.16},
    "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
    "vel_scale": 0.2,
    "acc_scale": 0.2,
    "plan_only": false,
    "frame_id": ""
  }
}
```

桥接层转发到 `/move_pose` action；结果通过 `event: move_pose_result` 异步广播。

---



## Request

```json
{
    "type": "request",
    "id": "req-1",
    "action": "<name>",
    "data": {}
}
```

立即回 `response`（`ok: true, message: "accepted"`），结果走 `event`。

### `startup`

拉起底层组件并等待 ROS 接口 ready。

```json
{
  "type": "request", "id": "s1", "action": "startup",
  "data": {
    "arm_mode": "trajectory",
    "components": ["arms", "lift", "waist", "emergency_stop"],
    "show_terminal": false,
    "ready_timeout_sec": 30.0
  }
}
```

成功 → `event: startup_completed`；超时 → `startup_failed`。ready 后自动 enable lift/waist。

### `emergency_stop`

调用 `/emergency_stop_node/trigger` → `event: emergency_stop`（含 `ok`）。

### `reset`

清空缓存/队列，停掉本桥接拉起的进程，模式回到 `trajectory` → `event: reset_completed`。

---



## State / Event

状态帧（约 50Hz）：

```json
{
  "type": "state", "seq": 42, "time": 1710000000.0,
  "data": {
    "arm_mode": "trajectory",
    "left_arm":  {"position":[], "velocity":[], "effort":[]},
    "right_arm": {"position":[], "velocity":[], "effort":[]},
    "lift":  {"position":0, "velocity":0, "enabled":true, "...":"..."},
    "waist": {"position":0, "velocity":0, "torque":0},
    "ee_poses": {
      "left":  {"position":{"x":0,"y":0,"z":0}, "orientation":{"x":0,"y":0,"z":0,"w":1}},
      "right": {"position":{"x":0,"y":0,"z":0}, "orientation":{"x":0,"y":0,"z":0,"w":1}}
    }
  }
}
```

`ee_poses` 来自 `/ee_poses`（`tf2_msgs/TFMessage`，`openarm_move_pose/ee_pose_publisher` 发布，5 Hz）；`child_frame_id` 为 `openarm_left_hand`/`openarm_right_hand` 映射到 `left`/`right`。


| event                                  | 含义                                                             |
| -------------------------------------- | -------------------------------------------------------------- |
| `startup_completed` / `startup_failed` | 启动结果（含 `launch`、`readiness`）                                   |
| `emergency_stop`                       | 急停调用结果                                                         |
| `reset_completed`                      | 复位完成                                                           |
| `move_pose_result`                     | `/move_pose` action 结果（`arm`、`success`、`error_code`、`message`） |


---



## 典型流程

1. 启动 `robot_bridge`
2. 连接 `ws://host:9000`
3. 发 `startup`，等 `startup_completed`
4. 按 `arm_mode` 发 `command`；订阅 `state`
5. 结束或换模式：`reset` 后再 `startup`

---



## 注意

- `arm_pd` 超时后**不再重发**，下层保持上一指令，避免发散
- 进程日志默认 `/tmp/robot_bridge_launch/<component>.log`；`show_terminal: true` 时尝试图形终端
- 关节点名：`openarm_{left|right}_joint1..7`


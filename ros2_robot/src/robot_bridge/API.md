# robot_bridge 外部接口

WebSocket + JSON 协议，将 OpenArm 双臂、升降、腰部、急停暴露给外部算法。

**连接**：`ws://<host>:9000`（默认 `0.0.0.0:9000`）  
**格式**：每条消息一个 JSON 对象，UTF-8 文本帧。

---

## 1. 消息约定

### 公共字段


| 字段     | 类型     | 说明                    |
| ------ | ------ | --------------------- |
| `type` | string | 帧类型（必填）               |
| `seq`  | int    | 发送方序号（可选）             |
| `time` | float  | Unix 时间戳，秒（桥接下发帧自动填充） |


### 帧类型


| type       | 方向    | 说明                 |
| ---------- | ----- | ------------------ |
| `command`  | 外 → 桥 | 运动控制               |
| `request`  | 外 → 桥 | 生命周期 / 急停 / 复位     |
| `response` | 桥 → 外 | `request` 受理应答（同步） |
| `state`    | 桥 → 外 | 周期状态（~50 Hz）       |
| `event`    | 桥 → 外 | 异步结果               |
| `error`    | 桥 → 外 | 协议或参数错误            |


---



## 2. Command（运动控制）

```json
{
  "type": "command",
  "seq": 1,
  "target": "<name>",
  "ttl_ms": 100,
  "data": {}
}
```


| 字段       | 说明                              |
| -------- | ------------------------------- |
| `target` | 控制目标（见下表）                       |
| `ttl_ms` | 仅 `arm_pd` 有效；超时后停止重发，默认 100 ms |
| `data`   | 目标参数                            |


未知 `target` 静默忽略，无应答。

### target 一览


| target          | 前置条件                  | data 字段                                                                       | 行为                              |
| --------------- | --------------------- | ----------------------------------------------------------------------------- | ------------------------------- |
| `chassis`       | —                     | `vx`/`vy` (m/s)，`wz` (rad/s)                                                   | 50 Hz 发布 `/cmd_vel`；输入中断 0.5 s 后归零 |
| `chassis_state` | —                     | `command`: `enable` / `disable` / `brake`                                      | 发布 `/chassis/state_cmd` |
| `arm_pd`        | `arm_mode=gravity_pd` | `left`/`right`: 7 关节角 (rad)；可选 `left_gripper`/`right_gripper`: 0~1            | 写入缓存，100 Hz ZOH 重发；TTL 到期停发     |
| `arm_traj`      | `arm_mode=trajectory` | `left`/`right`: 7 关节角；`duration` (s，默认 2)；可选 `left_velocity`/`right_velocity` | FollowJointTrajectory 单点轨迹      |
| `gripper`       | `arm_mode=trajectory` | `left`/`right`: 0~~1（映射 0~~0.044 m）                                           | 夹爪动作                            |
| `lift`          | —                     | `position` 和/或 `velocity`                                                     | 升降绝对位置 / 速度                     |
| `waist`         | —                     | 仅 `position` → 位置指令；含 `torque` 或 `velocity` → `[pos, vel, torque]`            | 腰部控制                            |
| `arm_cartesian` | —                     | `left`/`right` pose；可选 `duration`                                             | **未实现**，广播 `not_implemented` 事件 |




### 2.0 `chassis` — 移动底盘

按标准 `geometry_msgs/Twist` 语义发送速度。bridge 会按配置限幅；需要以 50 Hz 连续发送，停止或断线 0.5 秒后自动发布零速度。

```json
{
  "type": "command",
  "seq": 1,
  "target": "chassis",
  "data": {"vx": 0.3, "vy": 0.0, "wz": -0.5}
}
```


### 2.1 `arm_pd` — PD + 重力补偿直驱

**前置**：`startup` 时 `arm_mode` 设为 `gravity_pd`。  
**关节**：`left`/`right` 各 7 个弧度值，顺序 `joint1`…`joint7`。  
**夹爪**：`left_gripper`/`right_gripper` 可选，0 闭合、1 打开，随臂指令一并发送。  
**发送频率**：建议外部 ≥10 Hz 持续发送；桥接以 100 Hz ZOH 重发；`ttl_ms` 超时后停发。

双臂 + 双夹爪（完整字段）：

```json
{
  "type": "command",
  "seq": 10,
  "target": "arm_pd",
  "ttl_ms": 100,
  "data": {
    "left":  [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0],
    "right": [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0],
    "left_gripper": 1.0,
    "right_gripper": 1.0
  }
}
```



延长心跳有效期（低频发送场景，`ttl_ms` 可适当加大）：

```json
{
  "type": "command", "seq": 13, "target": "arm_pd", "ttl_ms": 200,
  "data": {
    "left":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "left_gripper": 0.0,
    "right_gripper": 0.0
  }
}
```



### 2.2 `arm_traj` — 关节轨迹

**前置**：`arm_mode=trajectory`。  
**关节**：`left`/`right` 各 7 个弧度值。  
**时长**：`duration` 到达目标的时间（秒），默认 2.0。  
**速度**：`left_velocity`/`right_velocity` 可选，各 7 个关节速度 (rad/s)。

双臂同步运动：

```json
{
  "type": "command",
  "seq": 20,
  "target": "arm_traj",
  "data": {
    "left":  [0.0, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0],
    "right": [0.0, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0],
    "duration": 3.0
  }
}
```

带关节速度约束：

```json
{
  "type": "command", "seq": 23, "target": "arm_traj",
  "data": {
    "left":  [0.0, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0],
    "right": [0.0, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0],
    "left_velocity":  [0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
    "right_velocity": [0.0, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0],
    "duration": 4.0
  }
}
```



### 2.3 `gripper` — 夹爪

**前置**：`arm_mode=trajectory`（`gravity_pd` 模式下夹爪应通过 `arm_pd` 的 `left_gripper`/`right_gripper` 发送）。  
**取值**：0.0 闭合，1.0 完全打开（底层映射 0~0.044 m）。

双手打开：

```json
{
  "type": "command", "seq": 30, "target": "gripper",
  "data": {"left": 1.0, "right": 1.0}
}
```

双手闭合：

```json
{
  "type": "command", "seq": 31, "target": "gripper",
  "data": {"left": 0.0, "right": 0.0}
}
```



### 2.4 `lift` — 升降

**单位**：`position` 为绝对位置 (mm)，范围 0~~400；~~`velocity` ~~为速度 (mm/s)，范围 0~~120。  
**说明**：`position` 与 `velocity` 可单独或同时发送；速度设置会影响后续位置运动。

绝对位置（移动到 100 mm）：

```json
{
  "type": "command", "seq": 40, "target": "lift",
  "data": {"position": 100.0}
}
```

设置运动速度：

```json
{
  "type": "command", "seq": 41, "target": "lift",
  "data": {"velocity": 20.0}
}
```

先设速度再移动（同帧发送）：

```json
{
  "type": "command", "seq": 42, "target": "lift",
  "data": {"velocity": 30.0, "position": 200.0}
}
```

回到零位：

```json
{
  "type": "command", "seq": 43, "target": "lift",
  "data": {"position": 0.0}
}
```



### 2.5 `waist` — 腰部

**单位**：`position` 弧度；`velocity` rad/s；`torque` N·m。  
**路由规则**：`data` 中含 `torque` 或 `velocity` 任一字段时，走 `[position, velocity, torque]` 复合指令；否则仅 `position` 时走位置指令。

仅位置控制：

```json
{
  "type": "command", "seq": 50, "target": "waist",
  "data": {"position": 0.5}
}
```

位置 + 速度 + 力矩（完整控制）：

```json
{
  "type": "command", "seq": 51, "target": "waist",
  "data": {
    "position": 0.5,
    "velocity": 0.3,
    "torque": 70.0
  }
}
```

回零（复合指令，速度力矩置零）：

```json
{
  "type": "command", "seq": 52, "target": "waist",
  "data": {"position": 0.0, "velocity": 0.0, "torque": 0.0}
}
```



### 2.6 `arm_cartesian` — 笛卡尔（预留）

**状态**：桥接层 IK/servo 未接入，发送后收到 `not_implemented` 事件。  
**pose 格式**：`position` [x, y, z] (m)，`orientation` [qx, qy, qz, qw]。

```json
{
  "type": "command", "seq": 60, "target": "arm_cartesian",
  "data": {
    "left": {
      "position": [0.3, 0.2, 0.4],
      "orientation": [0.0, 0.0, 0.0, 1.0]
    },
    "right": {
      "position": [0.3, -0.2, 0.4],
      "orientation": [0.0, 0.0, 0.0, 1.0]
    },
    "duration": 3.0
  }
}
```

预期下行事件：

```json
{
  "type": "event",
  "event": "not_implemented",
  "data": {"reason": "笛卡尔控制需桥接层接入 IK/servo，当前未实现"}
}
```

---



## 3. Request（一次性操作）

```json
{
  "type": "request",
  "id": "req-1",
  "action": "<name>",
  "data": {}
}
```

桥接**立即**返回 `response`，实际结果通过 `event` 异步通知。

### response（同步应答）

```json
{
  "type": "response",
  "id": "req-1",
  "seq": 1,
  "action": "startup",
  "ok": true,
  "message": "accepted",
  "data": {}
}
```

`ok=false` 时 `message` 含原因；非法 `action` 下发 `error` 帧。

### 3.1 `startup` — 启动底层组件

拉起底层 ROS 组件并等待接口 ready。成功 / 失败均通过 `event` 异步返回。

**轨迹模式（默认，双臂 + 升降 + 腰 + 急停）**

请求：

```json
{
  "type": "request",
  "id": "s1",
  "action": "startup",
  "data": {
    "arm_mode": "trajectory",
    "components": ["arms", "lift", "waist", "emergency_stop"],
    "show_terminal": false,
    "ready_timeout_sec": 30.0
  }
}
```

同步应答：

```json
{
  "type": "response",
  "id": "s1",
  "action": "startup",
  "ok": true,
  "message": "accepted",
  "data": {}
}
```

成功事件：

```json
{
  "type": "event",
  "id": "s1",
  "event": "startup_completed",
  "data": {
    "arm_mode": "trajectory",
    "launch": {
      "arms": {"status": "started", "pid": 12345, "cmd": ["ros2", "launch", "openarm_bringup", "openarm.bimanual.launch.py"]},
      "lift": {"status": "started", "pid": 12346},
      "waist": {"status": "started", "pid": 12347},
      "emergency_stop": {"status": "started", "pid": 12348}
    },
    "readiness": {
      "arms": {"ready": true, "left_trajectory_action": true, "right_trajectory_action": true},
      "lift": {"ready": true},
      "waist": {"ready": true},
      "emergency_stop": {"ready": true},
      "ready": true
    }
  }
}
```

**PD + 重力补偿模式**

```json
{
  "type": "request",
  "id": "s2",
  "action": "startup",
  "data": {
    "arm_mode": "gravity_pd",
    "components": ["arms", "lift", "waist", "emergency_stop"],
    "show_terminal": false,
    "ready_timeout_sec": 30.0
  }
}
```

启动后使用 `command: arm_pd` 控制双臂；`gripper` 应通过 `arm_pd` 的 `left_gripper`/`right_gripper` 发送。

**仅启动部分组件（例：只测升降）**

```json
{
  "type": "request",
  "id": "s3",
  "action": "startup",
  "data": {
    "arm_mode": "trajectory",
    "components": ["lift"],
    "show_terminal": false,
    "ready_timeout_sec": 15.0
  }
}
```

**调试模式（图形终端显示 launch 日志）**

```json
{
  "type": "request",
  "id": "s4",
  "action": "startup",
  "data": {
    "arm_mode": "trajectory",
    "show_terminal": true
  }
}
```

**启动超时**

```json
{
  "type": "event",
  "id": "s1",
  "event": "startup_failed",
  "data": {
    "arm_mode": "trajectory",
    "launch": {},
    "readiness": {"arms": {"ready": false}, "ready": false}
  }
}
```


| 字段                  | 默认           | 说明                                               |
| ------------------- | ------------ | ------------------------------------------------ |
| `arm_mode`          | `trajectory` | 见 §5                                             |
| `components`        | 全部四项         | `arms` / `lift` / `waist` / `emergency_stop`     |
| `show_terminal`     | `false`      | `true` 时尝试图形终端，否则日志写 `/tmp/robot_bridge_launch/` |
| `ready_timeout_sec` | `30.0`       | ready 等待超时                                       |


ready 后自动 enable 升降 / 腰部。

### 3.2 `emergency_stop` — 急停

请求：

```json
{
  "type": "request",
  "id": "e1",
  "action": "emergency_stop",
  "data": {}
}
```

同步应答：

```json
{
  "type": "response",
  "id": "e1",
  "action": "emergency_stop",
  "ok": true,
  "message": "accepted",
  "data": {}
}
```

结果事件（`ok=true` 表示急停服务调用成功）：

```json
{
  "type": "event",
  "id": "e1",
  "event": "emergency_stop",
  "data": {"ok": true}
}
```



### 3.3 `reset` — 复位

清空命令缓存与队列，停止桥接拉起的进程，`arm_mode` 恢复 `trajectory`。

请求：

```json
{
  "type": "request",
  "id": "r1",
  "action": "reset",
  "data": {}
}
```

同步应答：

```json
{
  "type": "response",
  "id": "r1",
  "action": "reset",
  "ok": true,
  "message": "accepted",
  "data": {}
}
```

完成事件：

```json
{
  "type": "event",
  "id": "r1",
  "event": "reset_completed",
  "data": {
    "arm_mode": "trajectory",
    "shutdown": {
      "arms": {"status": "terminated", "pid": 12345},
      "lift": {"status": "terminated", "pid": 12346}
    }
  }
}
```

---



## 4. 下行消息



### state（~50 Hz）

```json
{
  "type": "state",
  "seq": 42,
  "time": 1710000000.0,
  "data": {
    "arm_mode": "trajectory",
    "left_arm":  {"position": [], "velocity": [], "effort": []},
    "right_arm": {"position": [], "velocity": [], "effort": []},
    "chassis": {
      "battery_voltage_v": 48.0,
      "battery_temp1_c": 25.0,
      "battery_temp2_c": 25.5,
      "battery_soc": 80,
      "state_name": "enabled",
      "vx": 0.0, "vy": 0.0, "wz": 0.0
    },
    "lift": {
      "position": 0.0, "velocity": 0.0, "enabled": true,
      "error": false, "error_code": 0, "error_message": "", "status": ""
    },
    "waist": {"position": 0.0, "velocity": 0.0, "torque": 0.0}
  }
}
```

`lift` / `waist` 字段随底层话题到达情况而定，未收到时为空对象。

### event


| event               | 触发     | data 要点                           |
| ------------------- | ------ | --------------------------------- |
| `startup_completed` | 启动成功   | `arm_mode`, `launch`, `readiness` |
| `startup_failed`    | 启动超时   | 同上                                |
| `emergency_stop`    | 急停调用完成 | `ok`                              |
| `reset_completed`   | 复位完成   | `arm_mode`, `shutdown`            |
| `not_implemented`   | 未实现功能  | `reason`                          |


```json
{
  "type": "event",
  "id": "s1",
  "event": "startup_completed",
  "data": { "arm_mode": "trajectory", "launch": {}, "readiness": {} }
}
```



### error

```json
{
  "type": "error",
  "id": "req-1",
  "code": "BAD_TYPE",
  "message": "不支持的帧类型: foo"
}
```


| code            | 含义                   |
| --------------- | -------------------- |
| `BAD_TYPE`      | 非法 `type`            |
| `BAD_ACTION`    | 未知 `action`          |
| `BAD_ARG`       | 参数错误（如非法 `arm_mode`） |
| `SHUTTING_DOWN` | 桥接正在关闭               |


---



## 5. 双臂模式 `arm_mode`


| 值            | 说明                  | 可用 command            |
| ------------ | ------------------- | --------------------- |
| `trajectory` | ros2_control 轨迹（默认） | `arm_traj`, `gripper` |
| `gravity_pd` | PD + 重力补偿直驱         | `arm_pd`              |
| `cartesian`  | 预留，IK 未实现           | 无（等同 `trajectory` 启动） |


关节名：`openarm_{left|right}_joint1` … `joint7`，单位弧度。

---



## 6. 典型流程



### 6.1 轨迹模式完整流程

```
1. 连接 ws://host:9000
2. 发送 startup（arm_mode=trajectory）→ 等 startup_completed
3. 发送 arm_traj 移动双臂
4. 发送 gripper 开合夹爪
5. 发送 lift / waist 控制升降和腰部
6. 持续接收 state 监控状态
7. 必要时 emergency_stop
8. reset → 重新 startup（换模式时先 reset）
```

**时序示例（轨迹模式）**

```json
// ① 启动
{"type": "request", "id": "s1", "action": "startup",
 "data": {"arm_mode": "trajectory", "show_terminal": false}}

// ② 双臂运动 3 秒
{"type": "command", "seq": 1, "target": "arm_traj",
 "data": {
   "left":  [0.0, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0],
   "right": [0.0, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0],
   "duration": 3.0
 }}

// ③ 打开夹爪
{"type": "command", "seq": 2, "target": "gripper", "data": {"left": 1.0, "right": 1.0}}

// ④ 升降到 150 mm
{"type": "command", "seq": 3, "target": "lift", "data": {"velocity": 20.0, "position": 150.0}}

// ⑤ 腰部转到 0.5 rad
{"type": "command", "seq": 4, "target": "waist", "data": {"position": 0.5}}

// ⑥ 复位
{"type": "request", "id": "r1", "action": "reset", "data": {}}
```



### 6.2 PD 直驱模式完整流程

```
1. 连接 → startup（arm_mode=gravity_pd）→ 等 startup_completed
2. 以 ≥10 Hz 持续发送 arm_pd（含 ttl_ms 心跳）
3. 夹爪通过 arm_pd 的 left_gripper/right_gripper 控制
4. reset 结束
```

**时序示例（PD 模式，50 Hz 发送）**

```json
// ① 启动 PD 模式
{"type": "request", "id": "s2", "action": "startup",
 "data": {"arm_mode": "gravity_pd", "show_terminal": false}}

// ② 每 20 ms 重复发送（示例为一帧）
{"type": "command", "seq": 100, "target": "arm_pd", "ttl_ms": 100,
 "data": {
   "left":  [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0],
   "right": [0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0],
   "left_gripper": 0.5,
   "right_gripper": 0.5
 }}

// ③ 停止发送后 ttl_ms 到期，桥接自动停发
// ④ 复位
{"type": "request", "id": "r2", "action": "reset", "data": {}}
```

---



## 7. 注意事项

1. `**arm_pd` 心跳**：外部需以 ≥10 Hz 持续发送并设置合适 `ttl_ms`；超时后桥接停发，下层保持上一指令。
2. `**command` 无应答**：仅 `request` 有同步 `response`；控制效果通过 `state` 观察。
3. **模式匹配**：`arm_pd` 仅在 `gravity_pd` 下生效；`arm_traj`/`gripper` 仅在 `trajectory` 下生效。
4. **并发**：多客户端连接时，广播共享同一状态；`arm_pd` 缓存为全局最新值（后写覆盖）。


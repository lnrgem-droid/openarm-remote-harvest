# 外部接口 → ROS2 映射

> 从 `client/robot_client.py`、`src/robot_bridge/` 梳理。  
> 传输：`WebSocket ws://<host>:9000`（`ros2 launch robot_bridge bridge.launch.py`）。  
> Web 面板经 `web/server.py` 代理到同一桥接地址。

---

## 1. 外部 API 对照表

| 外部入口 | 桥接 `target` / `action` | ROS2 接口 | 类型 |
| --- | --- | --- | --- |
| `connect()` | — | — | 仅建 WS 连接 |
| `startup(arm_mode, components, …)` | `request` → `startup` | 按组件 `ros2 launch`（见 §2） | 进程管理 |
| `reset()` | `request` → `reset` | 停止 launch 进程 | 进程管理 |
| `emergency_stop()` | `request` → `emergency_stop` | `/emergency_stop_node/trigger` | `std_srvs/Trigger` |
| `set_arm_trajectory(...)` | `command` → `arm_traj` | `/left\|right_joint_trajectory_controller/follow_joint_trajectory` | `FollowJointTrajectory` action |
| `set_arm_gravity_pd(...)` + `start_control()` | `command` → `arm_pd`（50Hz 发，桥接 100Hz ZOH） | `/left\|right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` |
| `set_arm_cartesian(...)` | `command` → `arm_cartesian` | **未实现**（回 `not_implemented` 事件） | — |
| `set_gripper(left, right)` | `command` → `gripper` | `/left\|right_gripper_controller/gripper_cmd` | `GripperCommand` action |
| `set_lift_position(mm)` | `command` → `lift` `{position}` | `/motor/move_absolute` | `std_msgs/Float64` |
| `set_lift_velocity(mm/s)` | `command` → `lift` `{velocity}` | `/motor/set_velocity` | `std_msgs/Float64` |
| `set_waist_position(rad)` | `command` → `waist` `{position}` | `/waist_controller/position_command` | `std_msgs/Float64` |
| `set_waist_command(...)` | `command` → `waist` `{position,velocity,torque}` | `/waist_controller/command` | `Float64MultiArray` `[pos,vel,torque]` |
| `get_state()` / `get_*_state()` | 订阅 `type: state` 帧 | `/joint_states`、`/motor/*`、`/waist_controller/state` | 见 §4 |

**双臂关节**：每侧 7 个，`openarm_left_joint1..7` / `openarm_right_joint1..7`，单位 rad。  
**夹爪**：外部 `0.0` 闭合 ~ `1.0` 打开；桥接换算为 `0.0~0.044` m 后发 `GripperCommand` action。重力 PD 与 ros2_control 使用同一夹爪 action 接口。

---

## 2. `startup` 拉起的 ROS2 launch

| `arm_mode` | launch 命令 |
| --- | --- |
| `trajectory`（默认） | `ros2 launch openarm_bringup openarm.bimanual.launch.py` |
| `gravity_pd` | `ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py` |
| `cartesian` | 暂复用 `openarm.bimanual.launch.py` |

| `components` 项 | launch 命令 |
| --- | --- |
| `arms` | 见上 `arm_mode` |
| `lift` | `ros2 launch lift_motor_canopen lift_motor.launch.py` |
| `waist` | `ros2 launch openarm_waist_control waist.launch.py` |
| `emergency_stop` | `ros2 launch emergency_stop emergency_stop.launch.py` |

就绪后桥接自动调用：`/motor/enable`（含 lift 时）、`/waist_controller/enable` `true`（含 waist 时）。

---

## 3. JSON 帧结构

### 3.1 公共字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `type` | string | `command` \| `request` \| `response` \| `state` \| `event` \| `error` |
| `seq` | int | 发送方序号 |
| `time` | float | Unix 时间戳（秒） |

### 3.2 下行：控制命令 `type: command`

```json
{"type":"command","seq":1,"time":1719660000.1,"target":"<target>","data":{...}}
```

| `target` | `data` 字段 | 说明 |
| --- | --- | --- |
| `arm_traj` | `left`, `right`: `[7]float`; `duration`: float; 可选 `left_velocity`, `right_velocity` | 双臂轨迹 |
| `arm_pd` | `left`, `right`: `[7]float`; 可选 `left_gripper`, `right_gripper`: float | 重力 PD 模式，手臂走 forward position 话题；夹爪走 `GripperCommand` action；需 `ttl_ms`（默认 100） |
| `arm_cartesian` | `left`, `right`: `{position:[x,y,z], orientation:[qx,qy,qz,qw]}`; 可选 `duration` | 未实现 |
| `gripper` | 可选 `left`, `right`: float `0~1` | 夹爪 |
| `lift` | `position` mm `0~400` 和/或 `velocity` mm/s `0~120` | 升降 |
| `waist` | 仅 `position` 或 `position`+`velocity`+`torque` | 腰部 |

### 3.3 下行：管理请求 `type: request`

```json
{"type":"request","seq":2,"id":"req-2","time":1719660000.2,"action":"<action>","data":{...}}
```

| `action` | `data` 字段 | 说明 |
| --- | --- | --- |
| `startup` | `arm_mode`, `components[]`, `show_terminal`, `ready_timeout_sec` | 立即回 `response`；完成/失败发 `event` |
| `emergency_stop` | `{}` | 触发急停 |
| `reset` | `{}` | 停进程、清缓存 |

### 3.4 上行：应答与事件

**`response`**（匹配 `id`）：

```json
{"type":"response","id":"req-2","seq":2,"action":"startup","ok":true,"message":"accepted","data":{}}
```

**`event`**：

| `event` | `data` 要点 |
| --- | --- |
| `startup_completed` | `arm_mode`, `launch`, `readiness` |
| `startup_failed` | 同上（超时未 ready） |
| `reset_completed` | `arm_mode`, `shutdown` |
| `emergency_stop` | `{ok: bool}` |
| `not_implemented` | 笛卡尔等未接功能 |

**`state`**（50Hz 广播）：

```json
{
  "type": "state",
  "seq": 100,
  "time": 1719660001.0,
  "data": {
    "arm_mode": "trajectory",
    "left_arm":  {"position": [7], "velocity": [7], "effort": [7]},
    "right_arm": {"position": [7], "velocity": [7], "effort": [7]},
    "lift": {
      "position": 100.0, "velocity": 0.0, "enabled": true,
      "error": false, "error_code": 0, "error_message": "", "status": ""
    },
    "waist": {"position": 0.5, "velocity": 0.0, "torque": 0.0}
  }
}
```

| `data` 路径 | ROS2 来源 |
| --- | --- |
| `left_arm` / `right_arm` | `/joint_states`（按关节名含 `left`/`right` 拆分） |
| `lift.*` | `/motor/position`, `/velocity`, `/enabled`, `/error`, `/error_code`, `/error_message`, `/status` |
| `waist.*` | `/waist_controller/state` → `[pos, vel, torque]` |

**`error`**：

```json
{"type":"error","code":"BAD_TYPE","message":"...","id":"req-2","seq":2}
```

---

## 4. 示例（与 `client/example.py` 一致）

```python
robot = RobotClient("ws://127.0.0.1:9000")
robot.connect()
robot.startup(arm_mode="trajectory", show_terminal=False)

robot.set_arm_trajectory(
    left_positions=[0,0.15,0.15,0.15,0.15,0.15,0],
    right_positions=[0,0.15,0.15,0.15,0.15,0.15,0],
    duration=3.0,
)
robot.set_gripper(left=1.0, right=1.0)
robot.set_lift_position(100.0)
robot.set_waist_position(0.5)

print(robot.get_state())
robot.emergency_stop()
robot.reset()
```

对应 WS 命令帧示例：

```json
{"type":"command","seq":3,"time":1719660000.3,"target":"arm_traj","data":{"left":[0,0.15,0.15,0.15,0.15,0.15,0],"right":[0,0.15,0.15,0.15,0.15,0.15,0],"duration":3.0}}
```

---

## 5. 模式差异速查

| 模式 | 臂控制 | 夹爪 |
| --- | --- | --- |
| `trajectory` | `arm_traj` → action | 独立 `gripper` action |
| `gravity_pd` | 客户端 50Hz `arm_pd` → 桥接 100Hz → topic | 写入 `arm_pd.left_gripper` / `right_gripper` |
| `cartesian` | 接口保留，桥接未接 IK | 同 trajectory |

# 平台接口清单

## 1. 双臂控制

### 1.1 启动

```bash
# ros2_control 轨迹控制，默认推荐
ros2 launch openarm_bringup openarm.bimanual.launch.py

# ros2_control 位置控制
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller

# 1 kHz 重力补偿 PD 控制；接口为 `/left_arm/joint_command`、`/right_arm/joint_command`，并发布 `/joint_states`
ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py

# MoveIt 笛卡尔位姿一次启动；不可与 gravity_pd 同时开
# 终端 1: ros2 launch openarm_bimanual_moveit_config demo.launch.py
# 终端 2: ros2 launch openarm_move_pose move_pose.launch.py
# 笛卡尔位姿分步启动
ros2 launch openarm_bringup openarm.bimanual.launch.py
# 终端 1：MoveIt 规划/执行
ros2 launch openarm_bimanual_moveit_config move_group.launch.py
# 终端 2：/move_pose 封装
ros2 launch openarm_move_pose move_pose.launch.py
```

> **二选一**：`openarm_bringup` / MoveIt demo 与 `openarm_gravity_pd_control` 共用 CAN，不可同时启动。两者都会发布 `/joint_states`，不可并行。

### 1.2 接口

#### ros2_control（`openarm_bringup`）

| 用途          | 话题 / 动作 / 服务                                                 | 类型                                                                      | 方向  |
| ----------- | ------------------------------------------------------------ | ----------------------------------------------------------------------- | --- |
| 左臂轨迹 topic  | `/left_joint_trajectory_controller/joint_trajectory`         | `trajectory_msgs/JointTrajectory`；必须包含 7 个左臂关节                          | 订阅  |
| 右臂轨迹 topic  | `/right_joint_trajectory_controller/joint_trajectory`        | `trajectory_msgs/JointTrajectory`；必须包含 7 个右臂关节                          | 订阅  |
| 左臂轨迹 action | `/left_joint_trajectory_controller/follow_joint_trajectory`  | `control_msgs/action/FollowJointTrajectory`；桥接层轨迹控制实际使用这个 action        | 动作  |
| 右臂轨迹 action | `/right_joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory`；桥接层轨迹控制实际使用这个 action        | 动作  |
| 左臂位置控制      | `/left_forward_position_controller/commands`                 | `std_msgs/Float64MultiArray` 长度 7                                       | 订阅  |
| 右臂位置控制      | `/right_forward_position_controller/commands`                | `std_msgs/Float64MultiArray` 长度 7                                       | 订阅  |
| 左夹爪控制       | `/left_gripper_controller/gripper_cmd`                       | `control_msgs/action/GripperCommand`，`0.0` 闭合，`0.044` 打开                 | 动作  |
| 右夹爪控制       | `/right_gripper_controller/gripper_cmd`                      | `control_msgs/action/GripperCommand`，`0.0` 闭合，`0.044` 打开                 | 动作  |
| 双臂状态        | `/joint_states`                                              | `sensor_msgs/JointState`；由 `joint_state_broadcaster` 发布（CAN 真机反馈）         | 发布  |
| 左轨迹控制器状态    | `/left_joint_trajectory_controller/controller_state`         | `control_msgs/JointTrajectoryControllerState`                           | 发布  |
| 右轨迹控制器状态    | `/right_joint_trajectory_controller/controller_state`        | `control_msgs/JointTrajectoryControllerState`                           | 发布  |
| 控制器查询       | `/controller_manager/list_controllers`                       | `controller_manager_msgs/srv/ListControllers`                           | 服务  |
| 控制器切换       | `/controller_manager/switch_controller`                      | `controller_manager_msgs/srv/SwitchController`                          | 服务  |

#### 重力补偿 PD（`openarm_gravity_pd_control`）

| 用途     | 话题 / 动作 / 服务              | 类型                                                                 | 方向  |
| ------ | ------------------------- | ------------------------------------------------------------------ | --- |
| 左臂位置控制 | `/left_arm/joint_command`  | `sensor_msgs/JointState`；`position` 为 8 维：`[j1..j7, gripper]`，关节单位 rad，夹爪 `0=闭/1=开` | 订阅  |
| 右臂位置控制 | `/right_arm/joint_command` | `sensor_msgs/JointState`；`position` 为 8 维：`[j1..j7, gripper]`，关节单位 rad，夹爪 `0=闭/1=开` | 订阅  |
| 双臂状态   | `/joint_states`           | `sensor_msgs/JointState`；由 `openarm_gravity_pd_node` 从 CAN 真机反馈发布，默认 50 Hz；**不依赖** `joint_state_broadcaster` | 发布  |

`/joint_states` 关节名（重力 PD 模式）：

```text
openarm_left_joint1 .. openarm_left_joint7
openarm_left_finger_joint1
openarm_right_joint1 .. openarm_right_joint7
openarm_right_finger_joint1
```

字段：`position` 臂关节为 rad，夹爪为 m（约 0~0.044）；含 `velocity`（rad/s）、`effort`（N·m）。

数据来源：`CAN 电机反馈 → refresh_all/recv_all → get_position() → 合并发布`。与 `openarm_bringup` 的 `/joint_states` **发布者不同、不可同时存在**。

可调参数（`config/control_params.yaml` 或 launch 覆盖）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_joint_states` | `true` | 是否发布 `/joint_states` |
| `joint_states_rate` | `50.0` | 发布频率 [Hz] |

常见上游（向 `/left_arm/joint_command` 发命令）：

| 来源 | 说明 |
|------|------|
| `openarm_vr_ik_teleop/ik_teleop_gravity_pd_node` | VR IK 遥操作，订阅 `/joint_states` 做 Placo 同步 |
| `qnbot_teleoperator/exo_retargeting_node` | 外骨骼重定向 |
| 手动 `ros2 topic pub` | 调试 |

#### MoveIt 笛卡尔（`openarm_move_pose`）

依赖：`move_group.launch.py` + `move_pose.launch.py`。内部转发 `/move_action`。

| 用途 | 话题 / 动作 / 服务 | 类型 | 方向 |
|------|-------------------|------|------|
| 左右臂位姿规划执行 | `/move_pose` | `openarm_move_pose/action/MovePose`；`arm`: `left`/`right` | 动作 |

Goal 字段：`arm`，`pose`（`geometry_msgs/Pose`），`frame_id`（空则 `world`），`vel_scale`/`acc_scale`（≤0 默认 0.2），`plan_only`。  
末端：`left`→`openarm_left_hand`，`right`→`openarm_right_hand`。左臂姿态常用 `(1,0,0,0)`。

### 1.3 关节顺序

轨迹控制器配置了 `allow_partial_joints_goal: false`，所以不能只发 6 个关节，`joint_names` 和 `positions` 必须完整匹配 7 个关节。

左臂顺序：

```text
openarm_left_joint1
openarm_left_joint2
openarm_left_joint3
openarm_left_joint4
openarm_left_joint5
openarm_left_joint6
openarm_left_joint7
```

右臂顺序：

```text
openarm_right_joint1
openarm_right_joint2
openarm_right_joint3
openarm_right_joint4
openarm_right_joint5
openarm_right_joint6
openarm_right_joint7
```

### 1.4 命令示例

```bash
# 左臂轨迹 action：7 个 joint_names + 7 个 positions，少一个会被控制器拒绝
ros2 action send_goal /left_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
'{trajectory: {joint_names: ["openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7"], points: [{positions: [0.0, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}'

# 右臂轨迹 action：7 个 joint_names + 7 个 positions
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
'{trajectory: {joint_names: ["openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3", "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"], points: [{positions: [0.0, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}'

# 左臂轨迹 topic：同样必须完整 7 关节
ros2 topic pub --once /left_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0,0,0,0,0,0,0], time_from_start: {sec: 2}}]}"

# 右臂轨迹 topic：同样必须完整 7 关节
ros2 topic pub --once /right_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0,0,0,0,0,0,0], time_from_start: {sec: 2}}]}"


# 位置控制（openarm_bringup + forward_position_controller）
ros2 topic pub --once /left_forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0,0,0,0,0,0,0]"
ros2 topic pub --once /right_forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0,0,0,0,0,0,0]"

# 位置控制（openarm_gravity_pd_control）；8 维 position，最后一维为夹爪 0=闭/1=开
ros2 topic pub --once /left_arm/joint_command sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}"
ros2 topic pub --once /right_arm/joint_command sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}"


# 左夹爪打开 / 闭合；ros2_control 与重力 PD 通用，position 单位 m
ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.044, max_effort: 0.0}}"
ros2 action send_goal /left_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"

# 右夹爪打开 / 闭合；ros2_control 与重力 PD 通用，position 单位 m
ros2 action send_goal /right_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.044, max_effort: 0.0}}"
ros2 action send_goal /right_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"


# 查看双臂状态（两种控制模式均发布 /joint_states，但发布者不同，不可同时启动）
ros2 topic info /joint_states -v
ros2 topic echo /joint_states

# MoveIt 笛卡尔：左臂 / 右臂（需 move_group + move_pose 已启动）夹爪仍使用轨迹控制方式
ros2 action send_goal /move_pose openarm_move_pose/action/MovePose "{arm: 'left', pose: {position: {x: 0.15, y: 0.15, z: 0.16}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, vel_scale: 0.2, acc_scale: 0.2, plan_only: false}"
ros2 action send_goal /move_pose openarm_move_pose/action/MovePose "{arm: 'right', pose: {position: {x: 0.15, y: -0.15, z: 0.16}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}, vel_scale: 0.2, acc_scale: 0.2, plan_only: false}"
```

---



## 2. 升降控制

### 2.1 启动(自动使能)

```bash
ros2 launch lift_motor_canopen lift_motor.launch.py
```

### 2.2 接口


| 用途                       | 话题 / 服务                | 类型                       | 方向  |
| ------------------------ | ---------------------- | ------------------------ | --- |
| 绝对位置控制，单位 mm，范围 0-400    | `/motor/move_absolute` | `std_msgs/Float64`       | 订阅  |
| 相对位置控制，单位 mm，自动限制到 0-400 | `/motor/move_relative` | `std_msgs/Float64`       | 订阅  |
| 设置速度，单位 mm/s，范围 0-120    | `/motor/set_velocity`  | `std_msgs/Float64`       | 订阅  |
| 使能                       | `/motor/enable`        | `std_srvs/srv/Trigger`   | 服务  |
| 失能                       | `/motor/disable`       | `std_srvs/srv/Trigger`   | 服务  |
| 停止                       | `/motor/stop`          | `std_srvs/srv/Trigger`   | 服务  |
| 设置零点                     | `/motor/set_zero`      | `std_srvs/srv/Trigger`   | 服务  |
| 清除错误                     | `/motor/clear_error`   | `std_srvs/srv/Trigger`   | 服务  |
| 状态反馈                     | `/motor/status`        | `std_msgs/String`        | 发布  |
| 位置反馈，单位 mm               | `/motor/position`      | `std_msgs/Float64`       | 发布  |
| 速度反馈，单位 mm/s             | `/motor/velocity`      | `std_msgs/Float64`       | 发布  |
| 使能状态                     | `/motor/enabled`       | `std_msgs/Bool`          | 发布  |
| 错误状态                     | `/motor/error`         | `std_msgs/Bool`          | 发布  |
| 报警码                      | `/motor/error_code`    | `std_msgs/UInt16`        | 发布  |
| 报警信息                     | `/motor/error_message` | `std_msgs/String`        | 发布  |
| 关节状态                     | `/lift_joint_states`   | `sensor_msgs/JointState` | 发布  |


### 2.3 命令示例

```bash
# 使能
ros2 service call /motor/enable std_srvs/srv/Trigger "{}"
# 绝对位置控制，单位：mm，范围：0-400
ros2 topic pub --once /motor/move_absolute std_msgs/msg/Float64 "data: 100.0"
# 相对位置控制，单位：mm，自动限制到 0-400
ros2 topic pub --once /motor/move_relative std_msgs/msg/Float64 "data: 20.0"
# 设置速度，单位：mm/s，范围：0-120
ros2 topic pub --once /motor/set_velocity std_msgs/msg/Float64 "data: 20.0"
# 停止
ros2 service call /motor/stop std_srvs/srv/Trigger "{}"
# 查看状态
ros2 topic echo /motor/position
# 查看报警码和报警信息，卡死报警为 0x0004 / 电机卡死
ros2 topic echo /motor/error_code
ros2 topic echo /motor/error_message
# 清除报警
ros2 service call /motor/clear_error std_srvs/srv/Trigger "{}"
```



## 3. 腰部俯仰控制



### 3.1 启动(自动使能)

```bash
ros2 launch openarm_waist_control waist.launch.py
```



### 3.2 接口


| 用途             | 话题                                   | 类型                                                | 方向  |
| -------------- | ------------------------------------ | ------------------------------------------------- | --- |
| 位置控制           | `/waist_controller/position_command` | `std_msgs/Float64`                                | 订阅  |
| 完整 PosForce 控制 | `/waist_controller/command`          | `std_msgs/Float64MultiArray` `[pos, vel, torque]` | 订阅  |
| 使能 / 失能        | `/waist_controller/enable`           | `std_msgs/Bool`                                   | 订阅  |
| 状态反馈           | `/waist_controller/state`            | `std_msgs/Float64MultiArray` `[pos, vel, torque]` | 发布  |




### 3.3 命令示例

```bash
# 位置控制（限位 -0.1 ~ 1.51 rad）
ros2 topic pub --once /waist_controller/position_command std_msgs/msg/Float64 "data: 0.5"
# 位置+速度+输出轴力矩控制
ros2 topic pub --once /waist_controller/command std_msgs/msg/Float64MultiArray "data: [0.5, 0.1, 70.0]"
# 失能
ros2 topic pub --once /waist_controller/enable std_msgs/msg/Bool "data: false"
# 使能
ros2 topic pub --once /waist_controller/enable std_msgs/msg/Bool "data: true"
# 查看状态
ros2 topic echo /waist_controller/state
```

---



## 4. 急停



### 4.1 启动

```bash
ros2 launch emergency_stop emergency_stop.launch.py
```



### 4.2 接口


| 用途     | 话题 / 服务                        | 类型                     | 方向  |
| ------ | ------------------------------ | ---------------------- | --- |
| 触发急停   | `/emergency_stop_node/trigger` | `std_srvs/srv/Trigger` | 服务  |
| 本机键盘急停 | 终端按 `e`                        | -                      | 输入  |




### 4.3 急停范围


| CAN    | 对象           | 说明          |
| ------ | ------------ | ----------- |
| `can0` | 右臂 7 关节 + 夹爪 | 达妙失能帧       |
| `can1` | 左臂 7 关节 + 夹爪 | 达妙失能帧       |
| `can2` | 腰部           | 达妙失能帧       |
| `can3` | 升降           | CANopen 失能帧 |




### 4.4 命令示例

```bash
# 触发急停
ros2 service call /emergency_stop_node/trigger std_srvs/srv/Trigger "{}"
```



## 5. 底盘控制



### 5.1 启动(启动即使能)

```bash
ros2 launch chassis_control chassis_control.launch.py
```

> 启动即下发使能指令（控制帧 `0x050` 的 `byte12=2`），底盘使能后才进入正常速度控制。



### 5.2 接口


| 用途                 | 话题                 | 类型                        | 方向  |
| ------------------ | ------------------ | ------------------------- | --- |
| 速度控制               | `cmd_vel`          | `geometry_msgs/Twist`     | 订阅  |
| 状态控制（使能/失能/刹车）     | `chassis/state_cmd` | `std_msgs/String`         | 订阅  |
| 底盘状态反馈（JSON，50Hz）  | `chassis/status`    | `std_msgs/String`         | 发布  |


`chassis/state_cmd` 取值：`enable` / `disable` / `brake` / `init`。

`chassis/status` JSON 字段：`battery_voltage_v`、`battery_temp1_c`、`battery_temp2_c`、`battery_soc`、`battery_fault`、`chassis_state`、`state_name`、`hub_faults[3]`、`steer_faults[3]`。



### 5.3 命令示例

```bash
# 使能 / 刹车 / 失能
ros2 topic pub --once chassis/state_cmd std_msgs/msg/String "data: enable"
ros2 topic pub --once chassis/state_cmd std_msgs/msg/String "data: brake"
ros2 topic pub --once chassis/state_cmd std_msgs/msg/String "data: disable"

# 速度控制
ros2 topic pub --once cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"

# 查看底盘状态
ros2 topic echo chassis/status
```


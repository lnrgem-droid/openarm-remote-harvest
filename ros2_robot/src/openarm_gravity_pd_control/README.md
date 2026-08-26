# openarm_gravity_pd_control

订阅 `/left_arm/joint_command`、`/right_arm/joint_command`，经 CAN 以重力补偿 + PD（MIT）驱动双臂。**can0=右臂，can1=左臂，默认 500 Hz**；相邻 `joint_command` 按 `command_interp_s` 线性插值。发布 `/joint_states`（CAN 反馈，默认 100 Hz），不依赖 `openarm_bringup`。

**勿与 `openarm_bringup` 同开**（共用 CAN，且都会发 `/joint_states`）。

## 控制

```
τ = Kp·(q_des − q_act) + Kd·(−dq_act) + τ_gravity   # τ_gravity 来自 URDF+KDL，× grav_scale
```

夹爪仅 PD：`motor_rad = gripper × gripper_max_rad`（默认 `-1.0472`；**0=闭，1=开**）。

## 话题

| 方向 | 话题 | 说明 |
|------|------|------|
| 订阅 | `/left_arm/joint_command`、`/right_arm/joint_command` | `JointState.position = [j1..j7(rad), gripper∈[0,1]]` |
| 发布 | `/joint_states` | 左右臂 `openarm_*_joint1..7` + `*_finger_joint1`（夹爪约 0~0.044 m） |

## 编译与启动

```bash
colcon build --packages-up-to openarm_gravity_pd_control && source install/setup.bash

sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py
```

VR IK：本包 → XR（`/xr_pose`）→ `ros2 launch openarm_vr_ik_teleop ik_teleop_gravity_pd.launch.py`

外骨骼：`websocket_teleoperator` → `exo_retargeting` → 本包

测试：

```bash
ros2 topic pub --once /left_arm/joint_command sensor_msgs/msg/JointState \
  "{position: [0,0,0,0,0,0,0,1.0]}"
```

## 参数

`config/control_params.yaml`，或 launch / `-p` 覆盖。

| 参数 | 默认 | 说明 |
|------|------|------|
| `right_arm_can` / `left_arm_can` | `can0` / `can1` | CAN 口 |
| `grav_scale` | `0.95` | 重力缩放；无指令时上漂则减小，下沉则增大 |
| `kp` | `[70,70,60,50,10,10,10]` | 肩/肘可略增，腕宜低 |
| `kd` | `[2,2,1.5,1.5,0.5,0.5,0.4]` | 过大则钝，过小易振 |
| `gripper_kp` / `gripper_kd` | `10` / `1` | 过大易振/堵转 |
| `gripper_max_rad` | `-1.0472` | 张开角（负方向）；不够开则更负 |
| `publish_joint_states` / `joint_states_rate` | `true` / `50` | 反馈发布 |
| `log_interval` | `2.0` | 遥操日志最短间隔 [s] |

## 电机与限位

各臂独立总线，ID 相同：J1–J7 / 夹爪 发 `0x01–0x08`、收 `0x11–0x18`。型号：J1–2 DM8009，J3–4 DM4340，J5–7 与夹爪 DM4310。

软限位见 `openarm_constants.hpp`（J4 下限 **0**，防超伸）。上电以当前电机角为初始目标；无指令时保持姿态。CAN 需 `dialout` 或 sudo。

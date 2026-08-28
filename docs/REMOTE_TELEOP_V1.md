# 双机双臂遥操作 v1 基线

**状态：已冻结，用作三路 RGB-D 回传开发的上游基线。**  
**开发分支：`dev/remote-teleop-v1`**

本版本用于一套 OpenArm 双臂作为操作者主端、另一套双臂安装于 Jetson 作为从端的局域网遥操作。它只处理机械臂与夹爪；三台 Orbbec 相机、图像预览和数据录制不属于本版本。

## 已实现功能

- 主机右/左主臂与 Jetson 右/左从臂一一对应遥操，关节与夹爪均通过 UDP 传输。
- 主机与 Jetson 的 ROS 2 图通过 `ROS_LOCALHOST_ONLY=1` 隔离；跨机器只允许 UDP 动作和状态数据，避免 ROS 话题串机。
- UDP 目标与状态更新 250 Hz；Jetson 本地重力 PD 控制 500 Hz，并以 5 ms 插值平滑目标。
- 两端启动时按右臂、左臂顺序受控移动到**既有编码器** `q=0` 参考，不改写电机零点。
- Jetson 端独立看门狗状态机：`INIT → ALIGNING → READY → RUNNING`，以及 `FAULT/E_STOP`；只有 `RUNNING`、新鲜动作和健康看门狗同时满足时才接受远程目标。
- 运行前检查主从关节差不超过 `0.15 rad` 且持续 1 秒；运行瞬间自动记录主从相对参考，避免对齐残差导致跳变，也防止“显示 RUNNING 但从臂不跟随”。
- `hold/reset/align/run/status/disable` 本地控制命令。
- 主端保留重力补偿；双边反馈使用“从臂实际位置相对已应用动作的跟踪误差”生成有限虚拟反作用力。它不把网络延迟、从臂绝对位置或原始电机力矩直接施加到主臂。
- 双边力反馈默认限幅、低通滤波；空载应尽量保持主臂轻盈，从臂受阻/滞后时才产生反向阻力。实际接触手感仍应在抓取工况中逐步标定。

## 固定硬件与网络映射

| 角色 | 设备 | CAN |
| --- | --- | --- |
| 主端右臂 | x86 主机 | `can0` |
| 主端左臂 | x86 主机 | `can1` |
| 从端右臂 | Jetson | `can1` |
| 从端左臂 | Jetson | `can2` |

默认 Jetson 地址为 `192.168.50.2`，SSH 主机别名为 `openarm-jetson`。如地址或别名改变，可在主机启动前设置：

```bash
export JETSON_HOST=你的_ssh_主机名
export PEER_IP=Jetson_IP
```

动作 UDP 端口为 `50010`，从端状态 UDP 端口为 `50011`。两台机器必须在同一有线局域网，且防火墙不能阻断这两个 UDP 端口和 SSH。

## 工作目录与构建

两端均使用开发副本，不能在原始可用目录内修改：

| 机器 | 目录 |
| --- | --- |
| 主机 | `/home/openarm/dev/openarm-remote-harvest` |
| Jetson | `/home/nvidia/dev/openarm-remote-harvest` |

主机（x86）与 Jetson（ARM）必须各自编译，不能复制 `build_bimanual` 或 `install_bimanual` 目录。每次同步 Python/C++ 控制代码后，在对应机器执行：

```bash
cd /路径/openarm-remote-harvest/ros2_robot
source /opt/ros/humble/setup.bash
source /原始工作区/ros2_robot/install/setup.bash
colcon build --packages-select remote_teleop_runtime \
  --build-base build_bimanual --install-base install_bimanual
```

主机可以附加 `--symlink-install`。Jetson 不使用 x86 构建产物。

## 一键启动

现场必须先确认急停可用、人员到位、两端 CAN 已配置且机械臂周围无碰撞风险。随后只在**主机**执行：

```bash
/home/openarm/dev/openarm-remote-harvest/scripts/run_bimanual_remote_feedback.sh
```

脚本按以下顺序执行：

1. 通过 SSH 启动 Jetson 从端看门狗、500 Hz 从端控制器和 UDP 网关；
2. 从端右、左臂受控回既有 `q=0`；
3. 启动主机主端控制器和 UDP 网关，主端右、左臂受控回既有 `q=0`；
4. 等待 Jetson 收到新鲜主端会话；
5. 请求 `align`，仅在误差与持续时间检查通过后请求 `run`；
6. 保持主机终端前台运行，实时遥操开始。

脚本默认启用双边虚拟力反馈。临时只检查普通遥操可使用：

```bash
FORCE_FEEDBACK=false /home/openarm/dev/openarm-remote-harvest/scripts/run_bimanual_remote_feedback.sh
```

该命令只用于诊断，不能覆盖或删除已冻结的反馈实现。

## 正常停止与异常处理

- 正常停止：在主机脚本终端按 `Ctrl+C`，脚本会向 Jetson 发送 `hold`。确认机械臂稳定后，现场人员托住机械臂，再按需执行 Jetson `disable`。
- 出现异常运动、明显振荡、下坠、碰撞风险或过热：立即使用硬件急停，不要等待软件命令。
- `disable` 不是默认停止方式。断力矩可能造成重力下落，必须在现场支撑机械臂后使用。
- Jetson 死机、断电或 CAN 断开时，软件保持无效；测试和运行均须有人守在从臂旁。

## Jetson 本地控制命令

仅在 Jetson 执行。常用状态查询：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install/setup.bash
source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/setup.bash
ros2 run remote_teleop_runtime remote-teleop-control status
```

可用命令为 `status`、`align`、`run`、`hold`、`reset`、`disable`。一键脚本已自动处理 `align/run`；手动执行时不得绕过现场对齐和托举要求。

## 双边反馈实现说明

官方单机双边控制在同一进程的高频循环内直接交换主、从臂状态。双机系统存在网络传输延迟，直接照搬“从臂位置作为主臂位置目标”或把原始电机力矩直接反射给主臂，会把延迟、重力模型误差放大为沉重、抖动的主臂手感。

本版本的实现位于：

- `ros2_robot/src/remote_teleop_runtime/remote_teleop_runtime/leader.py`
- `ros2_robot/src/remote_teleop_runtime/remote_teleop_runtime/follower.py`
- `ros2_robot/src/openarm_gravity_pd_control/`

Jetson 将已真正应用的动作序号和从臂实际状态返回主机。主机用该动作恢复“理论应达到的从臂位置”，再计算实际位置偏差，形成有限虚拟反作用力。空载且从臂跟随正常时偏差接近零；从臂被阻住或夹爪受物体限制时才产生阻力。力反馈经过滤波和每关节限幅后以独立 ROS 话题送入主端重力 PD 控制器；它不控制主端位置。

## 后续相机阶段约束

三台 Gemini RGB-D 相机只连接 Jetson。后续图像阶段必须：

- 保持本版本的 UDP 控制/状态端口与 ROS 隔离策略不变；
- 由 Jetson 上唯一相机服务进程打开设备并保存 RGB/Depth；
- 主机只接收低分辨率 JPEG RGB 预览，不接管相机；
- 图像使用独立端口/进程，不能和控制 UDP 共用；
- 任何相机性能问题不得降低控制循环优先级或绕过看门狗。

建议从此提交创建相机功能分支，例如 `feat/jetson-rgb-preview`，保持 `dev/remote-teleop-v1` 可随时回退和复测。

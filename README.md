# OpenArm 双臂蘑菇采摘项目

这是当前 OpenArm 程序的**源码基线备份仓库**。它保存了现有双臂控制、LeRobot 数据采集桥接、相机预览和 Web 控制相关代码，作为后续开发远程主从遥操作与三路 RGB-D 数据采集功能的可回退起点。

> 本仓库当前记录的是单机可用程序基线，不代表计划中的“主机 + Jetson”远程系统已经完成。接真机前必须先确认 CAN 映射、急停和本地失效保护。

## 项目目标

- 主端：一套 OpenArm 双臂连接操作者主机，采集左右主臂关节与夹爪动作。
- 从端：另一套 OpenArm 双臂连接 Jetson，执行左对左、右对右的远程主从控制。
- 相机：从端左右腕部 Gemini 305 和胸部 Gemini 335 同步采集 RGB 与深度。
- 数据：Jetson 保存从臂观测、实际执行动作和三路 RGB-D，先形成 OpenArm 原始数据，再转换为 LeRobot Dataset。
- 预览：尽可能将从端 RGB 低延迟传到主端，供操作者遥操作观察；预览链路与原始录制链路相互独立。

## 当前仓库内容

| 路径 | 内容 |
| --- | --- |
| `ros2_robot/` | ROS 2 Humble 工作空间源码，包含 CAN、双臂控制、重力补偿、主从遥操作和 WebSocket bridge |
| `lerobot/` | LeRobot 0.6.2 源码快照 |
| `lerobot_robot_openarm_bridge/` | OpenArm 与 LeRobot 之间的可编辑 Python 插件 |
| `scripts/` | 相机推流和 Orbbec 示例程序 |
| `web/` | robot_bridge 的浏览器控制台 |
| `LeRobot_OpenArm_启动指南.md` | 当前本机的启动与采集操作记录 |
| `docs/BASELINE.md` | 本次备份范围、环境和回退说明 |
| `docs/GITHUB_BACKUP.md` | GitHub 建仓、推送和后续协作命令 |
| `docs/RESTORE_AND_TELEOP.md` | 从全新克隆恢复、编译并启动双臂主从遥操作 |
| `docs/VALIDATION_2026-08-26.md` | 本次 bundle、编译、假硬件和真机验证记录 |

## 当前运行基线

- 操作系统：Ubuntu 22.04，x86_64
- ROS：ROS 2 Humble
- LeRobot 源码版本：0.6.2
- 当前双边遥操作 CAN 默认映射：右从臂 `can0`、左从臂 `can1`、右主臂 `can2`、左主臂 `can3`
- `openarm_bringup`、`openarm_gravity_pd_control` 和直接占用相同 CAN 的遥操作程序不能同时运行
- 2026-08-26 已从 Git bundle 全新恢复并完成双臂真机主从遥操作；左右夹爪均被识别并进入控制线程，实际开合尚未单独记录验证

当前源码的具体编译和启动说明见：

- [ROS 2 工作空间说明](ros2_robot/README.md)
- [LeRobot × OpenArm 启动指南](LeRobot_OpenArm_启动指南.md)
- [ZMQ 相机推流说明](scripts/README.md)

旧文档中仍有 `/home/openarm/vla/...`、`~/ros2_arms` 等历史路径。使用本仓库时应替换为实际克隆路径；此次备份不修改这些原始运行文档，以保证基线内容可追溯。

## 从源码恢复

```bash
git clone <你的仓库地址> openarm-remote-harvest
cd openarm-remote-harvest/ros2_robot

# ROS Humble 使用系统 Python 3.10；不要在 Conda 环境中编译 ROS 工作空间
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash

# rosdep 尚未初始化的电脑只需执行一次：
# sudo rosdep init
# rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 仓库内的 openarm_can 是普通 CMake 包，先单独构建并导出其配置路径
colcon build --packages-select openarm_can
openarm_can_cmake_dir="$PWD/install/openarm_can/lib/cmake/OpenArmCAN"
colcon build --packages-skip openarm_can \
  --cmake-args -DOpenArmCAN_DIR="$openarm_can_cmake_dir"

source install/setup.bash
```

如果系统已经安装 `libopenarm-can-dev` 并能找到 `OpenArmCANConfig.cmake`，也可以直接执行普通的 `colcon build`。上述两步源码构建方式已经在本次恢复验证中通过 11/11 个 ROS 包。

LeRobot 和桥接插件建议安装到单独的 Conda 环境：

```bash
conda create -n openarm-lerobot python=3.12 -y
conda activate openarm-lerobot
pip install -e "./lerobot[core_scripts]"
pip install -e ./lerobot_robot_openarm_bridge
pip install "websockets>=12"
```

恢复后不要直接给机械臂上使能。应先完成依赖检查、CAN 口核对、仿真/无负载测试和急停验证。

真机主从遥操作请严格按 [恢复与主从遥操作手册](docs/RESTORE_AND_TELEOP.md) 执行。本程序启动时会使能四臂并将关节移动到预设初始位置；退出时会执行 `disable_all()`，必须提前支撑机械臂。

## 安全边界

- 网络超时处理不能替代 Jetson 本地看门狗。
- 必须实测控制进程崩溃、Jetson 死机或断电、CAN 适配器断开时电机的行为。
- 急停后的安全状态需要按机械臂负载决定，不能默认一律断力矩，以免机械臂因重力坠落。
- 服务自动重启只用于恢复运行；重启后必须停留在等待对齐状态，不得自行恢复跟随。
- 控制周期以实测可持续频率为准，目标可设为 1 kHz，但不能在测试前把它当成已满足的事实。

## 数据与隐私

Git 默认忽略 ROS 构建产物、虚拟环境、数据集、录像、模型权重、日志、环境变量文件和常见密钥文件。原始采集数据通常很大，也可能包含温室现场画面，不应直接提交到源码仓库。数据应使用独立存储或专门的数据版本管理方案。

## 许可证

本仓库包含多个来源的组件。各组件继续遵循其目录内已有的许可证和版权声明；本次源码备份不额外改变或统一这些许可证。

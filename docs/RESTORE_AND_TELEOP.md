# 从备份恢复并启动双臂主从遥操作

本文档记录已经在 2026-08-26 实际验证通过的恢复和启动顺序。目标映射如下：

| 设备 | CAN |
| --- | --- |
| 右从臂 | `can0` |
| 左从臂 | `can1` |
| 右主臂 | `can2` |
| 左主臂 | `can3` |

## 1. 安全准备

启动程序会立即执行以下动作：

1. 打开四路 SocketCAN。
2. 使能四根机械臂和夹爪。
3. 在约 2.2 秒内将每根机械臂插值移动到关节初始位置 `[0, 0, 0, π/5, 0, 0, 0]`，夹爪移动到零位。
4. 进入双边主从控制循环。

退出程序会调用 `disable_all()`。如果机械臂没有外部支撑，失去力矩后可能下落。因此必须满足：

- 四臂周围无人且没有障碍物。
- 四臂能够安全到达上述初始位置。
- 四臂已可靠支撑，停止后不会坠落。
- 操作者站在设备旁，物理急停在手。
- 没有运行重力补偿、ros2_control、其他遥操作或 CAN 测试程序。

## 2. 克隆与编译

```bash
git clone https://github.com/xuesheng1031/openarm-remote-harvest.git
cd openarm-remote-harvest/ros2_robot

conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash

# 如果 rosdep 尚未初始化，先按系统提示执行 sudo rosdep init 和 rosdep update。
rosdep install --from-paths src --ignore-src -r -y

# openarm_can 是普通 CMake 包，先独立构建。
colcon build --packages-select openarm_can

openarm_can_cmake_dir="$PWD/install/openarm_can/lib/cmake/OpenArmCAN"
colcon build --packages-skip openarm_can \
  --cmake-args -DOpenArmCAN_DIR="$openarm_can_cmake_dir"

source install/setup.bash
```

验收标准：`openarm_can` 构建成功，其余 10 个 ROS 包构建成功；编译器警告可以记录，但不能出现 `Failed <<<`。

## 3. 配置并核对 CAN

仅在接口尚未正确配置或机器重启后执行配置命令。不要在控制程序正在运行时重新配置 CAN。

```bash
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can3 -fd -b 1000000 -d 5000000
```

核对接口：

```bash
ip -brief link show type can
```

四路均应显示 `UP`。再确认没有其他控制程序：

```bash
pgrep -af 'bilateral_control|gravity_pd|gravity_comp|ros2_control_node'
```

在预期没有控制程序的情况下，这条命令应无输出。

## 4. 启动备份程序

在仓库的 `ros2_robot` 目录执行：

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash

WS_DIR="$PWD" bash "$PWD/src/openarm_teleop/script/launch_bimanual_bilateral.sh"
```

必须设置 `WS_DIR="$PWD"`，否则脚本默认寻找 `$HOME/openarm_robot/ros2_robot`，可能误用另一份源码。

正常启动日志应包含：

```text
right: leader=can2 follower=can0
left: leader=can3 follower=can1
Arm motor count: 7
Gripper motor count: 1
both arms running
TeleopRosPublisher ... @ 100 Hz
```

## 5. 运行监控

另开终端并进入相同工作空间：

```bash
cd <仓库路径>/ros2_robot
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash

pgrep -af bilateral_control
ros2 topic list | grep -E 'joint_states|joint_command'
ros2 topic hz /joint_states
```

应看到两个 `bilateral_control` 进程，以及以下数据流：

- `/joint_states`
- `/left_arm/joint_command`
- `/right_arm/joint_command`

依次小幅、缓慢验证右主到右从、左主到左从以及左右夹爪。不要快速甩动，不要接近关节极限。

## 6. 停止

先托住或支撑四臂，然后在启动终端按 `Ctrl+C`。确认：

```bash
pgrep -af bilateral_control
```

没有输出表示两个控制进程已经结束。程序正常退出会关闭电机力矩；不能把 `Ctrl+C` 等同于能承重的机械安全状态。


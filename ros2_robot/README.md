# OpenArm ROS2

双臂 OpenArm 控制系统：CAN 驱动真机、轨迹 / 重力补偿 PD / 笛卡尔位姿、VR 遥操作，以及 WebSocket 桥接与 Web 控制台。

```
外部算法 / Web / client          PICO VR
        │  ws://host:9000           │
        ▼                           ▼
   robot_bridge                 pico_xr → /xr_pose
        │                           │
   ┌────┴────┬──────────┬───────────┴──┐
   ▼         ▼          ▼              ▼
 双臂控制   升降/腰    急停    openarm_vr_ik_teleop (Placo IK)
   │
openarm_can → SocketCAN (can0 右臂 / can1 左臂)
```

**注意**：`openarm_bringup`（含 MoveIt 笛卡尔）与 `openarm_gravity_pd_control` 共用 CAN，不可同时启动。

## 目录


| 路径                               | 说明                               |
| -------------------------------- | -------------------------------- |
| `src/openarm_can`                | CAN / 达妙电机驱动库                    |
| `src/openarm_description`        | URDF / 可视化                       |
| `src/openarm_ros2`               | ros2_control 硬件接口、bringup、MoveIt |
| `src/openarm_gravity_pd_control` | 1 kHz 重力补偿 PD                    |
| `src/openarm_move_pose`          | `/move_pose` 笛卡尔封装 + `/ee_poses` |
| `src/openarm_teleop`             | 主从臂遥操作                           |
| `src/openarm_vr_ik_teleop`       | VR + Placo IK 遥操作                |
| `src/pico_xr`                    | PICO XR 位姿桥接                     |
| `src/qnbot_teleoperator`         | 外骨骼重定向                           |
| `src/emergency_stop`             | CAN 旁路急停                         |
| `src/robot_bridge`               | WebSocket/JSON 对外桥接              |
| `web/`                           | 浏览器控制台                           |
| `client/`                        | Python `RobotClient`             |




## 环境依赖

- Ubuntu 22.04 + ROS 2 Humble（建议 `ros-humble-desktop`）
- SocketCAN、CAN-FD 适配器；真机需配置 `can0`~`can3`

```bash
# OpenArm CAN 工具与库
sudo apt update
sudo apt install -y \
  can-utils iproute2 \
  libeigen3-dev liborocos-kdl-dev \
  liburdfdom-dev liburdfdom-headers-dev libyaml-cpp-dev \
  libopenarm-can-dev openarm-can-utils \
  nlohmann-json3-dev

# ROS 控制 / MoveIt
sudo apt install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-gripper-controllers \
  ros-humble-hardware-interface ros-humble-moveit \
  ros-humble-forward-command-controller

# 工作空间依赖 + Python（桥接 / Web）
cd ~/ros2_arms
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r web/requirements.txt   # websockets 等
```

VR IK 另需 Placo及 PICO PC Service，见 `src/pico_xr`、`src/openarm_vr_ik_teleop` README。

## 编译

```bash
cd ~/ros2_arms
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```



## 设置接口波特率



#### 选项 1：使用快捷指令

```bash
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can3 -fd -b 1000000 -d 5000000
```



#### 选项 2：手动设置

```bash
sudo ip link set can0 down
# configure CAN 2.0 with 1mbps
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

ip link set can0 down
# configure CAN FD with 5mbps
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```



## MoveIt 运行示例

```bash
# 仿真（假硬件，无需 CAN）
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true

# 真机（默认 use_fake_hardware:=false，先配好 can0/can1）
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

笛卡尔封装可另开：`ros2 launch openarm_move_pose move_pose.launch.py`。

## 常用启动

```bash
# 轨迹控制（默认推荐）
ros2 launch openarm_bringup openarm.bimanual.launch.py

# 重力补偿 PD位置控制
ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py

# 关节位置控制
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 robot_controller:=forward_position_controller launch_rviz:=false

# 笛卡尔：先 MoveIt demo / bringup+move_group，再
ros2 launch openarm_move_pose move_pose.launch.py

# 对外桥接（可拉起双臂/升降/腰/急停）
ros2 launch robot_bridge bridge.launch.py

# Web 控制台（需先起 bridge）
python3 -m uvicorn web.server:app --host 0.0.0.0 --port 8000
# → http://127.0.0.1:8000
```



## VR 遥操作

PICO 手柄位姿经 `pico_xr` 发布，`openarm_vr_ik_teleop` 用 Placo IK 解算后驱动双臂。握把按下激活跟随，扳机控夹爪（按下闭合、松开打开）。


| 模式        | 双臂底层                                             | XR 输入                       | IK 启动                            |
| --------- | ------------------------------------------------ | --------------------------- | -------------------------------- |
| 位置控制      | `openarm_bringup`（`forward_position_controller`） | `/vr/*/pose` 等              | `ik_teleop.launch.py`            |
| 重力 PD（推荐） | `openarm_gravity_pd_control`                     | `/xr_pose`（`picoxr talker`） | `ik_teleop_gravity_pd.launch.py` |


```bash
# 重力 PD + VR（推荐）
# 1) PICO PC Service 先开着
ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py
ros2 run picoxr talker
ros2 launch openarm_vr_ik_teleop ik_teleop_gravity_pd.launch.py

# bringup 位置控制 + VR（需另起 XR→/vr/* 桥接）
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  arm_type:=v10 robot_controller:=forward_position_controller
ros2 launch openarm_vr_ik_teleop ik_teleop.launch.py
```

细节见 [src/openarm_vr_ik_teleop/README.md](src/openarm_vr_ik_teleop/README.md)、[src/pico_xr/README.md](src/pico_xr/README.md)。

Python 客户端见 `client/API.md`；示例：

```python
from robot_client import RobotClient
robot = RobotClient("ws://127.0.0.1:9000")
robot.connect()
robot.startup()  # arm_mode: trajectory | gravity_pd | cartesian
```



## 文档


| 文档                                                                       | 内容                 |
| ------------------------------------------------------------------------ | ------------------ |
| [interface_list.md](interface_list.md)                                   | ROS 话题 / 动作 / 服务一览 |
| [src/robot_bridge/README.md](src/robot_bridge/README.md)                 | WebSocket 协议与启动流程  |
| [client/API.md](client/API.md)                                           | Python 客户端接口       |
| [笛卡尔控制接口.md](笛卡尔控制接口.md)                                                 | 笛卡尔位姿细节            |
| [external_interface_mapping.md](external_interface_mapping.md)           | 外部接口与 ROS 映射       |
| [src/openarm_vr_ik_teleop/README.md](src/openarm_vr_ik_teleop/README.md) | VR IK 话题与参数        |
| [src/pico_xr/README.md](src/pico_xr/README.md)                           | PICO XR 桥接         |




## 已知问题

- 笛卡尔目前是单点轨迹控制，连续发下一条目标时手臂可能短暂下沉，后续会优化。
- 双臂精度和软硬度请按自己机台与负载调整，默认参数仅供参考。


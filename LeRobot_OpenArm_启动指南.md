# OpenArm × LeRobot 3.0 启动指南

ROS2 控臂，LeRobot 采数。LeRobot 环境不装 `rclpy`，经 `robot_bridge` WebSocket 通信。


| 组件      | 路径                                               | 版本             |
| ------- | ------------------------------------------------ | -------------- |
| LeRobot | `/home/openarm/vla/lerobot-main`                 | **0.6.1（3.0）** |
| 采集插件    | `/home/openarm/vla/lerobot_robot_openarm_bridge` | pip 可编辑安装      |
| ROS2    | `/home/openarm/vla/ros2_arms`                    | Humble         |


---

## 1. 切到 LeRobot 3.0

```bash
conda activate lerobot

# 若是 0.5.2（指向 ~/lerobot），先卸再装 0.6.1
pip uninstall -y lerobot
pip install -e "/home/openarm/vla/lerobot-main[core_scripts]"

# pip install -e "/home/hahan/vla/lerobot-main[core_scripts]" \
#  -i https://pypi.tuna.tsinghua.edu.cn/simple \
#  --trusted-host pypi.tuna.tsinghua.edu.cn

pip install -e "/home/openarm/vla/lerobot_robot_openarm_bridge"
pip install "websockets>=12"

# 校验
python -c "import lerobot; print(lerobot.__version__, lerobot.__file__)"
# 期望：0.6.1 且路径含 vla/lerobot-main

python -c "
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.robots.config import RobotConfig
print('openarm_bridge' in RobotConfig.get_known_choices())
"
# 期望：True
```

不想动旧环境时：

```bash
conda create -n lerobot30 python=3.12 -y
conda activate lerobot30
# 再执行上面的 pip install 三步
```

---



## 2. 编译 ROS2 bridge

在 `ros2_arms` **目录**编译，不要在 `vla` 根目录跑 `colcon`。

```bash
cd /home/openarm/vla/ros2_arms
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bridge picoxr openarm_vr_ik_teleop openarm_gravity_pd_control openarm_teleop
source install/setup.bash
```

改过相关源码后需重新 build + source。

---



## 3. 启动 ROS2（4 个终端）

每个终端先：

```bash
source /opt/ros/humble/setup.bash
source /home/openarm/vla/ros2_arms/install/setup.bash
```

PICO PC Service 需先开着。

```bash
# 终端 1：重力补偿 PD（与 bringup 互斥，勿同时开）
ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py
```

```bash
# 终端 2：PICO XR → /xr_pose
ros2 run picoxr talker
```

```bash
# 终端 3：VR/IK（仅被动采集；推理时不要开）
ros2 launch openarm_vr_ik_teleop ik_teleop_gravity_pd.launch.py
```

```bash
# 终端 4：桥接，必须 gravity_pd
ros2 launch robot_bridge bridge.launch.py arm_mode:=gravity_pd
```

正常时 bridge 约每 2 秒一条日志，`teleop_valid=True` 表示 VR/IK action 有效。

---



## 3b. 主从臂双边力反馈采集

VR 路径与双边遥操作**二选一**。双边力控占用从臂 CAN，**不要**同时开 `openarm_gravity_pd_control`。

```bash
# 终端 1：左右主从双边力控（发布 /joint_states 与 /{left,right}_arm/joint_command）
# 默认：右从 can0 / 右主 can2；左从 can1 / 左主 can3
bash ~/openarm_robot/ros2_robot/src/openarm_teleop/script/launch_bimanual_bilateral.sh
```

```bash
# 终端 2：桥接，只转发，不拉起 PD
ros2 launch robot_bridge bridge.launch.py arm_mode:=gravity_pd
```

确认 `teleop_valid=True` 且左右 `pos=8` 后，按 §4 跑 `lerobot-record`（`--robot.control_authority=external`）。

推理时关掉 `openarm_teleop`，改开 `openarm_gravity_pd_control` + 同一 `robot_bridge`，LeRobot 用 `control_authority=policy`。

---

---



## 4. 被动采集

```bash
conda activate lerobot

# 新建数据集：目标目录不能已存在
rm -rf /home/openarm/openarm_robot/datasets/openarm_task

lerobot-record \
  --robot.type=openarm_bridge \
  --robot.control_authority=external \
  --robot.cameras='{cam_left: {type: opencv, index_or_path: "/dev/video0", width: 640, height: 480, fps: 30}, cam_right: {type: opencv, index_or_path: "/dev/video2", width: 640, height: 480, fps: 30}, cam_head: {type: opencv, index_or_path: "/dev/video4", width: 640, height: 480, fps: 30}}' \
  --teleop.type=openarm_bridge_teleop \
  --dataset.repo_id=local/openarm_task \
  --dataset.root=/home/openarm/openarm_robot/datasets/openarm_task \
  --dataset.single_task="描述当前任务" \
  --dataset.fps=30 \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=false \
  --dataset.video_encoding_batch_size=10 \
  --dataset.encoder_threads=2 \
  --display_data=true \
  --play_sounds=true
```


| 操作  | 说明                               |
| --- | -------------------------------- |
| 续录  | 加 `--resume=true`，不要 `rm` 目录     |
| 预览  | `--display_data=true` → Rerun 窗口 |
| 语音  | `--play_sounds=true/false`       |
| 键盘  | `n`/→ 下一条，`r`/← 重录，`q`/Esc 退出    |


相机分辨率须与设备一致；不对就查：

```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
# 或
lerobot-find-cameras
```

只测关节、不开相机：

```bash
--robot.cameras='{}'
```

---



## 5. 数据约定

- observation / action：左右各 7 关节（rad）+ 1 夹爪（0 闭 ~ 1 开），共 16 维
- observation 夹爪：ROS `0~0.044 m` → 归一化 `0~1`
- 相机：LeRobot 本机 OpenCV，不经 ROS
- `control_authority=external`：采集时不下发控制（VR/IK 或主从臂控机）
- `control_authority=policy`：推理时策略经 WS 发 `arm_pd`（不起 IK / 不起 openarm_teleop）

---



## 6. 常见问题


| 现象                                   | 处理                                                           |
| ------------------------------------ | ------------------------------------------------------------ |
| `Literal` / `control_authority` 解析失败 | 已用 `str` 修复；确认插件是 editable 最新代码                              |
| `File exists: .../openarm_task`      | 新建则 `rm -rf` 该目录；续录用 `--resume=true`                         |
| `failed to set capture_width`        | 改成相机真实分辨率                                                    |
| `teleop_valid=False`                 | 检查 `/left_arm/joint_command`、`/right_arm/joint_command` 是否在发 |
| bridge 策略不动作                         | 确认 `arm_mode:=gravity_pd`                                    |
| 版本仍是 0.5.2                           | 看 `__file__` 是否还在 `~/lerobot`，按 §1 重装                        |
| `vla/build` 出现                       | 在 `vla` 根目录误跑了 colcon，可删；应在 `ros2_arms` 编译                   |



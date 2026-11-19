# OpenArm 温室蘑菇采集

这个项目用两套 OpenArm 双臂记录人工示范。操作者在笔记本旁移动左右**主臂**，Jetson 控制温室里的左右**从臂**跟随。左臂负责采一朵蘑菇，右臂负责拉出或推回菌棒。每次示范保存为一条独立记录（episode），供回放、检查和训练数据转换使用。

## 从哪个分支开始

日常使用的「启动蘑菇采集控制台」同时依赖**遥操**和**采集**两个分支。要理解当前系统，可以先看桌面入口怎样打开界面，再看界面怎样向 Jetson 请求录制。

| 想了解什么 | 分支 | 建议先看 |
| --- | --- | --- |
| 主从臂怎样启动、对齐、跟随和停止 | `dev/remote-teleop-v1` | [桌面启动脚本](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/dev/remote-teleop-v1/scripts/open_mushroom_collection_terminal.sh) · [遥操运行代码](https://github.com/lnrgem-droid/openarm-remote-harvest/tree/dev/remote-teleop-v1/ros2_robot/src/remote_teleop_runtime) |
| 操作者看见什么，一条示范如何保存 | `feat/jetson-rgbd-preview` | [蘑菇采集界面](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/scripts/mushroom_collection_console.py) · [Jetson 录制管理器](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/scripts/jetson_record_manager.py) |
| 开发采集脚本时参考的原始工程 | `codex/reference-openarm-robot-arms` | [openarm_robot_arms 源码快照](https://github.com/lnrgem-droid/openarm-remote-harvest/tree/codex/reference-openarm-robot-arms) · [该目录附带的双机遥操入口](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/codex/reference-openarm-robot-arms/launch_bimanual_remote.sh) |
| 单机版的环境、编译与恢复 | `main` | [单机恢复手册](docs/RESTORE_AND_TELEOP.md) · [基线验证记录](docs/VALIDATION_2026-08-26.md) |

### 在新电脑上阅读源码

在一个空目录中执行下面两条命令，可以同时打开遥操和采集代码。两个目录名与现场启动器使用的名称一致。这一步只下载源码。

```bash
git clone --branch dev/remote-teleop-v1 https://github.com/lnrgem-droid/openarm-remote-harvest.git openarm-remote-harvest
git clone --branch feat/jetson-rgbd-preview https://github.com/lnrgem-droid/openarm-remote-harvest.git openarm-rgbd-preview
```

现场部署使用 Ubuntu、ROS 2 Humble、CAN 适配器和 Orbbec 相机。启动器引用笔记本上的 `/home/openarm/dev/`、Jetson 上的 `/home/nvidia/dev/` 和 SSH 别名 `openarm-jetson`。迁移设备时，应先核对路径、主从臂 CAN 映射、两端已编译的 ROS 工作空间和相机配置。[双机遥操部署说明](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/dev/remote-teleop-v1/docs/REMOTE_TELEOP_V1.md) · [相机与预览配置](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/docs/RGBD_PREVIEW_V1.md)

## 一条示范怎样录下来

1. **打开控制台。** 在已部署的笔记本上双击「启动蘑菇采集控制台」，选择采集批次。界面显示相机预览、遥操状态和各任务的条数。
2. **开始遥操。** 打开窗口和启动机械臂是两个独立操作。确认运动范围已清空、急停可用后，点击「开始主从遥操」，等待使能、归位和对齐完成。
3. **选任务并示范。** 把对应从臂移到任务起始位置，确认 READY。采蘑菇时还要确认左腕画面的目标框内只有一朵待采蘑菇。点击开始录制，等界面显示「正在录制」后再做动作。
4. **保存结果。** 按实际结果选择「成功并保存」「失败并保存」或「中止本条」。保存一条后，遥操会话可继续用于下一条示范。收工时先支撑机械臂，再按控制台提示单独停止遥操。

| 一条示范包含的动作 | 任务标识 | 批次内的保存目录 | 本条保存的 RGB-D 画面 |
| --- | --- | --- | --- |
| 左臂采一朵蘑菇 | `LEFT_PICK_ONE` | `episodes/left_pick/` | 左腕、胸前 |
| 右臂拉出菌棒 | `RIGHT_PULL_LOG` | `episodes/right_pull/` | 右腕、胸前 |
| 右臂推回菌棒 | `RIGHT_PUSH_LOG` | `episodes/right_push/` | 右腕、胸前 |

三类任务分别编号。例如 `episodes/right_push/episode_0001/` 是该批次第一条推回示范。笔记本可以预览左腕、右腕和胸前三路画面，Jetson 按任务保存表中对应的两路彩色与深度数据。另有独立的「单左臂菌棒采集」模式，用左臂演示拉出和推回，有自己的启动入口与数据目录。[单左臂模式说明](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/docs/LEFT_LOG_COLLECTION_20261114.md)

## 怎样查看或使用数据

- **核对一条记录。** 批次根目录的 `session.json` 给出批次信息。进入某个 `episode_NNNN/` 后，先看 `episode.json` 中的任务、人工结果和有效性，再看 `lerobot/` 下的状态、动作及带时间索引的 RGB-D 原始帧。实际采集数据保存在现场设备或采集硬盘上。
- **回放或转存。** 回放工具用于检查动作对应的彩色画面和深度，归档工具用于将记录转存到采集硬盘。[回放脚本](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/scripts/play_recorded_rgbd.py) · [归档脚本](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/scripts/archive_recordings.py)
- **准备训练数据。** 先在离线环境用转换脚本将选定的示范导出为 OpenArmDataset v0.4，再校验并按需转为 LeRobot v3.0。导出使用 RGB 图像，深度保留在原始记录中，清单保存其引用。[当前转换脚本](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/scripts/convert_recording_to_openarm_dataset.py) · [离线转换步骤](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/feat/jetson-rgbd-preview/docs/OFFICIAL_TRAINING_DATASET.md)

## 单机基线与许可证

`main` 保留了早期单机工程。需要复现当时的环境、构建命令或 CAN 配置时，可查阅[原版基线 README](https://github.com/lnrgem-droid/openarm-remote-harvest/blob/12a66ce1e23105746cda215146734b34e6fa4822/README.md)。仓库内的 OpenArm、LeRobot 和其他组件各自沿用其目录中的许可证与版权声明。

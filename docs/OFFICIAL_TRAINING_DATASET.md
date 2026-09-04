# 官方 OpenArm 训练数据转换

本项目的在线录制优先保障 Jetson 上三路 RGB-D 的持续采集和无损落盘。训练前采用 OpenArm 官方认可的两段离线路径，而不是在录制时直接编码六路图像：

```text
Jetson 原始录制（LeRobot state/action + RGB-D sidecar）
    ↓ scripts/convert_recording_to_openarm_dataset.py
官方 OpenArmDataset v0.4（16 维状态/动作 + 三路 RGB JPEG）
    ↓ 官方 openarm-dataset-validate / openarm-dataset-convert
LeRobot Dataset v3.0（Parquet + 三路 MP4）
```

## 与 OpenArm 官方流程的一致性

OpenArm 官方训练教程要求先把采集结果转换为 LeRobot，并以 `lerobot-train --policy.type=act` 训练。当前官方工具支持 `--format lerobot_v3.0`；OpenArmDataset v0.4 的每个 episode 包含：

- `obs/arms/right/state.parquet`、`obs/arms/left/state.parquet`：实际从臂状态；
- `action/arms/right/state.parquet`、`action/arms/left/state.parquet`：实际下发动作；
- `cameras/<name>/<unix_ns>.jpeg`：按时间戳保存的 RGB 相机帧；
- `metadata.yaml`：硬件、相机、频率、任务与 episode 成败标签。

OpenArm `OpenArm` embodiment 的固定顺序为：`right[关节1..7,夹爪] + left[关节1..7,夹爪]`，即 16 维。我们的输入和官方推理接口的 16 维动作顺序一致。

## 一次录制的转换命令

以下命令在 Jetson 的 `lerobot` 环境执行；源录制目录永不修改，输出目录必须不存在：

```bash
source /home/nvidia/miniconda3/bin/activate lerobot

python /home/nvidia/dev/openarm-rgbd-preview/scripts/convert_recording_to_openarm_dataset.py \
  /home/nvidia/datasets/openarm_rgbd_YYYYMMDD_HHMMSS \
  /home/nvidia/datasets/openarm_official_YYYYMMDD_HHMMSS \
  --task "左臂稳定菌棒，右臂采摘蘑菇"

openarm-dataset-validate /home/nvidia/datasets/openarm_official_YYYYMMDD_HHMMSS

openarm-dataset-convert \
  /home/nvidia/datasets/openarm_official_YYYYMMDD_HHMMSS \
  /home/nvidia/datasets/lerobot_v3_YYYYMMDD_HHMMSS \
  --format lerobot_v3.0 --fps 30 --state qpos
```

转换器默认按每个 30 Hz 机械臂时间步匹配三路最近 RGB 帧，最大允许误差为 100 ms；每次会输出 `depth_sidecar_manifest.json`，包含每路最大/平均匹配误差。超过阈值会拒绝输出，不会留半成品目录。

## RGB、Depth 与训练边界

当前官方 OpenArmDataset v0.4 和官方 LeRobot v3.0 转换器将相机定义为 RGB JPEG/MP4，未定义 RGB-D 深度视觉特征。因此：

- 官方可训练版本使用三路 RGB：`observation.images.wrist_left`、`observation.images.wrist_right`、`observation.images.chest`；
- Depth 继续无损保存在原始 `depth_raw/*.u16le` 与 `*.jsonl` 中，不会丢失；
- 要训练 ACT 等官方 RGB 策略，使用官方转换后的 RGB LeRobot v3.0 数据；
- 要训练真正使用 Depth 的自定义策略，需在官方 RGB 基线验证后，单独增加一个 Depth-aware 数据加载器/模型适配层，不能把 16-bit Depth 伪装为 RGB 视频。

## 当前已验证样本

已在 Jetson 上用真实录制 `openarm_rgbd_20260830_174714` 完成官方转换验证：

- OpenArmDataset 输出：`/home/nvidia/datasets/openarm_official_20260830_174714`
- 官方 LeRobot v3.0 输出：`/home/nvidia/datasets/lerobot_v3_20260830_174714`
- 输出 `808` 个 `30 Hz` 时间步、三路 RGB MP4、16 维 state/action；
- 三路 RGB 最近帧匹配的平均误差约 `8–10 ms`，最大误差约 `55 ms`；
- 原始 RGB-D 录制目录未修改。

对早期录制，LeRobot 机器人时间戳仅为相对时间，因此转换器以首个对齐 RGB-D 帧作为时间原点并在 manifest 中记录该假设。今后正式采蘑菇数据采集前，应新增并验证机器人绝对时间锚点；在此之前，每个转换出的 episode 都必须进行 RGB、深度、关节曲线的人工抽查后才可进入训练集。

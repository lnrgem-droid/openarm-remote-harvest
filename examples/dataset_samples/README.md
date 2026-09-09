# OpenArm 蘑菇采集数据格式示例

本目录只用于代码审查和格式说明，不是训练集。它包含一次真实成功左臂任务和
一次真实成功右臂任务转换后的完整 LeRobot Dataset v3.0 单 episode 示例：

- `left_episode_0001/`：任务 `LEFT_GRASP_LOG`，原始时长约 30.3 秒。
- `right_episode_0001/`：任务 `RIGHT_PICK_ONE`，原始时长约 16.9 秒。

每个示例包含：

```text
<side>_episode_0001/
├── source_episode.json       # 现场采集结果、相机健康、帧数和耗时
├── conversion_report.json    # RGB/机器人时间匹配报告
└── lerobot_v30/              # 官方工具生成的 LeRobot Dataset v3.0
    ├── data/chunk-000/file-000.parquet
    ├── meta/info.json
    ├── meta/stats.json
    ├── meta/tasks.parquet
    ├── meta/episodes/chunk-000/file-000.parquet
    └── videos/
        ├── observation.images.chest/chunk-000/file-000.mp4
        ├── observation.images.wrist_left/chunk-000/file-000.mp4
        └── observation.images.wrist_right/chunk-000/file-000.mp4
```

主要训练特征是16维 `observation.state`、16维 `action` 和三路 RGB。帧率为
30 FPS，状态和动作顺序为左臂7关节+左夹爪、右臂7关节+右夹爪。

Jetson 上的现场原始 episode 还包含以下无损 RGB-D sidecar，但因单条数据约
2–4 GB，没有提交到 GitHub：

```text
lerobot/
├── rgb_raw/
│   ├── left_wrist.rgb24
│   ├── right_wrist.rgb24
│   └── chest.rgb24
└── depth_raw/
    ├── left_wrist.u16le
    ├── left_wrist.jsonl
    ├── right_wrist.u16le
    ├── right_wrist.jsonl
    ├── chest.u16le
    └── chest.jsonl
```

`.u16le` 保存对齐到 RGB 的毫米深度，`.jsonl` 保存每帧序号、设备/主机时间戳、
尺寸、字节偏移和对齐状态。OpenArmDataset v0.4 官方默认训练转换使用 RGB；
Depth 保留在 Jetson 原始层，若训练策略需要 Depth，应另行注入 LeRobot 特征。

不要把本目录的两个 episode 当作可训练数据量。真实训练需要大量成功示范、
统一任务定义、场景变化和完整的数据质量筛选。

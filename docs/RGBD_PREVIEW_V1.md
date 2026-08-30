# Jetson RGB-D 本地录制与主机实时预览 v1

**状态：已在 `feat/jetson-rgbd-preview` 完成联调。**  
**上游遥操作基线：`remote-teleop-v1.0` / `dev/remote-teleop-v1`。**

本版本保持双机双臂遥操作的控制链路不变。三台 Orbbec 只由 Jetson 上的一个相机服务进程打开：优先持续采集并写入 Jetson NVMe，其次才把 RGB 低延迟传给主机。预览退出、断网或卡顿只会丢预览帧，不能阻塞机械臂控制或本地落盘。

## 固定设备与数据流

| 角色 | 设备 | 序列号 | 输出 |
| --- | --- | --- | --- |
| 左腕 | Gemini 305 | `CV2L360000D9` | RGB + 对齐 Depth |
| 右腕 | Gemini 305 | `CV2L360000NR` | RGB + 对齐 Depth |
| 胸部 | Gemini 335L | `CP3294Y0001E` | RGB + 对齐 Depth |

所有流固定为 `640x480@30 FPS`。RGB 为 `uint8 RGB`，Depth 为对齐 RGB 的 `uint16` 毫米值，`0` 表示无效深度。跨相机不做硬件同步；每帧保留设备时间、Jetson 单调时间与序列号供后续对齐。

```text
三台 Orbbec → Jetson 唯一 RGB-D 服务
                    ├─ 无损 RGB/Depth sidecar → Jetson NVMe
                    └─ JPEG 最新帧 TCP 5556 → 主机实时预览窗口
从臂 actual/action → LeRobot 30 Hz Parquet → Jetson NVMe
```

## 启动顺序

先按 [双机遥操 v1](REMOTE_TELEOP_V1.md) 在主机启动遥操作。确认现场人员、急停、CAN 和姿态安全后，在主机执行：

```bash
bash /home/openarm/dev/openarm-remote-harvest/scripts/run_bimanual_remote_feedback.sh
```

Jetson 相机服务和录制管理器通常常驻。若重启后没有运行，在 Jetson 执行：

```bash
cd /home/nvidia/dev/openarm-rgbd-preview
bash scripts/start_jetson_rgbd_service.sh
# 另开一个 Jetson 终端：
bash scripts/start_jetson_record_manager.sh
```

Orbbec 服务停止后，设备需要约 12 秒释放 USB 资源；需要重启时使用：

```bash
bash /home/nvidia/dev/openarm-rgbd-preview/scripts/restart_jetson_rgbd_service.sh
```

在**主机**打开实时预览和录制按钮：

```bash
/home/openarm/miniconda3/bin/python \
  /home/openarm/dev/openarm-rgbd-preview/scripts/rgb_preview_live.py \
  --jetson 192.168.50.2 --port 5556
```

窗口上方显示胸部画面，下方依次为左腕、右腕。它只显示最新 JPEG 帧，不是 Rerun 回放。点击绿色 **START LOCAL RGB-D RECORDING** 才会开始在 Jetson 录制；点击红色 **STOP & SAVE RECORDING** 会安全结束并刷新文件。打开窗口本身不会自动录制，也不会控制 CAN 或机械臂。

## 录制内容和目录

每次录制会在 Jetson 生成一个新目录：

```text
/home/nvidia/datasets/openarm_rgbd_YYYYMMDD_HHMMSS/
├── data/chunk-000/file-000.parquet   # 30 Hz 从臂实际 observation 与 applied action
├── meta/info.json                    # LeRobot 数据集配置和特征
├── meta/stats.json                   # 状态/动作统计值
├── meta/tasks.parquet                # 任务文字标签
├── rgb_raw/
│   ├── left_wrist.rgb24
│   ├── right_wrist.rgb24
│   └── chest.rgb24                   # 三路连续 uint8 RGB 原始帧
└── depth_raw/
    ├── left_wrist.u16le
    ├── right_wrist.u16le
    ├── chest.u16le                   # 三路连续对齐 uint16 mm 深度帧
    └── {left_wrist,right_wrist,chest}.jsonl
                                      # 帧号、设备/主机时间、字节偏移与长度
```

`data/` 与 `meta/` 是标准 LeRobot 机械臂数据。当前视觉数据采取无损原始 sidecar，以优先保证 Jetson 上的三路 `30 FPS` 采集和落盘；后续离线转换器会按 JSONL 时间/序列索引把六路视觉流注入为标准 LeRobot 图像特征。不要删除对应的 JSONL 索引。

## 性能与空间边界

- 空闲时每路相机目标约 `30 FPS`；预览目标 `15 FPS`。
- 录制时预览自动降为约 `5 FPS`，以避免 JPEG 编码与网络抢占本地采集和 NVMe 写入。
- 原始三路 RGB-D 数据量约 `8.1 GB/分钟`；录制开始前小于 `20 GB` 可用空间会被拒绝。
- 相机服务日志：`/home/nvidia/openarm-rgbd-runtime/camera-service.log`。
- 录制管理日志：`/home/nvidia/openarm-rgbd-runtime/record-last.log`。

## 运行边界

- 录制或相机异常不能自动失能机械臂；机械臂仍由遥操作的 `hold` 与硬件急停负责。
- 相机服务是唯一设备打开者；LeRobot、预览程序或其他测试程序不得直接再次打开 Orbbec。
- 不要将 `/home/nvidia/datasets/` 提交到 Git；它可能很大，并包含现场图像。
- 本分支不是对 `dev/remote-teleop-v1` 的替代。仅需复测机械臂时可随时切回冻结遥操作分支。

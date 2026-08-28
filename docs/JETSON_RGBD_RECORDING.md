# Jetson RGB-D 录制与主机预览

本功能位于 `feat/jetson-rgbd-preview`。它不修改双机遥操控制端口，也不会打开 CAN；三台 Orbbec 只由 Jetson 相机服务打开一次。

## 首次配置

在 Jetson 的 `lerobot` Conda 环境执行：

```bash
conda activate lerobot
pip install --upgrade pyorbbecsdk2 pyzmq pyyaml
python -c 'import pyorbbecsdk, zmq; print("SDK ready")'
```

按 Orbbec SDK 的 Linux 环境配置安装 udev 规则。不要在这一步升级相机固件。

识别两台 Gemini 305 的物理角色：

```bash
cd /home/nvidia/dev/openarm-rgbd-preview
python scripts/jetson_orbbec_identify.py --output-dir /tmp/openarm-orbbec-identify
```

查看输出图片，随后在 `config/orbbec_rgbd_jetson.yaml` 填入左腕和右腕对应的序列号。胸部 `CP3294Y0001E` 已固定。序列号未完整填写时服务会拒绝启动。

## 运行

Jetson 终端 1：

```bash
/home/nvidia/dev/openarm-rgbd-preview/scripts/start_jetson_rgbd_service.sh
```

主机终端：

```bash
JETSON_IP=192.168.50.2 /home/openarm/dev/openarm-rgbd-preview/scripts/start_host_rgb_preview.sh
```

主机将打开 Rerun，显示 `left_wrist`、`right_wrist`、`chest` 三路 RGB。预览仅订阅 JPEG；关闭主机或断网不会停止 Jetson 服务。

## LeRobot 录制接入

Jetson 终端 2 启动只读 bridge。它只订阅 `/follower/*`，不启动 CAN 或控制器：

```bash
/home/nvidia/dev/openarm-rgbd-preview/scripts/start_jetson_follower_record_bridge.sh
```

Jetson 终端 3 启动录制。它保留 LeRobot 的 `n/r/q` episode 操作，并以 30 Hz 保存三路 RGB 与三路对齐的 `uint16` 毫米 Depth：

```bash
DATASET_ID=openarm/mushroom-rgbd \
/home/nvidia/dev/openarm-rgbd-preview/scripts/record_jetson_rgbd_dataset.sh
```

录制脚本会自动创建带时间戳的新目录，并在开始前检查 20 GB 的可用空间。视频使用两 episode 后顺序编码，避免一次启动六路编码进程。

## 安全和资源规则

- 相机异常只能停止录制或预览，不能自动失能机械臂。
- 任一路 RGB-D 超过 100 ms 未刷新时，bridge 读取将报错，避免把旧帧写入数据集。
- 预览采用非阻塞、有限队列发送；资源不足时优先丢预览帧。
- 录制前保留至少 20 GB 空间；当前 Jetson NVMe 仍有约 152 GB 可用空间。

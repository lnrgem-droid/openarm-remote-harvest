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

Jetson 上的 `robot_bridge` 必须使用 `config/bridge_follower_recording.yaml`，只订阅 `/follower/*` 话题。录制时 `OpenArmBridge` 传入：

```text
--robot.control_authority=external
--robot.ws_url=ws://127.0.0.1:9000
--robot.rgbd_endpoint=ipc:///tmp/openarm_rgbd_raw.ipc
```

录制器会保存三路 RGB 和三路对齐的 uint16 毫米 Depth。默认后续启动脚本将采用 30 Hz、两 episode 后顺序编码，以避免六路并行编码影响遥操。

## 安全和资源规则

- 相机异常只能停止录制或预览，不能自动失能机械臂。
- 任一路 RGB-D 超过 100 ms 未刷新时，bridge 读取将报错，避免把旧帧写入数据集。
- 预览采用非阻塞、有限队列发送；资源不足时优先丢预览帧。
- 录制前保留至少 20 GB 空间；当前 Jetson NVMe 仍有约 152 GB 可用空间。

# Jetson OpenArm 全量源码备份与交接说明（2026-09-08）

## 备份范围

本次备份用于移交当前 OpenArm 双机双臂、三路 RGB-D 采集和 Jetson 本地数据录制环境。源码同时保存在 GitHub 与离线恢复包中。

GitHub 仓库：`https://github.com/lnrgem-droid/openarm-remote-harvest`

- `dev/remote-teleop-v1`：主机端双机遥操、自动归零/对齐、启动检查与双边反馈；备份提交 `a2a6141`。
- `feat/jetson-rgbd-preview`：三台 Orbbec RGB-D、Jetson 本地录制、主机预览、采集控制台与官方数据转换；备份提交 `7a22ffa`。
- `backup/jetson-arm64-20260908`：Jetson 当前部署的 ARM64 从端控制源码和真机参数；备份提交 `085db29f`。

## Jetson 离线恢复包

Jetson 本地：`/home/nvidia/backups/openarm_jetson_20260908/`

主机副本：`/home/openarm/Backups/openarm_jetson_20260908/`

目录包含源码压缩包、Git bundle、系统与依赖清单、udev 规则、网络/CAN硬件摘要以及 SHA256 校验文件。源码压缩包不包含数据集、运行日志、SSH密钥、Wi-Fi密码、GitHub令牌以及可重新构建的 ROS `build/install/log` 目录。

## 数据集边界

数据集继续独立保存在 Jetson 的 `/home/nvidia/datasets/`，没有上传公共 GitHub。其体积大且包含现场图像，应使用移动硬盘、NAS 或专门的数据仓库另行备份。

当前正式采集批次采用：

```text
<保存根目录>/mushroom_harvest_时间戳/
└── episodes/
    ├── left/episode_NNNN/
    └── right/episode_NNNN/
```

原始记录是 LeRobot 状态/动作加三路无损 RGB-D sidecar；训练前先转换为 OpenArmDataset v0.4，再使用官方工具转换为 LeRobot Dataset v3.0。

## 新 Jetson 恢复顺序

1. 安装与清单一致的 JetPack/Ubuntu、ROS 2 Humble、SocketCAN和Conda依赖。
2. 从 GitHub检出 `backup/jetson-arm64-20260908` 恢复从端控制源码，再检出 `feat/jetson-rgbd-preview` 获取相机与录制源码。
3. 或验证离线包的 `SHA256SUMS` 后解压源码快照，并使用 Git bundle恢复完整历史。
4. 将备份的 `45-kcan.rules` 与 `99-obsensor-libusb.rules` 安装到 `/etc/udev/rules.d/`，执行 `sudo udevadm control --reload-rules && sudo udevadm trigger`。
5. 按 `environment/` 中的清单安装依赖，重新执行 ROS colcon 构建；不要复制旧 ARM 构建缓存到不同系统。
6. 恢复主机与 Jetson 的有线静态地址，验证 `192.168.50.1 ↔ 192.168.50.2`、SSH和 CAN-FD `1M/5M`。
7. 依次进行无硬件测试、CAN枚举、单臂低速测试、双臂低速测试和三相机录制测试。不得直接跳到高速真机运行。

## 安全说明

- `systemd`或进程自动重启不是机械安全机制。
- 从臂控制故障时的目标行为是本地位置保持；Jetson断电时软件保持无效。
- 真机首次恢复必须有人守急停并托举从臂。
- 不要把主机和 Jetson 的 `build/install` 目录互相复制；x86_64 与 ARM64必须分别构建。

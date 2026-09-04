# 完整代码审查索引（2026-09-04）

本分支 `review/complete-code-20260904` 合并了主机双机遥操和 Jetson RGB-D 采集功能，供代码审查使用；它不是未经真机回归就可直接替代现场运行版本的发布标签。

审查范围：

- `ros2_robot/src/remote_teleop_runtime/`：主机/Jetson UDP 遥操网关。
- `ros2_robot/src/remote_teleop_follower_safety/`：Jetson 本地看门狗与状态机。
- `ros2_robot/src/openarm_gravity_pd_control/`：重力补偿、受控归零和启动保持。
- `scripts/jetson_orbbec_rgbd_service.py`：三相机唯一打开者和原始 RGB-D 写入。
- `scripts/jetson_record_manager.py`、`scripts/rgbd_collection_console.py`：会话和 episode 录制。
- `scripts/convert_recording_to_openarm_dataset.py`：离线 OpenArmDataset 转换器。
- `lerobot_robot_openarm_bridge/`：LeRobot 读写桥。

现场部署快照单独保留，避免丢失 Jetson 尚未合并的实际改动：

- `archive/jetson-live-20260904`：Jetson 遥操工作区的本地提交、未提交源码和现场脚本。
- `archive/jetson-rgbd-live-20260904`：Jetson RGB-D 无 Git 部署目录中与功能分支不同的源码文件。

不纳入 Git 的内容：编译产物、安装目录、Conda 环境、相机录制数据、模型权重、日志、设备缓存和密钥。它们不属于可审查源码，且可能包含大文件或现场数据。

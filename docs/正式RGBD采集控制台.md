# 正式 RGB-D 采集控制台

桌面入口仍为“启动主从遥操与 RGB-D 采集界面”。它完成 CAN、网线、Jetson、三相机检查和既有的自动归零/对齐；只有终端明确显示 `RUNNING` 后才允许移动主臂。

打开控制台时只创建一个空采集会话，**不会开始录制**。选择 A 接近、B 夹取拔下、C 放篮或 D 完整采摘任务后，点击“开始本 episode”才会在 Jetson 写入数据。每条结束时必须点击：

- “成功并保存”：操作任务成功；
- “失败并保存”：保留失败数据与失败标签；
- “中止”：仅用于无效尝试或安全中止。

结束后可立即选择下一任务并继续，所有 episode 保存在同一会话内。点击“结束采集会话”完成会话清单。若窗口意外关闭，启动脚本会安全结束当前 episode，但不会停止遥操。

数据目录在 Jetson：`/home/nvidia/datasets/openarm_harvest_sessions/<session>/episodes/<episode>/`。

- `lerobot/`：当前 OpenArmBridge/LeRobot 的从臂 observation 和实际下发 action 暂存数据；
- `lerobot/rgb_raw/` 与 `lerobot/depth_raw/`：三路 RGB 与无损 uint16 深度、逐帧时间戳 sidecar；
- `episode.json`：任务、成功/失败/中止、录制时间、相机健康、帧数、丢帧和可用性标记；
- 会话根目录的 `session.json`：该会话的全部 episode 清单。

`valid=true` 只表示操作员标记成功且三路相机在采集端健康、无 spool 丢帧、每路至少有 30 帧；正式训练前仍必须执行离线 OpenArm 数据验证和 RGB-D 注入转换。

# 正式 RGB-D 采集控制台

桌面入口仍为“启动主从遥操与 RGB-D 采集界面”。它完成 CAN、网线、Jetson、三相机检查和既有的自动归零/对齐；只有终端明确显示 `RUNNING` 后才允许移动主臂。

打开控制台时**不会开始录制**，而是先要求明确选择批次：

- “继续上次批次”：用于窗口闪退或当天继续采集，左右编号分别接着增加；
- “新建批次”：在指定的 Jetson 保存根目录中创建时间戳批次，左右编号都从 1 开始；
- “选择已有批次”：从历史列表恢复指定批次及左右各自的下一编号。

保存根目录必须位于 Jetson 的 `/home/nvidia/datasets` 内，程序不会覆盖已有批次或 episode。选定批次后，点击左臂或右臂的开始按钮才会在 Jetson 写入数据。每条结束时必须点击：

- “成功并保存”：操作任务成功；
- “失败并保存”：保留失败数据与失败标签；
- “中止”：仅用于无效尝试或安全中止。

结束后可立即选择下一任务并继续，所有 episode 保存在同一会话内。点击“结束采集会话”完成会话清单。若窗口意外关闭，启动脚本会安全结束当前 episode，但不会停止遥操。

默认数据目录在 Jetson：`/home/nvidia/datasets/openarm_harvest_sessions/<session>/episodes/<side>/episode_NNNN/`。若新建批次时选择了其他根目录，则 `<session>` 创建在该目录下。

- `lerobot/`：当前 OpenArmBridge/LeRobot 的从臂 observation 和实际下发 action 暂存数据；
- `lerobot/rgb_raw/` 与 `lerobot/depth_raw/`：三路 RGB 与无损 uint16 深度、逐帧时间戳 sidecar；
- `episode.json`：任务、成功/失败/中止、录制时间、相机健康、帧数、丢帧和可用性标记；
- 会话根目录的 `session.json`：该会话的全部 episode 清单。

`valid=true` 只表示操作员标记成功且三路相机在采集端健康、无 spool 丢帧、每路至少有 30 帧；正式训练前仍必须执行离线 OpenArm 数据验证和 RGB-D 注入转换。

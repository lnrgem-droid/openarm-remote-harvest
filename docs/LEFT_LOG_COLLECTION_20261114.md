# 当前启动方式：共同垂落初始位（2026-11-14）

已按用户确认，以左主臂当前自然垂落姿态作为主从共用初始位。启动/恢复后两臂缓慢对齐到保存的位置，通过主从角度检查才进入绝对角度跟随。相对跟随方案没有部署，也未修改电机零点。

录制方式保持不变：无需菌棒编号，勾选 READY 后选择拉出或推回，两类数据分别保存到固定文件夹。

以下历史段落中“各自当前姿态原位保持”已被共同低位对齐替代。

# 单左臂菌棒采集控制台

桌面入口：**单左臂菌棒采集**。原来的“单左臂遥操”终端入口和“启动蘑菇采集控制台”保留。

## 使用

1. 打开新入口，等待胸前、左腕相机就绪。打开窗口只连接采集服务，不启动机械臂。
2. 尚未启动遥操时，点击“开始左臂遥操…”，按对话框确认现场条件。沿用已经安装的单左臂启动器和预检。
3. 固定保存目录会自动准备，不用新建、继续或选择批次。左右任务卡会显示各自保存路径和下一条编号。
4. 手动把左臂调到本阶段 READY，勾选 READY；无需填写菌棒编号或目标说明。
5. 点击“开始录制拉出菌棒”或“开始录制推回菌棒”，等待顶部显示“正在录制”，再做动作。
6. 结束后选“成功并保存”“失败并保存”或“中止本条并保存”。等待封口完成，界面会显示保存结果并更新计数。

保存一条不会停止遥操。关闭界面不会让机械臂失能；要停止机械臂，先可靠支撑左主臂和左从臂，再使用“支撑后失能停止…”。“暂停跟随”请求保持；恢复使用现有启动流程，不能直接清除故障继续跟随。

FAULT、反馈过期、录制服务断连、相机不健康、READY 未确认时不能开始录制。采集中断会保留本条，等待人工标记，不会把中断片段自动算为有效成功。

## 数据

Jetson 固定根目录：`/home/nvidia/datasets/openarm_left_log_sessions/`（不再创建日期批次子目录）

- `episodes/left_pull/episode_NNNN/`：左臂拉出菌棒，任务 `LEFT_PULL_LOG`。
- `episodes/left_push/episode_NNNN/`：左臂推回菌棒，任务 `LEFT_PUSH_LOG`。
- `episode.json`：任务与自动编号、人工结果、有效性、异常原因和相机文件封口回执。
- `lerobot/data/chunk-000/file-000.parquet`：30 Hz 目标频率的实际左臂状态/已下发动作。每个向量为 8 维：J1…J7（rad）+ 夹爪开度比例（0 闭合、1 张开）。没有右臂占位数据。
- `lerobot/meta/info.json`：真实时间原点、采样数、任务、主臂会话号、录制错误。
- `lerobot/rgb_raw/`、`lerobot/depth_raw/`：两路无损 RGB、毫米单位 uint16 深度和逐帧时间索引。

`lerobot` 是沿用旧管理器的目录名；内容是 `openarm_left_rgbd_staging_v1` 中间格式，不是可直接宣称已验证的 LeRobot 训练集。

实际状态来自 ROS `/follower/joint_states`（domain 71），动作来自单左臂网关 `applied_axes[:8]`。录制器没有 CAN 发送、ROS publisher 或使能指令。采样时检查反馈、主臂会话、故障和间隔；少于 30 个机器人采样或录制异常的数据不计为有效成功。

离线转换使用 Jetson 独立环境：

```bash
cd /home/nvidia/openarm-left-collection-20261114
PYTHONNOUSERSITE=1 .venv/bin/python scripts/convert_recording_to_openarm_dataset.py \
  <episode目录>/lerobot <新的输出目录>
```

转换器按源元数据识别单左臂，输出 OpenArm v0.4 布局的左臂与两路 RGB，保留深度源文件引用，检查时间匹配，拒绝覆盖已有输出。训练前仍需完成对应训练工具的格式验证和实际画面/动作同步检查。

## 删除数据后的编号

两个任务各自按当前目录中最大的 `episode_NNNN` 编号加 1；没有条目时从 `episode_0001` 开始。空闲时界面会自动刷新，无需重启。

例如拉出目录只有 `episode_0001` 和 `episode_0002`，在录制已结束并保存后删除这两个完整文件夹，下一条拉出会重新保存为 `episode_0001`。推回目录独立计算，不受影响。如果还留有 `episode_0003`，下一条是 `episode_0004`，不会覆盖或填补前面的空号。

只删除文件夹内部的视频或数据，不等于删除整条：只要 `episode_NNNN` 条目还在，该编号就会保留。正在录制、封口或等待人工结果的数据仍占用当前编号；不要在这些阶段删除。

旧日期批次保留原位置，不自动搬迁或删除；若旧批次有未保存的一条，先完成该条保存，之后自动切换固定目录。

## 部署

- 主机 GUI：`~/.local/share/openarm-left-log/scripts/left_log_collection_console.py`。
- 主机单左臂启动器：从 `~/.local/share/openarm-left-only/desktop.json` 读取现有 release_root。
- Jetson 程序：`/home/nvidia/openarm-left-collection-20261114`。
- 用户服务：`openarm-left-log-recorder.service`、`openarm-left-log-camera.service`。不设置开机自启，由界面连接时启动。
- 状态端口 5597，预览端口 5596；录制标记和运行目录均与原双臂采集隔离。
- 相机保持原来的全局 SDK 单实例锁。旧双臂相机服务占用设备时，不会抢占或杀掉它；请先结束旧采集。
- 相机服务沿用 Jetson 的 `lerobot` 环境；状态录制用新建的 `.venv`，`PYTHONNOUSERSITE=1`，避免 Jetson 用户目录 NumPy 2 与系统 Pandas 的二进制冲突。现有遥操环境未改动。

独立环境安装命令（只安装到新目录）：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --without-pip --system-site-packages \
  /home/nvidia/openarm-left-collection-20261114/.venv
PYTHONNOUSERSITE=1 /home/nvidia/openarm-left-collection-20261114/.venv/bin/python -m pip install \
  numpy==1.26.4 pandas==2.3.3 pyarrow==25.0.0 pyzmq==27.1.0
```

## 本次验证

- 本机旧录制生命周期、任务划分和相机恢复回归：42 项通过。
- Jetson 新录制、真实 Parquet 写入、故障中断、人工结果封口、两相机 8 维转换和原回归：44 项通过。真实子进程测试全部使用合成输入、临时目录，不连接机器人。
- Tk 实际按钮测试：4 项通过，验证 READY/目标门槛、故障阻止录制、取消确认不调用运动、录制期间不能启动/失能。
- 两路真实 RGB-D 约 30 FPS，主机成功收到两路 640×480 预览；只读 ROS 订阅成功读到左臂 8 维状态。
- 界面已截图检查。没有执行实机动作或录制真实示教。
- 开始部署时现有左臂网关报告 FAULT（电机反馈缺失/过期），新界面会阻止录制。启动采集服务后的检查中，从臂原服务 MainPID 7057 保持不变。最终检查时左右两端的单左臂服务均为 inactive，当前没有录制；本次安装没有发出清除故障、使能、失能或遥操重启命令，未判定这次服务停止的原因。

## 2026-11-14 固定目录更新

- 笔记本安装界面和 Jetson 单左臂管理器均已更新。批次按钮已移除，显示固定路径和下一条编号。
- Jetson 的生产服务状态确认 `storage_mode=fixed`、`storage_ready=true`，两类初始编号都是 1。
- Jetson 实际环境完成 13 项测试，包含合成 Parquet 录制、保存、故障中断、旧批次恢复、删除后编号复用；本机实际 Tk 按钮测试 7 项通过。
- 只在录制空闲时重启 `openarm-left-log-recorder.service` 并重新打开笔记本 GUI。相机和遥操进程保持不变；没有发送运动、使能或失能命令。
- Jetson 备份与更新记录：`/home/nvidia/openarm-offline-updates/left-fixed-storage-20261114/`。
- 笔记本界面备份：`~/.local/share/openarm-left-log/backups/fixed-storage-20261114/`。

## 2026-11-14 移除手填目标

笔记本界面不再显示「菌棒编号 / 目标说明」，Jetson 不再要求该字段。准备好后勾选 READY，点击拉出或推回按钮即可开始；等待「正在录制」后做动作，结束点击结果保存。文件夹和编号规则保持不变。旧数据的目标字段保留，新录制的 target 可为空。更新前确认录制空闲，只重启录制服务和界面；7 项界面测试、13 项 Jetson 测试通过（含无目标字段的合成录制）。备份在 Jetson `openarm-offline-updates/left-no-target-20261114/backup` 和笔记本 `~/.local/share/openarm-left-log/backups/no-target-20261114/`。

## 2026-11-14 启动遥操保持自然垂落位置

单左臂采集的“启动遥操”使用原位启动版本：左主臂和左从臂各自保持启动时测得的姿态，不再走到固定的抬臂初始位。主从姿态差过大或反馈不健康时不进入跟随，也不自动移动对齐。先让主从自然垂落、姿态对应并静止，再按界面启动。此次更新不重启当前遥操；正常停止后下次启动生效。

## 2026-11-14 启动自动恢复

重复启动时，正常运行会话直接复用。故障旧会话且16颗电机均已确认失能时，自动清理后重新预检。仍带力矩时，采集界面弹出“支撑后重新启动”；确认两臂已可靠支撑后，程序先失能停止，再按原位启动流程重新启动。通信异常、缺失反馈、异常状态码和版本不一致均不会自动放行。

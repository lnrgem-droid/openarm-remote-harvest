# 左臂采摘与右臂菌棒分段采集（2026-11-12）

状态（2026-11-14）：已按用户请求更新 Jetson 原双臂录制管理器、控制器源码及实际加载的安装文件。已验证三类任务目录和能力标志，尚未执行实机示教。更新时原双臂程序未运行；下次启动原「启动蘑菇采集控制台」将加载新代码。当前运行的独立单左臂采集与遥操程序保持运行，未重启或改动。

## 使用方式

桌面入口仍是 `/home/openarm/桌面/启动蘑菇采集控制台.desktop`。

- 左臂：单朵蘑菇采摘。确认左臂 READY 和左腕目标框中的待采蘑菇，开始录制，结束后单独保存。
- 右臂：在右侧选择「① 拉出菌棒」或「② 推回菌棒」，确认当前阶段 READY，开始录制，结束后单独保存。切换阶段会清除右臂 READY 确认。
- 录制及等待保存期间禁止切换阶段。拉出与推回都可独立录制，不要求固定顺序或先录另一段。
- 右臂两阶段均要求左臂保持、右臂跟随，并等待运动切换完成。新菌棒任务不要求回到旧的右臂采摘起始位；推回可从已拉出的位置开始。
- 「主从一起回位」仍是原有姿态操作，不等于「推回菌棒」录制。推回示范由操作者遥操完成。

| 实际任务 | task | task_group / 目录 | arm_side |
| --- | --- | --- | --- |
| 左臂采蘑菇 | LEFT_PICK_ONE | left_pick | left |
| 右臂拉出菌棒 | RIGHT_PULL_LOG | right_pull | right |
| 右臂推回菌棒 | RIGHT_PUSH_LOG | right_push | right |

每类单独从 `episodes/<task_group>/episode_0001` 编号，分别统计成功、失败、中止。旧 `LEFT_GRASP_LOG` / `RIGHT_PICK_ONE` 的 `left` / `right` 数据保留原标签和目录，不改名、不混入新任务计数。批次继续和中断恢复保留新任务身份。

## 本机修改与待部署文件

主机界面已直接修改：
`/home/openarm/dev/openarm-rgbd-preview/scripts/mushroom_collection_console.py`

Jetson 需要更新以下两份代码，均已在本机准备：

1. `/home/openarm/dev/openarm-rgbd-preview/scripts/jetson_record_manager.py`
   → `/home/nvidia/dev/openarm-rgbd-preview/scripts/jetson_record_manager.py`
2. `/home/openarm/dev/openarm-remote-harvest/ros2_robot/src/remote_teleop_runtime/remote_teleop_runtime/collection_motion.py`
   → `/home/nvidia/dev/openarm-remote-harvest/ros2_robot/src/remote_teleop_runtime/remote_teleop_runtime/collection_motion.py`

待部署包：`/home/openarm/dev/openarm-rgbd-preview/deploy/mushroom-task-split-20261112/`。
包含两仓库的最小补丁、修改前基线、修改后快照及 SHA-256 清单。`host-only/` 是已应用的主机界面快照，无需部署到 Jetson。完整快照包含本次开始前已有的本地改动；远端如有差异，应合并最小补丁，不应直接覆盖。

## 部署复核步骤

1. 使用 `openarm-jetson`（当前配置为 `192.168.50.2`）连接，先读取录制/遥操状态及远端文件。核实没有正在录制、封装或待保存的 episode；保留现有数据和保存位姿。
2. 比较 `manifest.json` 中的基线/结果哈希与远端文件，备份远端当前版本。如果远端有新修改，按补丁合并并检查冲突，保留那些修改。补丁路径相对各自仓库根目录，可先用 `git apply --check` 检查；已经是目标结果的文件无需重复应用。
3. 对录制管理器和 `collection_motion.py` 做 Python 语法检查。按现有 ROS 安装方式核实 `remote_teleop_runtime.collection_motion.__file__`，确认实际使用的是新源码（必要时更新相应构建/安装产物）。
4. 录制服务空闲时更新它。运动控制进程的更新走现有停止/失能与重新启动流程；文件同步本身不应触发使能、回位或自动录制。若仍在带载遥操，先安排安全停止再重启控制进程。
5. 验证录制端 `supported_tasks` 包含三个新任务，控制器 `collection.split_log_tasks_supported` 为 `true`。新界面会阻止在旧录制端/旧控制器上开始新任务，以免误存入 `test` 或继续受旧采摘起始位限制。
6. 重新打开主机界面，核对左右用途、右臂阶段选择和目录。硬件录制验收时检查三类各自元数据、编号、RGB-D 数据和封口结果。

## 已完成的离线验证

全部使用模拟遥操/录制或临时目录，不连接机械臂、相机，不触发使能或运动。

- `tests/test_split_harvest_tasks.py`、`tests/test_mushroom_collection_console.py`、`tests/test_jetson_record_manager.py`、`tests/test_recording_lifecycle_regressions.py`：共 142 项通过。
- `remote_teleop_runtime/test/test_collection_motion.py`：34 项通过。
- Python 语法检查和 `git diff --check` 通过。
- 验证覆盖：三个任务的物理臂路由、独立编号和统计、保存与重启恢复、旧数据保持、新旧服务兼容拦截、重复开始拦截、阶段切换 READY 重确认、推回在不同位置开始、保持/跟随要求。

复现命令（本机）：

```bash
cd /home/openarm/dev/openarm-rgbd-preview
/usr/bin/python3 -m pytest tests/test_split_harvest_tasks.py tests/test_mushroom_collection_console.py tests/test_jetson_record_manager.py tests/test_recording_lifecycle_regressions.py -q
```

```bash
cd /home/openarm/dev/openarm-remote-harvest
source /opt/ros/humble/setup.bash
PYTHONPATH=ros2_robot/src/remote_teleop_runtime:ros2_robot/src/remote_teleop_protocol:ros2_robot/src/remote_teleop_follower_safety:$PYTHONPATH /usr/bin/python3 -m pytest ros2_robot/src/remote_teleop_runtime/test/test_collection_motion.py -q
```

## 2026-11-14 Jetson 部署记录

- 远端备份和部署清单：`/home/nvidia/openarm-offline-updates/mushroom-task-split-20261114/backup/`、`deployment.json`。
- 已合并原最小改动，保留远端原有的 `OPENARM_RGBD_RUNTIME`、`OPENARM_DATASET_ROOT` 及默认录制目录逻辑；未用主机后来新增的单左臂通用化改动覆盖远端。
- 控制器安装文件实际位置：`/home/nvidia/openarm-offline-updates/job-20261024-175511-dcf1c744/install/remote_teleop_runtime/lib/python3.10/site-packages/remote_teleop_runtime/collection_motion.py`。该文件与源码均已更新。
- Jetson 上合并后的代码通过 36 项离线测试：新任务路由、独立目录/编号、保存恢复及录制生命周期回归。测试需 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，避开现有系统 pytest 与用户 anyio 插件的版本冲突；未修改 Python 环境。
- 从生产路径重新导入代码验证：`supported_tasks` 含三个新任务、`split_log_tasks_supported=true`；临时目录中的 `left_pick`、`right_pull`、`right_push` 创建成功。
- 原双臂录制器与网关在更新时未运行，不需要重启。没有启动双臂遥操、使能、归位或真实录制。独立单左臂录制、相机和控制进程 PID 均保持不变。
- 此更新针对原双臂控制台。正在运行的「单左臂菌棒采集」是另一套程序，仍使用其原有 `left_pull`、`left_push` 目录。

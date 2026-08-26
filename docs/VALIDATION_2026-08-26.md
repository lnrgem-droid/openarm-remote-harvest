# 备份恢复与真机验证记录（2026-08-26）

## 验证对象

- 原程序目录：`/home/openarm/openarm_robot`
- 本地 Git 备份：`/home/openarm/dev/openarm-remote-harvest`
- 独立 bundle：`/home/openarm/Desktop/openarm-remote-harvest-baseline-2026-08-26.bundle`
- 被测试代码提交：`ff9298b`，其后的提交仅补充本次验证文档
- 系统：Ubuntu 22.04、ROS 2 Humble、x86_64
- LeRobot 环境：Python 3.12.13、LeRobot 0.6.2

验证不是在原程序目录中进行，而是从 Git bundle 克隆到全新的 `/tmp/openarm-restore-validation-fixed.*` 目录后完成。

## 验证结果

| 项目 | 结果 | 说明 |
| --- | --- | --- |
| Git bundle 克隆 | 通过 | 分支、标签和完整历史可恢复 |
| Git 对象完整性 | 通过 | `git fsck --full` 无错误 |
| 文件恢复 | 通过 | 1317 个 Git 跟踪文件全部恢复，工作区干净 |
| Python 语法 | 通过 | LeRobot、插件、bridge、scripts 和 web 通过 `compileall` |
| Python 导入 | 通过 | LeRobot 0.6.2 和 OpenArm bridge 插件成功导入 |
| ROS 包发现 | 通过 | 发现 11 个项目 ROS 包 |
| ROS 全量构建 | 通过 | 11/11 个软件包完成 |
| 假硬件启动 | 通过 | 左右 `mock_components/GenericSystem`、控制器、MoveIt、RViz 正常启动 |
| 真机双臂遥操作 | 通过 | 左右主从臂遥操作正常；左右夹爪均被识别并进入控制线程，实际开合未单独记录 |
| ROS 遥操作发布 | 通过 | 左右发布器以 100 Hz 启动 |

## 真机启动观测

恢复副本使用以下映射启动：

```text
right: leader=can2 follower=can0
left:  leader=can3 follower=can1
```

左右两套控制进程均成功识别：

```text
Arm motor count: 7
Gripper motor count: 1
leader arm motor num: 7
follower arm motor num: 7
leader hand motor num: 1
follower hand motor num: 1
```

随后左右 leader、follower、admin 控制线程均启动，ROS 发布器报告：

```text
/joint_states
/right_arm/joint_command
/left_arm/joint_command
100 Hz
```

操作者确认机械臂主从遥操作正常。

## 验证中发现并修正的问题

### Git 不保存空目录

原程序的 `ros2_robot/src/emergency_stop/config/` 是空目录，但 `CMakeLists.txt` 在安装阶段要求它存在。首次从 bundle 恢复后 ROS 构建因此失败。

已在该目录加入 `.gitkeep`，重新生成 bundle 后，`emergency_stop` 编译和安装通过。

### OpenArmCAN 构建顺序

仓库内 `openarm_can` 是普通 CMake 包，其导出的 `OpenArmCANConfig.cmake` 不会自动进入其他 ROS 包的搜索路径。可靠恢复方式是先构建 `openarm_can`，再将其 CMake 配置目录传给剩余包。操作命令已经写入根 README 和恢复手册。

### ROS 与 Conda Python

若在 Conda base 中编译，ROS Humble 可能误用 Conda Python，并因缺少 `catkin_pkg` 失败。ROS 工作空间编译前应退出 Conda，使用 `/usr/bin/python3`；LeRobot 仍在单独的 Python 3.12 环境运行。

## 已知警告

- `openarm_teleop` 有构造顺序和未使用变量等编译器警告，但不阻止构建。
- 假硬件测试中 MoveIt/RViz 能正常运行和执行规划；收到停止信号时存在未优雅退出现象，需要后续单独处理。
- MoveIt 对末端组、惯量和缺省加速度限制有警告，不影响本次主从遥操作验证，但正式自动规划前应复核。

## 本次没有验证

- 主机与 Jetson 分机后的网络遥操作协议
- Jetson 本地独立看门狗和崩溃/断电失效安全
- 三路 Orbbec RGB-D 同步采集、深度落盘和远程 RGB 预览
- OpenArm 原始数据到 LeRobot Dataset 的完整转换
- 控制周期的长期 p99 抖动和超周期统计
- CAN 断开、Jetson 断电、进程强制终止等故障注入

因此，本次结论是：**该备份可以恢复、编译并运行现有单机双臂主从遥操作；它不能证明尚未实现的主机—Jetson 远程系统已经可用。**

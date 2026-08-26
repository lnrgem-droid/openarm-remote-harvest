# 源码基线记录

## 快照信息

- 快照日期：2026-08-26（Asia/Shanghai）
- 原程序目录：`/home/openarm/openarm_robot`
- 独立 Git 备份目录：`/home/openarm/dev/openarm-remote-harvest`
- 目的：保留远程主从与 RGB-D 功能开发前的可回退源码基线
- 原目录处理：未删除、未移动、未覆盖原程序文件

## 纳入备份

- ROS 2 工作空间中的 `src/` 源码及项目文档
- LeRobot 0.6.2 源码快照
- `lerobot_robot_openarm_bridge` 插件源码
- ZMQ 相机脚本和 Orbbec 示例
- Web 控制台源码
- 当前启动指南和配置文件

## 未纳入 Git

- `ros2_robot/build/`、`ros2_robot/install/`、`ros2_robot/log/`
- Python 缓存、虚拟环境、editable install 生成的 `*.egg-info`
- `datasets/`、`outputs/`、录像、ROS bag、模型权重和临时日志
- `.env`、私钥、证书及常见凭据文件

这些内容不是恢复源码所必需的，且生成物、数据集或凭据不适合直接上传 GitHub。

## 已知基线限制

1. 当前源码来自一个没有根级 Git 历史的工作目录，因此无法记录可信的上游 commit；本次提交是新的 Git 历史起点。
2. `LeRobot_OpenArm_启动指南.md` 和若干子目录 README 含旧机器的绝对路径或旧版本文字，保留它们是为了忠实记录现状。根 README 已标出这一点。
3. 当前基线尚未完成两台电脑之间的正式远程主从拆分，也没有完成三路 Orbbec RGB-D 的统一时间合同。
4. 接真机正式遥操作前，仍需实现并验证独立于网络线程的从端本地看门狗和失效安全状态。

## 回退方法

开发必须在 Git 分支中进行。若某次改动失败，可先查看历史：

```bash
git log --oneline --decorate --graph --all
git status
```

推荐从基线标签创建一个干净的恢复分支：

```bash
git switch -c recovery baseline-single-host-2026-08-26
```

不要在机械臂运行时切换代码、覆盖工作目录或删除构建目录。先停止控制进程并确认设备处于安全状态。


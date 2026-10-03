# 主从不一致修正：真机验收（2026-09-20）

## 范围与起始条件

用户明确授权在安全环境中由程序进行真实机械臂测试。本次没有录制数据、没有修改编码器零点，
没有关闭看门狗、扩大验收容差或增加无限累积的纠偏项。使用原有归位轨迹、速度限制和对齐门控。
原 `install_bimanual` 二进制保留，修正构建位于两端 `ros2_robot/install_tracking_candidate`。

## 实际执行

1. 启动前读取真实状态：从端 ALIGNING、无故障；无录制、无左臂锁定、无右臂回位任务。
2. 从此前左从臂 J4 归位未到位的状态启动修正版本，两条从臂约2秒通过既有0.07rad归位门槛。
3. 第一轮主机控制器加载失败，原因是构建误用了 Conda 的 `libyaml-cpp.so.0.9`。
   已停止本轮启动，不冒充运行成功；将主机候选重新链接系统 `libyaml-cpp.so.0.7`。
   核心脚本新增两端 `ldd` 检查，在任何归位前拒绝缺库；主臂进程死亡不再等完整超时。
4. 第二轮四臂归位成功。32个真实驱动的使能检查通过；ALIGN前和解除启动保持后的实际对齐均通过。
5. 只读采样30秒，进入 RUNNING 后无故障，采样点主从误差均满足现有门槛。
6. 调用既有“左臂保持 → 右主从一起返回已保存位置”，不是注入假状态或替代主臂数据。
   约13.04秒完成；右主臂最大关节位移0.96742rad，从臂0.96704rad，约55.4°。
   左臂保持目标始终不变，检查没有超过原0.20rad保护边界，结束后右臂自动恢复跟随。
   用户原有保存位置没有被重写。
7. 恢复左臂跟随，再读30秒：双臂 FOLLOW、RUNNING、无故障，十四关节采样均通过误差门槛。
8. 将已实测构建选择接入原核心入口，并用原 `openarm-remote-teleop.service` 再次启动验证，
   而非只保留临时验收服务。启动仍执行全部物理使能、归位和对齐检查。
   标准入口四臂归位和两轮对齐检查通过；随后30秒288组状态全部通过。
   最终左右臂均FOLLOW、RUNNING、fault_bits=0，无录制。

## 量化结果与边界

第一轮完整启动的正常跟随静态测量（初始位附近）：

| 项目 | 实测 |
|---|---:|
| 左J4主从差 | 0.04005rad / 2.295° |
| 左J7主从差 | 0.01678rad / 0.962° |
| 右J7主从差 | 0.00687rad / 0.393° |

标准入口重启后的第二组实测：十四关节最大主从差1.945°，左J7约0.656°，右J7约0.087°。
30秒288组测量中没有超出上述验收门槛。它是短时静态验收，不代表10分钟全负载动态验收。

J1–J4门槛仍是0.05rad；J5–J7仍是0.035rad。通过不代表绝对零误差。
此前左J4的6.53°是旧版本相对初始目标的误差；上表是正常跟随时主从差，不应混为同一测量。
本次没有完整复测旧的左J7约−52°姿态，也没有做所有姿态、满载、碰撞或手感验收。
右主从保存位回位包含专用主臂伺服，不能用这一测试证明所有人工快速遥操动态性能。
没有人为拔CAN、切电、阻挡机械臂来测试失败；这些异常仍只有软件测试证据。

## 软件回归及保存位置

- 主机：111项测试通过。
- Jetson：43项相关测试通过。
- 实测原始采样、十四关节对齐报告：仓库 `validation/20260920-tracking/`。
- 原服务启动日志：主机 `/tmp/openarm-daily-start/teleop.log`。
- 从端归位日志：Jetson `/tmp/openarm_bimanual_follower.log`。
- 未推送GitHub。

## 入口与复现

桌面入口不需要重建，仍调用 `scripts/run_bimanual_remote_feedback.sh`。
核心的 `TRACKING_CANDIDATE` 默认选择true，使用两端独立修正构建；直接ROS launch的候选开关仍默认关闭，
避免其他独立启动命令无意改变参数。原安装保留仅用于明确回退，不保证旧版本能通过本次对齐要求。

主机重建时必须选择系统Python和系统yaml-cpp，避免Conda混入：

```bash
cd /home/openarm/dev/openarm-remote-harvest/ros2_robot
source /opt/ros/humble/setup.bash
source install/setup.bash
source install_bimanual/setup.bash
/usr/bin/colcon --log-base log_tracking_candidate build \
  --build-base build_tracking_candidate --install-base install_tracking_candidate \
  --packages-select openarm_gravity_pd_control remote_teleop_runtime \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp
```

Jetson同样在本机ARM64构建，不能复制x86二进制；其OpenArmCAN包路径为
`/home/nvidia/openarm_robot/ros2_robot/install/openarm_can/lib/cmake/OpenArmCAN`。
源码或依赖改变后要重编译，并再次验收，不能仅凭本次报告自动认定新构建通过。

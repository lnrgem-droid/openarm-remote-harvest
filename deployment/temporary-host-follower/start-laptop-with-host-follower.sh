#!/usr/bin/env bash
# Temporary arrangement for testing without Jetson. This launcher homes and
# enables all four arms after its on-screen physical safety gate.
set -euo pipefail

export JETSON_HOST=openarm-follower-test
export JETSON_ROOT=/home/openarm/dev/openarm-remote-harvest
export PEER_IP=192.168.50.3
export FOLLOWER_RIGHT_CAN=can0
export FOLLOWER_LEFT_CAN=can1
export OPENARM_TEST_HOST_MODE=true
export OPENARM_REMOTE_CAN_SETUP=/home/openarm/dev/openarm-remote-harvest/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan
export OPENARM_RECORDER_PYTHON=/home/openarm/miniconda3/envs/lerobot/bin/python
export OPENARM_COLLECTION_CONSOLE=/home/openarm/dev/openarm-rgbd-preview/scripts/mushroom_collection_console.py
export OPENARM_COLLECTION_PYTHON=/usr/bin/python3
export OPENARM_COLLECTION_CONSOLE_EXTRA_ARGS='--jetson-ssh openarm-follower-test'
export OPENARM_TELEOP_SERVICE=openarm-host-test-teleop.service
export OPENARM_TELEOP_SERVICE_SOURCE=/home/openarm/openarm-transfer/openarm-host-test-teleop.service

echo '当前配置：笔记本主臂 can0/can1 → 旧主机从臂 can0/can1、三台相机。'
echo '启动后四条机械臂将自动归位并使能，然后才能采图。'
echo '请确认四臂运动范围无人和无障碍物、无夹持物、急停可用且有人现场监护。'
read -r -p '符合上述条件时输入“启动”，其他输入取消：' confirmation
[[ "$confirmation" == 启动 ]] || { echo '已取消，机械臂未使能。'; exit 2; }

exec /usr/bin/bash /home/openarm/dev/openarm-remote-harvest/scripts/daily_start_teleop_rgbd.sh

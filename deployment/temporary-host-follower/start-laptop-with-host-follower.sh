#!/usr/bin/env bash
# Temporary arrangement for testing without Jetson. Open the console only;
# enabling and homing require the explicit start button in the console.
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

export OPENARM_TELEOP_PROFILE=host-test

echo '当前配置：笔记本主臂 → 旧主机从臂、三台相机。'
echo '只打开采集界面；需要运动时，请在界面点击“开始主从遥操…”并确认。'
exec /usr/bin/bash /home/openarm/dev/openarm-remote-harvest/scripts/daily_start_teleop_rgbd.sh

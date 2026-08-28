#!/usr/bin/env bash
# Read-only bridge for LeRobot recording. It never starts an arm controller.
set -euo pipefail

ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
CONFIG_FILE="$ROOT_DIR/config/bridge_follower_recording.yaml"

if ss -ltn '( sport = :9000 )' | grep -q LISTEN; then
  echo "Refusing to start: TCP 9000 is already in use. Do not replace a live bridge." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$ROOT_DIR/ros2_robot/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
exec ros2 launch robot_bridge bridge.launch.py config_file:="$CONFIG_FILE" arm_mode:=gravity_pd

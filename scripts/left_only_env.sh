#!/usr/bin/env bash
# Source only; this file never launches ROS nodes or sends CAN traffic.
LEFT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$LEFT_ROOT/scripts/left_only_paths.sh"
source "$LEFT_UNDERLAY/setup.bash"
source "$LEFT_ROOT/ros2_robot/install_left_only/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
# Isolate ROS services/topics from the original bimanual GUI and runtime.
export ROS_DOMAIN_ID=71

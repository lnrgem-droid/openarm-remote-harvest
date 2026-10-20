#!/usr/bin/env bash
# Build the effective read-only bridge config for the x86 package version.
set -eo pipefail
RGBD=/home/openarm/dev/openarm-rgbd-preview
RUNTIME=/home/openarm/.local/state/openarm-rgbd-host
mkdir -p "$RUNTIME"
/usr/bin/python3 - "$RGBD" "$RUNTIME/bridge-config.yaml" <<'PY'
import importlib.util
from pathlib import Path
import sys
import yaml

root, output = Path(sys.argv[1]), Path(sys.argv[2])
loader = root / 'ros2_robot/src/robot_bridge/robot_bridge/config_loader.py'
spec = importlib.util.spec_from_file_location('rgbd_config_loader', loader)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
config = module.load_config(str(root / 'config/bridge_follower_recording.yaml'))
assert config['startup']['default_components'] == []
assert config['topics']['joint_states'] == '/follower/joint_states'
output.write_text(yaml.safe_dump(config, allow_unicode=True), encoding='utf-8')
PY
source /opt/ros/humble/setup.bash
source /home/openarm/dev/openarm-remote-harvest/ros2_robot/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1
exec taskset --cpu-list 3-7 nice -n 2 \
  ros2 launch robot_bridge bridge.launch.py \
  config_file:="$RUNTIME/bridge-config.yaml" arm_mode:=gravity_pd

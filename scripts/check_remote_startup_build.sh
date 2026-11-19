#!/usr/bin/env bash
# Read-only preflight. Load the effective ROS overlays once for all checks.
# Invoked via `ssh ... bash -s -- ... < this-file`; no remote install needed.
set -eo pipefail
root=$1
overlay=$2
tracking=$3
cpuset=$4
action_timeout=$5
node="$root/ros2_robot/$overlay/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"

source /opt/ros/humble/setup.bash
source "$root/ros2_robot/install/setup.bash"
source "$root/ros2_robot/install_bimanual/setup.bash"
source "$root/ros2_robot/install_gripper_candidate/setup.bash"
source "$root/ros2_robot/$overlay/local_setup.bash"
set -u

dependencies=$(ldd "$node" 2>&1) || { echo "$dependencies" >&2; exit 1; }
if grep -q 'not found' <<<"$dependencies"; then
  echo "ERROR: Jetson 控制器缺少动态库：$dependencies" >&2
  exit 1
fi
if [[ "$action_timeout" == 300 ]]; then
  grep -q 'network_action_timeout_ms' "$root/ros2_robot/install_gripper_candidate/remote_teleop_runtime/share/remote_teleop_runtime/launch/bimanual_follower.launch.py" || {
    echo 'ERROR: 从端启动文件不支持 300 ms 动作超时阈值；尚未开始归位。' >&2; exit 1;
  }
fi
if [[ "$tracking" == true ]]; then
  strings "$node" | grep -F 'bounded tracking assist' >/dev/null || {
    echo 'ERROR: Jetson 控制器缺少 tracking assist；尚未开始归位。' >&2; exit 1;
  }
fi
if [[ ! -x "$node" ]] || ! strings "$node" | grep -F 'Startup homing to upstream OpenArm INITIAL_POSITION' >/dev/null; then
  echo 'ERROR: Jetson 控制器不是当前 INITIAL_POSITION 复位版本；尚未开始归位。' >&2
  exit 1
fi
taskset --cpu-list "$cpuset" true || {
  echo "ERROR: Jetson cannot reserve control CPUs $cpuset." >&2; exit 1;
}

# These are the same bilateral-hold and I/O compatibility assertions previously
# run by two separate SSH/ROS invocations. Any failure still blocks startup.
/usr/bin/python3 - <<'PY'
from remote_teleop_protocol.protocol import decode_collection_ack, COLLECTION_STATE_FLAGS
from remote_teleop_runtime.collection_motion import COMMANDS
import os, yaml
from ament_index_python.packages import get_package_share_directory
assert {"left_home", "right_home", "left_home_stop", "right_home_stop"}.issubset(COMMANDS)
share = get_package_share_directory("remote_teleop_runtime")
with open(os.path.join(share, "config", "bimanual_follower.yaml")) as f:
    assert yaml.safe_load(f)["/**"]["ros__parameters"]["startup_home_target"] == [0.]*7, "从臂启动目标必须为全零"
from remote_teleop_runtime.leader import LeaderGateway
from remote_teleop_runtime.follower import FollowerGateway
assert decode_collection_ack(48) == (0, 0)
assert {9, 17, 19, 23}.issubset(COLLECTION_STATE_FLAGS)
assert {"left_master_align", "left_lock_both", "left_hold_release"}.issubset(COMMANDS)
assert hasattr(LeaderGateway, "collection_acknowledgement")
from remote_teleop_runtime.follower_io import FollowerIOWorker, IO_PROTOCOL_VERSION
from remote_teleop_follower_safety.service import WATCHDOG_IO_PROTOCOL_VERSION
assert IO_PROTOCOL_VERSION == WATCHDOG_IO_PROTOCOL_VERSION == 1
# Verify the actual installed watchdog before any enable/homing. A stale
# package can turn a shutdown HOLD race into a new INVALID_COMMAND fault.
from remote_teleop_protocol import FaultBits
from remote_teleop_follower_safety.state_machine import SafetyStateMachine, SafetyReaction
machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
machine.boot()
machine.trip(FaultBits.CAN_ERROR, "shutdown compatibility check")
before_hold = machine.snapshot()
machine.request_hold()
assert machine.snapshot() == before_hold, "watchdog requires idempotent HOLD after fault"
PY

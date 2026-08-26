#!/bin/bash
#
# Dual-arm bilateral teleop for LeRobot recording.
# Right follower=can0 leader=can2; left follower=can1 leader=can3.
# Publishes /joint_states and /{left,right}_arm/joint_command.
# Do not run together with openarm_gravity_pd_control (same CAN).

set -eo pipefail

WS_DIR="${WS_DIR:-$HOME/openarm_robot/ros2_robot}"
ARM_TYPE="v10"
TMPDIR="/tmp/openarm_urdf_gen"
XACRO_PATH="$WS_DIR/src/openarm_description/urdf/robot/${ARM_TYPE}.urdf.xacro"
LEADER_URDF_PATH="$TMPDIR/${ARM_TYPE}_leader.urdf"
FOLLOWER_URDF_PATH="$TMPDIR/${ARM_TYPE}_follower.urdf"

RIGHT_LEADER_CAN=${1:-can2}
RIGHT_FOLLOWER_CAN=${2:-can0}
LEFT_LEADER_CAN=${3:-can3}
LEFT_FOLLOWER_CAN=${4:-can1}

if [ ! -f "$XACRO_PATH" ]; then
    echo "[ERROR] xacro not found: $XACRO_PATH" >&2
    exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

BIN_PATH="$(ros2 pkg prefix openarm_teleop)/lib/openarm_teleop/bilateral_control"
if [ ! -f "$BIN_PATH" ]; then
    echo "[ERROR] bilateral_control not found at: $BIN_PATH" >&2
    echo "Build first: colcon build --packages-select openarm_teleop && source install/setup.bash" >&2
    exit 1
fi

mkdir -p "$TMPDIR"
echo "[INFO] Generating URDFs..."
xacro "$XACRO_PATH" bimanual:=true -o "$LEADER_URDF_PATH"
cp "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH"

cleanup() {
    if [ -n "${RIGHT_PID:-}" ]; then kill "$RIGHT_PID" 2>/dev/null || true; fi
    if [ -n "${LEFT_PID:-}" ]; then kill "$LEFT_PID" 2>/dev/null || true; fi
    wait || true
}
trap cleanup EXIT INT TERM

echo "[INFO] right: leader=$RIGHT_LEADER_CAN follower=$RIGHT_FOLLOWER_CAN"
"$BIN_PATH" "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH" right_arm "$RIGHT_LEADER_CAN" "$RIGHT_FOLLOWER_CAN" &
RIGHT_PID=$!

echo "[INFO] left: leader=$LEFT_LEADER_CAN follower=$LEFT_FOLLOWER_CAN"
"$BIN_PATH" "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH" left_arm "$LEFT_LEADER_CAN" "$LEFT_FOLLOWER_CAN" &
LEFT_PID=$!

echo "[INFO] both arms running (pids $RIGHT_PID $LEFT_PID). Ctrl+C to stop."
wait

#!/usr/bin/env bash
# Start the verified dual-machine bimanual teleoperation stack.
#
# Host:   leaders, can0 (right) + can1 (left)
# Jetson: followers, can1 (right) + can2 (left)
#
# This script deliberately never bypasses the follower watchdog's ALIGNING gate.
# Both stacks move only to their existing calibrated encoder q=0 reference during
# startup; after that, ALIGN and RUN are requested only if the watchdog accepts it.
# ROS Humble setup scripts themselves read optional variables that may be unset.
# Enable nounset only after sourcing them.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DIR="$ROOT_DIR/ros2_robot"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
JETSON_ROOT="${JETSON_ROOT:-/home/nvidia/dev/openarm-remote-harvest}"
PEER_IP="${PEER_IP:-192.168.50.2}"
FORCE_FEEDBACK="${FORCE_FEEDBACK:-true}"
LOG_DIR="${LOG_DIR:-/tmp/openarm-remote-teleop}"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$ROS_DIR/install/setup.bash"
source "$ROS_DIR/install_bimanual/setup.bash"
set -u

remote_control() {
  local command="$1"
  case "$command" in status|align|run|hold|reset|disable) ;; *) return 64 ;; esac
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && ros2 run remote_teleop_runtime remote-teleop-control $command"
}

cleanup() {
  # A normal interrupt must request the tested hold behavior.  Do not disable
  # motors automatically: the operator must support the arms before disable.
  remote_control hold >/dev/null 2>&1 || true
  if [[ -n "${LEADER_PID:-}" ]] && kill -0 "$LEADER_PID" 2>/dev/null; then
    kill -INT "$LEADER_PID" 2>/dev/null || true
    wait "$LEADER_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

# Bracketed first characters prevent pgrep from matching its own remote command.
if ssh "$JETSON_HOST" "pgrep -af '[r]emote-teleop-follower-watchdog|[r]emote-teleop-follower|[f]ollower_gravity_pd' >/dev/null"; then
  echo "ERROR: a Jetson follower stack is already running. Stop it cleanly before using this script." >&2
  exit 1
fi
if pgrep -af '[l]eader_gravity_pd|[r]emote-teleop-leader' >/dev/null; then
  echo "ERROR: a host leader stack is already running. Stop it cleanly before using this script." >&2
  exit 1
fi

echo '[1/4] Starting Jetson follower stack and controlled q=0 return...'
ssh "$JETSON_HOST" "nohup bash -lc 'source /opt/ros/humble/setup.bash && source $JETSON_ROOT/ros2_robot/install/setup.bash && source $JETSON_ROOT/ros2_robot/install_bimanual/setup.bash && exec ros2 launch remote_teleop_runtime bimanual_follower.launch.py reaction_verified:=true startup_home:=true' > /tmp/openarm_bimanual_follower.log 2>&1 &"

echo '[2/4] Starting host leader stack and controlled q=0 return...'
ros2 launch remote_teleop_runtime bimanual_leader.launch.py "peer:=$PEER_IP" startup_home:=true "force_feedback:=$FORCE_FEEDBACK" >"$LOG_DIR/leader.log" 2>&1 &
LEADER_PID=$!

echo '[3/4] Waiting for both gateways and q=0 return to finish...'
for attempt in $(seq 1 50); do
  STATUS="$(remote_control status 2>/dev/null || true)"
  if grep -q '"leader_session_id": [1-9]' <<<"$STATUS" && grep -q '"state": "ALIGNING"' <<<"$STATUS"; then
    break
  fi
  sleep 1
done
if ! grep -q '"leader_session_id": [1-9]' <<<"${STATUS:-}"; then
  echo "ERROR: follower did not receive a live leader session. See $LOG_DIR/leader.log and Jetson /tmp/openarm_bimanual_follower.log." >&2
  exit 1
fi

echo '[4/4] Requesting safe ALIGN, then RUN...'
if ! remote_control align; then
  echo 'ALIGN was rejected. Keep the arms supported, manually make both sides closer, then restart this script.' >&2
  exit 2
fi
sleep 1
remote_control run
sleep 1
remote_control status
echo 'Bimanual remote teleoperation is RUNNING. Press Ctrl+C for HOLD (then support arms before disable).'
wait "$LEADER_PID"

#!/usr/bin/env bash
# Start the verified dual-machine bimanual teleoperation stack.
#
# Host:   leaders, can0 (right) + can1 (left)
# Jetson: followers, can1 (right) + can2 (left)
#
# This script deliberately never bypasses the follower watchdog's ALIGNING gate.
# Both stacks first reproduce the upstream openarm_teleop INITIAL_POSITION
# (J4=pi/5, all other arm joints=0); after that, ALIGN and RUN are requested
# only if the watchdog accepts it.
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
HOST_CONTROL_NODE="$ROS_DIR/install_bimanual/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
JETSON_CONTROL_NODE="$JETSON_ROOT/ros2_robot/install_bimanual/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
HOME_MARKER="Startup homing to upstream OpenArm INITIAL_POSITION"
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

verify_runtime_builds() {
  if [[ ! -x "$HOST_CONTROL_NODE" ]] || ! strings "$HOST_CONTROL_NODE" | grep -F "$HOME_MARKER" >/dev/null; then
    echo "ERROR: 主机 install_bimanual 不是当前 INITIAL_POSITION 复位版本，请先重新编译。" >&2
    return 1
  fi
  if ! ssh "$JETSON_HOST" "test -x '$JETSON_CONTROL_NODE' && strings '$JETSON_CONTROL_NODE' | grep -F '$HOME_MARKER' >/dev/null"; then
    echo "ERROR: Jetson install_bimanual 不是当前 INITIAL_POSITION 复位版本，请先同步并编译。" >&2
    return 1
  fi
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

# Always replace an old control stack. A gravity-PD node can remain alive after
# its motors were disabled (the disable service says "restart required"). Merely
# seeing that PID and issuing RUN then produces a dangerous false-positive:
# RUNNING in software, but no gravity compensation or motor torque.
echo '[0/5] Verifying identical startup-pose runtime on host and Jetson...'
verify_runtime_builds

echo '[1/5] Holding and replacing any previous teleoperation stack...'
remote_control hold >/dev/null 2>&1 || true
host_pids="$(pgrep -f '^/usr/bin/python3 .*/remote-teleop-leader( |$)|^/home/openarm/.*/openarm_gravity_pd_node .*__node:=leader_gravity_pd( |$)' || true)"
if [[ -n "$host_pids" ]]; then kill -INT $host_pids 2>/dev/null || true; fi
ssh "$JETSON_HOST" "pids=\$(pgrep -f '^/usr/bin/python3 .*/remote-teleop-follower-watchdog( |$)|^/usr/bin/python3 .*/remote-teleop-follower( |$)|^/home/nvidia/.*/openarm_gravity_pd_node .*__node:=follower_gravity_pd( |$)' || true); if [[ -n \"\$pids\" ]]; then kill -INT \$pids 2>/dev/null || true; fi"
sleep 3
if pgrep -f '^/usr/bin/python3 .*/remote-teleop-leader( |$)|^/home/openarm/.*/openarm_gravity_pd_node .*__node:=leader_gravity_pd( |$)' >/dev/null; then
  echo 'ERROR: old host control stack did not stop.' >&2; exit 1
fi
if ssh "$JETSON_HOST" "pgrep -f '^/usr/bin/python3 .*/remote-teleop-follower-watchdog( |$)|^/usr/bin/python3 .*/remote-teleop-follower( |$)|^/home/nvidia/.*/openarm_gravity_pd_node .*__node:=follower_gravity_pd( |$)' >/dev/null"; then
  echo 'ERROR: old Jetson control stack did not stop.' >&2; exit 1
fi

echo '[2/5] Starting Jetson follower stack and controlled OpenArm initial-pose return...'
ssh "$JETSON_HOST" "nohup bash -lc 'source /opt/ros/humble/setup.bash && source $JETSON_ROOT/ros2_robot/install/setup.bash && source $JETSON_ROOT/ros2_robot/install_bimanual/setup.bash && exec ros2 launch remote_teleop_runtime bimanual_follower.launch.py reaction_verified:=true startup_home:=true' > /tmp/openarm_bimanual_follower.log 2>&1 &"

echo '[3/5] Starting host leader stack and controlled OpenArm initial-pose return...'
ros2 launch remote_teleop_runtime bimanual_leader.launch.py "peer:=$PEER_IP" startup_home:=true "force_feedback:=$FORCE_FEEDBACK" >"$LOG_DIR/leader.log" 2>&1 &
LEADER_PID=$!

echo '[4/5] Waiting for both gateways and initial-pose return to finish...'
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

echo '[5/5] Requesting safe ALIGN, then RUN...'
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

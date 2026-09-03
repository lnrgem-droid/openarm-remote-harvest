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
JETSON_CONTROL_CPUSET="${JETSON_CONTROL_CPUSET:-0-2}"
HOST_CONTROL_NODE="$ROS_DIR/install_bimanual/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
JETSON_CONTROL_NODE="$JETSON_ROOT/ros2_robot/install_bimanual/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
HOME_MARKER="Startup homing to upstream OpenArm INITIAL_POSITION"
mkdir -p "$LOG_DIR"
if [[ ! "$JETSON_CONTROL_CPUSET" =~ ^[0-9,-]+$ ]]; then
  echo "ERROR: invalid JETSON_CONTROL_CPUSET: $JETSON_CONTROL_CPUSET" >&2
  exit 64
fi

source /opt/ros/humble/setup.bash
source "$ROS_DIR/install/setup.bash"
source "$ROS_DIR/install_bimanual/setup.bash"
set -u

remote_control() {
  local command="$1"
  case "$command" in status|align|run|hold|reset|disable) ;; *) return 64 ;; esac
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && ros2 run remote_teleop_runtime remote-teleop-control $command"
}

show_stage() {
  local number="$1" title="$2" motion="$3" control="$4" action="$5"
  printf '\n[%s/6] %s\n' "$number" "$title"
  printf '  机械臂可能运动：%s\n' "$motion"
  printf '  现在可以遥操：%s\n' "$control"
  printf '  你现在应当：%s\n' "$action"
}

release_startup_holds() {
  echo '  正在解除主臂和从臂的启动位保持，准备接受实时遥操指令…'
  ROS_LOCALHOST_ONLY=1 timeout 8 ros2 service call /leader/openarm_gravity_pd/startup_hold \
    std_srvs/srv/SetBool '{data: false}' >/dev/null
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && ROS_LOCALHOST_ONLY=1 timeout 8 ros2 service call /follower/openarm_gravity_pd/startup_hold std_srvs/srv/SetBool '{data: false}' >/dev/null"
}

check_jetson_python_runtime() {
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && /usr/bin/python3 -c 'from remote_teleop_runtime.common import FOLLOWER_LEFT_COMMAND_TOPIC; from remote_teleop_runtime.follower import FollowerGateway'"
}

repair_jetson_python_runtime() {
  local package_source="$ROS_DIR/src/remote_teleop_runtime/remote_teleop_runtime/"
  local config_source="$ROS_DIR/src/remote_teleop_runtime/config"
  local installed_package="$JETSON_ROOT/ros2_robot/install/remote_teleop_runtime/lib/python3.10/site-packages/remote_teleop_runtime/"
  local installed_config="$JETSON_ROOT/ros2_robot/install/remote_teleop_runtime/share/remote_teleop_runtime/config/"
  local bimanual_config="$JETSON_ROOT/ros2_robot/install_bimanual/remote_teleop_runtime/share/remote_teleop_runtime/config/"

  echo 'Jetson Python runtime is inconsistent; synchronizing the complete runtime package...'
  rsync -a --include='*.py' --exclude='*' "$package_source" "$JETSON_HOST:$installed_package"
  rsync -a "$config_source/bimanual_leader.yaml" "$config_source/bimanual_follower.yaml" \
    "$JETSON_HOST:$installed_config"
  rsync -a "$config_source/bimanual_leader.yaml" "$config_source/bimanual_follower.yaml" \
    "$JETSON_HOST:$bimanual_config"
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
  if ! ssh "$JETSON_HOST" "taskset --cpu-list '$JETSON_CONTROL_CPUSET' true"; then
    echo "ERROR: Jetson cannot reserve control CPUs $JETSON_CONTROL_CPUSET." >&2
    return 1
  fi
  if ! check_jetson_python_runtime; then
    repair_jetson_python_runtime
    if ! check_jetson_python_runtime; then
      echo "ERROR: Jetson remote_teleop_runtime 自动修复后仍无法导入。" >&2
      return 1
    fi
    echo 'Jetson Python runtime repair: OK'
  fi
}

cleanup() {
  # A normal interrupt must request the tested hold behavior.  Do not disable
  # motors automatically: the operator must support the arms before disable.
  # The watchdog intentionally rejects HOLD after it has already latched FAULT.
  # Do not turn a normal transport-timeout fault into an additional invalid-
  # command fault while cleaning up a terminated launcher.
  local final_status
  final_status="$(remote_control status 2>/dev/null || true)"
  if grep -Eq '"state": "(ALIGNING|READY|RUNNING)"' <<<"$final_status"; then
    remote_control hold >/dev/null 2>&1 || true
  fi
  if [[ -n "${LEADER_PID:-}" ]] && kill -0 "$LEADER_PID" 2>/dev/null; then
    kill -INT "$LEADER_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$LEADER_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$LEADER_PID" 2>/dev/null; then
      leader_children="$(pgrep -P "$LEADER_PID" || true)"
      [[ -z "$leader_children" ]] || kill -TERM $leader_children 2>/dev/null || true
      kill -TERM "$LEADER_PID" 2>/dev/null || true
    fi
    wait "$LEADER_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

# Always replace an old control stack. A gravity-PD node can remain alive after
# its motors were disabled (the disable service says "restart required"). Merely
# seeing that PID and issuing RUN then produces a dangerous false-positive:
# RUNNING in software, but no gravity compensation or motor torque.
echo '============================================================'
echo 'OpenArm 双机双臂启动：主机主臂 can0/can1 → Jetson 从臂 can1/can2'
echo '只有终端明确显示“现在可以遥操：是”以后，才能移动主机械臂。'
echo '============================================================'
show_stage 0 '检查主机与 Jetson 是否使用同一套归零程序' \
  '本步骤不发送新动作；若旧遥操仍在运行，旧控制仍可能生效' \
  '否' '保持四条机械臂静止，确认急停可用'
verify_runtime_builds

show_stage 1 '让旧遥操进入保持，并关闭旧控制进程' \
  '可能；切换保持或重启控制器时力矩手感可能变化' \
  '否' '不要推动主臂或从臂，人员在从臂旁监护'
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

show_stage 2 '启动 Jetson 从臂控制，并让左右从臂自动回到初始位' \
  '是；左右从臂会自动运动回初始位' \
  '否' '远离运动范围，不要拖动任何机械臂，随时准备急停'
ssh "$JETSON_HOST" "nohup bash -lc 'source /opt/ros/humble/setup.bash && source $JETSON_ROOT/ros2_robot/install/setup.bash && source $JETSON_ROOT/ros2_robot/install_bimanual/setup.bash && exec taskset --cpu-list $JETSON_CONTROL_CPUSET ros2 launch remote_teleop_runtime bimanual_follower.launch.py reaction_verified:=true startup_home:=true' > /tmp/openarm_bimanual_follower.log 2>&1 &"

show_stage 3 '等待左右从臂完成回初始位并进入 ALIGNING' \
  '是；从臂可能仍在缓慢归位' \
  '否' '继续等待，不要提前移动主臂；本步骤无需按键'
FOLLOWER_READY=false
for attempt in $(seq 1 45); do
  if ssh "$JETSON_HOST" "grep -q 'process has died' /tmp/openarm_bimanual_follower.log 2>/dev/null"; then
    echo 'ERROR: Jetson follower stack exited during startup:' >&2
    ssh "$JETSON_HOST" "tail -40 /tmp/openarm_bimanual_follower.log" >&2 || true
    exit 1
  fi
  STATUS="$(remote_control status 2>/dev/null || true)"
  # The watchdog may already report ALIGNING while gravity-PD is still
  # executing its sequential left/right startup trajectory.  Do not start the
  # leader, ALIGN, or print a teleoperation-ready message until *both* follower
  # arms have explicitly completed that trajectory.
  FOLLOWER_HOME_COUNT="$(ssh "$JETSON_HOST" "grep -c 'Startup homing command complete.' /tmp/openarm_bimanual_follower.log 2>/dev/null || true")"
  if [[ "$FOLLOWER_HOME_COUNT" -ge 2 ]] &&
     grep -q '"right_actual_rad"' <<<"$STATUS" &&
     grep -q '"left_actual_rad"' <<<"$STATUS" &&
     grep -q '"state": "ALIGNING"' <<<"$STATUS"; then
    FOLLOWER_READY=true
    break
  fi
  sleep 1
done
if [[ "$FOLLOWER_READY" != true ]]; then
  echo 'ERROR: Jetson follower homing did not become ready within 45 seconds.' >&2
  echo 'See Jetson /tmp/openarm_bimanual_follower.log.' >&2
  exit 1
fi

show_stage 4 '启动主机主臂控制，并让左右主臂自动回到初始位' \
  '是；左右主臂会自动运动回初始位' \
  '否' '松开主臂并远离夹点，等待自动归位完成'
ros2 launch remote_teleop_runtime bimanual_leader.launch.py "peer:=$PEER_IP" startup_home:=true "force_feedback:=$FORCE_FEEDBACK" >"$LOG_DIR/leader.log" 2>&1 &
LEADER_PID=$!

show_stage 5 '等待主臂归位，并检查主从关节差值是否稳定' \
  '可能；主臂可能仍在归位，主从两端随后保持当前位置' \
  '否' '保持所有机械臂不动；本步骤无需按键，程序会自动判断'
for attempt in $(seq 1 50); do
  STATUS="$(remote_control status 2>/dev/null || true)"
  # Same contract on the host: a live leader session alone is not evidence
  # that both leader arms have reached and are holding INITIAL_POSITION.
  LEADER_HOME_COUNT="$(grep -c 'Startup homing command complete.' "$LOG_DIR/leader.log" 2>/dev/null || true)"
  if [[ "$LEADER_HOME_COUNT" -ge 2 ]] &&
     grep -q '"leader_session_id": [1-9]' <<<"$STATUS" && grep -q '"state": "ALIGNING"' <<<"$STATUS"; then
    break
  fi
  sleep 1
done
if ! grep -q '"leader_session_id": [1-9]' <<<"${STATUS:-}"; then
  echo "ERROR: follower did not receive a live leader session. See $LOG_DIR/leader.log and Jetson /tmp/openarm_bimanual_follower.log." >&2
  exit 1
fi

show_stage 6 '自动执行 ALIGN，并请求进入 RUNNING' \
  '可能；RUNNING 生效后从臂将开始跟随主臂' \
  '暂时不可以' '保持主臂静止，等候最终绿色提示'
ALIGN_OK=false
for attempt in $(seq 1 12); do
  ALIGN_REPLY="$(remote_control align 2>&1 || true)"
  if grep -q '"state": "READY"' <<<"$ALIGN_REPLY"; then
    ALIGN_OK=true
    break
  fi
  if grep -q 'alignment must remain' <<<"$ALIGN_REPLY"; then
    sleep 0.25
    continue
  fi
  if grep -q '"error"' <<<"$ALIGN_REPLY"; then
    echo "$ALIGN_REPLY" >&2
    break
  fi
  # The command endpoint returns its current snapshot before the watchdog loop
  # consumes the request.  Poll the authoritative status briefly instead of
  # treating that expected stale ALIGNING reply as a failed alignment.
  for settle_attempt in $(seq 1 20); do
    sleep 0.10
    ALIGN_STATUS="$(remote_control status 2>&1 || true)"
    if grep -q '"state": "READY"' <<<"$ALIGN_STATUS"; then
      ALIGN_OK=true
      break 2
    fi
  done
  if ! grep -q '"state": "ALIGNING"' <<<"$ALIGN_STATUS"; then
    echo "$ALIGN_STATUS" >&2
    break
  fi
  sleep 0.25
done
if [[ "$ALIGN_OK" != true ]]; then
  echo 'ALIGN was rejected. The follower remains in HOLD; inspect the joint mismatch above.' >&2
  exit 2
fi
RUN_REPLY="$(remote_control run 2>&1 || true)"
RUN_OK=false
for attempt in $(seq 1 30); do
  RUN_STATUS="$(remote_control status 2>&1 || true)"
  if grep -q '"state": "RUNNING"' <<<"$RUN_STATUS"; then
    RUN_OK=true
    break
  fi
  sleep 0.10
done
if [[ "$RUN_OK" != true ]]; then
  echo "$RUN_REPLY" >&2
  echo "$RUN_STATUS" >&2
  echo 'RUN was not acknowledged; both arms remain in startup-pose hold.' >&2
  exit 3
fi
release_startup_holds
# The RUN acknowledgement is generated before the two gravity-PD startup
# holds are released.  Wait for that release to complete and for fresh control
# traffic before telling the operator that it is safe to move a leader arm.
sleep 2
RUN_STATUS="$(remote_control status 2>/dev/null || true)"
if ! grep -q '"state": "RUNNING"' <<<"$RUN_STATUS"; then
  echo "ERROR: startup holds were released but RUNNING was lost." >&2
  echo "$RUN_STATUS" >&2
  exit 3
fi
printf '%s\n' "$RUN_STATUS" >"$LOG_DIR/run-status.json"
echo
echo '============================================================'
echo '启动完成：状态 RUNNING，左右主从臂已建立一一对应跟随。'
echo '  机械臂可能运动：是；从臂会跟随主臂，力反馈也已启用'
echo '  现在可以遥操：是'
echo '  你现在应当：先小幅、低速移动主臂，确认方向正确后再正常操作'
echo '  停止方式：按 Ctrl+C 将从臂切换到位置保持；不要直接断电'
echo "  完整运行状态：$LOG_DIR/run-status.json"
echo '============================================================'
wait "$LEADER_PID"

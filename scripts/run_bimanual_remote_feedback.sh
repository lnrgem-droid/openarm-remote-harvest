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

ssh() { command ssh -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=2 "$@"; }

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DIR="$ROOT_DIR/ros2_robot"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
JETSON_ROOT="${JETSON_ROOT:-/home/nvidia/dev/openarm-remote-harvest}"
PEER_IP="${PEER_IP:-192.168.50.2}"
FORCE_FEEDBACK="${FORCE_FEEDBACK:-true}"
LOG_DIR="${LOG_DIR:-/tmp/openarm-remote-teleop}"
JETSON_CONTROL_CPUSET="${JETSON_CONTROL_CPUSET:-0-2}"
# Default selected after supervised homing + alignment + saved-pose return
# acceptance on 2026-09-20. Keep the old install intact for explicit rollback.
TRACKING_CANDIDATE="${TRACKING_CANDIDATE:-true}"
case "$TRACKING_CANDIDATE" in
  true) CONTROL_OVERLAY=install_tracking_candidate ;;
  false) CONTROL_OVERLAY=install_bimanual ;;
  *) echo 'ERROR: TRACKING_CANDIDATE must be true or false' >&2; exit 64 ;;
esac
GRIPPER_CONTACT_FEEDBACK="${GRIPPER_CONTACT_FEEDBACK:-true}"
case "$GRIPPER_CONTACT_FEEDBACK" in
  true) CONTROL_OVERLAY=install_gripper_spring_candidate ;;
  false) ;;
  *) echo 'ERROR: GRIPPER_CONTACT_FEEDBACK must be true or false' >&2; exit 64 ;;
esac
HOST_CONTROL_NODE="$ROS_DIR/$CONTROL_OVERLAY/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
JETSON_CONTROL_NODE="$JETSON_ROOT/ros2_robot/$CONTROL_OVERLAY/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
HOME_MARKER="Startup homing to upstream OpenArm INITIAL_POSITION"
mkdir -p "$LOG_DIR"
if [[ ! "$JETSON_CONTROL_CPUSET" =~ ^[0-9,-]+$ ]]; then
  echo "ERROR: invalid JETSON_CONTROL_CPUSET: $JETSON_CONTROL_CPUSET" >&2
  exit 64
fi

source /opt/ros/humble/setup.bash
source "$ROS_DIR/install/setup.bash"
source "$ROS_DIR/install_bimanual/setup.bash"
if [[ "$CONTROL_OVERLAY" != install_bimanual ]]; then
  # The spring overlay contains only the C++ controller. Preserve the tested
  # Python/launch/config underlay; setup.bash would re-source older parents.
  source "$ROS_DIR/install_gripper_candidate/setup.bash"
  source "$ROS_DIR/$CONTROL_OVERLAY/local_setup.bash"
  echo '使用 2026-09-20 真机归位/对齐验收构建；原控制构建保留，启动仍须通过全部检查。'
fi
set -u

remote_control() {
  local command="$1"
  case "$command" in status|align|run|hold|reset|disable) ;; *) return 64 ;; esac
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && ros2 run remote_teleop_runtime remote-teleop-control $command"
}

status_is_healthy_running() {
  /usr/bin/python3 "$ROOT_DIR/scripts/check_remote_running_status.py" \
    --startup-report "$LOG_DIR/alignment-following.pending.json" <<<"$1"
}

check_physical_enables() {
  echo '  被动检查主从 32 个电机的真实使能状态…'
  /usr/bin/python3 "$ROOT_DIR/scripts/check_motor_enable.py" --jetson "$JETSON_HOST" \
    >"$LOG_DIR/motor-enable-status.json" || {
      echo 'ERROR: 主从电机未全部确认使能，禁止继续自动对齐。' >&2
      cat "$LOG_DIR/motor-enable-status.json" >&2
      return 1
    }
}

verify_alignment() {
  local phase="$1"
  local report="$LOG_DIR/alignment-$phase.json"
  [[ "$phase" != following ]] || report="$LOG_DIR/alignment-following.pending.json"
  /usr/bin/python3 "$ROOT_DIR/scripts/verify_startup_alignment.py" \
    --jetson "$JETSON_HOST" --phase "$phase" \
    --report "$report"
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
  local leader_release follower_release
  leader_release="$(ROS_LOCALHOST_ONLY=1 timeout 8 ros2 service call /leader/openarm_gravity_pd/startup_hold \
    std_srvs/srv/SetBool '{data: false}')"
  grep -Eq 'success=(True|true)' <<<"$leader_release" || {
    echo "ERROR: 主臂启动位保持未确认解除：$leader_release" >&2
    return 1
  }
  follower_release="$(ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && ROS_LOCALHOST_ONLY=1 timeout 8 ros2 service call /follower/openarm_gravity_pd/startup_hold std_srvs/srv/SetBool '{data: false}'")"
  grep -Eq 'success=(True|true)' <<<"$follower_release" || {
    echo "ERROR: 从臂启动位保持未确认解除：$follower_release" >&2
    return 1
  }
}

check_jetson_python_runtime() {
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && /usr/bin/python3 -c 'from remote_teleop_runtime.collection_motion import CollectionMotion; from remote_teleop_runtime.follower import FollowerGateway; from remote_teleop_protocol import FollowerState; assert hasattr(FollowerState, \"leader_return_target\")'"
}

repair_jetson_python_runtime() {
  local package_source="$ROS_DIR/src/remote_teleop_runtime/remote_teleop_runtime/"
  local config_source="$ROS_DIR/src/remote_teleop_runtime/config"
  local installed_package="$JETSON_ROOT/ros2_robot/install/remote_teleop_runtime/lib/python3.10/site-packages/remote_teleop_runtime/"
  local installed_config="$JETSON_ROOT/ros2_robot/install/remote_teleop_runtime/share/remote_teleop_runtime/config/"
  local bimanual_config="$JETSON_ROOT/ros2_robot/install_bimanual/remote_teleop_runtime/share/remote_teleop_runtime/config/"

  echo 'Jetson Python runtime is inconsistent; synchronizing the complete runtime package...'
  rsync -a --include='*.py' --exclude='*' "$package_source" "$JETSON_HOST:$installed_package"
  rsync -a --include='*.py' --exclude='*' \
    "$ROS_DIR/src/remote_teleop_protocol/remote_teleop_protocol/" \
    "$JETSON_HOST:$JETSON_ROOT/ros2_robot/install_bimanual/remote_teleop_protocol/lib/python3.10/site-packages/remote_teleop_protocol/"
  rsync -a "$config_source/bimanual_leader.yaml" "$config_source/bimanual_follower.yaml" \
    "$JETSON_HOST:$installed_config"
  rsync -a "$config_source/bimanual_leader.yaml" "$config_source/bimanual_follower.yaml" \
    "$JETSON_HOST:$bimanual_config"
}

verify_runtime_builds() {
  # A successful build does not prove the service environment can load it
  # (e.g. accidental Conda yaml-cpp linkage). Check before touching motors.
  local host_dependencies remote_dependencies
  host_dependencies="$(ldd "$HOST_CONTROL_NODE" 2>&1)" || {
    echo "ERROR: 主机控制器依赖检查失败：$host_dependencies" >&2; return 1;
  }
  remote_dependencies="$(ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_gripper_candidate/setup.bash' && source '$JETSON_ROOT/ros2_robot/$CONTROL_OVERLAY/local_setup.bash' && ldd '$JETSON_CONTROL_NODE'" 2>&1)" || {
    echo "ERROR: Jetson 控制器依赖检查失败：$remote_dependencies" >&2; return 1;
  }
  if grep -q 'not found' <<<"$host_dependencies$remote_dependencies"; then
    echo 'ERROR: 控制器缺少动态库，尚未开始归位：' >&2
    printf '%s\n%s\n' "$host_dependencies" "$remote_dependencies" >&2
    return 1
  fi
  if [[ "$GRIPPER_CONTACT_FEEDBACK" == true ]]; then
    strings "$HOST_CONTROL_NODE" | grep -F 'Gripper contact reflection' >/dev/null || return 1
    /usr/bin/python3 -c 'from remote_teleop_runtime.common import gripper_contact_reference' || return 1
  fi
  if [[ "$TRACKING_CANDIDATE" == true ]]; then
    strings "$HOST_CONTROL_NODE" | grep -F 'bounded tracking assist' >/dev/null || return 1
    ssh "$JETSON_HOST" "strings '$JETSON_CONTROL_NODE' | grep -F 'bounded tracking assist' >/dev/null" || return 1
  fi
  /usr/bin/python3 -c 'from remote_teleop_runtime.collection_motion import CollectionMotion; from remote_teleop_protocol import FollowerState; assert hasattr(FollowerState, "leader_return_target")' || {
    echo 'ERROR: 主机遥操 Python 包未更新，请重新构建 remote_teleop_protocol 和 remote_teleop_runtime。' >&2
    return 1
  }
  if ! strings "$HOST_CONTROL_NODE" | grep -F 'collection_return_topic' >/dev/null; then
    echo 'ERROR: 主机控制器缺少右主臂回位功能，请先编译 openarm_gravity_pd_control。' >&2
    return 1
  fi
  if ! strings "$HOST_CONTROL_NODE" | grep -F 'collection_left_return_topic' >/dev/null; then
    echo 'ERROR: 主机控制器缺少左主臂自动对齐接口；尚未开始归位，请更新控制构建。' >&2
    return 1
  fi
  # Both running Python overlays must understand the independent left/right
  # servo acknowledgements before any homing or motor startup is attempted.
  local left_master_check
  left_master_check='from remote_teleop_protocol.protocol import decode_collection_ack, COLLECTION_STATE_FLAGS; from remote_teleop_runtime.collection_motion import COMMANDS; from remote_teleop_runtime.leader import LeaderGateway; from remote_teleop_runtime.follower import FollowerGateway; assert decode_collection_ack(16) == (0, 0); assert 9 in COLLECTION_STATE_FLAGS; assert "left_master_align" in COMMANDS; assert hasattr(LeaderGateway, "collection_acknowledgement")'
  /usr/bin/python3 -c "$left_master_check" || {
    echo 'ERROR: 主机实际 Python overlay 缺少左主臂对齐协议；尚未开始归位。' >&2
    return 1
  }
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_gripper_candidate/setup.bash' && source '$JETSON_ROOT/ros2_robot/$CONTROL_OVERLAY/local_setup.bash' && /usr/bin/python3 -c '$left_master_check'" || {
    echo 'ERROR: Jetson 实际 Python overlay 缺少左主臂对齐协议；尚未开始归位。' >&2
    return 1
  }
  # The dedicated I/O worker requires correlated watchdog replies. Check the
  # effective overlays together before a restart can begin motor homing.
  local follower_io_check
  follower_io_check='from remote_teleop_runtime.follower_io import FollowerIOWorker, IO_PROTOCOL_VERSION; from remote_teleop_runtime.follower import FollowerGateway; from remote_teleop_follower_safety.service import WATCHDOG_IO_PROTOCOL_VERSION; assert IO_PROTOCOL_VERSION == WATCHDOG_IO_PROTOCOL_VERSION == 1'
  /usr/bin/python3 -c "$follower_io_check" || {
    echo 'ERROR: 主机通信与 watchdog 修复版本不一致；尚未开始归位。' >&2
    return 1
  }
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source '$JETSON_ROOT/ros2_robot/install/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_bimanual/setup.bash' && source '$JETSON_ROOT/ros2_robot/install_gripper_candidate/setup.bash' && source '$JETSON_ROOT/ros2_robot/$CONTROL_OVERLAY/local_setup.bash' && /usr/bin/python3 -c '$follower_io_check'" || {
    echo 'ERROR: Jetson 通信与 watchdog 修复版本不一致；尚未开始归位。' >&2
    return 1
  }
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
  if [[ -n "${LINK_MONITOR_PID:-}" ]]; then
    kill -TERM "$LINK_MONITOR_PID" 2>/dev/null || true
    wait "$LINK_MONITOR_PID" 2>/dev/null || true
  fi
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
PEER_BOOT_ID="$(ssh "$JETSON_HOST" 'cat /proc/sys/kernel/random/boot_id')"
if [[ ! "$PEER_BOOT_ID" =~ ^[a-f0-9-]{36}$ ]]; then
  echo 'ERROR: 无法确认Jetson启动身份，不开始归位。' >&2; exit 1
fi

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
ssh "$JETSON_HOST" "nohup bash -lc 'source /opt/ros/humble/setup.bash && source $JETSON_ROOT/ros2_robot/install/setup.bash && source $JETSON_ROOT/ros2_robot/install_bimanual/setup.bash && source $JETSON_ROOT/ros2_robot/install_gripper_candidate/setup.bash && source $JETSON_ROOT/ros2_robot/$CONTROL_OVERLAY/local_setup.bash && exec taskset --cpu-list $JETSON_CONTROL_CPUSET ros2 launch remote_teleop_runtime bimanual_follower.launch.py reaction_verified:=true startup_home:=true tracking_candidate:=$TRACKING_CANDIDATE' > /tmp/openarm_bimanual_follower.log 2>&1 &"

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
  if ssh "$JETSON_HOST" "grep -q 'Startup homing timed out' /tmp/openarm_bimanual_follower.log"; then
    echo 'ERROR: 从臂未实际到达初始位，不能把归位超时当作成功。停止自动启动并保持，不重复强推。' >&2
    ssh "$JETSON_HOST" "tail -20 /tmp/openarm_bimanual_follower.log" >&2 || true
    exit 1
  fi
  # The watchdog may already report ALIGNING while gravity-PD is still
  # executing its sequential left/right startup trajectory.  Do not start the
  # leader, ALIGN, or print a teleoperation-ready message until *both* follower
  # arms have explicitly completed that trajectory.
  FOLLOWER_HOME_COUNT="$(ssh "$JETSON_HOST" "grep -c 'Startup homing reached upstream initial pose' /tmp/openarm_bimanual_follower.log 2>/dev/null || true")"
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
LEADER_ARGS=("peer:=$PEER_IP" startup_home:=true "force_feedback:=$FORCE_FEEDBACK" "tracking_candidate:=$TRACKING_CANDIDATE")
# Old overlays do not know this parameter; only send it to the new build.
if [[ "$GRIPPER_CONTACT_FEEDBACK" == true ]]; then LEADER_ARGS+=(gripper_contact_feedback:=true); fi
ros2 launch remote_teleop_runtime bimanual_leader.launch.py "${LEADER_ARGS[@]}" >"$LOG_DIR/leader.log" 2>&1 &
LEADER_PID=$!

show_stage 5 '等待主臂归位，并检查主从关节差值是否稳定' \
  '可能；主臂可能仍在归位，主从两端随后保持当前位置' \
  '否' '保持所有机械臂不动；本步骤无需按键，程序会自动判断'
LEADER_READY=false
for attempt in $(seq 1 50); do
  if ! kill -0 "$LEADER_PID" 2>/dev/null || grep -q 'process has died' "$LOG_DIR/leader.log"; then
    echo 'ERROR: 主臂控制进程退出，停止启动并保持；不是等待对齐。' >&2
    tail -25 "$LOG_DIR/leader.log" >&2 || true
    exit 1
  fi
  STATUS="$(remote_control status 2>/dev/null || true)"
  if grep -q 'Startup homing timed out' "$LOG_DIR/leader.log"; then
    echo 'ERROR: 主臂未实际到达初始位，停止自动启动，不进入遥操。' >&2
    tail -20 "$LOG_DIR/leader.log" >&2 || true
    exit 1
  fi
  # Same contract on the host: a live leader session alone is not evidence
  # that both leader arms have reached and are holding INITIAL_POSITION.
  LEADER_HOME_COUNT="$(grep -c 'Startup homing reached upstream initial pose' "$LOG_DIR/leader.log" 2>/dev/null || true)"
  if [[ "$LEADER_HOME_COUNT" -ge 2 ]] &&
     grep -q '"leader_session_id": [1-9]' <<<"$STATUS" && grep -q '"state": "ALIGNING"' <<<"$STATUS"; then
    LEADER_READY=true
    break
  fi
  sleep 1
done
if [[ "$LEADER_READY" != true ]]; then
  echo "ERROR: 两条主臂未在 50 秒内同时完成归零，或从端未收到有效 leader 会话。" >&2
  echo "See $LOG_DIR/leader.log and Jetson /tmp/openarm_bimanual_follower.log." >&2
  exit 1
fi

echo '  自动验收归位与主从对齐：J1–J4 差值 ≤0.05 rad，J5–J7 ≤0.035 rad；连续静止 1 秒。'
check_physical_enables || exit 3
verify_alignment aligning || exit 3

show_stage 6 '自动执行 ALIGN，并请求进入 RUNNING' \
  '可能；RUNNING 生效后从臂将开始跟随主臂' \
  '暂时不可以' '保持主臂静止，等候最终绿色提示'
ALIGN_OK=false
# The preceding stable-pose gate replaces blind repeated ALIGN requests.
# Send once, then poll authoritative state; never mistake a lost reply for a
# reason to issue a second transition or reset a safety fault.
ALIGN_REPLY="$(remote_control align 2>&1 || true)"
if grep -q '"error"' <<<"$ALIGN_REPLY"; then
  echo "$ALIGN_REPLY" >&2
  exit 2
fi
for attempt in $(seq 1 20); do
  ALIGN_STATUS="$(remote_control status 2>&1 || true)"
  if grep -q '"state": "READY"' <<<"$ALIGN_STATUS"; then
    ALIGN_OK=true
    break
  fi
  if grep -Eq '"state": "(FAULT|E_STOP)"' <<<"$ALIGN_STATUS"; then break; fi
  sleep 0.1
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
echo '  正在复查正常跟随增益下的实际到位情况；现在不能遥操，请保持主臂静止。'
verify_alignment following || exit 3
# The stable-pose verifier above already waited for release/settling. Require
# its certificate to match the live session; do not add an unobserved sleep.
FOLLOW_READY=false
for attempt in $(seq 1 20); do
  RUN_STATUS="$(remote_control status 2>/dev/null || true)"
  if status_is_healthy_running "$RUN_STATUS"; then
    FOLLOW_READY=true
    break
  fi
  sleep 0.1
done
if [[ "$FOLLOW_READY" != true ]]; then
  echo "ERROR: 启动位保持解除后，双臂跟随健康条件未全部满足。" >&2
  echo "要求：RUNNING、fault_bits=0、左右臂均启用、相对跟随参考已捕获。" >&2
  echo "$RUN_STATUS" >&2
  exit 3
fi
# Heartbeats and matching angles do not prove torque is enabled after a power
# cycle. Inspect all 32 physical drives before printing the success marker.
check_physical_enables || exit 3
RUN_STATUS="$(remote_control status 2>/dev/null || true)"
status_is_healthy_running "$RUN_STATUS" || {
  echo 'ERROR: 最终检查期间控制状态或会话发生变化，停止启动。' >&2
  exit 3
}
mv "$LOG_DIR/alignment-following.pending.json" "$LOG_DIR/alignment-following.json"
CURRENT_PEER_BOOT_ID="$(ssh "$JETSON_HOST" 'cat /proc/sys/kernel/random/boot_id')"
if [[ "$CURRENT_PEER_BOOT_ID" != "$PEER_BOOT_ID" ]]; then
  echo 'ERROR: 启动期间Jetson重启，会话作废，禁止宣布遥操就绪。' >&2; exit 3
fi
printf '%s\n' "$PEER_BOOT_ID" >"$LOG_DIR/peer-boot-id"
printf '%s\n' "$RUN_STATUS" >"$LOG_DIR/run-status.json"
/usr/bin/python3 "$ROOT_DIR/scripts/watch_teleop_link.py" --jetson "$JETSON_HOST" \
  --owner-pid "$$" --output "$LOG_DIR/live-health.json" &
LINK_MONITOR_PID=$!
echo '  正在等待本会话的持续电机健康检查，尚不能宣布可遥操…'
LIVE_READY=false
for attempt in $(seq 1 20); do
  if ! kill -0 "$LINK_MONITOR_PID" 2>/dev/null; then break; fi
  RUN_STATUS="$(remote_control status 2>/dev/null || true)"
  if /usr/bin/python3 "$ROOT_DIR/scripts/check_remote_running_status.py" \
      --health-report "$LOG_DIR/live-health.json" <<<"$RUN_STATUS"; then
    LIVE_READY=true; break
  fi
  sleep 0.5
done
if [[ "$LIVE_READY" != true ]]; then
  echo 'ERROR: 持续电机健康检查未通过，不能确认可遥操；不自动重新使能。' >&2
  exit 3
fi
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

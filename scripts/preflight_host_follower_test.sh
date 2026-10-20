#!/usr/bin/env bash
# Saved 2026-10-20 diagnostic snapshot of the runtime-build checks.
# This file ends at verify_runtime_builds; it does not launch/homing/enable arms.
# It may repair remote Python runtime files using rsync, so it is not read-only.
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
FOLLOWER_RIGHT_CAN="${FOLLOWER_RIGHT_CAN:-can1}"
FOLLOWER_LEFT_CAN="${FOLLOWER_LEFT_CAN:-can2}"
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
    --follower-interfaces "$FOLLOWER_RIGHT_CAN" "$FOLLOWER_LEFT_CAN" \
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


verify_runtime_builds

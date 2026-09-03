#!/usr/bin/env bash
# Daily operator entrypoint: teleoperation only, never start RGB-D recording.
set -euo pipefail

ROOT_DIR="/home/openarm/dev/openarm-remote-harvest"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
PEER_IP="${PEER_IP:-192.168.50.2}"
HOST_CAN_SETUP="$ROOT_DIR/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan"
JETSON_CAN_SETUP="/home/nvidia/openarm_robot/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan"
START_LOCK="/tmp/openarm-daily-teleop.lock"

exec 9>"$START_LOCK"
if ! flock -n 9; then
  echo "ERROR: 已有一个 OpenArm 启动或遥操流程正在运行，请勿重复点击。" >&2
  exit 4
fi

say() { printf '\n=== %s ===\n' "$*"; }

ensure_host_can() {
  local iface
  if [[ ! -x "$HOST_CAN_SETUP" ]]; then
    echo "ERROR: 主机 CAN 配置程序不存在：$HOST_CAN_SETUP" >&2
    exit 1
  fi
  for iface in can0 can1; do
    if ! ip link show "$iface" 2>/dev/null | grep -q 'state UP'; then
      echo "主机 $iface 未启用，正在配置 CAN FD（可能要求输入 sudo 密码）…"
      "$HOST_CAN_SETUP" "$iface" -fd -b 1000000 -d 5000000
    fi
    ip -brief link show "$iface"
  done
}

ensure_jetson_can() {
  if ssh "$JETSON_HOST" "ip link show can1 2>/dev/null | grep -q 'state UP' && ip link show can2 2>/dev/null | grep -q 'state UP'"; then
    ssh "$JETSON_HOST" 'ip -brief link show can1; ip -brief link show can2'
    return
  fi
  echo "Jetson can1/can2 未启用，正在配置 CAN FD（可能要求输入 Jetson sudo 密码）…"
  ssh -tt "$JETSON_HOST" "$JETSON_CAN_SETUP can1 -fd -b 1000000 -d 5000000 && $JETSON_CAN_SETUP can2 -fd -b 1000000 -d 5000000"
}

ensure_no_recording() {
  # Teleop-only mode must not quietly append data to a prior episode.
  ssh "$JETSON_HOST" '/home/nvidia/miniconda3/envs/lerobot/bin/python - <<'"'"'PY'"'"'
import sys, zmq
try:
    c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 3000); s.connect("tcp://127.0.0.1:5557")
    s.send_json({"command":"status"}); reply=s.recv_json(); print(reply)
except Exception as exc:
    print(f"Jetson recorder unavailable; teleop-only mode will not record ({exc}).")
    sys.exit(0)
if reply.get("running"):
    print("ERROR: Jetson is recording. Stop and save it before starting teleop-only mode.", file=sys.stderr)
    sys.exit(3)
PY'
}

say "1/4 检查主机 CAN"
echo "  机械臂可能运动：否（本步骤只读取/配置主机 CAN）"
echo "  现在可以遥操：否"
echo "  你现在应当：保持机械臂静止"
ensure_host_can
say "2/4 检查主机与 Jetson 网络"
echo "  机械臂可能运动：否（本步骤只检查网线、IP 和 SSH）"
echo "  现在可以遥操：否"
echo "  你现在应当：无需操作，等待网络检查完成"
ping -c 2 -W 1 "$PEER_IP"
ssh -o ConnectTimeout=8 "$JETSON_HOST" 'hostname; uptime -p'
say "3/4 检查 Jetson 从臂 CAN"
echo "  机械臂可能运动：否（本步骤只读取/配置 Jetson CAN）"
echo "  现在可以遥操：否"
echo "  你现在应当：保持从臂周围无人和无障碍物"
ensure_jetson_can
say "4/4 确认未采集并启动受控遥操"
echo "  机械臂可能运动：即将会；下一阶段主从臂都会自动回初始位"
echo "  现在可以遥操：否"
echo "  你现在应当：两人就位、急停可用，然后等待终端明确显示可以遥操"
ensure_no_recording

echo "检查通过。无需按键，程序将自动归零、对齐并进入 RUNNING。"
exec bash "$ROOT_DIR/scripts/run_bimanual_remote_feedback.sh"

#!/usr/bin/env bash
# Daily operator entrypoint: teleoperation only, never start RGB-D recording.
set -euo pipefail

ROOT_DIR="/home/openarm/dev/openarm-remote-harvest"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
PEER_IP="${PEER_IP:-192.168.50.2}"
HOST_CAN_SETUP="$ROOT_DIR/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan"
JETSON_CAN_SETUP="/home/nvidia/openarm_robot/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan"

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
ensure_host_can
say "2/4 检查主机与 Jetson 网络"
ping -c 2 -W 1 "$PEER_IP"
ssh -o ConnectTimeout=8 "$JETSON_HOST" 'hostname; uptime -p'
say "3/4 检查 Jetson 从臂 CAN"
ensure_jetson_can
say "4/4 确认未采集并启动受控遥操"
ensure_no_recording

echo "通过检查。即将自动归零、对齐并进入 RUNNING；请确认两人就位、急停可用。"
exec bash "$ROOT_DIR/scripts/run_bimanual_remote_feedback.sh"

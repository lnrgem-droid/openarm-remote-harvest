#!/usr/bin/env bash
# Daily operator entrypoint: teleoperation + Jetson-local RGB-D recording.
set -euo pipefail

TELEOP_ROOT="/home/openarm/dev/openarm-remote-harvest"
RGBD_ROOT="/home/openarm/dev/openarm-rgbd-preview"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
PEER_IP="${PEER_IP:-192.168.50.2}"
LOG_DIR="/tmp/openarm-daily-start"
mkdir -p "$LOG_DIR"

say() { printf '\n=== %s ===\n' "$*"; }

ensure_host_can() {
  local iface
  for iface in can0 can1; do
    if ! ip link show "$iface" 2>/dev/null | grep -q 'state UP'; then
      echo "主机 $iface 未启用，正在配置 CAN FD（可能要求输入 sudo 密码）…"
      openarm-can-configure-socketcan "$iface" -fd -b 1000000 -d 5000000
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
  ssh -tt "$JETSON_HOST" 'openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000 && openarm-can-configure-socketcan can2 -fd -b 1000000 -d 5000000'
}

ensure_jetson_rgbd_services() {
  ssh "$JETSON_HOST" 'set -e
runtime=/home/nvidia/openarm-rgbd-runtime
root=/home/nvidia/dev/openarm-rgbd-preview
mkdir -p "$runtime"
alive() { test -f "$1" && kill -0 "$(cat "$1")" 2>/dev/null; }
if ! alive "$runtime/camera-service.pid"; then
  echo "启动 Jetson 三相机服务…"
  nohup bash "$root/scripts/start_jetson_rgbd_service.sh" >"$runtime/camera-service.log" 2>&1 & echo $! >"$runtime/camera-service.pid"
fi
if ! alive "$runtime/record-manager.pid"; then
  echo "启动 Jetson 录制管理服务…"
  nohup bash "$root/scripts/start_jetson_record_manager.sh" >"$runtime/record-manager.log" 2>&1 & echo $! >"$runtime/record-manager.pid"
fi
for n in $(seq 1 20); do
  test -S /tmp/openarm_rgbd_raw.ipc && break
  sleep 1
done
test -S /tmp/openarm_rgbd_raw.ipc'
}

start_recording() {
  ssh "$JETSON_HOST" '/home/nvidia/miniconda3/envs/lerobot/bin/python - <<'"'"'PY'"'"'
import sys, zmq
c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 5000); s.connect("tcp://127.0.0.1:5557")
s.send_json({"command":"status"}); status=s.recv_json()
if status.get("running"):
    print("ERROR: a recording is already running: " + str(status.get("dataset_root")), file=sys.stderr)
    sys.exit(3)
s.close(0); c.term()
c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 5000); s.connect("tcp://127.0.0.1:5557")
s.send_json({"command":"start"}); reply=s.recv_json(); print(reply)
if not reply.get("ok") or not reply.get("running"):
    sys.exit(4)
PY'
}

stop_recording_on_exit() {
  # Closing the preview window must not leave a hidden recording running.
  ssh "$JETSON_HOST" '/home/nvidia/miniconda3/envs/lerobot/bin/python - <<'"'"'PY'"'"' || true
import zmq
c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 2000); s.connect("tcp://127.0.0.1:5557")
s.send_json({"command":"status"}); status=s.recv_json()
if status.get("running"):
    s.close(0); c.term(); c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 2000); s.connect("tcp://127.0.0.1:5557"); s.send_json({"command":"stop"}); print(s.recv_json())
PY
}

say "1/5 检查主机 CAN 与网络"
ensure_host_can
ping -c 2 -W 1 "$PEER_IP"
ssh -o ConnectTimeout=8 "$JETSON_HOST" 'hostname; uptime -p'
say "2/5 检查 Jetson 从臂 CAN 与 RGB-D 服务"
ensure_jetson_can
ensure_jetson_rgbd_services
say "3/5 启动受控双臂遥操"
nohup bash "$TELEOP_ROOT/scripts/run_bimanual_remote_feedback.sh" >"$LOG_DIR/teleop.log" 2>&1 &
for n in $(seq 1 60); do
  status=$(ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install/setup.bash && source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/setup.bash && ros2 run remote_teleop_runtime remote-teleop-control status" 2>/dev/null || true)
  if grep -q '"state": "RUNNING"' <<<"$status"; then break; fi
  sleep 1
done
grep -q '"state": "RUNNING"' <<<"${status:-}" || { echo "ERROR: 遥操未进入 RUNNING。查看 $LOG_DIR/teleop.log" >&2; exit 1; }
say "4/5 开始 Jetson 本地 RGB-D 采集"
start_recording
say "5/5 打开主机实时预览"
echo "采集已开始。点击窗口红色“停止并保存”结束；关闭窗口也会请求停止本次录制。"
trap stop_recording_on_exit EXIT INT TERM
/home/openarm/miniconda3/bin/python "$RGBD_ROOT/scripts/rgb_preview_live.py" --jetson "$PEER_IP" --port 5556

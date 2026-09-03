#!/usr/bin/env bash
# Daily operator entrypoint: teleoperation + RGB-D preview.
# Recording is started only by the operator button in the preview window.
set -euo pipefail

TELEOP_ROOT="/home/openarm/dev/openarm-remote-harvest"
RGBD_ROOT="/home/openarm/dev/openarm-rgbd-preview"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
PEER_IP="${PEER_IP:-192.168.50.2}"
LOG_DIR="/tmp/openarm-daily-start"
HOST_CAN_SETUP="$TELEOP_ROOT/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan"
JETSON_CAN_SETUP="/home/nvidia/openarm_robot/ros2_robot/install/openarm_can/bin/openarm-can-configure-socketcan"
START_LOCK="/tmp/openarm-daily-teleop.lock"
TELEOP_CORE="$TELEOP_ROOT/scripts/run_bimanual_remote_feedback.sh"
mkdir -p "$LOG_DIR"

exec 9>"$START_LOCK"
if ! flock -n 9; then
  echo "ERROR: 已有一个 OpenArm 启动或遥操流程正在运行，请勿重复点击。" >&2
  exit 4
fi

say() { printf '\n=== %s ===\n' "$*"; }

teleop_status() {
  ssh "$JETSON_HOST" "source /opt/ros/humble/setup.bash && source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install/setup.bash && source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/setup.bash && ros2 run remote_teleop_runtime remote-teleop-control status" 2>/dev/null || true
}

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

ensure_jetson_rgbd_services() {
  ssh "$JETSON_HOST" 'set -e
runtime=/home/nvidia/openarm-rgbd-runtime
root=/home/nvidia/dev/openarm-rgbd-preview
mkdir -p "$runtime"
alive() { test -f "$1" && kill -0 "$(cat "$1")" 2>/dev/null; }
camera_pid=$(pgrep -f "^/home/nvidia/miniconda3/envs/lerobot/bin/python $root/scripts/jetson_orbbec_rgbd_service.py" | head -n1 || true)
if test -n "$camera_pid" && ! test -S /tmp/openarm_rgbd_raw.ipc; then
  echo "相机进程存在但本地采集 IPC 丢失，正在自动重启修复…"
  kill -TERM "$camera_pid" 2>/dev/null || true
  for n in $(seq 1 20); do
    kill -0 "$camera_pid" 2>/dev/null || break
    sleep 0.25
  done
  camera_pid=""
  rm -f "$runtime/camera-service.pid"
fi
if test -n "$camera_pid"; then
  echo "$camera_pid" >"$runtime/camera-service.pid"
elif ! alive "$runtime/camera-service.pid"; then
  echo "启动 Jetson 三相机服务…"
  nohup bash "$root/scripts/start_jetson_rgbd_service.sh" >"$runtime/camera-service.log" 2>&1 & echo $! >"$runtime/camera-service.pid"
fi
manager_pid=$(pgrep -f "^/home/nvidia/miniconda3/envs/lerobot/bin/python $root/scripts/jetson_record_manager.py" | head -n1 || true)
if test -n "$manager_pid"; then
  echo "$manager_pid" >"$runtime/record-manager.pid"
elif ! alive "$runtime/record-manager.pid"; then
  echo "启动 Jetson 录制管理服务…"
  nohup bash "$root/scripts/start_jetson_record_manager.sh" >"$runtime/record-manager.log" 2>&1 & echo $! >"$runtime/record-manager.pid"
fi
# Enforce the data-plane CPU partition even when services were started by an
# older launcher or a manual command. Camera/recording work must never run on
# CPUs 0-2 reserved for follower safety and CAN control.
for pid_file in "$runtime/camera-service.pid" "$runtime/record-manager.pid"; do
  if alive "$pid_file"; then
    taskset --all-tasks --pid --cpu-list 3-7 "$(cat "$pid_file")" >/dev/null
  fi
done
bridge_pids=$(pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch robot_bridge|^/usr/bin/python3 /home/nvidia/dev/openarm-rgbd-preview/ros2_robot/install/robot_bridge/.*/bridge_node" || true)
for bridge_pid in $bridge_pids; do
  taskset --all-tasks --pid --cpu-list 3-7 "$bridge_pid" >/dev/null 2>&1 || true
  renice -n 2 -p "$bridge_pid" >/dev/null 2>&1 || true
done
for n in $(seq 1 20); do
  test -S /tmp/openarm_rgbd_raw.ipc && break
  sleep 1
done
test -S /tmp/openarm_rgbd_raw.ipc'
}

stop_recording_on_exit() {
  # Closing the preview window must not leave a hidden recording running.
  ssh "$JETSON_HOST" '/home/nvidia/miniconda3/envs/lerobot/bin/python - <<'"'"'PY'"'"' || true
import zmq
c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 2000); s.connect("tcp://127.0.0.1:5557")
s.send_json({"command":"status"}); status=s.recv_json()
if status.get("running"):
    s.close(0); c.term(); c=zmq.Context(); s=c.socket(zmq.REQ); s.setsockopt(zmq.RCVTIMEO, 2000); s.connect("tcp://127.0.0.1:5557"); s.send_json({"command":"stop"}); print(s.recv_json())
PY'
}

ensure_recording_idle() {
  # A previous UI crash must not make a newly opened window look as if it
  # started recording by itself.  Finish any orphaned recorder first.
  ssh "$JETSON_HOST" '/home/nvidia/miniconda3/envs/lerobot/bin/python - <<'"'"'PY'"'"'
import time, zmq
endpoint = "tcp://127.0.0.1:5557"
def request(command):
    context = zmq.Context(); socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.RCVTIMEO, 3000)
    socket.connect(endpoint); socket.send_json({"command": command})
    response = socket.recv_json(); socket.close(0); context.term(); return response
status = request("status")
if status.get("running"):
    print("发现上次遗留录制，正在停止并保存：" + str(status.get("dataset_root")))
    request("stop")
    for _ in range(12):
        time.sleep(1); status = request("status")
        if not status.get("running"):
            break
if status.get("running"):
    raise SystemExit("ERROR: 遗留录制未能停止，请查看 Jetson 录制日志")
print("录制状态：未录制（等待界面按钮）")
PY'
}

say "1/5 检查主机 CAN 与网络"
echo "  机械臂可能运动：否（本步骤只检查主机 CAN、网线、IP 和 SSH）"
echo "  现在可以遥操：否"
echo "  你现在应当：保持机械臂静止，等待检查完成"
ensure_host_can
ping -c 2 -W 1 "$PEER_IP"
ssh -o ConnectTimeout=8 "$JETSON_HOST" 'hostname; uptime -p'
say "2/5 检查 Jetson 从臂 CAN 与 RGB-D 服务"
echo "  机械臂可能运动：否（本步骤只检查 CAN、相机和录制服务）"
echo "  现在可以遥操：否"
echo "  你现在应当：保持从臂周围无人和无障碍物"
ensure_jetson_can
ensure_jetson_rgbd_services
say "3/5 启动受控双臂遥操"
echo "  机械臂可能运动：是；主从左右臂将依次自动回到初始位"
echo "  现在可以遥操：否"
echo "  你现在应当：不要触碰机械臂，无需按键，等待终端显示 RUNNING"
[[ -f "$TELEOP_CORE" ]] || { echo "ERROR: 遥操核心脚本不存在：$TELEOP_CORE" >&2; exit 1; }
echo "使用与“启动主从遥操”完全相同的遥操核心：$TELEOP_CORE"
# The detached teleop process must not inherit descriptor 9. Otherwise it keeps
# the daily-start lock forever after this UI/recording wrapper exits.
nohup bash "$TELEOP_CORE" >"$LOG_DIR/teleop.log" 2>&1 9>&- &
teleop_pid=$!
progress_count=0
for n in $(seq 1 60); do
  status=$(teleop_status)
  mapfile -t progress_lines < <(grep -E '^\[[0-6]/6\]' "$LOG_DIR/teleop.log" 2>/dev/null || true)
  while (( progress_count < ${#progress_lines[@]} )); do
    echo "  遥操启动进度：${progress_lines[$progress_count]}"
    progress_count=$((progress_count + 1))
  done
  if grep -q '"state": "RUNNING"' <<<"$status"; then break; fi
  if ! kill -0 "$teleop_pid" 2>/dev/null; then
    echo "ERROR: 遥操启动脚本已退出，未进入 RUNNING。最后日志如下：" >&2
    tail -n 30 "$LOG_DIR/teleop.log" >&2 || true
    exit 2
  fi
  sleep 1
done
grep -q '"state": "RUNNING"' <<<"${status:-}" || { echo "ERROR: 遥操未在 60 秒内进入 RUNNING。查看 $LOG_DIR/teleop.log" >&2; exit 1; }
echo "  机械臂可能运动：是；从臂会跟随主臂"
echo "  现在可以遥操：是"
echo "  遥操状态：RUNNING，左右臂开始一一对应跟随。"
say "4/5 准备 Jetson 本地 RGB-D 采集"
echo "  机械臂可能运动：是（遥操继续运行，本步骤不会额外驱动机械臂）"
echo "  现在可以遥操：是"
echo "  你现在应当：等待确认录制状态；此时不会自动开始录制"
ensure_recording_idle
trap stop_recording_on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
say "5/5 打开主机实时预览"
echo "  机械臂可能运动：是（遥操继续运行）"
echo "  现在可以遥操：是"
echo "当前只打开实时预览，不会自动录制。"
echo "点击绿色“开始本地录制”才开始；点击红色“停止并保存”一次即可结束。"
echo "停止录制只停止数据保存，不会停止机械臂遥操。"
echo "相机预览和本地录制独立运行；遥操故障不会再关闭采图窗口。"
QT_QPA_FONTDIR=/usr/share/fonts/truetype/dejavu \
  /home/openarm/miniconda3/bin/python "$RGBD_ROOT/scripts/rgb_preview_live.py" \
    --jetson "$PEER_IP" --port 5556 \
    2> >(grep -v -E '^(QFontDatabase: Cannot find font directory|Note that Qt no longer ships fonts)' >&2)

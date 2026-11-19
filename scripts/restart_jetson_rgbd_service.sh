#!/usr/bin/env bash
# Orbbec USB devices need a real release interval after a Pipeline exits.
# A fast kill/start can leave a Gemini 305 enumerated but producing no frames.
set -euo pipefail

ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
RUNTIME_DIR="${OPENARM_RGBD_RUNTIME:-/home/nvidia/openarm-rgbd-runtime}"
PID_FILE="$RUNTIME_DIR/camera-service.pid"
LOG_FILE="$RUNTIME_DIR/camera-service.log"
mkdir -p "$RUNTIME_DIR"

# Do not interrupt a recording or kill an unrelated process after PID reuse.
python3 - "$PID_FILE" <<'PY'
import json, sys
from pathlib import Path
import zmq
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.setsockopt(zmq.RCVTIMEO, 4000)
socket.setsockopt(zmq.SNDTIMEO, 4000)
try:
    socket.connect("tcp://127.0.0.1:5557")
    socket.send_json({"command": "status"})
    status = socket.recv_json()
    if (status.get("ok") is not True or status.get("phase") != "idle"
            or status.get("running") or status.get("active_episode")
            or Path("/tmp/openarm-rgbd-recording.active").exists()):
        raise SystemExit("camera restart refused: recorder is not confirmed idle")
finally:
    socket.close(0)
    context.term()
pid_file = Path(sys.argv[1])
if pid_file.exists():
    pid = pid_file.read_text().strip()
    if not pid.isdecimal():
        raise SystemExit("camera restart refused: invalid PID file")
    proc = Path("/proc") / pid / "cmdline"
    if proc.exists() and b"jetson_orbbec_rgbd_service.py" not in proc.read_bytes():
        raise SystemExit("camera restart refused: PID belongs to a different process")
PY

if [[ -f "$PID_FILE" ]]; then
  old_pid=$(<"$PID_FILE")
  if kill -0 "$old_pid" 2>/dev/null; then
    kill "$old_pid"
    for _ in $(seq 1 50); do
      kill -0 "$old_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$old_pid" 2>/dev/null; then
      echo "camera process did not exit; refusing to start another owner" >&2
      exit 1
    fi
  fi
fi

# Required by the observed 305/335 SDK release behaviour on this Jetson.
sleep 12
nohup "$ROOT_DIR/scripts/start_jetson_rgbd_service.sh" >"$LOG_FILE" 2>&1 < /dev/null &
echo $! >"$PID_FILE"
echo "camera service started: pid $(<"$PID_FILE"); wait 12 s then inspect $LOG_FILE"

#!/usr/bin/env bash
# Orbbec USB devices need a real release interval after a Pipeline exits.
# A fast kill/start can leave a Gemini 305 enumerated but producing no frames.
set -euo pipefail

ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
RUNTIME_DIR="${OPENARM_RGBD_RUNTIME:-/home/nvidia/openarm-rgbd-runtime}"
PID_FILE="$RUNTIME_DIR/camera-service.pid"
LOG_FILE="$RUNTIME_DIR/camera-service.log"
mkdir -p "$RUNTIME_DIR"

if [[ -f "$PID_FILE" ]]; then
  old_pid=$(<"$PID_FILE")
  if kill -0 "$old_pid" 2>/dev/null; then
    kill "$old_pid"
    for _ in $(seq 1 50); do
      kill -0 "$old_pid" 2>/dev/null || break
      sleep 0.1
    done
  fi
fi

# Required by the observed 305/335 SDK release behaviour on this Jetson.
sleep 12
nohup "$ROOT_DIR/scripts/start_jetson_rgbd_service.sh" >"$LOG_FILE" 2>&1 < /dev/null &
echo $! >"$PID_FILE"
echo "camera service started: pid $(<"$PID_FILE"); wait 12 s then inspect $LOG_FILE"

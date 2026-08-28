#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
exec /home/nvidia/miniconda3/envs/lerobot/bin/python "$ROOT_DIR/scripts/jetson_orbbec_rgbd_service.py" \
  --config "$ROOT_DIR/config/orbbec_rgbd_jetson.yaml" --ipc ipc:///tmp/openarm_rgbd_raw.ipc --preview-port 5556

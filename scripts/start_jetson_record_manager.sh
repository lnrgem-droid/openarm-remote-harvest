#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
RGBD_CPUSET="${RGBD_CPUSET:-3-7}"
exec taskset --cpu-list "$RGBD_CPUSET" \
  /home/nvidia/miniconda3/envs/lerobot/bin/python "$ROOT_DIR/scripts/jetson_record_manager.py" --root "$ROOT_DIR"

#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="${OPENARM_RGBD_ROOT:-/home/openarm/dev/openarm-rgbd-preview}"
exec /home/openarm/miniconda3/bin/python "$ROOT_DIR/scripts/rgb_preview_live.py" --jetson "${JETSON_IP:-192.168.50.2}" --port 5556

#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ROOT_DIR/scripts/rgb_preview_rerun.py" --jetson "${JETSON_IP:-192.168.50.2}" --port 5556

#!/usr/bin/env bash
# Temporary follower/camera host for the laptop test. Starts no arm controller.
set -euo pipefail

RUNTIME=/home/openarm/.local/state/openarm-rgbd-host
DATASETS=/home/openarm/datasets
mkdir -p "$RUNTIME" "$DATASETS"

echo '启动旧主机三相机 RGB-D 服务（不控制机械臂）…'
systemctl --user start openarm-host-camera.service

healthy=false
for attempt in $(seq 1 15); do
  if /usr/bin/python3 - <<'PY'
import json, time
from pathlib import Path
p = Path('/tmp/openarm-rgbd-camera-status.json')
try:
    report = json.loads(p.read_text())
    assert time.time() - report['updated_unix_s'] < 4
    assert all(report['cameras'][role]['healthy'] for role in ('left_wrist', 'right_wrist', 'chest'))
except (OSError, ValueError, KeyError, AssertionError):
    raise SystemExit(1)
PY
  then
    healthy=true
    break
  fi
  sleep 2
done
if [[ "$healthy" != true ]]; then
  echo 'ERROR: 三相机未同时健康；旧主机暂不能采集。当前状态：' >&2
  cat /tmp/openarm-rgbd-camera-status.json 2>/dev/null >&2 || true
  tail -10 "$RUNTIME/camera.log" 2>/dev/null >&2 || true
  exit 2
fi

echo '启动旧主机录制管理服务…'
systemctl --user start openarm-host-record-manager.service
echo '启动只读从臂录制桥（不控制机械臂）…'
systemctl --user start openarm-host-record-bridge.service
for attempt in $(seq 1 15); do
  if ss -ltn '( sport = :5557 )' | grep -q LISTEN &&
     ss -ltn '( sport = :9000 )' | grep -q LISTEN; then
    echo '旧主机三相机、录制管理、只读状态桥均已就绪。'
    exit 0
  fi
  sleep 1
done
echo 'ERROR: 录制管理或只读状态桥未启动。' >&2
tail -n 20 "$RUNTIME/record-manager.log" "$RUNTIME/record-bridge.log" >&2 || true
exit 2

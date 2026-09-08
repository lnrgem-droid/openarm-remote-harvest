#!/usr/bin/env bash
# New desktop launcher for the task-specific mushroom collection console.
# It deliberately delegates all CAN/network/teleop startup to the established
# daily launcher; only the host UI is different.
set -euo pipefail

TELEOP_ROOT="/home/openarm/dev/openarm-remote-harvest"
CONSOLE="/home/openarm/dev/openarm-rgbd-preview/scripts/mushroom_collection_console.py"
LOG_FILE="/tmp/openarm-mushroom-collection.log"

[[ -f "$CONSOLE" ]] || { echo "ERROR: 新采集界面不存在：$CONSOLE" >&2; exit 1; }

exec gnome-terminal --title="OpenArm｜蘑菇采集控制台" -- bash -lc '
set -o pipefail
export OPENARM_COLLECTION_CONSOLE="/home/openarm/dev/openarm-rgbd-preview/scripts/mushroom_collection_console.py"
export OPENARM_COLLECTION_PYTHON="/usr/bin/python3"
bash /home/openarm/dev/openarm-remote-harvest/scripts/daily_start_teleop_rgbd.sh 2>&1 | tee /tmp/openarm-mushroom-collection.log
status=${PIPESTATUS[0]}
printf "\\n执行结束，退出码：%s。日志：/tmp/openarm-mushroom-collection.log\\n" "$status"
read -r -p "按 Enter 关闭此窗口…"
exit "$status"
'

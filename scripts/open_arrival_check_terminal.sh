#!/usr/bin/env bash
# Desktop wrapper: keep the read-only arrival-check result visible.
set -euo pipefail
LOG_FILE=/tmp/openarm-arrival-check.log
printf '%s arrival check launcher invoked\n' "$(date --iso-8601=seconds)" >> "$LOG_FILE"
exec gnome-terminal --title="OpenArm｜到场设备检查" -- bash -lc '
set -o pipefail
bash /home/openarm/dev/openarm-remote-harvest/scripts/openarm_arrival_check.sh 2>&1 | tee /tmp/openarm-arrival-check.log
status=${PIPESTATUS[0]}
printf "\n检查结束，退出码：%s。日志：/tmp/openarm-arrival-check.log\n" "$status"
read -r -p "按 Enter 关闭此窗口…"
exit "$status"
'

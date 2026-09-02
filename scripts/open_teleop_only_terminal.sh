#!/usr/bin/env bash
# Desktop launcher wrapper. Keep a visible terminal on both success and error.
set -euo pipefail
LOG_FILE=/tmp/openarm-daily-teleop-only.log
# Record that the desktop launcher itself was reached before requesting a
# terminal. This makes a post-reboot double-click failure diagnosable even if
# GNOME Terminal cannot create a window.
printf '%s desktop launcher invoked\n' "$(date --iso-8601=seconds)" >> "$LOG_FILE"
exec gnome-terminal --title="OpenArm｜主从遥操（不采集）" -- bash -lc '
set -o pipefail
bash /home/openarm/dev/openarm-remote-harvest/scripts/daily_start_teleop_only.sh 2>&1 | tee /tmp/openarm-daily-teleop-only.log
status=${PIPESTATUS[0]}
printf "\\n执行结束，退出码：%s。日志：/tmp/openarm-daily-teleop-only.log\\n" "$status"
read -r -p "按 Enter 关闭此窗口…"
exit "$status"
'

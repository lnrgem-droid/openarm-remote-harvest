#!/usr/bin/env bash
set -euo pipefail
exec gnome-terminal --title='OpenArm 临时主从采集测试' -- bash -lc '
set -o pipefail
bash /home/openarm/openarm-transfer/start-laptop-with-host-follower.sh 2>&1 | tee /tmp/openarm-host-follower-test.log
status=${PIPESTATUS[0]}
printf "\n执行结束，退出码：%s。日志：/tmp/openarm-host-follower-test.log\n" "$status"
read -r -p "按 Enter 关闭此窗口…" _
exit "$status"
'

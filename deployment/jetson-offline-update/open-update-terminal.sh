#!/usr/bin/env bash
set -euo pipefail
exec gnome-terminal --title="Jetson 离线更新" -- bash -c '
set -o pipefail
root=/home/openarm/openarm-transfer/jetson-update-20261020
/usr/bin/python3 -u "$root/update-jetson.py" 2>&1 | tee -a "$root/update-laptop.log"
result=${PIPESTATUS[0]}
if [ "$result" -eq 0 ]; then
  echo "软件更新成功。下一步：打开采集控制台，验证三相机和短录制。"
else
  echo "本次未确认更新成功，请查看上方原因。日志：$root/update-laptop.log"
fi
read -r -p "按 Enter 关闭此窗口…"
exit "$result"
'

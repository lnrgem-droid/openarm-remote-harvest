#!/usr/bin/env bash
set -u
pkexec /bin/bash /home/openarm/configure-can-today.sh > /home/openarm/configure-can-today.log 2>&1
result=$?
cat /home/openarm/configure-can-today.log
if (( result == 0 )); then
  echo 'CAN 接口配置完成。可以关闭这个窗口。'
else
  echo "CAN 接口配置失败（退出码 $result）。请保留窗口并告知 Codex。"
fi
read -r -p '按回车关闭窗口...' _
exit "$result"

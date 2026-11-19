#!/usr/bin/env bash
set -eo pipefail
LEFT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
role="${1:?leader or follower}"; bus="${2:?physical left CAN interface}"
case "$role" in leader|follower) ;; *) exit 64 ;; esac
exec 8>/tmp/openarm-left-controller.lock
flock -n 8 || { echo '单左臂控制器已存在'; exit 1; }
# Shares the original desktop launch lock: do not race the bimanual launcher.
exec 9>/tmp/openarm-daily-teleop.lock
flock -n 9 || { echo '另一个启动流程仍占用机械臂'; exit 1; }
/usr/bin/python3 "$LEFT_ROOT/scripts/verify_left_release.py"
/usr/bin/python3 "$LEFT_ROOT/scripts/left_only_device.py" --role "$role" --bus "$bus"
mkdir -p "$HOME/.local/state/openarm-left-only"
exec >"$HOME/.local/state/openarm-left-only/$role.log" 2>&1
source "$LEFT_ROOT/scripts/left_only_env.sh"
export OPENARM_LEFT_STARTUP_POLICY="$LEFT_ROOT/scripts/left_startup_alignment_policy.json"
exec ros2 launch remote_teleop_runtime "left_only_${role}.launch.py" left_can:="$bus" peer:="${3:-192.168.50.2}"

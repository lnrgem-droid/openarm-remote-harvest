#!/usr/bin/env bash
# Terminal entry point for this laptop + Jetson, using the installed network
# equivalent of launch_bimanual_bilateral.sh. Run this file ON THE LAPTOP.
# Dependencies live in /home/openarm/dev/openarm-remote-harvest on both peers
# (Jetson home is /home/nvidia). This is not a portable standalone controller.
set -Eeuo pipefail

ROOT=/home/openarm/dev/openarm-remote-harvest
export OPENARM_TELEOP_PROFILE=jetson
export JETSON_HOST=openarm-jetson
export JETSON_ROOT=/home/nvidia/dev/openarm-remote-harvest
export PEER_IP=192.168.50.2
export FOLLOWER_RIGHT_CAN=can1 FOLLOWER_LEFT_CAN=can2
export FORCE_FEEDBACK=true TRACKING_CANDIDATE=true GRIPPER_CONTACT_FEEDBACK=true
export LOG_DIR=/tmp/openarm-remote-teleop
export OPENARM_NETWORK_ACTION_TIMEOUT_MS=150
export JETSON_CONTROL_CPUSET=0-2

usage() {
  cat <<'EOF'
用法：bash launch_bimanual_remote.sh [start|--check|status|hold]
  start     默认：检查连接与旧进程，配置 CAN，归位、对齐并启动双臂主从。
  --check   只检查 SSH、接口、控制器依赖和旧进程；不配置 CAN、不使能。
  status    读取 Jetson 当前遥操状态。
  hold      请求从臂位置保持，暂停跟随；不是失能或断电。

固定接线：笔记本右主臂 can0、左主臂 can1；Jetson 右从臂 can1、左从臂 can2。
SSH：openarm-jetson (nvidia@192.168.50.2)。实时主从数据由现有网络程序传输。
启动会使四臂自动回初始位；请预先清空运动范围、松开主臂并准备好急停。
等待“现在可以遥操：是”再操作。Ctrl+C 请求从臂保持，不代表所有电机已失能。
本入口只接受无旧控制进程的启动；已有会话请先通过原控制台正常停止。
不会启动相机或录制。依赖本机与 Jetson 已安装的 openarm-remote-harvest。
EOF
}

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
ssh_peer() {
  command ssh -o BatchMode=yes -o ConnectTimeout=5 \
    -o ServerAliveInterval=2 -o ServerAliveCountMax=2 "$JETSON_HOST" "$@"
}

[[ $# -le 1 ]] || { usage >&2; exit 64; }
action=${1:-start}
case "$action" in
  -h|--help) usage; exit 0 ;;
  start|--check|status|hold) ;;
  *) usage >&2; exit 64 ;;
esac
[[ -r "$ROOT/scripts/run_bimanual_remote_feedback.sh" ]] || fail "找不到跨电脑控制程序：$ROOT"
if [[ "$action" == status ]]; then
  exec /usr/bin/python3 "$ROOT/scripts/read_remote_teleop_status.py" --peer "$JETSON_HOST"
fi
if [[ "$action" == hold ]]; then
  ssh_peer 'set -e
source /opt/ros/humble/setup.bash
source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install/setup.bash
source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/setup.bash
exec ros2 run remote_teleop_runtime remote-teleop-control hold'
  exit $?
fi

# Match the existing desktop startup lock. Keep it across exec into the core.
if [[ "$action" == start ]]; then
  exec 9>/tmp/openarm-daily-teleop.lock
  flock -n 9 || fail '已有启动或遥操流程，不能重复启动。'
fi

printf '主臂：本机 can0/can1；从臂：%s can1/can2\n' "$JETSON_HOST"
# The same read-only inspection runs on each host. Do not infer bus identity
# just from canN; verify the dual-channel USB adapter before any later repair.
endpoint_check=$(cat <<'PY'
import json, pathlib, subprocess, sys
buses = sys.argv[1:]
names = {'openarm_gravity_pd_node', 'remote-teleop-leader',
         'remote-teleop-follower', 'remote-teleop-follower-watchdog',
         'bilateral_control', 'unilateral_control', 'gravity_comp',
         'bimanual_leader.launch.py', 'bimanual_follower.launch.py'}
busy = []
for file in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
    try:
        args = file.read_bytes().decode(errors='replace').split('\0')
    except FileNotFoundError:
        continue
    if any(pathlib.Path(a).name in names for a in args if a):
        busy.append(file.parent.name)
if busy or pathlib.Path('/tmp/openarm_remote_runtime.sock').exists():
    raise SystemExit('已有控制进程或旧遥操 socket；请先在原控制台正常停止。PID=' + ','.join(busy))
devices = []
for bus in buses:
    usb = (pathlib.Path('/sys/class/net') / bus / 'device').resolve().parent
    if ((usb / 'idVendor').read_text().strip(), (usb / 'idProduct').read_text().strip()) != ('0c72', '0011'):
        raise SystemExit(bus + ' 不是预期的双通道 CAN 盒，停止检查')
    devices.append(usb)
    link = json.loads(subprocess.check_output(['ip', '-j', '-d', 'link', 'show', bus]))[0]
    data = link['linkinfo']['info_data']
    if 'UP' not in link['flags'] and data['state'] == 'STOPPED':
        print(bus + ': DOWN；start 时配置为 CAN-FD 1M/5M')
    elif ('UP' in link['flags'] and data['state'] == 'ERROR-ACTIVE'
          and 'FD' in data.get('ctrlmode', [])
          and data.get('bittiming', {}).get('bitrate') == 1000000
          and data.get('data_bittiming', {}).get('bitrate') == 5000000):
        print(bus + ': CAN-FD 1M/5M 正常')
    else:
        raise SystemExit(bus + ': 总线故障或参数不匹配；不会自动重置')
if len(set(devices)) != 1:
    raise SystemExit('两路 CAN 不属于同一只双通道盒')
PY
)
/usr/bin/python3 - can0 can1 <<<"$endpoint_check"
ssh_peer '/usr/bin/python3 - can1 can2' <<<"$endpoint_check"

node="$ROOT/ros2_robot/install_gripper_spring_candidate/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
[[ -x "$node" ]] || fail "主臂控制器不存在：$node"
dependencies=$(
  set +u
  source /opt/ros/humble/setup.bash
  source "$ROOT/ros2_robot/install/setup.bash"
  source "$ROOT/ros2_robot/install_bimanual/setup.bash"
  source "$ROOT/ros2_robot/install_gripper_candidate/setup.bash"
  source "$ROOT/ros2_robot/install_gripper_spring_candidate/local_setup.bash"
  ldd "$node"
)
if [[ "$dependencies" == *'not found'* ]]; then fail "主臂控制器缺少动态库：$dependencies"; fi
ssh_peer 'bash -s -- /home/nvidia/dev/openarm-remote-harvest install_gripper_spring_candidate true 0-2 150' \
  <"$ROOT/scripts/check_remote_startup_build.sh"
if [[ "$action" == --check ]]; then
  echo '只读检查通过。尚未检查电机供电、反馈和归位；没有启动机械臂。'
  exit 0
fi

# Refuse an active recording without requiring camera/recorder startup.
ssh_peer '/home/nvidia/miniconda3/envs/lerobot/bin/python -' <<'PY'
import sys, zmq
with zmq.Context() as context:
    with context.socket(zmq.REQ) as client:
        client.setsockopt(zmq.LINGER, 0)
        client.setsockopt(zmq.SNDTIMEO, 2000)
        client.setsockopt(zmq.RCVTIMEO, 2000)
        client.connect('tcp://127.0.0.1:5557')
        try:
            client.send_json({'command': 'status'})
            reply = client.recv_json()
        except zmq.Again:
            print('录制服务未响应；本脚本不启动录制。')
        else:
            if reply.get('running') is not False or reply.get('active_episode') is not None:
                sys.exit('录制尚未空闲，或状态无法确认；先停止并保存录制。')
PY

echo '检查并配置未启用的 CAN 接口；只配置已核验的双通道盒。'
/usr/bin/python3 - "$ROOT/scripts" <<'PY'
import sys
sys.path.insert(0, sys.argv[1])
from inspect_teleop_recovery import connection_config
from preflight_motor_feedback import verify
try:
    verify(connection_config(), repair=True)
except Exception as exc:
    sys.exit(str(exc))
PY
echo '即将启动：四臂自动归位、对齐，完成后进入带力反馈的主从跟随。'
echo '日志：/tmp/openarm-remote-teleop/leader.log；Jetson /tmp/openarm_bimanual_follower.log'
# The core checks fresh feedback from all 32 disabled motors before enabling,
# then performs homing, alignment, network watchdog and readiness checks.
exec bash "$ROOT/scripts/run_bimanual_remote_feedback.sh"

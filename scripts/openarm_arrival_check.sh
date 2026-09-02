#!/usr/bin/env bash
# Read-only preflight for the transported OpenArm host + Jetson system.
set -u

ROOT_DIR="/home/openarm/dev/openarm-remote-harvest"
JETSON_HOST="${JETSON_HOST:-openarm-jetson}"
JETSON_IP="${JETSON_IP:-192.168.50.2}"
HOST_WIRED_IFACE="${HOST_WIRED_IFACE:-eno1}"
JETSON_WIRED_IFACE="${JETSON_WIRED_IFACE:-eno1}"
JETSON_ROOT="${JETSON_ROOT:-/home/nvidia/dev/openarm-remote-harvest}"
JETSON_RGBD_ROOT="${JETSON_RGBD_ROOT:-/home/nvidia/dev/openarm-rgbd-preview}"
JETSON_PYTHON="/home/nvidia/miniconda3/envs/lerobot/bin/python"
EXPECTED_CAMERAS=(CV2L360000D9 CV2L360000NR CP3294Y0001E)
PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() { printf '  [通过] %s\n' "$*"; PASS_COUNT=$((PASS_COUNT + 1)); }
warn() { printf '  [警告] %s\n' "$*"; WARN_COUNT=$((WARN_COUNT + 1)); }
fail() { printf '  [失败] %s\n' "$*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
section() { printf '\n=== %s ===\n' "$*"; }

check_can_local() {
  local iface="$1" detail
  detail="$(ip -details link show "$iface" 2>/dev/null || true)"
  if [[ -z "$detail" ]]; then
    fail "主机不存在 $iface"
  elif [[ "$detail" != *"state UP"* ]]; then
    fail "主机 $iface 未启用"
  elif [[ "$detail" == *"state BUS-OFF"* ]]; then
    fail "主机 $iface 为 BUS-OFF"
  elif [[ "$detail" != *"<FD>"* ]]; then
    fail "主机 $iface 未配置为 CAN FD"
  else
    pass "主机 $iface 已启用，CAN FD 状态正常"
  fi
}

check_can_remote() {
  local iface="$1" detail
  detail="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_HOST" \
    "ip -details link show '$iface'" 2>/dev/null || true)"
  if [[ -z "$detail" ]]; then
    fail "Jetson 不存在或无法读取 $iface"
  elif [[ "$detail" != *"state UP"* ]]; then
    fail "Jetson $iface 未启用"
  elif [[ "$detail" == *"state BUS-OFF"* ]]; then
    fail "Jetson $iface 为 BUS-OFF"
  elif [[ "$detail" != *"<FD>"* ]]; then
    fail "Jetson $iface 未配置为 CAN FD"
  else
    pass "Jetson $iface 已启用，CAN FD 状态正常"
  fi
}

section "1/6 有线网络与 SSH"
host_ip="$(ip -4 -o address show dev "$HOST_WIRED_IFACE" 2>/dev/null | awk '{print $4}' | head -1)"
if [[ "$host_ip" == "192.168.50.1/24" ]]; then
  pass "主机 $HOST_WIRED_IFACE 固定地址为 $host_ip"
else
  fail "主机 $HOST_WIRED_IFACE 地址异常：${host_ip:-未配置}，应为 192.168.50.1/24"
fi

if ping -c 2 -W 1 "$JETSON_IP" >/dev/null 2>&1; then
  pass "主机与 Jetson 网线连通，地址 $JETSON_IP"
else
  fail "无法 ping 通 $JETSON_IP；检查网线和两端 eno1"
fi

if ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_HOST" true 2>/dev/null; then
  pass "SSH 别名 $JETSON_HOST 可免密连接"
  jetson_ip="$(ssh -o BatchMode=yes "$JETSON_HOST" \
    "ip -4 -o address show dev '$JETSON_WIRED_IFACE' 2>/dev/null | tr -s ' ' | cut -d' ' -f4 | head -1" 2>/dev/null)"
  if [[ "$jetson_ip" == "192.168.50.2/24" ]]; then
    pass "Jetson $JETSON_WIRED_IFACE 固定地址为 $jetson_ip"
  else
    fail "Jetson $JETSON_WIRED_IFACE 地址异常：${jetson_ip:-未配置}，应为 192.168.50.2/24"
  fi
else
  fail "SSH 无法连接 $JETSON_HOST；确认网线、IP 和 SSH 密钥"
fi

section "2/6 主机 CAN"
check_can_local can0
check_can_local can1

section "3/6 Jetson CAN"
if ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_HOST" true 2>/dev/null; then
  check_can_remote can1
  check_can_remote can2
else
  fail "SSH 不通，无法检查 Jetson can1/can2"
fi

section "4/6 三台 Orbbec 相机"
camera_check="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_HOST" \
  "$JETSON_PYTHON -" 2>/dev/null <<'PY'
from pathlib import Path
import os
import time

pid_path = Path("/home/nvidia/openarm-rgbd-runtime/camera-service.pid")
log_path = Path("/home/nvidia/openarm-rgbd-runtime/camera-service.log")
if pid_path.exists():
    try:
        os.kill(int(pid_path.read_text().strip()), 0)
        last_line = log_path.read_text(errors="replace").splitlines()[-1]
        fresh = time.time() - log_path.stat().st_mtime <= 6.0
        roles = ("left_wrist", "right_wrist", "chest")
        if fresh and all(role in last_line for role in roles) and last_line.count("'healthy': True") >= 3:
            print("SERVICE_HEALTHY")
            raise SystemExit(0)
    except (OSError, ValueError, IndexError):
        pass

from pyorbbecsdk import Context

devices = Context().query_devices()
serials = []
for index in range(devices.get_count()):
    serials.append(devices.get_device_by_index(index).get_device_info().get_serial_number())
print("\n".join(serials))
PY
)"
if [[ "$camera_check" == *"SERVICE_HEALTHY"* ]]; then
  pass "三路相机服务正在输出实时 RGB-D，三路均健康"
  pass "左腕相机 CV2L360000D9 角色配置正确"
  pass "右腕相机 CV2L360000NR 角色配置正确"
  pass "胸部相机 CP3294Y0001E 角色配置正确"
elif [[ -z "$camera_check" ]]; then
  fail "Jetson 未枚举到 Orbbec 相机，或相机 SDK 无法运行"
else
  for serial in "${EXPECTED_CAMERAS[@]}"; do
    case "$camera_check" in
      *"$serial"*) pass "已识别相机 $serial" ;;
      *) fail "缺少相机 $serial" ;;
    esac
  done
fi

section "5/6 Jetson 磁盘与录制状态"
available_bytes="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_HOST" \
  "df -B1 --output=avail /home/nvidia/datasets 2>/dev/null | tail -1 | tr -d ' '" 2>/dev/null)"
if [[ "$available_bytes" =~ ^[0-9]+$ ]]; then
  available_gib=$((available_bytes / 1024 / 1024 / 1024))
  if (( available_gib >= 20 )); then
    pass "Jetson 数据盘剩余约 ${available_gib} GiB"
  elif (( available_gib >= 10 )); then
    warn "Jetson 数据盘仅剩约 ${available_gib} GiB，不应开始新的长时间录制"
  else
    fail "Jetson 数据盘仅剩约 ${available_gib} GiB"
  fi
else
  fail "无法读取 Jetson 数据盘剩余空间"
fi

recording_status="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_HOST" \
  "$JETSON_PYTHON -" 2>/dev/null <<'PY'
import zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 1500)
sock.connect("tcp://127.0.0.1:5557")
sock.send_json({"command": "status"})
print(sock.recv_json().get("running", False))
PY
)"
if [[ "$recording_status" == "True" ]]; then
  warn "Jetson 当前正在录制，请先确认是否需要停止并保存"
elif [[ "$recording_status" == "False" ]]; then
  pass "Jetson 当前没有后台录制"
else
  warn "录制管理服务尚未启动；组合启动按钮会自动启动它"
fi

section "6/6 遥操运行文件版本"
host_commit="$(git -C "$ROOT_DIR" rev-parse --short HEAD 2>/dev/null || true)"
host_branch="$(git -C "$ROOT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
printf '  主机源码：%s @ %s\n' "${host_branch:-未知分支}" "${host_commit:-未知版本}"

runtime_rel="ros2_robot/src/remote_teleop_runtime/remote_teleop_runtime/follower.py"
host_runtime_sha="$(sha256sum "$ROOT_DIR/$runtime_rel" 2>/dev/null | awk '{print $1}')"
jetson_source_sha="$(ssh -o BatchMode=yes "$JETSON_HOST" \
  "sha256sum '$JETSON_ROOT/$runtime_rel' 2>/dev/null | cut -d' ' -f1" 2>/dev/null)"
jetson_installed_sha="$(ssh -o BatchMode=yes "$JETSON_HOST" \
  "sha256sum '$JETSON_ROOT/ros2_robot/install/remote_teleop_runtime/lib/python3.10/site-packages/remote_teleop_runtime/follower.py' 2>/dev/null | cut -d' ' -f1" 2>/dev/null)"
if [[ -n "$host_runtime_sha" && "$host_runtime_sha" == "$jetson_source_sha" && "$host_runtime_sha" == "$jetson_installed_sha" ]]; then
  pass "主机、Jetson 源码和 Jetson ARM 运行文件一致"
else
  fail "主机与 Jetson 的遥操运行文件不一致"
fi

host_control="$ROOT_DIR/ros2_robot/install_bimanual/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
jetson_control="$JETSON_ROOT/ros2_robot/install_bimanual/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
home_marker="Startup homing to upstream OpenArm INITIAL_POSITION"
if [[ -x "$host_control" ]] && strings "$host_control" | grep -Fq "$home_marker" && \
   ssh -o BatchMode=yes "$JETSON_HOST" \
     "test -x '$jetson_control' && strings '$jetson_control' | grep -Fq '$home_marker'" 2>/dev/null; then
  pass "两端控制程序均为当前自动归零版本"
else
  fail "两端控制程序版本不完整或缺少自动归零功能"
fi

printf '\n========================================\n'
printf '检查完成：通过 %d 项，警告 %d 项，失败 %d 项。\n' \
  "$PASS_COUNT" "$WARN_COUNT" "$FAIL_COUNT"
if (( FAIL_COUNT == 0 )); then
  printf '结论：设备连接条件通过，可以在两人就位后启动遥操。\n'
  exit 0
fi
printf '结论：暂不要启动机械臂，请先处理上面的失败项目。\n'
exit 1

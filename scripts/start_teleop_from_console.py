#!/usr/bin/env python3
"""Operator-confirmed lifecycle start. No hardware actions occur on import."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time

from inspect_teleop_recovery import (connection_config, inspect as inspect_recovery,
                                    consume_power_down_receipt)
from teleop_readiness import apply_report, apply_current_report
from power_down_arms import emit, ssh_args, RecorderClient, require_reply


def start_decision(result):
    """An explicit click permits cold start or a narrowly verified recovery."""
    e = result.get("evidence") or {}
    remote = e.get("remote") or {}
    unit = e.get("host_unit") or {}
    status = remote.get("status") or {}
    if not (e.get("ssh_ok") and e.get("host_can_ok") and remote.get("can_ok")):
        raise RuntimeError("网络或 CAN 检查未通过，未启动遥操")
    if not remote.get("recorder_idle"):
        raise RuntimeError("请先保存当前条目并等待录制器空闲")
    if (unit.get("ActiveState") == "active" and e.get("health_report_fresh")
            and apply_report(status, e.get("health_report") or {}).get("teleop_ready")):
        return "reuse"
    if result.get("confirmation_required") is True and result.get("code") in {
            "OPERATOR_POWER_DOWN_CONFIRM", "DISABLED_TIMEOUT_CONFIRM", "PEER_REBOOT_CONFIRM"}:
        return "restart"
    # A genuinely empty stack is the first-start path. Missing/malformed
    # evidence and any live/faulted old controller are not an empty stack.
    motors = e.get("motor_report") or {}
    cold = (unit.get("LoadState") == "loaded"
            and unit.get("ActiveState") in ("inactive", "failed")
            and unit.get("MainPID") == "0" and unit.get("ControlGroup") == ""
            and remote.get("processes") == []
            and remote.get("runtime_socket_exists") is False
            and remote.get("status") is None
            and e.get("local_control_processes") == []
            and motors.get("result") in ("DISABLED", "UNKNOWN"))
    for side in ("host", "follower"):
        part = motors.get(side) or {}
        values = part.get("motors") or {}
        expected = {f"{bus}/{motor}" for bus in connection_config()[side + "_interfaces"]
                    for motor in range(1, 9)}
        cold = cold and part.get("result") in ("DISABLED", "UNKNOWN") and set(values) == expected
        cold = cold and all(all(state == 0 for state in v.get("states", [])) for v in values.values())
    if cold:
        return "cold"
    raise RuntimeError("现有遥操状态不允许重新启动：" + str(result.get("reason", "状态未确认")) +
                       "。请先停止遥操并确认失能；故障需排查后恢复。")


def identity(result):
    e = result["evidence"]
    return [e.get("profile"), e.get("peer"), e.get("host_boot_id"), e.get("host_unit"),
            e["remote"].get("boot_id"), e["remote"].get("process_identities"),
            (e["remote"].get("status") or {}).get("leader_session_id"),
            (e.get("power_down_receipt") or {}).get("shutdown_id")]


def unit_status(service):
    result = subprocess.run(["systemctl", "--user", "show", service, "-p", "ActiveState",
                             "-p", "MainPID", "-p", "InvocationID", "-p", "Environment"],
                            capture_output=True, text=True, check=True, timeout=5)
    return dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)


def follower_status(peer):
    code = '''
import json,socket,tempfile
with tempfile.TemporaryDirectory(prefix="oa_start_status_") as d:
 with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
  s.bind(d+"/reply");s.settimeout(2)
  s.sendto(b'{"command":"status"}',"/tmp/openarm_remote_runtime.sock")
  print(s.recv(65536).decode())
'''
    p = subprocess.run(ssh_args(peer) + ["/usr/bin/python3 -"], input=code,
                       capture_output=True, text=True, check=True, timeout=6)
    return json.loads(p.stdout)


def start(config, *, inspect_fn=inspect_recovery):
    peer, service = config["peer"], config["service"]
    emit({"event": "progress", "message": "检查录制、连接、CAN 与现有控制会话…"})
    before = inspect_fn(peer)
    mode = start_decision(before)
    if mode == "reuse":
        return {"event": "complete", "ok": True, "message": "主从遥操已运行，已复用当前会话；未重新归位。"}
    # A second independent read immediately before the mutation rejects stale
    # approval/evidence (another UI, recorder, reboot or service replacement).
    again = inspect_fn(peer)
    if start_decision(again) != mode or identity(before) != identity(again):
        raise RuntimeError("检查期间控制会话发生变化，未启动；请重新确认")
    state = require_reply(RecorderClient(f"tcp://{config['peer_ip']}:5557")("status"))
    if state.get("running") is not False or state.get("active_episode") is not None:
        raise RuntimeError("录制器状态已改变，未启动")
    unit_before = unit_status(service)
    if config["profile"] == "host-test":
        import shlex
        environment = dict(item.split("=", 1) for item in shlex.split(unit_before["Environment"]) if "=" in item)
        if any(environment.get(key) != value for key, value in {
                "JETSON_HOST": peer, "PEER_IP": config["peer_ip"],
                "FOLLOWER_RIGHT_CAN": "can0", "FOLLOWER_LEFT_CAN": "can1"}.items()):
            raise RuntimeError("测试遥操服务的连接配置不匹配，未启动")
    if again.get("code") == "OPERATOR_POWER_DOWN_CONFIRM":
        consume_power_down_receipt(again["evidence"]["power_down_receipt"]["shutdown_id"])
    previous_session = (again["evidence"]["remote"].get("status") or {}).get("leader_session_id")
    emit({"event": "progress", "message": "正在启动：四臂将使能、归位并对齐，请勿触碰。"})
    started_ns = time.monotonic_ns()
    subprocess.run(["systemctl", "--user", "restart", service], check=True, timeout=25)
    invocation = unit_status(service).get("InvocationID")
    if not invocation or invocation == unit_before.get("InvocationID"):
        raise RuntimeError("未确认新的遥操服务实例")
    deadline = time.monotonic() + 180
    last_message = None
    while time.monotonic() < deadline:
        unit = unit_status(service)
        if unit.get("ActiveState") != "active" or unit.get("InvocationID") != invocation:
            raise RuntimeError("启动服务已退出或被替换；未确认可遥操，请查看启动日志")
        try:
            status = follower_status(peer)
            ready = apply_current_report(status)
            from teleop_readiness import read_report
            health = read_report()
            acceptance = json.loads(Path("/tmp/openarm-remote-teleop/alignment-following.json").read_text())
            if (ready.get("teleop_ready") is True
                    and status.get("leader_session_id") != previous_session
                    and health.get("probe_started_monotonic_ns", 0) >= started_ns
                    and acceptance.get("accepted") is True
                    and acceptance.get("leader_session_id") == status.get("leader_session_id")):
                return {"event": "complete", "ok": True,
                        "message": "主从遥操已运行：归位和对齐通过，32/32 电机使能已核验。"}
            message = "正在归位、对齐和核验电机，当前状态：" + str(status.get("state", "未知"))
        except (OSError, ValueError, subprocess.SubprocessError):
            message = "等待两端控制器启动及状态反馈…"
        if message != last_message:
            emit({"event": "progress", "message": message})
            last_message = message
        time.sleep(.5)
    raise RuntimeError("启动超时，尚未确认可遥操；控制器可能仍在运行。请使用停止遥操并失能处理，不要重复启动。")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--confirm", action="store_true")
    p.add_argument("--status", action="store_true", help="read-only stopped-session verification")
    p.add_argument("--profile", choices=("jetson", "host-test"), default="jetson")
    p.add_argument("--jetson", required=True)
    p.add_argument("--recorder", required=True)
    args = p.parse_args()
    if not args.confirm and not args.status:
        p.error("请先确认四臂无夹持物、归位路径无人且无障碍、急停可用并有人监护")
    os.environ["OPENARM_TELEOP_PROFILE"] = args.profile
    config = connection_config()
    if args.jetson != config["peer"] or args.recorder != f"tcp://{config['peer_ip']}:5557":
        p.error("连接目标与遥操配置不一致")
    if args.status:
        result = inspect_recovery(config["peer"])
        stopped = result.get("code") == "OPERATOR_POWER_DOWN_CONFIRM"
        emit({"event": "complete", "ok": True, "stopped": stopped,
              "leader_session_id": ((result.get("evidence") or {}).get("power_down_receipt") or {}).get("leader_session_id"),
              "message": "遥操已停止 · 32/32 电机失能回执已核验" if stopped else "未确认正常停止记录"})
        return 0
    try:
        with open("/tmp/openarm-teleop-lifecycle.lock", "a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # Cooperate with the existing desktop initializer too.
            with open("/tmp/openarm-daily-teleop.lock", "a") as daily:
                fcntl.flock(daily, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = start(config)
    except Exception as exc:
        result = {"event": "complete", "ok": False,
                  "message": "启动未确认：" + str(exc) + "；不会自动重试。"}
    emit(result)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Laptop entrypoint. Upload offline material and follow a detached Jetson job."""
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

BUNDLE = Path(__file__).resolve().parent
STATE = Path.home() / ".local/state/openarm"
PENDING = STATE / "jetson-update-pending.json"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
       "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=3",
       "-o", "ServerAliveCountMax=2", "openarm-jetson"]


def remote(code, timeout=25):
    result = subprocess.run(SSH + ["/usr/bin/python3", "-"], input=code,
                            capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "Jetson SSH 执行失败")
    return result.stdout


def follow(job):
    offset = 0
    disconnects = 0
    while True:
        code = """
import json,pathlib
p=pathlib.Path(JOB)
log=p/'update.log'
data=log.read_bytes() if log.exists() else b''
result=p/'result.json'
pidfile=p/'supervisor.pid'
alive=False
if pidfile.exists():
 try:
  command=(pathlib.Path('/proc')/pidfile.read_text().strip()/'cmdline').read_bytes()
  alive=str(p/'run_update_job.py').encode() in command.split(b'\\0')
 except (FileNotFoundError,ProcessLookupError):pass
print(json.dumps({'text':data[OFFSET:].decode(errors='replace'),'offset':len(data),
 'alive':alive,
 'result':json.loads(result.read_text()) if result.exists() else None}))
""".replace("JOB", repr(job)).replace("OFFSET", str(offset))
        try:
            state = json.loads(remote(code))
            disconnects = 0
        except (RuntimeError, subprocess.TimeoutExpired) as exc:
            disconnects += 1
            print("连接暂时中断；Jetson 会继续更新，不会自动开始遥操。", flush=True)
            if disconnects >= 3:
                raise RuntimeError("暂时无法读取结果。恢复网线后再次双击此入口，会继续读取同一任务；"
                                   "请勿同时启动采集。\n" + str(exc))
            time.sleep(3)
            continue
        print(state["text"], end="", flush=True)
        offset = state["offset"]
        if state["result"] is not None:
            code = state["result"]["returncode"]
            STATE.mkdir(parents=True, exist_ok=True)
            (STATE / "jetson-update-last-result.json").write_text(
                json.dumps({"job": job, **state["result"]}, indent=2))
            PENDING.unlink(missing_ok=True)
            return code
        if not state["alive"]:
            PENDING.unlink(missing_ok=True)
            raise RuntimeError("上次更新任务已中断或未成功启动，未确认安装完成。"
                               "再次双击入口将先恢复中断的切换，再重新更新。")
        time.sleep(3)


def main():
    print("Jetson 离线更新：请先连接网线并让 Jetson 开机；不要同时打开采集程序。", flush=True)
    print("更新不会启动遥操、使能或归位。无需下载软件；现有编译依赖缺失时会明确报错。", flush=True)
    # Same locks as the existing local startup and stop/start button helpers.
    locks = []
    for name in ("/tmp/openarm-teleop-lifecycle.lock", "/tmp/openarm-daily-teleop.lock"):
        stream = open(name, "w")
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("当前有遥操启停/采集启动流程正在执行，请结束后再更新。")
        locks.append(stream)
    STATE.mkdir(parents=True, exist_ok=True)
    if PENDING.exists():
        job = json.loads(PENDING.read_text())["job"]
        print("继续读取上次任务：" + job, flush=True)
        return follow(job)
    for unit in ("openarm-remote-teleop.service", "openarm-host-test-teleop.service"):
        result = subprocess.run(["systemctl", "--user", "show", unit, "-p", "MainPID", "--value"],
                                capture_output=True, text=True)
        if result.stdout.strip() not in ("", "0"):
            raise RuntimeError("笔记本遥操仍在运行。请先安全停止遥操并失能，再退出采集界面。")
    manifest = json.loads((BUNDLE / "manifest.json").read_text())
    for name in ("remote-control", "rgbd-capture"):
        path = BUNDLE / (name + ".tar.gz")
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest[name]["sha256"]:
            raise RuntimeError("本地离线包损坏：" + name)
    print("正在连接之前配置的 Jetson 192.168.50.2…", flush=True)
    identity = json.loads(remote(
        "import platform,pathlib,json;print(json.dumps({'arch':platform.machine(),"
        "'home':str(pathlib.Path.home()),'host':platform.node()}))"))
    if identity["arch"] != "aarch64" or identity["home"] != "/home/nvidia":
        raise RuntimeError("目标不是预期的 Jetson nvidia/aarch64，拒绝安装：" + str(identity))
    print("已连接：" + identity["host"], flush=True)
    job = "/home/nvidia/openarm-update-inbox/" + uuid.uuid4().hex
    remote("from pathlib import Path;Path(" + repr(job) + ").mkdir(parents=True)")
    files = ["remote-control.tar.gz", "rgbd-capture.tar.gz", "manifest.json",
             "install_on_jetson.py", "run_update_job.py"]
    print("传输更新包…", flush=True)
    subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                    "-o", "ConnectTimeout=5", *[str(BUNDLE / name) for name in files],
                    "openarm-jetson:" + job + "/"], check=True)
    # Save identity before dispatch: SSH loss after spawning must never trigger
    # an automatic second installation.
    PENDING.write_text(json.dumps({"job": job}))
    code = """
import pathlib,subprocess
p=pathlib.Path(JOB)
with (p/'update.log').open('ab') as log:
 child=subprocess.Popen(['/usr/bin/python3','-u',str(p/'run_update_job.py')],
  cwd=p,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (p/'supervisor.pid').write_text(str(child.pid))
print('Jetson 更新任务已启动')
""".replace("JOB", repr(job))
    print(remote(code), end="", flush=True)
    return follow(job)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print("\n更新未确认完成：" + str(exc), file=sys.stderr)
        print("请保留日志。不要把连接成功或更新包已传输当作安装成功。", file=sys.stderr)
        sys.exit(1)

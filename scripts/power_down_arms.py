#!/usr/bin/env python3
"""Explicit four-arm torque disable; never powers off Linux or re-enables motors.

The coordinator seals the recorder first. Two bounded subprocesses arm passive
CAN listeners before either receives permission to call its ROS disable service.
Newline-delimited JSON is the UI protocol. No hardware access occurs on import.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import fcntl
import inspect
import json
import os
from pathlib import Path
import select
import shlex
import signal
import socket
import struct
import subprocess
import sys
import threading
import time


class ShutdownError(RuntimeError):
    pass


def emit(event):
    print(json.dumps(event, ensure_ascii=False), flush=True)


def require_reply(value):
    if not isinstance(value, dict) or value.get("ok") is not True:
        raise ShutdownError(str(value.get("error", "录制服务未确认请求")) if isinstance(value, dict) else "无效录制状态")
    return value


def recording_saved(episode):
    """Success of persistence is independent of the operator's success label."""
    receipt = episode.get("rgbd_writer_receipt") or {}
    return (episode.get("recorder_returncode") == 0
            and receipt.get("complete") is True
            and bool(episode.get("lerobot_root"))
            and receipt.get("dataset_root") == episode.get("lerobot_root")
            and episode.get("ended_unix_s") is not None)


def seal_recording(request, *, timeout=45., clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + timeout
    state = require_reply(request("status"))
    if state.get("running") is not False and state.get("running") is not True:
        raise ShutdownError("录制状态未知，未执行下电")
    session = state.get("session_root")
    episode = state.get("active_episode")
    if state["running"]:
        if not episode or not episode.get("episode_root"):
            raise ShutdownError("无法确认当前条目身份，未执行下电")
        if state.get("phase") in {"starting", "recording"}:
            require_reply(request("episode_stop", result="aborted", failure_code="operator_power_down"))
        elif state.get("phase") != "stopping":
            raise ShutdownError("当前录制状态异常，未执行下电")
        while clock() < deadline:
            state = require_reply(request("status"))
            if state.get("session_root") != session:
                raise ShutdownError("保存期间采集会话改变，未执行下电")
            if state.get("running") is False:
                last = state.get("last_episode") or {}
                if last.get("episode_root") != episode["episode_root"] or not recording_saved(last):
                    raise ShutdownError("本条保存未确认完整，未执行下电；请检查录制日志和数据")
                break
            if state.get("running") is not True:
                raise ShutdownError("保存状态未知，未执行下电")
            sleep(.2)
        else:
            raise ShutdownError("等待保存超时，未执行下电")
    if state.get("phase") != "idle" or state.get("running") is not False:
        raise ShutdownError("录制器未确认空闲，未执行下电")
    # Do not let retry silently bypass an incomplete preceding finalization.
    if state.get("last_episode") and not recording_saved(state["last_episode"]):
        raise ShutdownError("上一条保存不完整，未执行下电；请先处理该条数据")
    return state


class RecorderClient:
    def __init__(self, endpoint):
        self.endpoint = endpoint

    def __call__(self, command, **fields):
        import zmq
        context = zmq.Context()
        channel = context.socket(zmq.REQ)
        channel.setsockopt(zmq.LINGER, 0)
        channel.setsockopt(zmq.SNDTIMEO, 2000)
        channel.setsockopt(zmq.RCVTIMEO, 3000)
        try:
            channel.connect(self.endpoint)
            channel.send_json({"command": command, **fields})
            return channel.recv_json()
        finally:
            channel.close(0)
            context.term()


class DisableEvidence:
    """Command-window evidence: disabled motors may stop broadcasting afterwards."""
    def __init__(self, interfaces):
        self.samples = {(iface, motor): [] for iface in interfaces for motor in range(1, 9)}
        self.started = None
        self.lock = threading.Lock()

    def begin(self, now):
        with self.lock:
            self.started = now
            for values in self.samples.values():
                values.clear()

    def observe(self, interface, decoded, received):
        with self.lock:
            if self.started is None or received < self.started:
                return
            key = (interface, decoded[0])
            if key in self.samples:
                values = self.samples[key]
                values.append(decoded[1])
                del values[:-3]

    def report(self):
        with self.lock:
            motors = {f"{iface}/{motor}": {"states": list(values),
                      "disabled": len(values) == 3 and values == [0, 0, 0]}
                      for (iface, motor), values in self.samples.items()}
        return {"verified": all(item["disabled"] for item in motors.values()), "motors": motors}


class PassiveListener:
    def __init__(self, interfaces):
        self.evidence = DisableEvidence(interfaces)
        self.sockets = {}
        self.stop = threading.Event()
        self.error = None
        self.thread = None
        try:
            from check_motor_enable import SO_TIMESTAMPNS
            for interface in interfaces:
                sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[sock] = interface
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                    b"".join(struct.pack("=II", i, 0xC00007FF) for i in range(0x11, 0x19)))
                sock.bind((interface,))
                sock.setblocking(False)
            self.thread = threading.Thread(target=self._listen, daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise

    def _listen(self):
        from check_motor_enable import decode_feedback, feedback_receive_time
        try:
            while not self.stop.is_set():
                ready, _, _ = select.select(list(self.sockets), [], [], .02)
                for sock in ready:
                    try:
                        packet, ancillary, flags, _ = sock.recvmsg(72, socket.CMSG_SPACE(16))
                    except BlockingIOError:
                        continue
                    received = feedback_receive_time(ancillary, flags, time.time_ns(), time.monotonic())
                    decoded = decode_feedback(packet) if received is not None else None
                    if decoded is not None:
                        self.evidence.observe(self.sockets[sock], decoded, received)
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1)
        for sock in self.sockets:
            sock.close()


def disable_worker(side):
    """Runs with ROS environment on the machine owning these CAN interfaces."""
    import rclpy
    from std_srvs.srv import Trigger
    interfaces = ["can0", "can1"] if side == "host" else ["can1", "can2"]
    service = f"/{'leader' if side == 'host' else 'follower'}/openarm_gravity_pd/disable"
    listener = None
    node = None
    rclpy.init(args=[])
    try:
        node = rclpy.create_node("operator_power_down_" + side)
        client = node.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=4.):
            raise ShutdownError("电机失能服务不可用：" + service)
        listener = PassiveListener(interfaces)
        emit({"event": "armed", "side": side})
        if not select.select([sys.stdin], [], [], 12.)[0] or sys.stdin.readline().strip() != "disable":
            raise ShutdownError("未收到本次下电许可，未调用失能服务")
        listener.evidence.begin(time.monotonic())
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.)
        reply = future.result() if future.done() else None
        # Include queued responses without requiring motors to keep transmitting.
        time.sleep(.2)
        report = listener.evidence.report()
        acknowledged = reply is not None and reply.success is True
        missing = [motor for motor, value in report["motors"].items() if not value["disabled"]]
        emit({"event": "device_result", "side": side, **report,
              "acknowledged": acknowledged,
              "ok": acknowledged and report["verified"] and listener.error is None,
              "error": (listener.error or ("失能服务回复失败或超时" if not acknowledged else None)
                        or ("缺少连续失能反馈：" + ", ".join(missing) if missing else None))})
    except Exception as exc:
        emit({"event": "device_result", "side": side, "ok": False, "verified": False, "error": str(exc)})
    finally:
        if listener:
            listener.close()
        if node:
            node.destroy_node()
        rclpy.shutdown()


class DeviceProcess:
    def __init__(self, side, peer):
        self.side = side
        self.buffer = b""
        # Stream the exact local implementation, avoiding stale remote files.
        checker = Path(__file__).with_name("check_motor_enable.py").read_text()
        payload = ("import types,sys\nm=types.ModuleType('check_motor_enable')\n"
                   + "exec(" + repr(checker) + ",m.__dict__)\nsys.modules[m.__name__]=m\n"
                   + "ns={'__name__':'power_down_worker'}\nexec(" + repr(Path(__file__).read_text())
                   + ",ns)\nns['disable_worker'](" + repr(side) + ")\n")
        # A single JSON line carries code; unbuffered os.read leaves subsequent
        # authorization bytes available to select()/readline() in the worker.
        bootstrap = "import os,json\nb=b''\nwhile not b.endswith(b'\\n'): b+=os.read(0,1)\nexec(json.loads(b))"
        command = "source /opt/ros/humble/setup.bash && export ROS_LOCALHOST_ONLY=1 && exec /usr/bin/python3 -u -c " + shlex.quote(bootstrap)
        argv = ["bash", "-c", command] if side == "host" else ssh_args(peer) + ["bash -c " + shlex.quote(command)]
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, bufsize=0)
        try:
            self.send((json.dumps(payload) + "\n").encode(), timeout=7.)
        except Exception:
            self.close()
            raise

    def send(self, data, timeout=2.):
        done = threading.Event()
        errors = []
        def write():
            try:
                remaining = memoryview(data)
                while remaining:
                    written = os.write(self.process.stdin.fileno(), remaining)
                    if written <= 0:
                        raise ShutdownError("下电进程输入已关闭")
                    remaining = remaining[written:]
            except Exception as exc:
                errors.append(exc)
            finally:
                done.set()
        writer = threading.Thread(target=write, daemon=True)
        writer.start()
        if not done.wait(timeout):
            self.process.terminate()
            writer.join(timeout=1.)
            raise ShutdownError("向下电进程发送请求超时")
        if errors:
            raise ShutdownError(str(errors[0]))

    def receive(self, expected, timeout):
        deadline = time.monotonic() + timeout
        detail = ""
        while time.monotonic() < deadline:
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                detail = line.decode(errors="replace")[-500:]
                try:
                    value = json.loads(line)
                except ValueError:
                    continue
                if isinstance(value, dict) and value.get("event") == expected:
                    return value
                if isinstance(value, dict) and value.get("event") == "device_result":
                    raise ShutdownError(str(value.get("error", "下电准备失败")))
            if not select.select([self.process.stdout], [], [], min(.2, max(0., deadline-time.monotonic())))[0]:
                continue
            data = os.read(self.process.stdout.fileno(), 65536)
            if not data:
                raise ShutdownError("下电进程已退出：" + detail)
            self.buffer += data
        raise ShutdownError("等待下电反馈超时：" + detail)

    def arm(self):
        return self.receive("armed", 9.)

    def go(self):
        self.send(b"disable\n")

    def result(self):
        return self.receive("device_result", 12.)

    def close(self):
        if self.process.stdin:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.terminate()  # Only our bounded helper, never a controller.
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.process.stdout.close()


def ssh_args(peer):
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2", peer]


def confirm_hold(request, *, clock=time.monotonic, sleep=time.sleep):
    status = request("status")
    if status.get("collection", {}).get("right_mode") == "RETURNING":
        request("right_pause")
    if status.get("state") in ("ALIGNING", "READY", "RUNNING"):
        request("hold")
    # The gateway's immediate command reply can contain an older watchdog
    # heartbeat. Require a fresh status read instead of accepting that reply.
    deadline = clock() + 2.
    while clock() < deadline:
        status = request("status")
        if (status.get("state") in ("ALIGNING", "READY", "FAULT", "E_STOP")
                and status.get("collection", {}).get("right_mode") != "RETURNING"):
            return status
        sleep(.05)
    raise RuntimeError("停止跟随未确认")


def hold_following(peer):
    code = 'import time\n' + inspect.getsource(confirm_hold) + '''
import socket,tempfile,json
from pathlib import Path
with tempfile.TemporaryDirectory(prefix="power_down_hold_") as directory:
 with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as channel:
  channel.bind(str(Path(directory)/"reply")); channel.settimeout(2)
  def request(command):
   channel.sendto(json.dumps({"command":command}).encode(),"/tmp/openarm_remote_runtime.sock")
   value=json.loads(channel.recv(65536))
   if "error" in value: raise RuntimeError(value["error"])
   return value
  print(json.dumps(confirm_hold(request)))
'''
    result = subprocess.run(ssh_args(peer) + ["/usr/bin/python3 -"], input=code,
                            capture_output=True, text=True, timeout=12)
    if result.returncode:
        raise ShutdownError("停止跟随失败：" + result.stderr[-500:])
    return json.loads(result.stdout)


def stop_service():
    service = "openarm-remote-teleop.service"
    subprocess.run(["systemctl", "--user", "stop", service], check=True,
                   capture_output=True, text=True, timeout=22)
    result = subprocess.run(["systemctl", "--user", "show", service,
                            "--property=ActiveState", "--property=MainPID", "--property=ControlGroup"],
                            check=True, capture_output=True, text=True, timeout=4)
    if not service_stopped(result.stdout):
        raise ShutdownError("电机已失能，但遥操服务停止未确认")


def service_stopped(output):
    fields = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
    # The existing shell supervisor exits 143 on SIGTERM. systemd records
    # 'failed' even after a successful intentional stop; require no processes.
    return (fields.get("ActiveState") in {"inactive", "failed"}
            and fields.get("MainPID") == "0" and fields.get("ControlGroup") == "")


def coordinate(request, hold, make_device, stop, progress=emit, *, capture=None, finish=None):
    devices = {}
    reports = {}
    stage = "saving"
    messages = {"saving": "保存数据", "holding": "停止跟随", "arming": "准备两端电机反馈监听",
                "disabling": "主从电机失能", "verifying": "核验结果", "stopping_service": "停止遥操服务"}
    def announce(name):
        progress({"event": "progress", "stage": name, "message": messages[name]})
    try:
        announce(stage)
        seal_recording(request)
        identity = capture() if capture else None
        stage = "holding"; announce(stage)
        hold()
        stage = "arming"; announce(stage)
        for side in ("host", "follower"):
            devices[side] = make_device(side)
        # Both listeners must be armed before sending either disable command.
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = {side: pool.submit(device.arm) for side, device in devices.items()}
            for side, future in futures.items():
                try:
                    future.result()
                    reports[side] = {"armed": True}
                except Exception as exc:
                    reports[side] = {"ok": False, "error": str(exc)}
        if not all(item.get("armed") for item in reports.values()):
            raise ShutdownError("两端失能服务和反馈监听未全部就绪，未发送失能命令")
        stage = "disabling"; announce(stage)
        for side, device in devices.items():
            try:
                device.go()
            except Exception as exc:
                reports[side] = {"ok": False, "error": str(exc)}
        stage = "verifying"; announce(stage)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = {side: pool.submit(device.result) for side, device in devices.items()
                       if reports[side].get("armed")}
            for side, future in futures.items():
                try:
                    reports[side] = future.result()
                except Exception as exc:
                    reports[side] = {"ok": False, "error": str(exc)}
                progress({"event": "device_result", "side": side, **reports[side]})
        if not all(item.get("ok") is True and item.get("verified") is True for item in reports.values()):
            raise ShutdownError("下电未完全确认，请检查两端结果；不会自动恢复跟随")
        stage = "stopping_service"; announce(stage)
        stop()
        if finish:
            finish(identity, reports)
        result = {"event": "complete", "ok": True, "stage": "complete", "devices": reports,
                  "message": "主从电机已确认失能，可手动关闭机械臂电源。Jetson 保持开机。"}
    except Exception as exc:
        result = {"event": "complete", "ok": False, "stage": stage, "devices": reports,
                  "message": "下电未完成：" + str(exc)}
    finally:
        for device in devices.values():
            try:
                device.close()
            except Exception:
                pass
    progress(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm", action="store_true", help="operator has placed and supported all four arms")
    parser.add_argument("--jetson", default="openarm-jetson")
    parser.add_argument("--recorder", default="tcp://192.168.50.2:5557")
    args = parser.parse_args()
    if not args.confirm:
        parser.error("必须先确认四臂已支撑、夹爪无物，再使用 --confirm")
    if args.jetson.startswith("-"):
        parser.error("invalid SSH destination")
    def interrupted(_signum, _frame):
        raise ShutdownError("下电协调程序被中断；结果未确认，不会自动恢复运动")
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    with open("/tmp/openarm-power-down.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            emit({"event": "complete", "ok": False, "message": "已有下电操作正在执行"})
            return 1
        from inspect_teleop_recovery import inspect as inspect_recovery, save_power_down_receipt
        def capture():
            evidence = inspect_recovery(args.jetson)['evidence']
            if not evidence.get('ssh_ok') or not evidence.get('host_unit') or not evidence['remote'].get('process_identities'):
                raise ShutdownError('无法取得两端控制会话身份，未执行下电')
            return evidence
        def finish(before, reports):
            save_power_down_receipt(before, capture(), reports)
        result = coordinate(RecorderClient(args.recorder), lambda: hold_following(args.jetson),
                            lambda side: DeviceProcess(side, args.jetson), stop_service,
                            capture=capture, finish=finish)
        return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())

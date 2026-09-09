#!/usr/bin/env python3
"""Reliable task-specific OpenArm mushroom RGB-D collection console.

The UI is a client of Jetson's recorder manager. It never opens cameras,
accesses CAN, or changes teleoperation. Native Tk buttons are used instead of
image-coordinate hit testing, so display scaling cannot make visible controls
unclickable.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

import zmq
from PIL import Image, ImageDraw, ImageTk


ROLES = ("chest", "left_wrist", "right_wrist")
CAMERA_TITLES = {"chest": "胸部全局相机", "left_wrist": "左腕相机", "right_wrist": "右腕相机"}
TASKS = {
    "left": ("LEFT_GRASP_LOG", "左臂｜菌棒夹持", "#58a9ef"),
    "right": ("RIGHT_PICK_ONE", "右臂｜单朵蘑菇采摘", "#72c94c"),
}
BG = "#090d10"; PANEL = "#10161a"; BORDER = "#344047"; TEXT = "#edf1f2"; MUTED = "#9ba8ad"
LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")


def parse_json_output(output: str) -> dict[str, Any]:
    """Parse a JSON object even when ROS writes harmless lines around it."""
    decoder = json.JSONDecoder()
    for index, character in enumerate(output):
        if character != "{":
            continue
        try:
            value, _ = decoder.raw_decode(output[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    raise ValueError("no JSON status object in command output")


def decode_jpeg(value: str) -> Image.Image:
    """Decode with Pillow so the UI can use Ubuntu's system Tk/font stack.

    The Conda Python bundled a legacy Tk build which only saw old X bitmap
    fonts.  That produced missing Chinese glyphs even though Noto CJK is
    installed.  Avoiding OpenCV lets this process run with /usr/bin/python3,
    whose Tk has Fontconfig/Noto support.
    """
    with Image.open(io.BytesIO(base64.b64decode(value))) as image:
        return image.convert("RGB").copy()


def recorder_request(endpoint: str, payload: dict[str, Any], timeout_ms: int = 5000) -> dict[str, Any]:
    """Perform one bounded recorder-manager request before the main UI starts."""
    context = zmq.Context(); socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.SNDTIMEO, 3000)
    socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
    try:
        socket.connect(endpoint); socket.send_json(payload)
        value = socket.recv_json()
        if not isinstance(value, dict):
            raise RuntimeError("Jetson 返回了无效响应")
        return value
    finally:
        socket.close(0); context.term()


def choose_collection_session(root: tk.Tk, endpoint: str) -> bool:
    """Require an explicit batch choice; never silently reuse episode numbers."""
    try:
        catalog = recorder_request(endpoint, {"command": "session_catalog"})
    except Exception as exc:
        messagebox.showerror("无法连接录制服务", f"无法读取 Jetson 采集批次：\n{exc}")
        return False
    if not catalog.get("ok"):
        messagebox.showerror("无法读取批次", str(catalog.get("error", "未知错误")))
        return False
    if catalog.get("running"):
        messagebox.showerror("已有录制正在运行", "请先完成并封装当前 episode，再重新打开采集界面。")
        return False

    # Use the real Tk window for the chooser.  A child Toplevel owned by a
    # withdrawn root is positioned relative to an invisible WM frame on some
    # GNOME/X11 multi-monitor layouts and can end up entirely off-screen.
    dialog = root; dialog.title("选择 Jetson 采集批次")
    dialog.geometry("900x600"); dialog.minsize(760, 520); dialog.configure(bg=BG)
    dialog.update_idletasks()
    screen_x = max(20, (dialog.winfo_screenwidth() - 900) // 2)
    screen_y = max(20, (dialog.winfo_screenheight() - 600) // 2)
    dialog.geometry(f"900x600+{screen_x}+{screen_y}")
    dialog.deiconify()
    dialog.lift(); dialog.attributes("-topmost", True)
    dialog.after(500, lambda: dialog.attributes("-topmost", False) if dialog.winfo_exists() else None)
    dialog.focus_force()
    selected = {"ok": False}
    finished = tk.BooleanVar(master=root, value=False)
    tk.Label(dialog, text="开始采集前请选择批次", bg=BG, fg=TEXT,
             font=("Noto Sans CJK SC", 18, "bold")).pack(anchor="w", padx=28, pady=(22, 5))
    current = catalog.get("current_session_root")
    current_text = current or "没有可恢复的当前批次"
    tk.Label(dialog, text=f"上次批次：{current_text}", bg=BG, fg="#79cfe8",
             font=("Noto Sans CJK SC", 10), wraplength=830, justify="left").pack(anchor="w", padx=28)

    def perform(payload: dict[str, Any]) -> None:
        try:
            response = recorder_request(endpoint, {"command": "session_start", **payload})
        except Exception as exc:
            messagebox.showerror("操作失败", str(exc), parent=dialog); return
        if not response.get("ok"):
            messagebox.showerror("操作被拒绝", str(response.get("error", "未知错误")), parent=dialog); return
        selected["ok"] = True; finished.set(True)

    tk.Button(dialog, text="继续上次批次（编号接着增加）",
              command=lambda: perform({"mode": "continue"}), bg="#2475a8", fg="white",
              font=("Noto Sans CJK SC", 12, "bold"), relief="flat", pady=9).pack(fill="x", padx=28, pady=(14, 10))

    new_box = tk.LabelFrame(dialog, text="新建批次（左右都从第 1 条开始）", bg=PANEL, fg=TEXT,
                            font=("Noto Sans CJK SC", 11, "bold"), padx=12, pady=10)
    new_box.pack(fill="x", padx=28, pady=5)
    storage_var = tk.StringVar(value=str(catalog.get("default_storage_base") or "/home/nvidia/datasets/openarm_harvest_sessions"))
    tk.Label(new_box, text="Jetson 保存根目录：", bg=PANEL, fg=TEXT,
             font=("Noto Sans CJK SC", 10)).pack(anchor="w")
    tk.Entry(new_box, textvariable=storage_var, font=("Noto Sans Mono CJK SC", 10)).pack(fill="x", pady=5)
    tk.Label(new_box, text="只允许 /home/nvidia/datasets 内的目录；旧批次不会覆盖。", bg=PANEL, fg=MUTED,
             font=("Noto Sans CJK SC", 9)).pack(anchor="w")
    tk.Button(new_box, text="在上述目录新建批次", command=lambda: perform({"mode": "new", "storage_base": storage_var.get().strip()}),
              bg="#3d913d", fg="white", font=("Noto Sans CJK SC", 11, "bold"), relief="flat", pady=7).pack(fill="x", pady=(7, 0))

    old_box = tk.LabelFrame(dialog, text="选择已有批次继续", bg=PANEL, fg=TEXT,
                            font=("Noto Sans CJK SC", 11, "bold"), padx=12, pady=10)
    old_box.pack(fill="both", expand=True, padx=28, pady=(8, 12))
    sessions = list(catalog.get("sessions") or [])
    labels = [f"{item['session_id']}　左 {item['episode_counts']['left']} 条 / 右 {item['episode_counts']['right']} 条　{item['session_root']}" for item in sessions]
    session_combo = ttk.Combobox(old_box, values=labels, state="readonly", font=("Noto Sans CJK SC", 9))
    session_combo.pack(fill="x", pady=(2, 8))
    if labels: session_combo.current(0)

    def select_existing() -> None:
        index = session_combo.current()
        if index < 0:
            messagebox.showwarning("未选择批次", "请先从列表选择一个已有批次。", parent=dialog); return
        perform({"mode": "select", "session_root": sessions[index]["session_root"]})

    tk.Button(old_box, text="继续所选已有批次", command=select_existing, bg="#6b5ca5", fg="white",
              font=("Noto Sans CJK SC", 11, "bold"), relief="flat", pady=7).pack(fill="x")
    dialog.protocol("WM_DELETE_WINDOW", lambda: finished.set(True))
    root.wait_variable(finished)
    for child in root.winfo_children():
        child.destroy()
    root.withdraw()
    return bool(selected["ok"])


class SessionControl:
    """Single-flight request client; duplicate clicks cannot create duplicate episodes."""

    def __init__(self, jetson: str, port: int) -> None:
        self.endpoint = f"tcp://{jetson}:{port}"
        self._lock = threading.RLock(); self._request_lock = threading.Lock()
        self._value: dict[str, Any] = {"phase": "idle", "running": False}
        self._message = "正在连接 Jetson 本地录制服务…"
        self._pending = False

    def request(self, command: str, **extra: str) -> bool:
        with self._lock:
            if self._pending and command != "status":
                self._message = "上一个操作仍在处理，请勿重复点击。"
                return False
            if command != "status":
                self._pending = True
                # Reflect a stop click immediately.  The recorder process
                # remains alive while it flushes parquet and RGB-D metadata,
                # but no new camera frames belong to the episode after this
                # point.  Do not mislabel that finalization time as recording.
                if command == "episode_stop":
                    self._value = dict(self._value)
                    self._value["phase"] = "stopping"
                self._message = {
                    "session_start": "正在创建采集会话…",
                    "episode_start": "正在启动本条录制…",
                    "episode_stop": "正在停止并封口，请勿重复点击…",
                    "session_close": "正在结束本次采集会话…",
                }.get(command, "正在请求 Jetson…")

        def worker() -> None:
            if command == "status" and not self._request_lock.acquire(blocking=False):
                return
            if command != "status":
                self._request_lock.acquire()
            context = zmq.Context(); socket = context.socket(zmq.REQ)
            socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.SNDTIMEO, 3000); socket.setsockopt(zmq.RCVTIMEO, 5000)
            try:
                socket.connect(self.endpoint); socket.send_json({"command": command, **extra}); response = socket.recv_json()
                with self._lock:
                    self._value = response
                    if not response.get("ok"):
                        self._message = "请求被拒绝：" + str(response.get("error", "未知错误"))
                    elif response.get("phase") == "stopping":
                        self._message = "正在安全封口；机械臂遥操继续运行。"
                    elif response.get("running"):
                        episode = response.get("active_episode") or {}
                        self._message = f"正在录制 {episode.get('task', '当前任务')}：数据写入 Jetson"
                    elif response.get("last_episode"):
                        episode = response["last_episode"]
                        self._message = f"已封口 {episode.get('episode_id')}：{episode.get('result')}，有效={episode.get('valid')}"
                    elif response.get("session_id"):
                        self._message = "会话就绪：确认 READY 后开始对应任务。"
                    else:
                        self._message = str(response.get("message", "未在录制"))
            except Exception as exc:
                with self._lock:
                    self._message = f"Jetson 录制服务未连接：{exc}"
            finally:
                with self._lock:
                    if command != "status":
                        self._pending = False
                socket.close(0); context.term(); self._request_lock.release()

        threading.Thread(target=worker, daemon=True, name=f"collection-{command}").start()
        return True

    def snapshot(self) -> tuple[dict[str, Any], str, bool]:
        with self._lock:
            return dict(self._value), self._message, self._pending

    def abort_synchronously(self, reason: str) -> None:
        """Seal an active episode as aborted without touching teleoperation."""
        context = zmq.Context(); socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.SNDTIMEO, 2000); socket.setsockopt(zmq.RCVTIMEO, 4000)
        try:
            socket.connect(self.endpoint); socket.send_json({"command": "status"}); state = socket.recv_json()
            if state.get("running"):
                socket.close(0); context.term(); context = zmq.Context(); socket = context.socket(zmq.REQ)
                socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.SNDTIMEO, 2000); socket.setsockopt(zmq.RCVTIMEO, 4000)
                socket.connect(self.endpoint)
                socket.send_json({"command": "episode_stop", "result": "aborted", "failure_code": reason})
                socket.recv_json()
        except Exception:
            pass
        finally:
            socket.close(0); context.term()

    def close_session_synchronously(self) -> None:
        """Close an idle collection batch before a normal window exit.

        A process crash still leaves the active-session marker in place for
        recovery.  A deliberate window close, however, is a clean end of the
        operator's current batch, so the next launch must start at episode 1
        in a new timestamped session instead of silently continuing an older
        batch.
        """
        context = zmq.Context(); socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0); socket.setsockopt(zmq.SNDTIMEO, 2000); socket.setsockopt(zmq.RCVTIMEO, 4000)
        try:
            socket.connect(self.endpoint)
            socket.send_json({"command": "session_close"})
            socket.recv_json()
        except Exception:
            pass
        finally:
            socket.close(0); context.term()


class PreviewReceiver:
    """Receive only the newest preview packet; recording never waits for this thread."""

    def __init__(self, endpoint: str) -> None:
        self.endpoint = endpoint
        self.frames: queue.Queue[tuple[dict[str, Image.Image], dict[str, float]]] = queue.Queue(maxsize=1)
        self.stop = threading.Event(); self.error = "等待三路实时画面…"
        self.last_packet_monotonic = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True, name="rgb-preview")

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        context = zmq.Context(); socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b""); socket.setsockopt(zmq.CONFLATE, 1); socket.setsockopt(zmq.LINGER, 0)
        socket.connect(self.endpoint); poller = zmq.Poller(); poller.register(socket, zmq.POLLIN)
        last_at = {role: 0.0 for role in ROLES}; smoothed = {role: 0.0 for role in ROLES}
        try:
            while not self.stop.is_set():
                if socket not in dict(poller.poll(timeout=250)):
                    continue
                try:
                    now = time.time(); packet = json.loads(socket.recv_string()); images: dict[str, Image.Image] = {}; metrics: dict[str, float] = {}
                    for role in ROLES:
                        images[role] = decode_jpeg(packet["images"][role])
                        previous = last_at[role]
                        if previous:
                            current = 1.0 / max(now - previous, 1e-3)
                            smoothed[role] = current if smoothed[role] == 0 else 0.85 * smoothed[role] + 0.15 * current
                        last_at[role] = now
                        metrics[role + "_fps"] = smoothed[role]
                        metrics[role + "_age_ms"] = (now - float(packet["timestamps"][role])) * 1000.0
                    try:
                        self.frames.get_nowait()
                    except queue.Empty:
                        pass
                    self.frames.put_nowait((images, metrics)); self.error = ""
                    self.last_packet_monotonic = time.monotonic()
                except Exception as exc:
                    # A torn/malformed JPEG must cost at most one preview frame.
                    # The operator controls and subsequent frames remain live.
                    self.error = f"丢弃一帧异常预览：{exc}"
        finally:
            socket.close(0); context.term()

    def latest(self) -> tuple[dict[str, Image.Image], dict[str, float]] | None:
        value = None
        try:
            while True:
                value = self.frames.get_nowait()
        except queue.Empty:
            return value

    def age_s(self) -> float:
        return time.monotonic() - self.last_packet_monotonic if self.last_packet_monotonic else float("inf")


class TeleopMonitor:
    """Read the follower watchdog's authoritative state without controlling it."""

    def __init__(self, ssh_host: str) -> None:
        self.ssh_host = ssh_host
        self.stop = threading.Event()
        self._lock = threading.Lock()
        self._value: dict[str, Any] = {"state": "CHECKING", "fault_bits": None, "connected": False}
        self._last_running = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True, name="teleop-status-monitor")

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        remote = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install/setup.bash && "
            "source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/setup.bash && "
            "ros2 run remote_teleop_runtime remote-teleop-control status"
        )
        while not self.stop.is_set():
            try:
                result = subprocess.run(
                    ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=2", self.ssh_host, remote],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=4, check=True,
                )
                value = parse_json_output(result.stdout)
                value["connected"] = True
                if value.get("state") == "RUNNING" and int(value.get("fault_bits", 0)) == 0:
                    self._last_running = time.monotonic()
            except Exception as exc:
                value = {"state": "DISCONNECTED", "fault_bits": None, "connected": False, "error": str(exc)}
            value["last_running_age_s"] = (
                time.monotonic() - self._last_running if self._last_running else float("inf")
            )
            with self._lock:
                self._value = value
            self.stop.wait(1.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._value)


class MushroomCollectionApp:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root; self.args = args
        self.control = SessionControl(args.jetson, args.record_port)
        self.preview = PreviewReceiver(f"tcp://{args.jetson}:{args.preview_port}")
        self.teleop = TeleopMonitor(args.jetson_ssh)
        self.last_status = 0.0; self.photos: dict[str, ImageTk.PhotoImage] = {}; self.close_requested = False
        self.closing = False
        self.teleop_abort_requested = False
        self.operator_notice = ""
        self.operator_notice_until = 0.0
        self.left_ready = tk.BooleanVar(value=False); self.right_ready = tk.BooleanVar(value=False)
        self.target_confirmed = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="正在连接…"); self.message_var = tk.StringVar(value="正在初始化采集会话…")
        self.camera_metric_vars = {role: tk.StringVar(value="等待实时画面…") for role in ROLES}
        self.count_vars = {"left": tk.StringVar(value="有效成功：0 条"), "right": tk.StringVar(value="有效成功：0 条")}
        self.subcount_vars = {"left": tk.StringVar(value="失败 0　中止 0"), "right": tk.StringVar(value="失败 0　中止 0")}
        self.storage_vars = {
            "left": tk.StringVar(value="保存位置：episodes/left/episode_0001"),
            "right": tk.StringVar(value="保存位置：episodes/right/episode_0001"),
        }
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.on_window_close)
        self.root.bind("<Escape>", self.on_ignored_key); self.root.bind("q", self.on_ignored_key); self.root.bind("Q", self.on_ignored_key)
        self.preview.start(); self.teleop.start(); self.control.request("status")
        self.root.after(50, self.tick)
        if args.automated_smoke_test:
            self.smoke_phase = 0; self.smoke_started = time.monotonic(); self.root.after(500, self.smoke_tick)

    def _build(self) -> None:
        self.root.title("OpenArm 蘑菇采摘 RGB-D 数据采集控制台")
        self.root.geometry("1500x820"); self.root.minsize(1100, 680); self.root.configure(bg=BG)
        top = tk.Frame(self.root, bg=BG, height=52); top.pack(fill="x", padx=14, pady=(7, 3)); top.pack_propagate(False)
        top.grid_columnconfigure(1, weight=1)
        tk.Label(top, text="OpenArm｜蘑菇采摘 RGB-D 数据采集控制台", bg=BG, fg=TEXT,
                 font=("Noto Sans CJK SC", 17, "bold")).grid(row=0, column=0, sticky="w", padx=6, pady=7)
        self.teleop_status_label = tk.Label(top, textvariable=self.status_var, bg=BG, fg="#79d65a",
                                            font=("Noto Sans CJK SC", 10), anchor="center")
        self.teleop_status_label.grid(row=0, column=1, sticky="ew", padx=10)
        self.close_button = tk.Button(top, text="结束本次采集会话", command=self.on_close_session,
                                      bg="#3b4449", fg="white", activebackground="#56636a",
                                      font=("Noto Sans CJK SC", 10, "bold"), relief="flat", padx=12)
        self.close_button.grid(row=0, column=2, sticky="e", padx=6, pady=7)

        cameras = tk.Frame(self.root, bg=BG); cameras.pack(fill="both", expand=True, padx=18, pady=4)
        for column in range(3): cameras.grid_columnconfigure(column, weight=1, uniform="camera")
        for column, role in enumerate(ROLES):
            panel = tk.Frame(cameras, bg=PANEL, highlightbackground=BORDER, highlightthickness=2)
            panel.grid(row=0, column=column, sticky="nsew", padx=6)
            tk.Label(panel, text=CAMERA_TITLES[role], bg=PANEL, fg=TEXT,
                     font=("Noto Sans CJK SC", 16, "bold")).pack(pady=(8, 2))
            tk.Label(panel, textvariable=self.camera_metric_vars[role], bg=PANEL, fg="#6fdb59",
                     font=("Noto Sans CJK SC", 10)).pack()
            image = tk.Label(panel, bg="#000000", text="等待实时画面…", fg=MUTED,
                             font=("Noto Sans CJK SC", 14))
            image.pack(fill="both", expand=True, padx=6, pady=(4, 6)); setattr(self, role + "_image", image)

        tasks = tk.Frame(self.root, bg=BG, height=205); tasks.pack(fill="x", padx=18, pady=5); tasks.pack_propagate(False)
        tasks.grid_columnconfigure(0, weight=1, uniform="task"); tasks.grid_columnconfigure(1, weight=1, uniform="task")
        for column, side in enumerate(("left", "right")):
            _, title, color = TASKS[side]
            card = tk.Frame(tasks, bg=PANEL, highlightbackground=color, highlightthickness=2)
            card.grid(row=0, column=column, sticky="nsew", padx=6); tasks.grid_rowconfigure(0, weight=1)
            tk.Label(card, text=title, bg=PANEL, fg=color, font=("Noto Sans CJK SC", 15, "bold")).pack(anchor="w", padx=18, pady=(8, 2))
            tk.Label(card, textvariable=self.count_vars[side], bg=PANEL, fg=TEXT,
                     font=("Noto Sans CJK SC", 19, "bold")).pack(anchor="w", padx=24)
            tk.Label(card, textvariable=self.subcount_vars[side], bg=PANEL, fg=MUTED,
                     font=("Noto Sans CJK SC", 11)).pack(anchor="w", padx=30)
            tk.Label(card, textvariable=self.storage_vars[side], bg=PANEL, fg="#79cfe8",
                     font=("Noto Sans Mono CJK SC", 10)).pack(anchor="w", padx=30, pady=(2, 0))
            ready = self.left_ready if side == "left" else self.right_ready
            arm_name = "左" if side == "left" else "右"
            tk.Checkbutton(card, text=f"我已通过遥操将{arm_name}从臂调至 READY", variable=ready,
                           bg=PANEL, fg=TEXT, selectcolor="#263238", activebackground=PANEL,
                           activeforeground=TEXT, font=("Noto Sans CJK SC", 10)).pack(anchor="w", padx=21, pady=(4, 1))
            if side == "right":
                tk.Checkbutton(card, text="右腕目标框内只有一朵明确待采蘑菇", variable=self.target_confirmed,
                               bg=PANEL, fg=TEXT, selectcolor="#263238", activebackground=PANEL,
                               activeforeground=TEXT, font=("Noto Sans CJK SC", 10)).pack(anchor="w", padx=21)
            button = tk.Button(card, text=f"开始录制{arm_name}臂第 0001 条", command=lambda value=side: self.on_start(value),
                               bg=color, fg="#071009", activebackground=color, font=("Noto Sans CJK SC", 12, "bold"),
                               relief="flat", pady=5)
            button.pack(fill="x", padx=21, pady=(4, 7)); setattr(self, "start_" + side + "_button", button)

        footer = tk.Frame(self.root, bg=PANEL, height=112, highlightbackground=BORDER, highlightthickness=1)
        footer.pack(fill="x", padx=24, pady=(0, 10)); footer.pack_propagate(False)
        tk.Label(footer, textvariable=self.message_var, bg=PANEL, fg="#5be1e6",
                 font=("Noto Sans CJK SC", 10)).pack(fill="x", padx=18, pady=(6, 4))
        row = tk.Frame(footer, bg=PANEL); row.pack(fill="both", expand=True, padx=14, pady=(0, 8))
        common = dict(fg="white", relief="flat", font=("Noto Sans CJK SC", 11, "bold"), pady=6)
        self.success_button = tk.Button(row, text="成功并保存", command=lambda: self.on_result("success"), bg="#18883e", **common)
        self.failure_button = tk.Button(row, text="失败并保存", command=lambda: self.on_result("failure"), bg="#a46620", **common)
        self.abort_button = tk.Button(row, text="中止本条", command=lambda: self.on_result("aborted"), bg="#525d62", **common)
        self.safe_end_button = tk.Button(row, text="安全结束本条采集（中止并保存）", command=self.on_safe_end, bg="#c53232", **common)
        for button in (self.success_button, self.failure_button, self.abort_button, self.safe_end_button):
            button.pack(side="left", fill="both", expand=True, padx=6)

    def on_ignored_key(self, _event: tk.Event) -> str:
        self.set_notice("Esc/Q 不会结束采集；请使用可见的结果按钮或“安全结束本条采集”。")
        return "break"

    def set_notice(self, message: str, seconds: float = 5.0) -> None:
        """Keep operator feedback visible instead of losing it on the next 50 ms refresh."""
        self.operator_notice = message
        self.operator_notice_until = time.monotonic() + seconds
        self.message_var.set(message)

    def on_start(self, side: str) -> None:
        state, _, pending = self.control.snapshot()
        if pending:
            self.set_notice("上一个操作仍在处理，请勿重复点击。")
            return
        if state.get("running"):
            self.set_notice("已有一条 episode 正在录制。")
            return
        teleop = self.teleop.snapshot()
        if teleop.get("state") != "RUNNING" or int(teleop.get("fault_bits") or 0) != 0:
            messagebox.showerror(
                "遥操未运行",
                f"当前从端状态：{teleop.get('state', '未知')}。\n"
                "本条不会开始录制。请先用桌面一键启动器完成自动归零并进入 RUNNING。",
            )
            return
        if state.get("camera_health", {}).get("ok") is not True:
            messagebox.showwarning("不能开始", "三路 RGB-D 相机尚未全部健康。")
            return
        ready = self.left_ready.get() if side == "left" else self.right_ready.get()
        if not ready:
            messagebox.showwarning("READY 未确认", f"请先确认{'左' if side == 'left' else '右'}从臂已经到达对应 READY。")
            return
        if side == "right" and not self.target_confirmed.get():
            messagebox.showwarning("目标未确认", "请确认右腕目标框内只有一朵明确待采蘑菇。")
            return
        task = TASKS[side][0]
        if self.args.automated_smoke_test:
            task = "TEST_" + task
        self.control.request("episode_start", task=task, target="ui_smoke_test" if self.args.automated_smoke_test else "")

    def on_result(self, result: str) -> None:
        state, _, pending = self.control.snapshot()
        if not state.get("running") or pending:
            self.set_notice("当前没有可结束的活动 episode，或封口仍在进行。")
            return
        if result in {"success", "failure"} and state.get("phase") != "recording":
            self.set_notice("本条仍在初始化，尚未进入正式录制；请等待状态显示“正在录制”后再标记结果。")
            return
        failure_code = "operator_marked_failure" if result == "failure" else "operator_aborted" if result == "aborted" else ""
        self.control.request("episode_stop", result=result, failure_code=failure_code)

    def on_safe_end(self) -> None:
        state, _, pending = self.control.snapshot()
        if not state.get("running") or pending:
            self.set_notice("当前没有正在录制的 episode。")
            return
        if not self.args.automated_smoke_test and not messagebox.askyesno(
                "安全结束本条采集", "本条将标记为中止并保存，不计入成功训练数据。\n机械臂遥操将继续运行。是否继续？"):
            return
        self.control.request("episode_stop", result="aborted", failure_code="operator_safe_end")

    def on_close_session(self) -> None:
        state, _, pending = self.control.snapshot()
        if state.get("running"):
            messagebox.showwarning("正在录制", "请先使用成功、失败或安全结束按钮封口当前 episode。")
            return
        if pending:
            self.set_notice("请等待当前操作完成。")
            return
        if not self.args.automated_smoke_test and not messagebox.askyesno(
                "结束本次采集会话", "结束会话只关闭采集界面，不会停止机械臂遥操。是否继续？"):
            return
        self.close_requested = self.control.request("session_close")

    def on_window_close(self) -> None:
        state, _, _ = self.control.snapshot()
        if state.get("running"):
            if not messagebox.askyesno("正在录制", "关闭窗口会把当前条标记为中止并封口，遥操继续运行。是否关闭？"):
                return
            self.control.abort_synchronously("collection_window_closed")
        else:
            # Closing an idle window is an intentional end of this collection
            # batch.  Unexpected process termination never reaches this code,
            # so crash recovery remains intact.
            self.control.close_session_synchronously()
        self.preview.stop.set(); self.teleop.stop.set(); self.root.destroy()

    def _set_button_states(self, state: dict[str, Any], pending: bool) -> None:
        running = bool(state.get("running")); phase = str(state.get("phase", "idle"))
        # Keep buttons physically clickable whenever no request is in flight.
        # Their callbacks explain unmet prerequisites (READY, camera health or
        # no active episode). A grey inert button was indistinguishable from a
        # broken UI to operators and provided no corrective guidance.
        can_start = not running and not pending and phase not in {"starting", "stopping"}
        can_finish = not pending and phase == "recording"
        can_abort = not pending and phase in {"starting", "recording"}
        for button in (self.start_left_button, self.start_right_button): button.configure(state="normal" if can_start else "disabled")
        for button in (self.success_button, self.failure_button): button.configure(state="normal" if can_finish else "disabled")
        for button in (self.abort_button, self.safe_end_button): button.configure(state="normal" if can_abort else "disabled")
        self.close_button.configure(state="normal" if not running and not pending else "disabled")

    def _tick_once(self) -> None:
        now = time.time(); latest = self.preview.latest()
        if latest:
            frames, metrics = latest
            for role in ROLES:
                frame = frames[role]
                if role == "right_wrist":
                    frame = frame.copy(); w, h = frame.size
                    ImageDraw.Draw(frame).rectangle(
                        (int(w * .40), int(h * .30), int(w * .62), int(h * .66)),
                        outline=(65, 220, 80), width=2,
                    )
                pil = frame; pil.thumbnail((440, 230), LANCZOS)
                photo = ImageTk.PhotoImage(pil); self.photos[role] = photo
                getattr(self, role + "_image").configure(image=photo, text="")
                self.camera_metric_vars[role].set(f"{metrics[role + '_fps']:.1f} FPS　{metrics[role + '_age_ms']:.0f} ms")
        preview_age = getattr(self.preview, "age_s", lambda: 0.0)()
        if preview_age > 1.5:
            for role in ROLES:
                getattr(self, role + "_image").configure(image="", text="实时画面已中断\n等待自动恢复…")
                self.camera_metric_vars[role].set("预览断线")
        if now - self.last_status >= 1.0:
            self.control.request("status"); self.last_status = now
        state, message, pending = self.control.snapshot(); health = state.get("camera_health", {})
        teleop = self.teleop.snapshot()
        running = bool(state.get("running")); phase = str(state.get("phase", "idle"))
        active = state.get("active_episode") or {}
        free_gb = state.get("free_gb", "?"); camera_text = "3/3 健康" if health.get("ok") else "相机异常"
        teleop_running = teleop.get("state") == "RUNNING" and int(teleop.get("fault_bits") or 0) == 0
        self.teleop_status_label.configure(fg="#79d65a" if teleop_running else "#ff5656")
        recording_label = {
            "starting": "录制准备中",
            "recording": "正在录制",
            "stopping": "正在封装",
            "error": "录制异常",
        }.get(phase, "未录制")
        self.status_var.set(
            f"遥操 {teleop.get('state', 'CHECKING')}　｜　相机 {camera_text}　｜　"
            f"磁盘 {free_gb} GB　｜　{recording_label}"
        )
        # A single SSH poll can fail transiently. Abort only after the follower
        # has not been authoritatively RUNNING for three seconds.
        if (running and not pending and not self.teleop_abort_requested
                and float(teleop.get("last_running_age_s", float("inf"))) > 3.0):
            self.teleop_abort_requested = self.control.request(
                "episode_stop", result="aborted", failure_code="teleoperation_not_running"
            )
            message = "遥操已离开 RUNNING，本条正在自动中止并封口。"
        elif not running:
            self.teleop_abort_requested = False
        if not teleop_running and not running:
            message = f"禁止开始采集：遥操状态为 {teleop.get('state', 'CHECKING')}，请先通过桌面一键启动进入 RUNNING。"
        elif health.get("ok") is not True and not running:
            message = "禁止开始采集：三路 RGB-D 相机尚未全部健康。"
        if time.monotonic() < self.operator_notice_until:
            message = self.operator_notice
        elapsed = ""
        recording_started = active.get("recording_started_unix_s")
        if phase == "recording" and recording_started:
            elapsed = f"　已录制 {int(now - float(recording_started))} 秒"
        active_detail = ""
        if active:
            active_detail = f"　编号：{active.get('episode_id')}　任务：{active.get('task')}"
        self.message_var.set(message + active_detail + elapsed)
        statistics = state.get("task_statistics", {})
        next_numbers = state.get("next_episode_by_task", {})
        for side in ("left", "right"):
            values = statistics.get(side, {})
            next_number = int(next_numbers.get(side, 1))
            self.count_vars[side].set(
                f"本批次有效成功：{int(values.get('valid_success', 0))} 条　｜　本批次下一条：第 {next_number} 条"
            )
            self.subcount_vars[side].set(f"失败 {int(values.get('failure', 0))}　中止 {int(values.get('aborted', 0))}")
            self.storage_vars[side].set(f"保存位置：episodes/{side}/episode_{next_number:04d}")
            arm_name = "左" if side == "left" else "右"
            getattr(self, "start_" + side + "_button").configure(
                text=f"开始录制{arm_name}臂｜本批次第 {next_number:04d} 条"
            )
        self._set_button_states(state, pending)
        # Close only after the manager acknowledges that the session is gone.
        # A rejected/failed request must leave the UI open with its error.
        if self.close_requested and not pending and not state.get("session_id"):
            if not self.closing:
                if self.args.automated_smoke_test:
                    print("UI_SMOKE_TEST: PASS", flush=True)
                self.closing = True
                self.preview.stop.set(); self.teleop.stop.set(); self.root.after(250, self.root.destroy)

    def tick(self) -> None:
        """Keep the operator controls alive even if one preview frame is bad."""
        try:
            self._tick_once()
        except Exception as exc:
            self.set_notice(f"画面刷新异常，控制按钮仍可使用：{exc}")
            print(f"UI_REFRESH_ERROR: {exc}", file=sys.stderr, flush=True)
        if not self.closing:
            self.root.after(50, self.tick)

    def smoke_tick(self) -> None:
        """Exercise the exact Tk button callbacks against live Jetson services."""
        state, _, pending = self.control.snapshot(); phase = str(state.get("phase", "idle")); now = time.monotonic()
        if now - self.smoke_started > 150:
            print("UI_SMOKE_TEST: TIMEOUT", file=sys.stderr, flush=True)
            self.preview.stop.set(); self.teleop.stop.set(); self.root.destroy(); return
        teleop = self.teleop.snapshot()
        teleop_ready = teleop.get("state") == "RUNNING" and int(teleop.get("fault_bits") or 0) == 0
        if (self.smoke_phase == 0 and state.get("session_id")
                and state.get("camera_health", {}).get("ok") and teleop_ready and not pending):
            self.left_ready.set(True); self.start_left_button.invoke(); self.start_left_button.invoke(); self.smoke_phase = 1
        elif self.smoke_phase == 1 and state.get("running") and phase == "recording":
            self.smoke_recording_at = now; self.smoke_phase = 2
        elif self.smoke_phase == 2 and now - self.smoke_recording_at >= 3:
            self.safe_end_button.invoke(); self.smoke_phase = 3
        elif self.smoke_phase == 3 and not state.get("running") and not pending and (state.get("last_episode") or {}).get("result") == "aborted":
            # Call the same callback directly.  A freshly-finalized manager
            # state can arrive a few milliseconds before Tk redraws the button
            # from disabled to normal; invoke() on that stale visual state is
            # ignored and used to make the smoke test wait forever.
            self.right_ready.set(True); self.target_confirmed.set(True); self.on_start("right"); self.smoke_phase = 4
        elif self.smoke_phase == 4 and state.get("running") and phase == "recording":
            self.smoke_recording_at = now; self.smoke_phase = 5
        elif self.smoke_phase == 5 and now - self.smoke_recording_at >= 3:
            self.failure_button.invoke(); self.smoke_phase = 6
        elif self.smoke_phase == 6 and not state.get("running") and not pending and (state.get("last_episode") or {}).get("result") == "failure":
            print("UI_SMOKE_TEST: EPISODE_BUTTONS_PASS", flush=True)
            self.close_button.invoke(); self.smoke_phase = 7
        self.root.after(200, self.smoke_tick)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--jetson", default="192.168.50.2")
    parser.add_argument("--jetson-ssh", default="openarm-jetson")
    parser.add_argument("--preview-port", type=int, default=5556)
    parser.add_argument("--record-port", type=int, default=5557)
    parser.add_argument("--automated-smoke-test", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = tk.Tk()
    # A Toplevel's geometry is relative to its Tk root on X11.  When the root
    # is withdrawn before it has ever been placed, GNOME may assign it an
    # off-screen position (especially after a monitor/layout change).  The
    # chooser then exists and has the correct size, but is translated outside
    # the visible desktop.  Anchor the hidden owner at the global origin first.
    root.geometry("1x1+0+0")
    root.update_idletasks()
    root.withdraw()
    # GNOME already applies desktop scaling. Conda/system Tk reported ~2.25
    # and scaled point fonts a second time, clipping every footer control.
    root.tk.call("tk", "scaling", 1.0)
    endpoint = f"tcp://{args.jetson}:{args.record_port}"
    if not choose_collection_session(root, endpoint):
        root.destroy(); return
    root.deiconify()
    MushroomCollectionApp(root, args); root.mainloop()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Reliable task-specific OpenArm mushroom RGB-D collection console.

The UI is a client of Jetson's recorder manager. It never opens cameras,
accesses CAN. Explicit posture buttons use the follower control socket via SSH.
Native Tk buttons are used instead of
image-coordinate hit testing, so display scaling cannot make visible controls
unclickable.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
from pathlib import Path
import queue
import select
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Any

import zmq
from PIL import Image, ImageDraw, ImageOps, ImageTk

# Same contract as the startup/continuous monitor, not another RUNNING test.
_teleop_root = Path(os.environ.get('OPENARM_TELEOP_ROOT',
    str(Path(__file__).resolve().parents[2] / 'openarm-remote-harvest')))
sys.path.insert(0, str(_teleop_root / 'scripts'))
try:
    from teleop_readiness import apply_current_report
except ImportError:
    def apply_current_report(value):
        return {**value, 'control_state':value.get('state'), 'state':'HEALTH_UNAVAILABLE',
                'teleop_ready':False, 'readiness_reason':'缺少电机健康检查模块，禁止开始采集'}

sys.path.insert(0, str(Path(__file__).resolve().parent))
from manual_left_alignment import manual_left_alignment


ROLES = ("chest", "left_wrist", "right_wrist")
CAMERA_TITLES = {"chest": "胸部全局相机", "left_wrist": "左腕相机", "right_wrist": "右腕相机"}
TASKS = {
    "left": ("LEFT_GRASP_LOG", "左臂｜菌棒夹持", "#58a9ef"),
    "right": ("RIGHT_PICK_ONE", "右臂｜单朵蘑菇采摘", "#72c94c"),
}
BG = "#090d10"; PANEL = "#10161a"; BORDER = "#344047"; TEXT = "#edf1f2"; MUTED = "#9ba8ad"
LANCZOS = getattr(getattr(Image, "Resampling", Image), "LANCZOS")


MASTER_ALIGN_PHASES = {"resetting", "preparing", "moving", "settling", "stopping", "releasing"}


def left_master_servo_free(status):
    """Only a fresh, side-specific physical ACK can prove servo release."""
    collection = status.get('collection') or {}
    detail = collection.get('left_master_align_detail') or {}
    return (isinstance(detail, dict) and collection.get('left_master_align_supported') is True
            and status.get('connected') is True
            and all(type(status.get(key)) in (int, float) and math.isfinite(status[key])
                    and 0 <= status[key] < 100 for key in ('action_age_ms', 'feedback_age_ms'))
            and detail.get('servo_released') is True
            and type(detail.get('leader_left_mode')) is int and detail['leader_left_mode'] == 0)


def left_master_busy(status):
    """Uncertain starts/stops remain motion until a later physical confirmation."""
    collection = status.get('collection') or {}
    phase = collection.get('left_master_align_phase', 'idle')
    detail = collection.get('left_master_align_detail') or {}
    return bool(status.get('left_master_align_may_be_active') or status.get('left_master_align_stop_requested')
                or collection.get('left_master_align_active') is True or phase in MASTER_ALIGN_PHASES
                or (isinstance(detail, dict) and detail.get('leader_left_mode') in (1, 2))
                or (phase in {'paused', 'failed', 'completed'}
                    and (not isinstance(detail, dict) or detail.get('servo_released') is not True
                         or type(detail.get('leader_left_mode')) is not int or detail['leader_left_mode'] != 0)))


def right_master_servo_free(status):
    """Only a fresh, side-specific physical ACK can prove servo release."""
    collection = status.get('collection') or {}
    detail = collection.get('right_master_align_detail') or {}
    return (isinstance(detail, dict) and collection.get('right_master_align_supported') is True
            and status.get('connected') is True
            and all(type(status.get(key)) in (int, float) and math.isfinite(status[key])
                    and 0 <= status[key] < 100 for key in ('action_age_ms', 'feedback_age_ms'))
            and detail.get('servo_released') is True
            and type(detail.get('leader_right_mode')) is int and detail['leader_right_mode'] == 0)


def right_master_busy(status):
    """Uncertain starts/stops remain motion until a later physical confirmation."""
    collection = status.get('collection') or {}
    phase = collection.get('right_master_align_phase', 'idle')
    detail = collection.get('right_master_align_detail') or {}
    return bool(status.get('right_master_align_may_be_active') or status.get('right_master_align_stop_requested')
                or collection.get('right_master_align_active') is True or phase in MASTER_ALIGN_PHASES
                or (phase != 'idle' and isinstance(detail, dict) and detail.get('leader_right_mode') in (1, 2))
                or (phase in {'paused', 'failed', 'completed'}
                    and (not isinstance(detail, dict) or detail.get('servo_released') is not True
                         or type(detail.get('leader_right_mode')) is not int or detail['leader_right_mode'] != 0)))


def right_servo_release_needed(status):
    collection = status.get('collection') or {}
    detail = collection.get('left_master_align_detail') or {}
    mode = detail.get('leader_right_mode') if isinstance(detail, dict) else None
    return (collection.get('right_mode') == 'HOLD'
            and (collection.get('right_servo_release_required') is True
                 or (type(mode) is int and mode in (1, 2))))


def right_servo_release_reason(status):
    """Gate an explicit HOLD-to-free servo release; ordinary stop is separate."""
    collection = status.get('collection') or {}
    if collection.get('right_pause_release_supported') is not True:
        return '当前控制器尚未加载右主臂伺服解除功能；需重新加载控制器后才能解除。'
    if collection.get('right_servo_release_pending') is True:
        return '正在等待右主臂伺服解除确认；右从臂保持，不会自动恢复跟随。'
    if (status.get('connected') is not True or status.get('teleop_ready') is not True
            or status.get('state') != 'RUNNING' or type(status.get('fault_bits')) is not int
            or status['fault_bits'] != 0):
        return '当前遥操健康状态不可确认，暂不能解除右主臂伺服：' + str(status.get('readiness_reason') or '等待实时健康检查')
    if not all(type(status.get(key)) in (int, float) and math.isfinite(status[key])
               and 0 <= status[key] < 100 for key in ('action_age_ms', 'feedback_age_ms')):
        return '主从状态已过期或缺失，等待实时反馈后才能解除右主臂伺服。'
    if collection.get('right_mode') != 'HOLD':
        return '右臂状态已变化，需确认右从臂保持后再解除主臂伺服。'
    if (collection.get('right_servo_release_required') is not True
            or collection.get('right_servo_release_pending') is not False):
        return '右主臂待解除状态未确认，暂不发送解除命令。'
    detail = collection.get('left_master_align_detail') or {}
    if (not isinstance(detail, dict) or type(detail.get('leader_right_mode')) is not int
            or detail['leader_right_mode'] not in (0, 1, 2)
            or type(detail.get('leader_left_mode')) is not int or detail['leader_left_mode'] != 0):
        return '左右主臂伺服状态不完整或左主臂仍在伺服，暂不能解除右主臂伺服。'
    if (left_master_busy(status) or collection.get('left_align_active') is not False
            or status.get('left_align_may_be_active') or status.get('left_align_stop_requested')):
        return '请先停止左臂对齐并确认结束，再解除右主臂伺服。'
    if collection.get('recording'):
        return '请先结束并保存本条录制，再解除右主臂伺服。'
    if not isinstance(collection.get('transitioning_arms'), list) or collection['transitioning_arms']:
        return '运动衔接尚未结束或状态不可确认，暂不能解除右主臂伺服。'
    return ''


def right_master_start_reason(status):
    collection = status.get('collection') or {}
    if collection.get('right_master_align_supported') is not True:
        return '控制器尚未加载右主臂自动对齐功能，请先加载修复。'
    if right_master_busy(status):
        return '右主臂正在对齐或伺服释放尚未确认，请先停止右主臂对齐并等待确认。'
    if (collection.get('right_master_align_active') is not False
            or collection.get('right_master_align_phase') not in {'idle', 'paused', 'failed', 'completed'}):
        return '右主臂自动对齐状态不完整，暂不能启动。'
    if (status.get('connected') is not True or status.get('teleop_ready') is not True
            or status.get('state') != 'RUNNING' or type(status.get('fault_bits')) is not int or status['fault_bits'] != 0
            or not all(type(status.get(key)) in (int, float) and math.isfinite(status[key])
                       and 0 <= status[key] < 100 for key in ('action_age_ms', 'feedback_age_ms'))):
        return '遥操健康状态或实时反馈不可确认，暂不能启动右主臂对齐。'
    if collection.get('left_mode') != 'HOLD' or collection.get('right_mode') != 'HOLD':
        return '请先确认左臂及右从臂保持，再自动对齐右主臂。'
    if (left_master_busy(status) or collection.get('left_align_active') is not False
            or status.get('left_align_may_be_active') or status.get('left_align_stop_requested')
            or not isinstance(collection.get('transitioning_arms'), list) or collection['transitioning_arms']):
        return '其他对齐或跟随衔接尚未结束，请等待确认。'
    if collection.get('recording'):
        return '请先结束并保存本条录制，再自动对齐右主臂。'
    if collection.get('right_servo_release_pending') is True:
        return '正在等待右主臂伺服解除确认。'
    if right_servo_release_needed(status):
        return '右主臂回位伺服尚未解除，请先点击“解除右主臂回位伺服”并确认。'
    detail = collection.get('right_master_align_detail') or {}
    if (not right_master_servo_free(status) or type(detail.get('leader_left_mode')) is not int
            or detail['leader_left_mode'] != 0):
        return '左右主臂伺服尚未确认释放，暂不能开始右主臂对齐。'
    for key in ('leader_axes', 'applied_axes'):
        axes = status.get(key)
        if (not isinstance(axes, (list, tuple)) or len(axes) != 16
                or not all(type(v) in (int, float) and math.isfinite(v) for v in axes[8:])):
            return '右臂关节及夹爪数据不完整，暂不能开始自动对齐。'
    return ''


def right_recording_posture_reason(status):
    collection = status.get('collection') or {}
    prefix = '录右臂需要：左臂保持，右臂跟随。\n'
    if collection.get('left_mode') != 'HOLD':
        return prefix + '请点击左侧“保持左臂及夹爪”。'
    prefix += '左臂已经保持，无需恢复左臂跟随。\n'
    if collection.get('right_servo_release_pending') is True:
        return prefix + '正在等待右主臂伺服解除确认；右从臂保持，不会自动恢复跟随。'
    if right_master_busy(status):
        return prefix + '右主臂自动对齐或伺服释放尚未结束，请等待完成；需中断时点击“停止右主臂对齐”。'
    if collection.get('right_mode') == 'RETURNING':
        return prefix + '右臂正在回位或等待伺服释放，请等待完成。'
    if collection.get('right_mode') == 'FOLLOW':
        return ''
    if right_servo_release_needed(status):
        return prefix + (right_servo_release_reason(status) or '请点击右侧“解除右主臂回位伺服”并确认；解除后可自动对齐右主臂。')
    return prefix + (right_master_start_reason(status) or '请点击右侧“主臂自动对齐并恢复右臂跟随”，按弹窗确认；无需手动对齐关节或夹爪。')


def left_master_start_reason(status):
    """Shared button and transport gate; the runtime rechecks before motion."""
    collection = status.get('collection') or {}
    if collection.get('left_master_align_supported') is not True:
        return '运行中的控制器尚未加载左主臂自动对齐功能；需确认现场可四臂归位后重启加载。'
    if right_master_busy(status):
        return "请先停止右主臂对齐并确认释放，再启动左主臂对齐。"
    if left_master_busy(status):
        return '左主臂对齐或伺服释放尚未确认结束，请先停止并等待确认。'
    if collection.get('left_master_align_active') is not False or collection.get('left_master_align_phase') not in {'idle', 'paused', 'failed', 'completed'}:
        return '左主臂对齐状态不完整，暂不能启动。'
    if collection.get('right_servo_release_pending') is True:
        return '正在等待右主臂伺服解除确认；确认后才能启动左主臂对齐。'
    # The manual readout already validates every joint and gripper, freshness,
    # HOLD, recording, transitions and the backend's held-target reference.
    check = manual_left_alignment(status)
    if not check.get('can_adjust'):
        return check['reason']
    detail = collection.get('left_master_align_detail') or {}
    if right_servo_release_needed(status):
        if collection.get('right_pause_release_supported') is not True:
            return '右主臂回位伺服尚未解除，阻止左主臂对齐；当前控制器需重新加载解除功能。'
        return '右主臂回位伺服尚未解除；请先点击“解除右主臂回位伺服”，确认解除后再启动左主臂对齐。'
    if (not left_master_servo_free(status) or type(detail.get('leader_right_mode')) is not int
            or detail['leader_right_mode'] != 0):
        return '等待左右主臂伺服均已解除的实时确认。'
    return ''


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
    if (catalog.get("phase") == "awaiting_result" and catalog.get("session_id")
            and catalog.get("active_episode")):
        # An interrupted episode already belongs to this batch. Reopen its
        # result controls without creating a batch or advancing its number.
        return True
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
                if not isinstance(response, dict):
                    raise ValueError("录制服务返回的状态不是 JSON 对象")
                with self._lock:
                    if command == "status" and self._pending and self._value.get("phase") == "stopping":
                        response["phase"] = "stopping"
                    self._value = response
                    if not response.get("ok"):
                        self._message = "请求被拒绝：" + str(response.get("error", "未知错误"))
                    elif response.get("phase") == "stopping":
                        self._message = "正在安全封口；机械臂遥操继续运行。"
                    elif response.get("phase") == "starting":
                        self._message = "本条正在准备，尚未正式录制；请等待“正在录制”提示。"
                    elif response.get("phase") == "awaiting_result":
                        episode = response.get("active_episode") or {}
                        reason = response.get("capture_interrupted") or episode.get("capture_interrupted") or "录制已中断"
                        self._message = f"本条已中断，等待保存：{reason}。请选择“成功并保存”或“失败并保存”；中断数据不会标为有效。"
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
        self.requests = queue.Queue(maxsize=1)
        self.results = queue.Queue()
        self.pending = False
        self.request_error = ""
        self.left_align_may_be_active = False
        self.left_align_stop_requested = False
        self.left_master_align_may_be_active = False
        self.left_master_align_stop_requested = False
        self.right_master_align_may_be_active = False
        self.right_master_align_stop_requested = False
        self.thread = threading.Thread(target=self._run, daemon=True, name="teleop-status-monitor")

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        status_probe = '''import json,socket,tempfile
with tempfile.TemporaryDirectory(prefix="oa_ui_status_") as temporary:
 with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as client:
  client.bind(temporary+"/reply");client.settimeout(.5)
  client.sendto(b'{"command":"status"}',"/tmp/openarm_remote_runtime.sock")
  reply=client.recv(65536).decode();print(reply)
  if "error" in json.loads(reply):raise SystemExit(2)
'''
        remote = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install/setup.bash && "
            "source /home/nvidia/dev/openarm-remote-harvest/ros2_robot/install_bimanual/setup.bash && "
            "ros2 run remote_teleop_runtime remote-teleop-control "
        )
        while not self.stop.is_set():
            try:
                command = self.requests.get_nowait()
            except queue.Empty:
                command = "status"
            try:
                if command == 'right_servo_release':
                    reason = right_servo_release_reason(self.snapshot())
                    if reason:
                        self.request_error = reason
                        self.results.put((command, {'error': reason}))
                        self.pending = not self.requests.empty()
                        continue
                if command == 'right_master_align':
                    # Ignore only this worker's own queued-start uncertainty.
                    current = self.snapshot()
                    current['right_master_align_may_be_active'] = False
                    reason = right_master_start_reason(current)
                    if reason:
                        self.request_error = reason
                        self.right_master_align_may_be_active = self.right_master_align_stop_requested
                        self.results.put((command, {'error': reason}))
                        self.pending = not self.requests.empty()
                        continue
                # New alignment commands use the same runtime socket without
                # ROS startup latency; the runtime retains all motion gates.
                fast_socket = command in {"status", "left_follow", "left_align_pause", "left_master_align", "left_master_pause", "right_master_align", "right_master_pause", "right_pause", "right_servo_release"}
                remote_command = "/usr/bin/python3 -" if fast_socket else remote + command
                payload = ({'command': 'right_pause', 'release_servo': True}
                           if command == 'right_servo_release' else {'command': command})
                probe = status_probe.replace('{"command":"status"}', json.dumps(payload, separators=(',', ':')))
                result = subprocess.run(
                    ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=2", self.ssh_host, remote_command],
                    input=probe if fast_socket else None,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                    timeout=3 if fast_socket else 4, check=True,
                )
                value = parse_json_output(result.stdout)
                if "error" in value:
                    raise RuntimeError(value["error"])
                value["connected"] = True
                collection = value.get("collection") or {}
                align_done = (collection.get("left_align_active") is False
                              and collection.get("left_align_phase") in {"completed", "failed", "paused"})
                stop_ack = (command == 'left_align_pause' and collection.get('left_align_active') is False
                            and collection.get('left_mode') == 'HOLD')
                if stop_ack:
                    # An explicit stop ACK may remain idle if the queued
                    # start was cancelled before it reached the controller.
                    align_done = True
                # An older status poll may finish after the operator enqueues
                # start. Its previous terminal phase cannot clear that request.
                can_confirm_end = command in {'left_align_follow', 'left_align_pause'} or (command == 'status' and not self.pending)
                hold_confirmed = (collection.get('left_align_active') is False
                                  and collection.get('left_mode') == 'HOLD'
                                  and (command == 'left_align_pause' or (command == 'status' and not self.pending)))
                if self.left_align_stop_requested and hold_confirmed:
                    self.left_align_stop_requested = False
                    self.left_align_may_be_active = False
                elif not self.left_align_stop_requested and can_confirm_end and align_done and (stop_ack or collection.get("left_align_phase") != "completed" or (
                        collection.get("left_mode") == "FOLLOW" and
                        "left" not in collection.get("transitioning_arms", []))):
                    self.left_align_may_be_active = False
                if left_master_busy(value):
                    self.left_master_align_may_be_active = True
                master_idle = (collection.get('left_master_align_active') is False
                               and left_master_servo_free(value))
                master_phase = collection.get('left_master_align_phase')
                master_status_current = command == 'status' and not self.pending
                master_hold = master_idle and collection.get('left_mode') == 'HOLD'
                if (self.left_master_align_stop_requested and master_hold
                        and master_phase in {'idle', 'paused', 'failed'}
                        and (command == 'left_master_pause' or master_status_current)):
                    self.left_master_align_stop_requested = False
                    self.left_master_align_may_be_active = False
                elif (not self.left_master_align_stop_requested and master_idle
                      and (command in {'left_master_align', 'left_master_pause'} or master_status_current)):
                    completed = (master_phase == 'completed' and collection.get('left_mode') == 'FOLLOW'
                                 and 'left' not in collection.get('transitioning_arms', ['left']))
                    if completed or (master_hold and master_phase in {'paused', 'failed'}):
                        self.left_master_align_may_be_active = False
                if right_master_busy(value):
                    self.right_master_align_may_be_active = True
                master_idle = (collection.get('right_master_align_active') is False
                               and right_master_servo_free(value))
                master_phase = collection.get('right_master_align_phase')
                master_status_current = command == 'status' and not self.pending
                master_hold = master_idle and collection.get('right_mode') == 'HOLD'
                if (self.right_master_align_stop_requested and master_hold
                        and master_phase in {'idle', 'paused', 'failed'}
                        and (command == 'right_master_pause' or master_status_current)):
                    self.right_master_align_stop_requested = False
                    self.right_master_align_may_be_active = False
                elif (not self.right_master_align_stop_requested and master_idle
                      and (command in {'right_master_align', 'right_master_pause'} or master_status_current)):
                    completed = (master_phase == 'completed' and collection.get('right_mode') == 'FOLLOW'
                                 and 'right' not in collection.get('transitioning_arms', ['right']))
                    if completed or (master_hold and master_phase in {'paused', 'failed'}):
                        self.right_master_align_may_be_active = False
                if apply_current_report(value).get('teleop_ready') is True:
                    self._last_running = time.monotonic()
            except Exception as exc:
                detail = str(exc)
                rejected = False
                if isinstance(exc, subprocess.CalledProcessError):
                    try:
                        reply = parse_json_output(exc.stdout)
                        detail = reply.get("error", detail)
                        rejected = bool(reply.get("error"))
                    except ValueError:
                        pass
                if rejected:
                    # A valid rejection (e.g. not aligned) proves connectivity;
                    # it is not a network outage and must not falsify RUNNING.
                    with self._lock:
                        value = {**self._value, "connected": True, "error": detail}
                    if command == "left_align_follow":
                        self.left_align_may_be_active = self.left_align_stop_requested
                    if command == 'left_master_align':
                        self.left_master_align_may_be_active = self.left_master_align_stop_requested
                    if command == 'right_master_align':
                        self.right_master_align_may_be_active = self.right_master_align_stop_requested
                else:
                    # Keep the last collection mode for a manual stop while
                    # marking it unconfirmed; never replay a timed-out motion.
                    with self._lock:
                        value = {**self._value, "state": "DISCONNECTED", "fault_bits": None,
                                 "connected": False, "error": "请求或状态未确认：" + detail}
            if command != "status":
                self.results.put((command, value))
                self.pending = not self.requests.empty()
            value["last_running_age_s"] = (
                time.monotonic() - self._last_running if self._last_running else float("inf")
            )
            with self._lock:
                self._value = value
            self.stop.wait(0.25)

    def request(self, command):
        self.request_error = ""
        if command == 'left_align_follow':
            self.request_error = "从臂自动对齐入口已关闭，请使用“主臂自动对齐并恢复左臂跟随”。"
            return False
        if command not in {"left_lock", "left_follow", "left_align_follow", "left_align_pause",
                           "left_master_align", "left_master_pause", "right_master_align", "right_master_pause",
                           "right_save", "right_return", "right_pause", "right_servo_release", "right_follow"}:
            self.request_error = "不支持的姿态操作。"
            return False
        snapshot = self.snapshot()
        collection = snapshot.get('collection') or {}
        if command == 'left_lock' and left_master_busy(snapshot):
            command = 'left_master_pause'
        if command == 'right_master_align':
            self.request_error = right_master_start_reason(snapshot)
            if self.request_error:
                return False
        elif command == 'right_master_pause':
            if collection.get('right_master_align_supported') is not True and not right_master_busy(snapshot):
                self.request_error = '当前控制器未确认支持右主臂停止命令。'
                return False
        elif command not in {'left_master_pause', 'left_align_pause', 'right_pause'} and right_master_busy(snapshot):
            self.request_error = '请先停止右主臂对齐并确认伺服解除。'
            return False
        if command == 'right_servo_release':
            self.request_error = right_servo_release_reason(snapshot)
            if self.request_error:
                return False
        if command == 'left_master_align':
            self.request_error = left_master_start_reason(snapshot)
            if self.request_error:
                return False
        elif command == 'left_master_pause':
            # A lost capability/status sample must never remove the stop path
            # for a motion already observed or requested by this process.
            if collection.get('left_master_align_supported') is not True and not left_master_busy(snapshot):
                self.request_error = '当前控制器未确认支持左主臂停止命令。'
                return False
        elif command not in {'left_align_pause', 'right_pause', 'right_master_pause'} and left_master_busy(snapshot):
            self.request_error = '请先停止左主臂对齐并确认主臂伺服解除，再执行其他姿态操作。'
            return False
        if command in {'left_align_follow', 'left_align_pause'} and collection.get('left_align_supported') is not True:
            self.request_error = "当前控制器不支持左臂低速对齐，请先更新控制器；本次未发送运动命令。"
            return False
        if command == 'left_align_follow' and self.left_align_may_be_active:
            self.request_error = "左臂对齐可能仍在进行，请先核实状态或点击“停止从臂对齐”。"
            return False
        # Explicit stop may queue behind one in-flight request. Never queue a
        # second movement request or silently repeat an uncertain command.
        if self.pending:
            if command not in {'left_align_pause', 'left_master_pause', 'right_master_pause'}:
                self.request_error = "前一个姿态操作尚未完成，请等待控制器状态。"
                return False
            try:
                queued = self.requests.get_nowait()
            except queue.Empty:
                queued = None  # The worker is already executing the request.
            cancellable = {'left_master_pause': 'left_master_align', 'right_master_pause': 'right_master_align'}.get(command, 'left_align_follow')
            if queued is not None and queued != cancellable:
                self.requests.put_nowait(queued)
                self.request_error = "已有停止或姿态请求等待确认，请勿重复点击。"
                return False
            # If start was still queued, replace it with one explicit stop;
            # keep uncertainty until the controller acknowledges the stop.
        if command not in {'left_lock','right_pause','left_align_pause','left_master_pause','right_master_pause'} and not snapshot.get('teleop_ready'):
            self.request_error = "不能执行姿态操作：" + snapshot.get('readiness_reason', '遥操健康检查未通过')
            return False
        try:
            self.requests.put_nowait(command)
        except queue.Full:
            self.request_error = "已有姿态请求等待处理，请勿重复点击。"
            return False
        self.pending = True
        if command in {'left_align_follow', 'left_align_pause'}:
            self.left_align_may_be_active = True
        if command == 'left_align_pause':
            self.left_align_stop_requested = True
        if command in {'right_master_align', 'right_master_pause'}:
            self.right_master_align_may_be_active = True
        if command == 'right_master_pause':
            self.right_master_align_stop_requested = True
        if command in {'left_master_align', 'left_master_pause'}:
            self.left_master_align_may_be_active = True
        if command == 'left_master_pause':
            self.left_master_align_stop_requested = True
        return True

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            value = dict(self._value)
        value = apply_current_report(value)
        value['left_align_may_be_active'] = self.left_align_may_be_active
        value['left_align_stop_requested'] = self.left_align_stop_requested
        value['left_master_align_may_be_active'] = self.left_master_align_may_be_active
        value['left_master_align_stop_requested'] = self.left_master_align_stop_requested
        value['right_master_align_may_be_active'] = self.right_master_align_may_be_active
        value['right_master_align_stop_requested'] = self.right_master_align_stop_requested
        value['last_running_age_s'] = time.monotonic()-self._last_running if self._last_running else float('inf')
        return value


class PowerDownClient:
    """Read coordinator progress off the Tk thread; never access robot hardware here."""

    def __init__(self, args):
        self.args = args
        self.events = queue.Queue()
        self.pending = False

    def start(self):
        if self.pending:
            return False
        self.pending = True
        threading.Thread(target=self._run, daemon=True, name="arm-power-down").start()
        return True

    def _run(self):
        process = None
        completed = False
        try:
            command = ["/usr/bin/python3", str(_teleop_root / "scripts/power_down_arms.py"),
                       "--confirm", "--jetson", self.args.jetson_ssh,
                       "--recorder", f"tcp://{self.args.jetson}:{self.args.record_port}"]
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 150
            buffer = b""
            detail = ""
            while time.monotonic() < deadline:
                if not select.select([process.stdout], [], [], .2)[0]:
                    continue
                chunk = os.read(process.stdout.fileno(), 65536)
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    detail = line.decode(errors="replace")[-500:]
                    try:
                        event = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(event, dict) and event.get("event") in {"progress", "device_result", "complete"}:
                        self.events.put(event)
                        completed = completed or event["event"] == "complete"
            if not completed:
                raise RuntimeError("下电结果未确认（进程退出或超时），不会自动恢复运动。" + detail)
        except Exception as exc:
            self.events.put({"event": "complete", "ok": False, "message": str(exc)})
        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()  # The coordinator only; never kill a control service.
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=2)
                process.stdout.close()
            self.pending = False


class MushroomCollectionApp:
    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root; self.args = args
        self.control = SessionControl(args.jetson, args.record_port)
        self.preview = PreviewReceiver(f"tcp://{args.jetson}:{args.preview_port}")
        self.teleop = TeleopMonitor(args.jetson_ssh)
        self.power_down = PowerDownClient(args)
        self.power_down_busy = False
        self.power_down_locked = False
        self.power_down_confirming = False
        self.power_down_message = ""
        self.power_down_succeeded = False
        self.power_down_dialog = None
        self.power_down_detail = tk.StringVar(value="")
        self.last_status = 0.0; self.photos: dict[str, ImageTk.PhotoImage] = {}; self.close_requested = False
        self.closing = False
        self.operator_notice = ""
        self.left_align_notice = ""
        self.left_align_may_be_active = False
        self.left_align_stop_requested = False
        self.left_master_align_notice = ""
        self.right_master_align_notice = ""
        self.right_master_align_may_be_active = False
        self.right_master_align_stop_requested = False
        self.left_master_align_may_be_active = False
        self.left_master_align_stop_requested = False
        self.manual_left_dialog = None
        self.manual_left_table = None
        self.manual_left_summary = tk.StringVar(value="")
        self.manual_left_result = ""
        self.manual_left_feedback = tk.StringVar(value="")
        self.manual_left_follow_attempted = False
        self.manual_left_completed = False
        self.operator_notice_until = 0.0
        self.motion_vars = {side: tk.StringVar(value="正在读取姿态控制状态…") for side in ("left", "right")}
        self.motion_buttons = {}
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
        height = min(1100, max(680, self.root.winfo_screenheight() - 100))
        self.root.geometry(f"1800x{height}"); self.root.minsize(1100, 680); self.root.configure(bg=BG)
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
        self.power_down_button = tk.Button(top, text="机械臂下电…", command=self.on_power_down,
            bg="#933a35", fg="white", activebackground="#b74a42",
            font=("Noto Sans CJK SC", 10, "bold"), relief="flat", padx=12)
        self.power_down_button.grid(row=0, column=3, sticky="e", padx=6, pady=7)

        cameras = tk.Frame(self.root, bg=BG); cameras.pack(fill="both", expand=True, padx=18, pady=4)
        for column in range(3): cameras.grid_columnconfigure(column, weight=1, uniform="camera")
        cameras.grid_rowconfigure(0, weight=1)
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

        tasks = tk.Frame(self.root, bg=BG); tasks.pack(fill="x", padx=18, pady=5)
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
            controls = tk.Frame(card, bg=PANEL); controls.pack(fill="x", padx=12, pady=3)
            choices = (("left_lock", "保持左臂及夹爪"),
                       ("left_master_align", "主臂自动对齐并恢复左臂跟随"),
                       ("left_master_pause", "停止主臂对齐"),
                       ("left_align_pause", "停止从臂对齐")) if side == "left" else (
                ("right_save", "保存主从起始位"), ("right_return", "主从一起回位"),
                ("right_pause", "停止双端回位"), ("right_master_align", "主臂自动对齐并恢复右臂跟随"), ("right_master_pause", "停止右主臂对齐"))
            for index, (command, label) in enumerate(choices):
                controls.grid_columnconfigure(index % 2, weight=1)
                button = tk.Button(controls, text=label, command=lambda name=command: self.on_motion(name),
                    bg="#34434c", fg="white", font=("Noto Sans CJK SC", 11), pady=5)
                button.grid(row=index // 2, column=index % 2, sticky="ew", padx=3, pady=2)
                self.motion_buttons[command] = button
            tk.Label(card, textvariable=self.motion_vars[side], bg=PANEL, fg="#79cfe8",
                     font=("Noto Sans CJK SC", 10), wraplength=650).pack(fill="x", padx=15)
            if side == 'right':
                tk.Label(card, text='右臂录制：左臂保持，右臂跟随；无需恢复左臂跟随。',
                         bg=PANEL, fg=TEXT, font=("Noto Sans CJK SC", 10)).pack(anchor='w', padx=18)
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
        if self.power_down_locked or self.power_down_confirming:
            self.set_notice("下电流程中或机械臂已下电；恢复使用请重新通过桌面启动器启动。")
            return
        state, _, pending = self.control.snapshot()
        if pending:
            self.set_notice("上一个操作仍在处理，请勿重复点击。")
            return
        if getattr(self.teleop, "pending", False):
            self.set_notice("姿态操作尚未确认完成，请稍候。")
            return
        if state.get("running"):
            self.set_notice("已有一条 episode 尚未保存，请先完成成功或失败保存。")
            return
        teleop = self.teleop.snapshot()
        if self.master_alignment_busy(teleop) or self.right_master_alignment_busy(teleop):
            self.set_notice("主臂对齐或停止尚未确认结束，不能开始录制。")
            return
        if teleop.get("state") != "RUNNING" or int(teleop.get("fault_bits") or 0) != 0:
            messagebox.showerror(
                "遥操未运行",
                f"当前状态：{teleop.get('state', '未知')}。\n{teleop.get('readiness_reason', '')}\n"
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
        collection = teleop.get("collection", {})
        if not self.args.automated_smoke_test:
            reason = None
            if not collection:
                reason = "控制器尚未更新或状态不可用"
            elif collection.get("right_servo_release_pending") is True:
                reason = "正在等待右主臂伺服解除确认；右从臂保持，不会自动恢复跟随。"
            elif collection.get("right_mode") == "RETURNING":
                reason = "右臂仍在回位"
            elif collection.get("transitioning_arms"):
                reason = "正在平滑恢复跟随，请稍候再开始录制"
            elif collection.get("left_align_active") is True:
                reason = "左从臂仍在低速对齐，请等待恢复跟随后再开始录制"
            elif side == "left" and collection.get("left_mode") != "FOLLOW":
                reason = "请先对齐并恢复左臂跟随"
            elif side == "right" and (collection.get("left_mode") != "HOLD" or collection.get("right_mode") != "FOLLOW"):
                reason = right_recording_posture_reason(teleop)
            elif side == "right" and (collection.get("right_ready_error_rad") is None or collection["right_ready_error_rad"] > 0.07):
                reason = "请先保存起始位，并将右臂回到该位置"
            if reason:
                messagebox.showwarning("采集姿态未准备好", reason); return
        task = TASKS[side][0]
        if self.args.automated_smoke_test:
            task = "TEST_" + task
        self.control.request("episode_start", task=task, target="ui_smoke_test" if self.args.automated_smoke_test else "")

    def on_motion(self, command):
        if command == 'right_servo_release':
            self.set_notice('请点击“解除右主臂回位伺服”并确认后再解除。')
            return
        if command == "left_align_follow":
            self.left_align_notice = "从臂自动对齐入口已关闭，请使用“主臂自动对齐并恢复左臂跟随”。"
            self.set_notice(self.left_align_notice)
            return
        if command == 'right_follow':
            self.set_notice('请使用“主臂自动对齐并恢复右臂跟随”，无需手动对齐。')
            return
        if command in {"left_manual_guide", "left_follow"}:
            self.set_notice("手动对齐入口已移除，请使用“主臂自动对齐并恢复左臂跟随”。")
            return
        if self.power_down_locked or self.power_down_confirming:
            self.set_notice("下电流程已锁定姿态操作，不会自动恢复运动。")
            return
        teleop = self.teleop.snapshot()
        if command == 'left_lock' and self.master_alignment_busy(teleop):
            command = 'left_master_pause'
        state, _, pending = self.control.snapshot()
        if command not in {"right_pause", "left_align_pause", "left_master_pause", "right_master_pause"} and (
                state.get("running") or pending or (command in {'left_master_align', 'right_master_align'} and state.get('active_episode'))):
            self.set_notice("请先结束并保存本条数据，再进行姿态操作。")
            return
        teleop = self.teleop.snapshot()
        collection = teleop.get("collection") or {}
        if command == 'right_pause' and self.right_master_alignment_busy(teleop):
            command = 'right_master_pause'
        release_right_servo = command == 'right_pause' and collection.get('right_mode') != 'RETURNING'
        if command == 'right_pause' and collection.get('right_servo_release_pending') is True:
            self.set_notice('正在等待右主臂伺服解除确认；右从臂保持，不会自动恢复跟随。')
            return
        if release_right_servo:
            reason = right_servo_release_reason(self.master_alignment_snapshot(teleop))
            if state.get('running') or state.get('active_episode') or pending:
                reason = '请先结束并保存本条数据，再解除右主臂伺服。'
            if reason or getattr(self.teleop, 'pending', False):
                self.set_notice(reason or '前一个姿态请求尚未确认，请稍候。')
                return
            if not messagebox.askyesno('解除右主臂回位伺服',
                    '右主臂及其夹爪将退出回位伺服，回到重力补偿状态。\n'
                    '请扶稳右主臂并确认安全；右从臂及夹爪保持当前目标，不会自动恢复跟随。\n'
                    '收到解除确认后，可使用主臂自动对齐按钮；本次不会自动启动对齐。\n'
                    '是否解除右主臂回位伺服？'):
                return
            # The confirmation dialog can process new status/recording events.
            state, _, pending = self.control.snapshot()
            reason = right_servo_release_reason(self.master_alignment_snapshot())
            if state.get('running') or state.get('active_episode') or pending:
                reason = '请先结束并保存本条数据，再解除右主臂伺服。'
            if (reason or getattr(self.teleop, 'pending', False)
                    or self.power_down_locked or self.power_down_confirming):
                self.set_notice(reason or '当前状态已变化，本次未解除右主臂伺服。')
                return
            command = 'right_servo_release'
        if command == 'right_master_align':
            reason = right_master_start_reason(self.right_master_alignment_snapshot(teleop))
            if reason or getattr(self.teleop, 'pending', False):
                self.right_master_align_notice = reason or '前一个姿态请求尚未确认，请稍候。'
                self.set_notice(self.right_master_align_notice)
                return
            if not messagebox.askyesno('右主臂及夹爪将主动运动',
                    '右主臂的 7 个关节和夹爪将主动低速移动，对齐右从臂当前保持目标。\n'
                    '右从臂及其夹爪全程保持原目标不动。请松开右主臂及夹爪，确认主臂路径无障碍。\n'
                    '全部对齐且确认主臂伺服解除后，才自动恢复右臂跟随；不会自动录制。\n'
                    '需要中断时点击“停止右主臂对齐”。是否开始？'):
                return
            # Tk dialogs run a nested event loop; recheck recording and state
            # after confirmation instead of trusting the pre-dialog sample.
            state, _, pending = self.control.snapshot()
            reason = right_master_start_reason(self.right_master_alignment_snapshot())
            if state.get('running') or state.get('active_episode'):
                reason = '请先结束并保存本条数据，再进行主臂自动对齐。'
            if reason or state.get('running') or state.get('active_episode') or pending or self.power_down_locked or self.power_down_confirming:
                self.set_notice(reason or '当前状态已变化，本次未启动主臂对齐。')
                return
        elif command not in {'right_master_pause', 'left_master_pause', 'left_align_pause', 'right_pause'} and self.right_master_alignment_busy(teleop):
            self.set_notice('请先停止右主臂对齐并确认伺服解除。')
            return
        if command == 'left_master_align':
            reason = left_master_start_reason(self.master_alignment_snapshot(teleop))
            if reason or getattr(self.teleop, 'pending', False):
                self.left_master_align_notice = reason or '前一个姿态请求尚未确认，请稍候。'
                self.set_notice(self.left_master_align_notice)
                return
            if not messagebox.askyesno('左主臂及夹爪将主动运动',
                    '左主臂的 7 个关节和夹爪将主动低速移动，对齐左从臂当前保持目标。\n'
                    '左从臂及其夹爪全程保持原目标不动。请松开左主臂及夹爪，确认主臂路径无障碍。\n'
                    '全部对齐且确认主臂伺服解除后，才自动恢复左臂跟随；不会自动录制。\n'
                    '需要中断时点击“停止主臂对齐”。是否开始？'):
                return
            # Tk dialogs run a nested event loop; recheck recording and state
            # after confirmation instead of trusting the pre-dialog sample.
            state, _, pending = self.control.snapshot()
            reason = left_master_start_reason(self.master_alignment_snapshot())
            if state.get('running') or state.get('active_episode'):
                reason = '请先结束并保存本条数据，再进行主臂自动对齐。'
            if reason or state.get('running') or state.get('active_episode') or pending or self.power_down_locked or self.power_down_confirming:
                self.set_notice(reason or '当前状态已变化，本次未启动主臂对齐。')
                return
        elif command not in {'left_master_pause', 'right_master_pause', 'left_align_pause', 'right_pause'} and self.master_alignment_busy(teleop):
            self.set_notice('请先停止左主臂对齐并确认伺服解除，再进行其他姿态操作。')
            return
        if command == "left_align_pause":
            if collection.get("left_align_supported") is not True:
                self.left_align_notice = "当前控制器不支持左臂低速对齐，请先更新控制器；本次未发送运动命令。"
                self.set_notice(self.left_align_notice)
                return
        if command == "right_return" and not messagebox.askyesno("右主从臂一起回位",
                "右主臂和右从臂都将低速运动，两个夹爪也会恢复保存的开合。\n"
                "请松开右主臂及夹爪，放好所持物体，确认两端路径无障碍。\n"
                "两端到位并确认解除回位伺服后自动恢复跟随；不会自动录制。"):
            return
        if command == "right_save" and self.teleop.snapshot().get("collection", {}).get("saved"):
            if not messagebox.askyesno("覆盖右臂起始位", "保存当前右主臂、右从臂和两端夹爪，替换原起始位？"):
                return
        if self.teleop.request(command):
            self.set_notice("操作已发送，正在等待控制器确认…")
            if release_right_servo:
                self.set_notice('已请求解除右主臂回位伺服，等待实时确认；右从臂保持，不会自动跟随。')
            if command == "left_lock":
                self.manual_left_follow_attempted = self.manual_left_completed = False
                self.manual_left_result = ""
            if command in {"left_align_follow", "left_align_pause"}:
                self.left_align_may_be_active = True
                if command == "left_align_pause":
                    self.left_align_stop_requested = True
                self.left_align_notice = ("正在请求停止从臂对齐，尚未确认停止。" if command == "left_align_pause"
                                          else "对齐请求已发送，等待控制器确认；尚未恢复跟随。")
            if command in {'left_master_align', 'left_master_pause'}:
                self.left_master_align_may_be_active = True
                if command == 'left_master_pause':
                    self.left_master_align_stop_requested = True
                else:
                    self.manual_left_follow_attempted = self.manual_left_completed = False
                    self.manual_left_result = ''
                self.left_master_align_notice = ('已请求停止主臂对齐，等待左从臂保持及主臂伺服解除确认。'
                    if command == 'left_master_pause' else '主臂对齐请求已发送，状态待确认；尚未恢复跟随。')
                self.set_notice(self.left_master_align_notice)
            if command in {'right_master_align', 'right_master_pause'}:
                self.right_master_align_may_be_active = True
                if command == 'right_master_pause':
                    self.right_master_align_stop_requested = True
                self.right_master_align_notice = ('已请求停止主臂对齐，等待右从臂保持及主臂伺服解除确认。'
                    if command == 'right_master_pause' else '主臂对齐请求已发送，状态待确认；尚未恢复跟随。')
                self.set_notice(self.right_master_align_notice)
        else:
            reason = getattr(self.teleop, "request_error", "")
            if not reason:
                reason = ("前一个姿态操作尚未完成。" if getattr(self.teleop, "pending", False)
                          else "姿态请求未发送：" + self.teleop.snapshot().get("readiness_reason", "请检查控制器状态"))
            self.set_notice(reason)
            if command in {"left_align_follow", "left_align_pause"}:
                self.left_align_notice = reason
            if command in {'left_master_align', 'left_master_pause'}:
                self.left_master_align_notice = reason
            if command in {'right_master_align', 'right_master_pause'}:
                self.right_master_align_notice = reason

    def master_alignment_snapshot(self, teleop=None):
        status = dict(self.teleop.snapshot() if teleop is None else teleop)
        for name in ('left_master_align_may_be_active', 'left_master_align_stop_requested'):
            status[name] = bool(status.get(name, getattr(self, name)))
        return status

    def master_alignment_busy(self, teleop=None):
        return left_master_busy(self.master_alignment_snapshot(teleop))

    def right_master_alignment_snapshot(self, teleop=None):
        status = dict(self.teleop.snapshot() if teleop is None else teleop)
        for name in ('right_master_align_may_be_active', 'right_master_align_stop_requested'):
            status[name] = bool(status.get(name, getattr(self, name)))
        return status

    def right_master_alignment_busy(self, teleop=None):
        return right_master_busy(self.right_master_alignment_snapshot(teleop))

    def open_manual_left_guide(self) -> None:
        """Open a read-only live guide; opening and reaching tolerance do not move anything."""
        if self.manual_left_dialog is not None and self.manual_left_dialog.winfo_exists():
            self.manual_left_dialog.lift()
            self.update_manual_left_guide()
            return
        dialog = tk.Toplevel(self.root)
        self.manual_left_dialog = dialog
        dialog.title("左臂手动对齐指引")
        dialog.geometry("1000x730")
        dialog.minsize(820, 700)
        dialog.configure(bg=BG)
        dialog.protocol("WM_DELETE_WINDOW", self.close_manual_left_guide)
        tk.Label(dialog, text="仅在左从臂已确认保持时，手动调整左主臂及夹爪", bg=BG, fg=TEXT,
                 font=("Noto Sans CJK SC", 16, "bold")).pack(anchor="w", padx=20, pady=(15, 5))
        tk.Label(dialog, text="按角度读数调整左主臂，不要推动左从臂。指引只读；全部达标后也不会自动恢复。\n"
                 "保持目标是点击保持时保存的控制目标；从臂实测可能因负载略有偏差，以保持目标为准。\n"
                 "7 个关节及夹爪的差值均需 ≤0.06 rad（约 3.4°）。\n"
                 "请确认夹持物安全。点击下方恢复按钮后，从臂才会平滑恢复跟随。",
                 bg=BG, fg=MUTED, font=("Noto Sans CJK SC", 11), justify="left").pack(anchor="w", padx=20)
        tk.Label(dialog, textvariable=self.manual_left_summary, bg=BG, fg="#79cfe8",
                 font=("Noto Sans CJK SC", 12, "bold"), justify="left", wraplength=940).pack(fill="x", padx=20, pady=10)
        style = ttk.Style(dialog)
        style.configure("ManualLeft.Treeview", background=PANEL, fieldbackground=PANEL,
                        foreground=TEXT, rowheight=34, font=("Noto Sans CJK SC", 11))
        columns = ("axis", "leader", "target", "delta", "instruction")
        table = ttk.Treeview(dialog, columns=columns, show="headings", height=8, style="ManualLeft.Treeview")
        self.manual_left_table = table
        for key, label, width in (("axis", "关节", 100), ("leader", "当前主臂角度", 140),
                                  ("target", "从臂保持目标", 140), ("delta", "需调整差值", 140),
                                  ("instruction", "手动调整方向", 310)):
            table.heading(key, text=label)
            table.column(key, width=width, stretch=False, anchor="center" if key != "instruction" else "w")
        table.tag_configure("unknown", foreground=MUTED)
        table.tag_configure("aligned", foreground="#79d65a")
        table.tag_configure("adjust", foreground="#ffd078")
        table.pack(fill="both", expand=True, padx=20)
        scrollbar = ttk.Scrollbar(dialog, orient="horizontal", command=table.xview)
        table.configure(xscrollcommand=scrollbar.set)
        scrollbar.pack(fill="x", padx=20)
        tk.Label(dialog, textvariable=self.manual_left_feedback, bg=BG, fg=TEXT,
                 font=("Noto Sans CJK SC", 11), justify="left", wraplength=940).pack(fill="x", padx=20, pady=10)
        footer = tk.Frame(dialog, bg=BG)
        footer.pack(fill="x", padx=20, pady=(0, 15))
        self.manual_left_confirm_button = tk.Button(footer, text="已对齐，恢复左臂跟随", command=self.confirm_manual_left_follow,
            bg="#2475a8", fg="white", font=("Noto Sans CJK SC", 12), padx=15, pady=8)
        self.manual_left_confirm_button.pack(side="right")
        tk.Button(footer, text="关闭指引（不发动作）", command=self.close_manual_left_guide,
                  bg="#34434c", fg="white", font=("Noto Sans CJK SC", 12), padx=15, pady=8).pack(side="left")
        self.update_manual_left_guide()

    def close_manual_left_guide(self) -> None:
        dialog = self.manual_left_dialog
        self.manual_left_dialog = self.manual_left_table = None
        if dialog is not None and dialog.winfo_exists():
            dialog.destroy()

    def manual_left_check(self, teleop=None, recording=None, pending=None):
        teleop = self.teleop.snapshot() if teleop is None else dict(teleop)
        teleop['left_align_may_be_active'] = bool(teleop.get('left_align_may_be_active', self.left_align_may_be_active))
        teleop['left_align_stop_requested'] = bool(teleop.get('left_align_stop_requested', self.left_align_stop_requested))
        check = dict(manual_left_alignment(teleop))
        if recording is None or pending is None:
            recording, _, pending = self.control.snapshot()
        reason = None
        if self.power_down_locked or self.power_down_confirming:
            reason = "下电流程已锁定操作，不能恢复跟随。"
        elif recording.get("running") or pending:
            reason = "请先结束并保存本条数据，再恢复左臂跟随。"
        elif self.master_alignment_busy(teleop):
            reason = '左主臂自动对齐或伺服释放尚未确认结束，请暂停手动调整。'
        elif getattr(self.teleop, "pending", False):
            reason = "前一个姿态请求仍在等待确认，请稍候。"
        if reason:
            check.update(can_resume=False, can_adjust=False, reason=reason)
        return check

    def confirm_manual_left_follow(self) -> None:
        check = self.manual_left_check()
        if not check.get('can_resume'):
            self.manual_left_result = check.get('reason') or "尚未满足手动对齐恢复条件。"
            self.update_manual_left_guide()
            return
        if self.teleop.request('left_follow'):
            self.manual_left_follow_attempted = True
            self.manual_left_completed = False
            self.manual_left_result = "恢复请求已发送，等待控制器确认；尚未确认恢复完成。"
        else:
            self.manual_left_result = getattr(self.teleop, 'request_error', '') or "恢复请求未发送，请检查控制器状态。"
        self.update_manual_left_guide()

    def update_manual_left_guide(self, teleop=None, recording=None, pending=None) -> None:
        teleop = self.teleop.snapshot() if teleop is None else teleop
        collection = teleop.get('collection') or {}
        if self.manual_left_completed and collection.get('left_mode') == 'HOLD':
            self.manual_left_follow_attempted = self.manual_left_completed = False
            self.manual_left_result = ""
        current_ready = (teleop.get('teleop_ready') is True and teleop.get('state') == 'RUNNING'
                         and teleop.get('connected') is not False
                         and all(type(teleop.get(key)) in (int, float) and 0 <= teleop[key] < 100
                                 for key in ('action_age_ms', 'feedback_age_ms')))
        if self.manual_left_completed and (not current_ready or 'left' in collection.get('transitioning_arms', [])):
            self.manual_left_result = "上次已恢复；当前状态不可确认，请暂停操作。"
        elif (self.manual_left_follow_attempted and current_ready
                and collection.get('left_mode') == 'FOLLOW'
                and 'left' not in collection.get('transitioning_arms', [])):
            self.manual_left_result = "左臂已恢复跟随（控制器已确认 FOLLOW，平滑衔接完成）。"
            self.manual_left_completed = True
        dialog = self.manual_left_dialog
        if dialog is None or not dialog.winfo_exists():
            return
        check = self.manual_left_check(teleop, recording, pending)
        usable = check.get('can_adjust') is True
        reason = check.get('reason') or "等待状态"
        worst = check.get('worst_axis')
        error = check.get('max_error_deg')
        completed_now = (self.manual_left_completed and current_ready and collection.get('left_mode') == 'FOLLOW'
                         and 'left' not in collection.get('transitioning_arms', []))
        if completed_now:
            summary = "手动对齐完成，左臂已恢复跟随，可关闭指引。"
        elif not usable:
            summary = "暂停调整，下表仅供参考；等待可确认的保持状态和实时读数。"
        elif check.get('aligned'):
            summary = "全部 8 轴已达到对齐门槛；请保持位置，主动点击恢复按钮。"
        else:
            summary = f"优先调整 ★ {worst}，最大差 {error:.2f}°。" if worst and isinstance(error, (int, float)) else "8 轴数据尚不完整。"
        self.manual_left_summary.set(summary if completed_now else summary + "\n" + reason)
        rows = check.get('rows') or []
        table = self.manual_left_table
        for existing in table.get_children():
            table.delete(existing)
        for row in rows:
            def degrees(value):
                return f"{value:+.2f}°" if isinstance(value, (int, float)) else "—"
            axis = row.get('axis', '—')
            label = '★ ' + axis if usable and axis == worst and not row.get('aligned') else axis
            instruction = ('已恢复跟随，无需继续对齐' if completed_now else
                           row.get('instruction', '等待数据') if usable else '暂停调整，等待状态确认')
            values = (label, degrees(row.get('leader_deg')), degrees(row.get('target_deg')),
                      degrees(row.get('delta_deg')), instruction)
            tag = 'unknown' if not usable else 'aligned' if row.get('aligned') else 'adjust'
            table.insert('', 'end', iid=row['axis'], values=values, tags=(tag,))
        self.manual_left_feedback.set(self.manual_left_result or "尚未发送恢复请求。达标后请主动点击恢复按钮。")
        self.manual_left_confirm_button.configure(state='normal' if check.get('can_resume') else 'disabled')

    def on_result(self, result: str) -> None:
        if self.power_down_busy or self.power_down_confirming:
            return
        state, _, pending = self.control.snapshot()
        if not state.get("running") or pending:
            self.set_notice("当前没有可结束的活动 episode，或封口仍在进行。")
            return
        if result in {"success", "failure"} and state.get("phase") not in {"recording", "awaiting_result"}:
            self.set_notice("本条仍在初始化，尚未进入正式录制；请等待状态显示“正在录制”后再标记结果。")
            return
        failure_code = "operator_marked_failure" if result == "failure" else "operator_aborted" if result == "aborted" else ""
        self.control.request("episode_stop", result=result, failure_code=failure_code)

    def on_safe_end(self) -> None:
        if self.power_down_busy or self.power_down_confirming:
            return
        state, _, pending = self.control.snapshot()
        if not state.get("running") or pending:
            self.set_notice("当前没有正在录制的 episode。")
            return
        if not self.args.automated_smoke_test and not messagebox.askyesno(
                "安全结束本条采集", "本条将标记为中止并保存，不计入成功训练数据。\n机械臂遥操将继续运行。是否继续？"):
            return
        self.control.request("episode_stop", result="aborted", failure_code="operator_safe_end")

    def on_close_session(self) -> None:
        if self.power_down_busy or self.power_down_confirming:
            self.set_notice("正在处理机械臂下电，请等待结果后关闭。")
            return
        teleop = self.teleop.snapshot()
        if not self.power_down_succeeded and self.right_master_alignment_busy(teleop):
            self.on_motion('right_master_pause')
            self.set_notice('正在停止右主臂对齐，确认伺服解除且右从臂保持后，再次关闭。')
            return
        if not self.power_down_succeeded and self.master_alignment_busy(teleop):
            self.on_motion('left_master_pause')
            self.set_notice('正在请求停止主臂对齐；需确认左从臂保持、左主臂伺服解除后，再次结束会话。')
            return
        left_align_uncertain = bool(teleop.get("left_align_may_be_active", self.left_align_may_be_active))
        if not self.power_down_succeeded and ((teleop.get("collection") or {}).get("left_align_active") is True or left_align_uncertain):
            self.on_motion("left_align_pause")
            self.set_notice("正在请求停止从臂对齐，请等待控制器确认保持后再次结束会话。")
            return
        if not self.power_down_succeeded and self.teleop.snapshot().get("collection", {}).get("right_mode") == "RETURNING":
            self.on_motion("right_pause")
            self.set_notice("正在停止右臂回位，确认保持后再结束会话。")
            return
        state, _, pending = self.control.snapshot()
        if state.get("running"):
            messagebox.showwarning("本条尚未保存", "请先使用成功、失败或安全结束按钮封口当前 episode。")
            return
        if pending:
            self.set_notice("请等待当前操作完成。")
            return
        if not self.args.automated_smoke_test and not messagebox.askyesno(
                "结束本次采集会话", "结束会话只关闭采集界面，不会停止机械臂遥操。是否继续？"):
            return
        self.close_requested = self.control.request("session_close")

    def on_window_close(self) -> None:
        if self.power_down_busy or self.power_down_confirming:
            self.set_notice("正在处理机械臂下电，请等待结果后关闭。")
            return
        teleop = self.teleop.snapshot()
        if not self.power_down_succeeded and self.right_master_alignment_busy(teleop):
            self.on_motion('right_master_pause')
            self.set_notice('正在停止右主臂对齐，确认伺服解除且右从臂保持后，再次关闭。')
            return
        if not self.power_down_succeeded and self.master_alignment_busy(teleop):
            self.on_motion('left_master_pause')
            self.set_notice('正在请求停止主臂对齐；需确认左从臂保持、左主臂伺服解除后，再次关闭窗口。')
            return
        left_align_uncertain = bool(teleop.get("left_align_may_be_active", self.left_align_may_be_active))
        if not self.power_down_succeeded and ((teleop.get("collection") or {}).get("left_align_active") is True or left_align_uncertain):
            self.on_motion("left_align_pause")
            self.set_notice("正在请求停止从臂对齐，请等待控制器确认保持后再次关闭窗口。")
            return
        if not self.power_down_succeeded and self.teleop.snapshot().get("collection", {}).get("right_mode") == "RETURNING":
            self.on_motion("right_pause")
            self.set_notice("正在停止右臂回位，确认保持后再关闭窗口。")
            return
        state, _, _ = self.control.snapshot()
        if state.get("running"):
            if not messagebox.askyesno("本条尚未保存", "关闭窗口会把当前条标记为中止并封口，遥操继续运行。是否关闭？"):
                return
            self.control.abort_synchronously("collection_window_closed")
        else:
            # Closing an idle window is an intentional end of this collection
            # batch.  Unexpected process termination never reaches this code,
            # so crash recovery remains intact.
            self.control.close_session_synchronously()
        self.preview.stop.set(); self.teleop.stop.set(); self.root.destroy()

    def confirm_power_down(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("确认机械臂下电")
        dialog.transient(self.root); dialog.resizable(False, False)
        dialog.configure(bg=PANEL)
        confirmed = {"value": False}
        text = ("主从四臂及夹爪将失去驱动力。\n请先放下夹持物、安置并支撑好四臂。\n\n"
                "正在录制的本条将中止并保存；已在封装的条目保留原结果。\n"
                "只有保存完成后才执行失能。Jetson 保持开机。\n"
                "本操作停止电机出力，硬件电源仍需手动关闭。")
        tk.Label(dialog, text=text, justify="left", bg=PANEL, fg=TEXT,
                 font=("Noto Sans CJK SC", 12), padx=24, pady=20).pack()
        buttons = tk.Frame(dialog, bg=PANEL); buttons.pack(fill="x", padx=20, pady=(0, 18))
        def accept():
            confirmed["value"] = True
            dialog.destroy()
        cancel = tk.Button(buttons, text="取消", command=dialog.destroy, width=14)
        cancel.pack(side="left", padx=8)
        tk.Button(buttons, text="确认下电", command=accept, width=14,
                  bg="#933a35", fg="white").pack(side="right", padx=8)
        dialog.bind("<Escape>", lambda _event: dialog.destroy())
        dialog.protocol("WM_DELETE_WINDOW", dialog.destroy)
        dialog.grab_set(); cancel.focus_set()
        self.root.wait_window(dialog)
        return confirmed["value"]

    def on_power_down(self):
        if (self.power_down_busy or self.power_down_confirming or self.power_down_succeeded
                or self.power_down.pending or self.close_requested):
            return
        _, _, pending = self.control.snapshot()
        if pending or getattr(self.teleop, "pending", False):
            self.set_notice("请等待当前操作回复后再下电。")
            return
        self.power_down_confirming = True
        try:
            confirmed = self.confirm_power_down()
        finally:
            self.power_down_confirming = False
        if not confirmed:
            return
        self.power_down_busy = True
        self.power_down_locked = True
        self.power_down_message = "正在确认录制状态…"
        self.power_down_detail.set("保存数据 → 停止跟随 → 主从电机失能 → 核验结果\n" + self.power_down_message)
        if self.power_down_dialog is None or not self.power_down_dialog.winfo_exists():
            self.power_down_dialog = tk.Toplevel(self.root)
            self.power_down_dialog.title("机械臂下电进度")
            self.power_down_dialog.transient(self.root)
            self.power_down_dialog.configure(bg=PANEL)
            tk.Label(self.power_down_dialog, textvariable=self.power_down_detail,
                     bg=PANEL, fg=TEXT, font=("Noto Sans CJK SC", 12), justify="left",
                     wraplength=650, padx=24, pady=24).pack()
            tk.Button(self.power_down_dialog, text="关闭此提示", command=self.close_power_down_dialog).pack(pady=(0, 16))
            self.power_down_dialog.protocol("WM_DELETE_WINDOW", self.close_power_down_dialog)
        self.power_down_dialog.lift()
        if not self.power_down.start():
            self.power_down_busy = False
            self.power_down_message = "已有下电操作正在执行，请等待结果。"

    def close_power_down_dialog(self):
        if not self.power_down_busy and self.power_down_dialog is not None:
            self.power_down_dialog.destroy()
            self.power_down_dialog = None

    def poll_power_down(self):
        try:
            while True:
                event = self.power_down.events.get_nowait()
                if event["event"] == "device_result":
                    continue  # The final result below includes both endpoints.
                self.power_down_message = event.get("message", "下电结果未知")
                detail = self.power_down_message
                if event["event"] == "complete":
                    self.power_down_busy = False
                    self.power_down_succeeded = event.get("ok") is True
                    for side, label in (("host", "主机主臂"), ("follower", "Jetson 从臂")):
                        report = event.get("devices", {}).get(side, {})
                        text = "已确认失能" if report.get("ok") and report.get("verified") else report.get("error") or "未确认失能"
                        detail += f"\n{label}：{text}"
                    if not self.power_down_succeeded:
                        detail += "\n不会自动使能或归位。处理问题后可再次点击“机械臂下电…”重试。"
                self.power_down_detail.set("保存数据 → 停止跟随 → 主从电机失能 → 核验结果\n" + detail)
        except queue.Empty:
            pass

    def _set_button_states(self, state: dict[str, Any], pending: bool) -> None:
        running = bool(state.get("running")); phase = str(state.get("phase", "idle"))
        # Keep buttons physically clickable whenever no request is in flight.
        # Their callbacks explain unmet prerequisites (READY, camera health or
        # no active episode). A grey inert button was indistinguishable from a
        # broken UI to operators and provided no corrective guidance.
        motion_active = ((self.teleop.snapshot().get("collection") or {}).get("left_align_active") is True
                         or self.left_align_may_be_active or self.master_alignment_busy())
        can_start = not running and not pending and not motion_active and phase not in {"starting", "stopping", "awaiting_result"}
        can_finish = running and not pending and phase in {"recording", "awaiting_result"}
        can_abort = running and not pending and phase in {"starting", "recording", "awaiting_result"}
        for button in (self.start_left_button, self.start_right_button): button.configure(state="normal" if can_start else "disabled")
        for button in (self.success_button, self.failure_button): button.configure(state="normal" if can_finish else "disabled")
        for button in (self.abort_button, self.safe_end_button): button.configure(state="normal" if can_abort else "disabled")
        self.close_button.configure(state="normal" if not running and not pending else "disabled")
        self.power_down_button.configure(state="disabled" if (
            self.power_down_busy or self.power_down_confirming or self.power_down_succeeded
            or self.power_down.pending or pending or getattr(self.teleop, "pending", False)
            or self.close_requested) else "normal")
        if self.power_down_locked or self.power_down_confirming:
            for button in (self.start_left_button, self.start_right_button, *self.motion_buttons.values()):
                button.configure(state="disabled")
        if self.power_down_busy or self.power_down_confirming:
            for button in (self.success_button, self.failure_button, self.abort_button,
                           self.safe_end_button, self.close_button):
                button.configure(state="disabled")

    def _tick_once(self) -> None:
        self.poll_power_down()
        if hasattr(self.teleop, "results"):
            try:
                while True:
                    command, reply = self.teleop.results.get_nowait()
                    if command in {"left_align_follow", "left_align_pause"}:
                        self.left_align_notice = reply.get("error") or ""
                        self.set_notice(self.left_align_notice or "控制器已回复，正在核实左臂对齐状态。", 8)
                    elif command == "left_follow":
                        self.manual_left_result = reply.get("error") or "控制器已回复，正在等待真实 FOLLOW 和平滑衔接完成。"
                        self.set_notice(self.manual_left_result, 8)
                    elif command in {'left_master_align', 'left_master_pause'}:
                        self.left_master_align_notice = reply.get('error') or ''
                        self.set_notice(self.left_master_align_notice or '控制器已回复，正在核实主臂对齐及伺服释放状态。', 8)
                    else:
                        self.set_notice(reply.get("error") or reply.get("collection", {}).get("note", "操作已确认"), 8)
                    if command in {"right_return", "right_save"}:
                        self.right_ready.set(False)
            except queue.Empty:
                pass
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
                image_label = getattr(self, role + "_image")
                # Fill the camera panel dynamically while preserving the
                # native 4:3 image ratio. This responds to both maximising and
                # manual window resizing instead of freezing previews at the
                # former 440x230 thumbnail size.
                target_w = max(160, image_label.winfo_width() - 12)
                target_h = max(120, image_label.winfo_height() - 12)
                pil = ImageOps.contain(frame, (target_w, target_h), method=LANCZOS)
                photo = ImageTk.PhotoImage(pil); self.photos[role] = photo
                image_label.configure(image=photo, text="")
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
        self.left_align_may_be_active = bool(teleop.get('left_align_may_be_active', self.left_align_may_be_active))
        self.left_align_stop_requested = bool(teleop.get('left_align_stop_requested', self.left_align_stop_requested))
        self.left_master_align_may_be_active = bool(teleop.get('left_master_align_may_be_active', self.left_master_align_may_be_active))
        self.left_master_align_stop_requested = bool(teleop.get('left_master_align_stop_requested', self.left_master_align_stop_requested))
        self.right_master_align_may_be_active = bool(teleop.get('right_master_align_may_be_active', self.right_master_align_may_be_active))
        self.right_master_align_stop_requested = bool(teleop.get('right_master_align_stop_requested', self.right_master_align_stop_requested))
        self.update_manual_left_guide(teleop, state, pending)
        running = bool(state.get("running")); phase = str(state.get("phase", "idle"))
        active = state.get("active_episode") or {}
        free_gb = state.get("free_gb", "?"); camera_text = "3/3 健康" if health.get("ok") else "相机异常"
        teleop_running = teleop.get("state") == "RUNNING" and int(teleop.get("fault_bits") or 0) == 0
        collection = teleop.get("collection", {})
        labels = {"FOLLOW": "跟随主臂", "HOLD": "保持中", "RETURNING": "低速回位中"}
        for side in ("left", "right"):
            mode = collection.get(side + "_mode")
            error = collection.get(side + "_alignment_error_rad", 0.0)
            extra = f"；主从对齐差 {error:.3f} rad（需 ≤0.06）" if mode == "HOLD" else ""
            if side == 'left' and mode == 'HOLD':
                extra = f'；可保持此状态录右臂；录左臂前才需自动对齐并恢复左臂跟随（当前差 {error:.3f} rad）'
            if side in collection.get("transitioning_arms", []):
                extra = "；正在平滑衔接，请稍候再录制"
            if side == "right" and mode == "RETURNING":
                extra = {"resetting": "；准备双端回位，请松开右主臂",
                         "preparing": "；等待主臂控制确认，请松开右主臂",
                         "moving": "；主从一起运动，请勿触碰",
                         "releasing": "；已到位，确认恢复跟随中"}.get(collection.get("return_phase"), "")
            saved = ("；起始位已保存" if collection.get("saved") else "；尚未保存起始位") if side == "right" else ""
            motion_text = (labels.get(mode, "等待新版控制器") if teleop_running
                           else '不可确认可遥操（原模式 '+str(mode)+'）') + extra + saved
            if side == 'right':
                if collection.get('right_servo_release_pending') is True:
                    motion_text = ('右从臂保持当前目标' if teleop_running else '右侧解除请求状态待确认')
                    motion_text += '；正在等待右主臂伺服解除确认，不会自动恢复跟随'
                elif right_servo_release_needed(teleop):
                    motion_text += '；' + (right_servo_release_reason(self.master_alignment_snapshot(teleop))
                        or '右主臂回位伺服尚未解除；可点击“解除右主臂回位伺服”')
                elif mode == 'HOLD' and collection.get('left_mode') == 'HOLD':
                    motion_text += '\n' + right_recording_posture_reason(teleop)
                release_detail = collection.get('right_servo_release_detail') or {}
                if isinstance(release_detail, dict) and release_detail.get('message'):
                    motion_text += '；' + str(release_detail['message'])
            if side == "left":
                align_phase = collection.get("left_align_phase", "idle")
                detail = collection.get("left_align_detail") or {}
                detail_text = str(detail.get("message", "")) if isinstance(detail, dict) else str(detail)
                if self.left_align_may_be_active and collection.get("left_align_active") is not True:
                    motion_text = "左臂对齐请求状态待确认，可能仍在运动；可点击“停止从臂对齐”"
                elif collection.get("left_align_active") is True:
                    label = "等待从臂稳定到位" if align_phase == "settling" else "左从臂及夹爪正在低速对齐"
                    motion_text = label + ("：" + detail_text if detail_text else "")
                    if isinstance(detail, dict) and isinstance(detail.get("progress"), (int, float)):
                        motion_text += f"；进度 {max(0., min(1., detail['progress'])):.0%}"
                    if teleop_running:
                        self.left_align_notice = ""
                    else:
                        motion_text = "状态待核实（上次状态）：" + motion_text
                elif align_phase in {"failed", "paused"}:
                    motion_text = ("左臂仍保持；对齐失败" if align_phase == "failed" else "左臂仍保持；已停止对齐")
                    motion_text += "：" + detail_text if detail_text else ""
                    if not teleop_running:
                        motion_text = "状态待核实（上次状态）：" + motion_text
                elif align_phase == "completed":
                    if mode == "FOLLOW" and "left" not in collection.get("transitioning_arms", []) and teleop_running:
                        motion_text = "左臂已恢复跟随（对齐及平滑衔接完成）"
                        self.left_align_notice = ""
                    else:
                        motion_text = "对齐后正在核实跟随衔接，尚未确认恢复完成"
                if self.left_align_notice:
                    motion_text += "；" + self.left_align_notice
                if self.left_align_stop_requested:
                    motion_text += "；已请求停止，尚未确认保持，可再次点击停止"
                elif self.manual_left_result:
                    motion_text += "；" + self.manual_left_result
                master_phase = collection.get('left_master_align_phase', 'idle')
                master_detail = collection.get('left_master_align_detail') or {}
                if not isinstance(master_detail, dict):
                    master_detail = {}
                if self.master_alignment_busy(teleop):
                    if self.left_master_align_stop_requested or master_phase in {'stopping', 'releasing'}:
                        motion_text = '左从臂保持；等待左主臂伺服解除确认，尚不能按重力补偿状态操作'
                    elif master_phase == 'failed':
                        motion_text = '对齐失败，轨迹已停止；左从臂保持，左主臂伺服尚未确认解除，请点击“停止主臂对齐”'
                    elif collection.get('left_master_align_active') is True and teleop_running:
                        motion_text = '左主臂及夹爪正在低速对齐；左从臂保持不动'
                        progress = master_detail.get('progress')
                        if type(progress) in (int, float) and math.isfinite(progress):
                            motion_text += f'；进度 {max(0., min(1., progress)):.0%}'
                    else:
                        motion_text = '主臂对齐状态待确认，可能仍在运动；请使用“停止主臂对齐”'
                    if master_detail.get('message'):
                        motion_text += '；' + str(master_detail['message'])
                elif master_phase in {'paused', 'failed'} and mode == 'HOLD':
                    if left_master_servo_free(teleop):
                        motion_text = ('主臂对齐已停止' if master_phase == 'paused' else '主臂对齐未完成')
                        motion_text += '；左从臂保持，左主臂伺服已解除（重力补偿）'
                    else:
                        motion_text = '上次主臂对齐已结束；当前伺服释放状态待确认'
                    if master_detail.get('message'):
                        motion_text += '；' + str(master_detail['message'])
                elif master_phase == 'completed' and mode == 'FOLLOW':
                    if (teleop_running and teleop.get('teleop_ready') is True and left_master_servo_free(teleop)
                            and 'left' not in collection.get('transitioning_arms', ['left'])):
                        motion_text = '左主臂及夹爪已对齐，伺服已解除；左臂已恢复跟随'
                    else:
                        motion_text = '主臂对齐后正在核实跟随及释放状态，尚未确认恢复完成'
                elif mode == 'HOLD':
                    blocked = left_master_start_reason(self.master_alignment_snapshot(teleop))
                    if blocked:
                        motion_text += '；' + blocked
                if self.left_master_align_notice:
                    motion_text += '；' + self.left_master_align_notice
            if side == 'right' and (self.right_master_alignment_busy(teleop)
                    or collection.get('right_master_align_phase') in {'paused', 'failed', 'completed'}):
                detail = collection.get('right_master_align_detail') or {}
                motion_text = str(detail.get('message') or '右主臂对齐状态待确认')
                phase = collection.get('right_master_align_phase')
                if phase == 'completed':
                    if (teleop_running and teleop.get('teleop_ready') is True and right_master_servo_free(teleop)
                            and mode == 'FOLLOW' and 'right' not in collection.get('transitioning_arms', ['right'])):
                        motion_text = '右主臂及夹爪已自动对齐，伺服已解除；右臂已恢复跟随'
                    else:
                        motion_text = '右主臂对齐后正在核实释放和跟随衔接，尚未确认恢复完成'
                elif phase in {'paused', 'failed'} and not self.right_master_alignment_busy(teleop):
                    motion_text = ('右主臂对齐已停止' if phase == 'paused' else '右主臂对齐未完成')
                    motion_text += '；右从臂保持，右主臂伺服已解除' if right_master_servo_free(teleop) else '；伺服释放状态待确认'
                if self.right_master_alignment_busy(teleop):
                    motion_text += '；需中断时点击“停止右主臂对齐”'
                if self.right_master_align_stop_requested:
                    motion_text += '；已请求停止，尚未确认释放'
                if not teleop_running:
                    motion_text = '状态待确认：' + motion_text
            self.motion_vars[side].set(motion_text)
        for command, button in self.motion_buttons.items():
            available = (not getattr(self.teleop, "pending", False) and teleop_running and bool(collection)
                         and not running and not pending and not collection.get("recording"))
            if command == 'right_master_pause':
                available = self.right_master_alignment_busy(teleop)
            elif command == 'right_master_align':
                available = available and not state.get('active_episode') and not right_master_start_reason(self.right_master_alignment_snapshot(teleop))
            elif command == "right_pause":
                releasing = collection.get('right_servo_release_pending') is True
                if releasing:
                    button.configure(text='正在解除右主臂伺服…')
                    available = False
                elif right_servo_release_needed(teleop):
                    button.configure(text='解除右主臂回位伺服')
                    available = (available and not state.get('active_episode')
                                 and not right_servo_release_reason(self.master_alignment_snapshot(teleop)))
                else:
                    button.configure(text='停止双端回位')
                    available = collection.get("right_mode") == "RETURNING" and not getattr(self.teleop, "pending", False)
            elif command == "left_align_pause":
                available = (collection.get("left_align_supported") is True
                             and (collection.get("left_align_active") is True or self.left_align_may_be_active))
                if available:
                    button.grid()
                else:
                    button.grid_remove()
            elif command == 'left_master_pause':
                available = self.master_alignment_busy(teleop)
            elif command == 'left_master_align':
                available = available and not state.get('active_episode') and not left_master_start_reason(self.master_alignment_snapshot(teleop))
            elif command == 'left_lock' and self.master_alignment_busy(teleop):
                available = True  # Routes to pause, including an uncertain queued start.
            elif self.master_alignment_busy(teleop) or self.right_master_alignment_busy(teleop):
                available = False
            elif collection.get("left_align_active") is True or self.left_align_may_be_active:
                available = False
            if command == 'right_pause' and self.right_master_alignment_busy(teleop):
                button.configure(text='停止右主臂对齐')
                available = True
            button.configure(state="normal" if available else "disabled")
        self.teleop_status_label.configure(fg="#79d65a" if teleop_running else "#ff5656")
        recording_label = {
            "starting": "录制准备中",
            "recording": "正在录制",
            "awaiting_result": "本条已中断，等待保存",
            "stopping": "正在封装",
            "error": "录制异常",
        }.get(phase, "未录制")
        self.status_var.set(
            f"遥操 {teleop.get('state', 'CHECKING')}　｜　相机 {camera_text}　｜　"
            f"磁盘 {free_gb} GB　｜　{recording_label}"
        )
        # Teleoperation diagnostics never choose an episode result. The
        # operator owns success/failure; real capture failures are reported by
        # the recorder as awaiting_result while preserving this episode.
        if not teleop_running and not running:
            message = '禁止开始采集：'+teleop.get('readiness_reason',str(teleop.get('state','CHECKING')))
        elif health.get("ok") is not True and not running:
            message = "禁止开始采集：三路 RGB-D 相机尚未全部健康。"
        if time.monotonic() < self.operator_notice_until:
            message = self.operator_notice
        if phase == "awaiting_result":
            reason = state.get("capture_interrupted") or active.get("capture_interrupted") or "录制已中断"
            message = f"本条已中断，等待保存：{reason}。请选择“成功并保存”或“失败并保存”；中断数据不会标为有效。"
        elif phase == "recording" and not teleop_running:
            reason = teleop.get('readiness_reason') or teleop.get('error') or str(teleop.get('state', 'CHECKING'))
            message = f"警告：遥操健康状态不可确认（{reason}）。本条录制继续，请手动点击“成功并保存”或“失败并保存”结束；请勿依据录制状态判断可遥操。"
        if self.power_down_locked:
            message = self.power_down_message
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
        active_side = active.get("task_group") or next(
            (side for side, task in TASKS.items() if task[0] == active.get("task")), None)
        for side in ("left", "right"):
            values = statistics.get(side, {})
            next_number = int(next_numbers.get(side, 1))
            count_text = f"本批次有效成功：{int(values.get('valid_success', 0))} 条"
            self.subcount_vars[side].set(f"失败 {int(values.get('failure', 0))}　中止 {int(values.get('aborted', 0))}")
            arm_name = "左" if side == "left" else "右"
            if running and active_side == side:
                current_number = int(active.get("episode_number") or next_number)
                action = {"recording": f"正在录制{arm_name}臂", "starting": f"正在准备{arm_name}臂",
                          "stopping": f"正在保存{arm_name}臂",
                          "awaiting_result": f"{arm_name}臂本条已中断，等待保存"}.get(phase, f"{arm_name}臂本条未保存")
                button_text = f"{action}｜第{current_number:04d}条"
                count_text += f"　｜　当前第 {current_number:04d} 条（未保存）"
                storage_text = f"保存位置：episodes/{side}/episode_{current_number:04d}"
            elif running:
                button_text = f"{arm_name}臂待录制｜请先保存当前条"
                storage_text = f"保存位置：episodes/{side}/"
            else:
                button_text = f"开始录制{arm_name}臂｜本批次第 {next_number:04d} 条"
                count_text += f"　｜　本批次下一条：第 {next_number} 条"
                storage_text = f"保存位置：episodes/{side}/episode_{next_number:04d}"
            self.count_vars[side].set(count_text)
            self.storage_vars[side].set(storage_text)
            getattr(self, "start_" + side + "_button").configure(text=button_text)
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

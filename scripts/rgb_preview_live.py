#!/usr/bin/env python3
"""Minimal real-time RGB monitor for remote OpenArm teleoperation.

Unlike Rerun this has no recording or timeline: every redraw is the newest
three-camera packet from Jetson.  The monitor never requests historical data.
"""
from __future__ import annotations

import argparse
import base64
import json
import threading
import time

import cv2
import numpy as np
import zmq
from PIL import Image, ImageDraw, ImageFont


ROLES = ("left_wrist", "right_wrist", "chest")
TITLES = {
    "chest": "胸部全局相机",
    "left_wrist": "左腕相机",
    "right_wrist": "右腕相机",
}
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def chinese_text(image: np.ndarray, text: str, xy: tuple[int, int], size: int,
                 color: tuple[int, int, int]) -> np.ndarray:
    """Draw UTF-8 operator labels; cv2.putText cannot render Chinese."""
    canvas = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(canvas)
    draw.text(xy, text, font=ImageFont.truetype(FONT_PATH, size), fill=color)
    return cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)


def decode(encoded: str) -> np.ndarray:
    raw = np.frombuffer(base64.b64decode(encoded), np.uint8)
    image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("invalid JPEG preview frame")
    return image


def label(image: np.ndarray, role: str, age_ms: float, sequence: int, fps: float) -> np.ndarray:
    panel = image.copy()
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 44), (0, 0, 0), -1)
    return chinese_text(panel, f"{TITLES[role]}  实时 {fps:.1f} FPS  延迟 {age_ms:.0f} ms  帧 {sequence}",
                        (10, 8), 19, (0, 255, 0))


class RecordControl:
    def __init__(self, jetson: str, port: int) -> None:
        self.endpoint = f"tcp://{jetson}:{port}"
        self.status = "正在连接 Jetson 录制服务…"
        self.running = False
        self.dataset_root: str | None = None
        self.started_unix_s: float | None = None
        self._lock = threading.Lock()

    def request(self, command: str) -> None:
        def run() -> None:
            context = zmq.Context()
            socket = context.socket(zmq.REQ)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVTIMEO, 2500)
            socket.setsockopt(zmq.SNDTIMEO, 2500)
            try:
                socket.connect(self.endpoint)
                socket.send_json({"command": command})
                response = socket.recv_json()
                with self._lock:
                    self.running = bool(response.get("running", False))
                    self.dataset_root = response.get("dataset_root")
                    self.started_unix_s = response.get("started_unix_s")
                    self.status = ("录制中：Jetson 正在本地保存" if self.running else "未录制：Jetson 本地保存已停止")
                    if not response.get("ok", False):
                        self.status += f"（{response.get('error', response.get('message', '未知错误'))}）"
            except Exception as exc:
                with self._lock:
                    self.status = f"Jetson 录制服务未连接：{exc}"
                    self.running = False
            finally:
                socket.close(0); context.term()
        threading.Thread(target=run, daemon=True).start()

    def snapshot(self) -> tuple[str, bool, str | None, float | None]:
        with self._lock:
            return self.status, self.running, self.dataset_root, self.started_unix_s


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--jetson", default="192.168.50.2")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--record-port", type=int, default=5557)
    args = parser.parse_args()

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect(f"tcp://{args.jetson}:{args.port}")

    control = RecordControl(args.jetson, args.record_port)
    control.request("status")
    # OpenCV Qt on this host does not create a callback-capable native window
    # when the window title contains CJK text.  Operator-facing labels inside
    # the canvas remain Chinese.
    name = "OpenArm Live Camera Preview (Esc closes)"
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, 1280, 920)
    # Qt creates the native handler on its first event cycle.  Without this
    # short pump, some desktops reject setMouseCallback with a null handler.
    cv2.waitKey(1)
    # Mouse coordinates are in the 1280x1160 composed canvas.
    start_button = {"x1": 165, "y1": 1060, "x2": 615, "y2": 1140}
    stop_button = {"x1": 665, "y1": 1060, "x2": 1115, "y2": 1140}
    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONUP:
            return
        _, running, _, _ = control.snapshot()
        if start_button["x1"] <= x <= start_button["x2"] and start_button["y1"] <= y <= start_button["y2"] and not running:
            control.request("start")
        elif stop_button["x1"] <= x <= stop_button["x2"] and stop_button["y1"] <= y <= stop_button["y2"] and running:
            control.request("stop")
    cv2.setMouseCallback(name, on_mouse)
    last_status_check = 0.0
    last_packet_at: dict[str, float | None] = {role: None for role in ROLES}
    display_fps: dict[str, float] = {role: 0.0 for role in ROLES}
    while True:
        packet = json.loads(socket.recv_string())
        now = time.time()
        if now - last_status_check >= 2.0:
            control.request("status")
            last_status_check = now
        panels = {}
        for role in ROLES:
            image = decode(packet["images"][role])
            age_ms = (now - float(packet["timestamps"][role])) * 1000.0
            previous = last_packet_at[role]
            if previous is not None and now > previous:
                instant_fps = 1.0 / (now - previous)
                display_fps[role] = instant_fps if display_fps[role] == 0.0 else 0.85 * display_fps[role] + 0.15 * instant_fps
            last_packet_at[role] = now
            panels[role] = label(image, role, age_ms, int(packet["frame_seq"][role]), display_fps[role])
        # Chest view is deliberately centred on top; the two wrist views are
        # side-by-side below, matching the operator's left/right arms.
        canvas = np.zeros((1040, 1280, 3), dtype=np.uint8)
        canvas[35:515, 320:960] = panels["chest"]
        canvas[550:1030, 0:640] = panels["left_wrist"]
        canvas[550:1030, 640:1280] = panels["right_wrist"]
        canvas = chinese_text(canvas, "OpenArm 双臂遥操｜实时 RGB 预览（不回放，不控制机械臂）", (18, 5), 20, (220, 220, 220))
        footer = np.zeros((120, canvas.shape[1], 3), dtype=np.uint8)
        status, running, dataset_root, started = control.snapshot()
        cv2.rectangle(footer, (start_button["x1"], 12), (start_button["x2"], 92), (0, 145, 0) if not running else (70, 70, 70), -1)
        cv2.rectangle(footer, (stop_button["x1"], 12), (stop_button["x2"], 92), (0, 0, 210) if running else (70, 70, 70), -1)
        footer = chinese_text(footer, "开始本地录制", (start_button["x1"] + 115, 31), 27, (255, 255, 255))
        footer = chinese_text(footer, "停止并保存", (stop_button["x1"] + 135, 31), 27, (255, 255, 255))
        runtime = f"  已录制 {int(now - started)} 秒" if running and started else ""
        footer = chinese_text(footer, status + runtime, (22, 96), 17, (0, 255, 255))
        if dataset_root:
            footer = chinese_text(footer, f"保存位置：{dataset_root}", (530, 96), 15, (180, 180, 180))
        cv2.imshow(name, cv2.vconcat([canvas, footer]))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
    socket.close(0)
    context.term()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

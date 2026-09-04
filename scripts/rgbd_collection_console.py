#!/usr/bin/env python3
"""Operator console for real-time RGB preview and multi-episode collection.

The console is intentionally only a client of Jetson's recorder manager.  It
cannot open cameras, write datasets, access CAN, or change teleoperation.
Closing the window therefore never stops robot motion; an active episode must
be explicitly saved as success/failure/abort before the session can close.
"""
from __future__ import annotations

import argparse
import base64
import json
import threading
import time
from typing import Any

import cv2
import numpy as np
import zmq
from PIL import Image, ImageDraw, ImageFont


ROLES = ("left_wrist", "right_wrist", "chest")
TITLES = {"chest": "胸部全局相机", "left_wrist": "左腕相机", "right_wrist": "右腕相机"}
TASKS = [
    ("A", "接近蘑菇"), ("B", "夹取并拔下蘑菇"),
    ("C", "放入篮筐"), ("D", "完整采摘流程"),
]
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def chinese(canvas: np.ndarray, text: str, xy: tuple[int, int], size: int,
            color: tuple[int, int, int] = (255, 255, 255)) -> np.ndarray:
    image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    ImageDraw.Draw(image).text(xy, text, font=ImageFont.truetype(FONT_PATH, size), fill=color)
    return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)


def decode(value: str) -> np.ndarray:
    decoded = cv2.imdecode(np.frombuffer(base64.b64decode(value), np.uint8), cv2.IMREAD_COLOR)
    if decoded is None:
        raise ValueError("bad JPEG frame")
    return decoded


class SessionControl:
    def __init__(self, jetson: str, port: int) -> None:
        self.endpoint = f"tcp://{jetson}:{port}"
        self.lock = threading.RLock(); self.request_lock = threading.Lock()
        self.value: dict[str, Any] = {"phase": "idle", "running": False}
        self.message = "正在连接 Jetson 采集服务…"
        self.pending = False

    def request(self, command: str, **extra: str) -> None:
        with self.lock:
            if self.pending and command != "status":
                return
            if command != "status":
                self.pending = True
                self.message = {"session_start": "正在创建采集会话…", "episode_start": "正在启动本 episode 录制…",
                                "episode_stop": "正在停止并封口，请勿重复点击…", "session_close": "正在生成会话报告…"}.get(command, "正在请求…")
        def worker() -> None:
            if command == "status" and not self.request_lock.acquire(blocking=False):
                return
            if command != "status": self.request_lock.acquire()
            context = zmq.Context(); sock = context.socket(zmq.REQ)
            sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.SNDTIMEO, 3000); sock.setsockopt(zmq.RCVTIMEO, 4000)
            try:
                sock.connect(self.endpoint); sock.send_json({"command": command, **extra}); response = sock.recv_json()
                with self.lock:
                    self.value = response
                    if response.get("ok"):
                        if response.get("running"):
                            self.message = "正在本地录制：RGB-D 与从臂状态/实际动作写入 Jetson"
                        elif response.get("phase") == "stopping":
                            self.message = "正在停止并封口，请等待完成…"
                        elif response.get("last_episode"):
                            episode = response["last_episode"]
                            self.message = f"已保存 {episode['episode_id']}：{episode['result']}，可开始下一条"
                        elif response.get("session_id"):
                            self.message = "会话已就绪：选择任务后点击“开始本 episode”"
                        else:
                            self.message = str(response.get("message", "未在录制"))
                    else:
                        self.message = "请求被拒绝：" + str(response.get("error", "未知错误"))
            except Exception as exc:
                with self.lock: self.message = f"Jetson 采集服务未连接：{exc}"
            finally:
                with self.lock:
                    if command != "status": self.pending = False
                sock.close(0); context.term(); self.request_lock.release()
        threading.Thread(target=worker, daemon=True, name=f"collection-{command}").start()

    def snapshot(self) -> tuple[dict[str, Any], str, bool]:
        with self.lock: return dict(self.value), self.message, self.pending

    def stop_active_episode_before_exit(self) -> None:
        """Best-effort synchronous stop for a window close.

        The daily launcher also has a shell-level guard, but keeping this in
        the GUI itself makes a direct/manual launch equally safe.  It affects
        only recorder persistence, never the independent teleoperation stack.
        """
        context = zmq.Context(); sock = context.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.SNDTIMEO, 2000); sock.setsockopt(zmq.RCVTIMEO, 3000)
        try:
            sock.connect(self.endpoint); sock.send_json({"command": "status"}); state = sock.recv_json()
            if state.get("running"):
                sock.close(0); context.term()
                context = zmq.Context(); sock = context.socket(zmq.REQ)
                sock.setsockopt(zmq.LINGER, 0); sock.setsockopt(zmq.SNDTIMEO, 2000); sock.setsockopt(zmq.RCVTIMEO, 3000)
                sock.connect(self.endpoint)
                sock.send_json({"command": "episode_stop", "result": "aborted", "failure_code": "collection_window_closed"})
                sock.recv_json()
        except Exception:
            pass
        finally:
            sock.close(0); context.term()


def button(canvas: np.ndarray, box: tuple[int, int, int, int], title: str, color: tuple[int, int, int], enabled: bool) -> np.ndarray:
    x1, y1, x2, y2 = box
    cv2.rectangle(canvas, (x1, y1), (x2, y2), color if enabled else (70, 70, 70), -1)
    return chinese(canvas, title, (x1 + 18, y1 + 15), 21, (255, 255, 255))


def inside(box: tuple[int, int, int, int], x: int, y: int) -> bool:
    return box[0] <= x <= box[2] and box[1] <= y <= box[3]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--jetson", default="192.168.50.2")
    parser.add_argument("--preview-port", type=int, default=5556)
    parser.add_argument("--record-port", type=int, default=5557)
    args = parser.parse_args()
    context = zmq.Context(); stream = context.socket(zmq.SUB)
    stream.setsockopt(zmq.SUBSCRIBE, b""); stream.setsockopt(zmq.CONFLATE, 1)
    stream.connect(f"tcp://{args.jetson}:{args.preview_port}")
    poller = zmq.Poller(); poller.register(stream, zmq.POLLIN)
    control = SessionControl(args.jetson, args.record_port)
    control.request("session_start")
    title = "OpenArm Collection Console"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL); cv2.resizeWindow(title, 1280, 1260); cv2.waitKey(1)
    task_index = 1
    footer_y = 1050
    task_boxes = [(25 + n * 315, footer_y + 75, 325 + n * 315, footer_y + 130) for n in range(4)]
    start_box = (25, footer_y + 145, 395, footer_y + 205); success_box = (415, footer_y + 145, 795, footer_y + 205)
    failure_box = (815, footer_y + 145, 1105, footer_y + 205); abort_box = (1120, footer_y + 145, 1270, footer_y + 205)
    close_box = (1040, footer_y + 5, 1270, footer_y + 45)
    def footer_box(box: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
        return (box[0], box[1] - footer_y, box[2], box[3] - footer_y)
    def click(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
        nonlocal task_index
        if event != cv2.EVENT_LBUTTONUP: return
        state, _, pending = control.snapshot(); running = bool(state.get("running")); phase = state.get("phase")
        for index, item in enumerate(task_boxes):
            if inside(item, x, y) and not running and not pending:
                task_index = index; return
        if inside(start_box, x, y) and not running and phase not in {"starting", "stopping"} and not pending:
            code, label = TASKS[task_index]; control.request("episode_start", task=f"{code}. {label}"); return
        if running and not pending:
            if inside(success_box, x, y): control.request("episode_stop", result="success")
            elif inside(failure_box, x, y): control.request("episode_stop", result="failure", failure_code="operator_marked_failure")
            elif inside(abort_box, x, y): control.request("episode_stop", result="aborted", failure_code="operator_aborted")
        elif inside(close_box, x, y) and not pending:
            control.request("session_close")
    cv2.setMouseCallback(title, click)
    last_status = 0.0; packet = None; panels: dict[str, np.ndarray] | None = None
    last_at = {role: None for role in ROLES}; fps = {role: 0.0 for role in ROLES}
    while True:
        received = stream in dict(poller.poll(timeout=20)); now = time.time()
        if received:
            packet = json.loads(stream.recv_string()); panels = {}
            for role in ROLES:
                image = decode(packet["images"][role]); earlier = last_at[role]
                if earlier:
                    current = 1.0 / max(now - earlier, 1e-3); fps[role] = current if not fps[role] else 0.85 * fps[role] + 0.15 * current
                last_at[role] = now
                cv2.rectangle(image, (0, 0), (640, 40), (0, 0, 0), -1)
                age = (now - float(packet["timestamps"][role])) * 1000
                panels[role] = chinese(image, f"{TITLES[role]}  {fps[role]:.1f} FPS  {age:.0f} ms", (10, 7), 18, (0, 255, 0))
        if now - last_status > 1.0:
            control.request("status"); last_status = now
        if panels is None:
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")): break
            continue
        canvas = np.zeros((1050, 1280, 3), dtype=np.uint8)
        canvas[35:515, 320:960] = panels["chest"]
        canvas[550:1030, :640] = panels["left_wrist"]
        canvas[550:1030, 640:] = panels["right_wrist"]
        canvas = chinese(canvas, "OpenArm 蘑菇采摘｜实时 RGB 预览与正式数据采集", (18, 5), 20, (225, 225, 225))
        state, message, pending = control.snapshot(); running = bool(state.get("running")); phase = str(state.get("phase", "idle"))
        footer = np.zeros((210, 1280, 3), dtype=np.uint8)
        session = state.get("session_id") or "正在创建会话…"
        health = state.get("camera_health", {}); camera_ok = health.get("ok") is True
        footer = chinese(footer, f"会话：{session}    相机：{'三路健康' if camera_ok else '等待三路相机健康'}    遥操：由独立遥操程序持续控制", (22, 7), 18, (0, 255, 255))
        footer = chinese(footer, "选择当前任务（只影响本 episode 的元数据）：", (25, 75), 18, (220, 220, 220))
        for index, (code, label) in enumerate(TASKS):
            footer = button(footer, footer_box(task_boxes[index]), f"{code}  {label}", (35, 105, 180) if index == task_index else (75, 75, 75), not running and not pending)
        footer = button(footer, footer_box(close_box), "结束采集会话", (80, 80, 80), not running and not pending)
        footer = button(footer, footer_box(start_box), "开始本 episode", (0, 145, 0), not running and camera_ok and not pending)
        footer = button(footer, footer_box(success_box), "成功并保存", (0, 125, 0), running and not pending)
        footer = button(footer, footer_box(failure_box), "失败并保存", (0, 80, 200), running and not pending)
        footer = button(footer, footer_box(abort_box), "中止", (100, 70, 30), running and not pending)
        elapsed = f"  已录制 {int(now - state['started_unix_s'])} 秒" if running and state.get("started_unix_s") else ""
        footer = chinese(footer, message + elapsed, (22, 187), 17, (0, 255, 255))
        cv2.imshow(title, cv2.vconcat([canvas, footer]))
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
    control.stop_active_episode_before_exit()
    stream.close(0); context.term(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

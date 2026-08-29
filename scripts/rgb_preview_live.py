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


ROLES = ("left_wrist", "right_wrist", "chest")
TITLES = {
    "chest": "CHEST CAMERA  |  GLOBAL WORKSPACE VIEW",
    "left_wrist": "LEFT WRIST CAMERA  |  LEFT GRIPPER VIEW",
    "right_wrist": "RIGHT WRIST CAMERA  |  RIGHT GRIPPER VIEW",
}


def decode(encoded: str) -> np.ndarray:
    raw = np.frombuffer(base64.b64decode(encoded), np.uint8)
    image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("invalid JPEG preview frame")
    return image


def label(image: np.ndarray, role: str, age_ms: float, sequence: int) -> np.ndarray:
    panel = image.copy()
    text = f"{TITLES[role]}  |  LIVE  |  {age_ms:.0f} ms  |  #{sequence}"
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(panel, text, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 1, cv2.LINE_AA)
    return panel


class RecordControl:
    def __init__(self, jetson: str, port: int) -> None:
        self.endpoint = f"tcp://{jetson}:{port}"
        self.status = "RECORDER: checking..."
        self.running = False
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
                    self.status = ("RECORDER: " + ("RUNNING" if self.running else "IDLE")
                                   + (f" — {response.get('error', response.get('message', 'ready'))}" if not response.get("ok", False) else ""))
            except Exception as exc:
                with self._lock:
                    self.status = f"RECORDER OFFLINE: {exc}"
                    self.running = False
            finally:
                socket.close(0); context.term()
        threading.Thread(target=run, daemon=True).start()

    def snapshot(self) -> tuple[str, bool]:
        with self._lock:
            return self.status, self.running


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
    name = "OpenArm LIVE RGB Preview  (q/Esc: close)"
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, 1280, 920)
    # Mouse coordinates are in the 1280x1160 composed canvas.
    button = {"x1": 330, "y1": 1050, "x2": 950, "y2": 1130}
    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONUP and button["x1"] <= x <= button["x2"] and button["y1"] <= y <= button["y2"]:
            _, running = control.snapshot()
            control.request("stop" if running else "start")
    cv2.setMouseCallback(name, on_mouse)
    while True:
        packet = json.loads(socket.recv_string())
        now = time.time()
        panels = {}
        for role in ROLES:
            image = decode(packet["images"][role])
            age_ms = (now - float(packet["timestamps"][role])) * 1000.0
            panels[role] = label(image, role, age_ms, int(packet["frame_seq"][role]))
        # Chest view is deliberately centred on top; the two wrist views are
        # side-by-side below, matching the operator's left/right arms.
        canvas = np.zeros((1040, 1280, 3), dtype=np.uint8)
        canvas[35:515, 320:960] = panels["chest"]
        canvas[550:1030, 0:640] = panels["left_wrist"]
        canvas[550:1030, 640:1280] = panels["right_wrist"]
        cv2.putText(canvas, "OpenArm remote teleoperation - LIVE RGB only", (18, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (220, 220, 220), 1, cv2.LINE_AA)
        footer = np.zeros((120, canvas.shape[1], 3), dtype=np.uint8)
        status, running = control.snapshot()
        color = (0, 0, 220) if running else (0, 150, 0)
        # Keep one large, unambiguous control. It starts/stops only Jetson
        # dataset recording; it never controls CAN or robot motion.
        bx1, bx2 = button["x1"], button["x2"]
        cv2.rectangle(footer, (bx1, 10), (bx2, 90), color, -1)
        action = "STOP & SAVE RECORDING" if running else "START LOCAL RGB-D RECORDING"
        cv2.putText(footer, action, (bx1 + 38, 57), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(footer, "Saves RGB-D + follower state/action on Jetson. Does NOT move the robot.",
                    (250, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(footer, status, (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.imshow(name, cv2.vconcat([canvas, footer]))
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
    socket.close(0)
    context.term()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

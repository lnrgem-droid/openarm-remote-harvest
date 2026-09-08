#!/usr/bin/env python3
"""Hardware-free regression tests for the operator collection console."""
from __future__ import annotations

import importlib.util
import base64
import io
import json
import threading
import time
import tkinter as tk
import unittest
from pathlib import Path

import zmq
from PIL import Image


SCRIPT = Path(__file__).parents[1] / "scripts" / "mushroom_collection_console.py"
SPEC = importlib.util.spec_from_file_location("mushroom_collection_console", SCRIPT)
ui = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ui)
REAL_PREVIEW_RECEIVER = ui.PreviewReceiver


class FakeControl:
    def __init__(self, *_args) -> None:
        self.calls: list[tuple[str, dict]] = []
        self.pending = False
        self.message = "idle"
        self.value = {
            "ok": True, "running": False, "phase": "idle", "session_id": None,
            "free_gb": 100.0, "camera_health": {"ok": True},
            "next_episode_by_task": {"left": 1, "right": 1, "test": 1},
            "task_statistics": {"left": {}, "right": {}}, "active_episode": None,
        }

    def request(self, command: str, **extra: str) -> bool:
        if self.pending and command != "status":
            return False
        self.calls.append((command, dict(extra)))
        if command == "session_start":
            self.value["session_id"] = "simulation"
        elif command == "episode_start":
            self.value.update(running=True, phase="recording", active_episode={"task": extra["task"]})
        elif command == "episode_stop":
            self.value.update(running=False, phase="idle", active_episode=None,
                              last_episode={"result": extra["result"]})
        elif command == "session_close":
            self.value["session_id"] = None
        return True

    def snapshot(self):
        return dict(self.value), self.message, self.pending

    def abort_synchronously(self, reason: str) -> None:
        self.calls.append(("abort_synchronously", {"reason": reason}))

    def close_session_synchronously(self) -> None:
        self.calls.append(("close_session_synchronously", {}))
        self.value["session_id"] = None


class FakePreview:
    def __init__(self, *_args) -> None:
        self.stop = threading.Event()
        self.preview_age_s = 0.0

    def start(self) -> None:
        pass

    def latest(self):
        return None

    def age_s(self):
        return self.preview_age_s


class FakeTeleop:
    def __init__(self, *_args) -> None:
        self.stop = threading.Event()
        self.value = {"state": "RUNNING", "fault_bits": 0, "last_running_age_s": 0.0}

    def start(self) -> None:
        pass

    def snapshot(self):
        return dict(self.value)


class Args:
    jetson = "127.0.0.1"
    jetson_ssh = "unused"
    preview_port = 5556
    record_port = 5557
    automated_smoke_test = False


class ConsoleSimulationTest(unittest.TestCase):
    def setUp(self) -> None:
        ui.SessionControl = FakeControl
        ui.PreviewReceiver = FakePreview
        ui.TeleopMonitor = FakeTeleop
        self.warnings: list[tuple[str, str]] = []
        self.errors: list[tuple[str, str]] = []
        ui.messagebox.showwarning = lambda title, body: self.warnings.append((title, body))
        ui.messagebox.showerror = lambda title, body: self.errors.append((title, body))
        ui.messagebox.askyesno = lambda *_args: True
        self.root = tk.Tk(); self.root.withdraw(); self.root.tk.call("tk", "scaling", 1.0)
        self.app = ui.MushroomCollectionApp(self.root, Args())
        self.app._tick_once()

    def tearDown(self) -> None:
        try:
            self.app.preview.stop.set(); self.app.teleop.stop.set(); self.root.destroy()
        except tk.TclError:
            pass

    def test_complete_operator_state_matrix(self) -> None:
        control = self.app.control
        self.assertIn("0001", self.app.start_left_button.cget("text"))
        self.assertIn("episodes/left/episode_0001", self.app.storage_vars["left"].get())
        self.assertEqual(self.app.success_button.cget("state"), "disabled")

        self.app.on_result("success")
        self.assertIn("没有", self.app.message_var.get())

        self.app.start_left_button.invoke()
        self.assertTrue(self.warnings and "READY" in self.warnings[-1][1])
        before = len(control.calls)
        self.app.left_ready.set(True); self.app.start_left_button.invoke()
        self.assertEqual(control.calls[-1][0], "episode_start")
        self.assertGreater(len(control.calls), before)

        control.value.update(running=True, phase="starting")
        self.app._tick_once()
        self.assertEqual(self.app.failure_button.cget("state"), "disabled")
        self.assertIn("录制准备中", self.app.status_var.get())
        before = len(control.calls); self.app.on_result("failure")
        self.assertEqual(len(control.calls), before)
        self.assertIn("初始化", self.app.message_var.get())

        control.value.update(running=True, phase="recording", active_episode={"task": "LEFT_GRASP_LOG"})
        self.app._tick_once()
        self.assertEqual(self.app.failure_button.cget("state"), "normal")
        self.assertIn("正在录制", self.app.status_var.get())
        self.app.teleop.value = {"state": "FAULT", "fault_bits": 1, "last_running_age_s": 4.0}
        self.app._tick_once()
        self.assertEqual(control.calls[-1], (
            "episode_stop", {"result": "aborted", "failure_code": "teleoperation_not_running"}
        ))

        control.value.update(running=False, phase="idle", active_episode=None)
        self.app._tick_once()
        before = len(control.calls); self.app.start_left_button.invoke()
        self.assertEqual(len(control.calls), before)
        self.assertTrue(self.errors and "未运行" in self.errors[-1][0])

        self.app.teleop.value = {"state": "RUNNING", "fault_bits": 0, "last_running_age_s": 0.0}
        control.pending = True; before = len(control.calls); self.app.start_left_button.invoke()
        self.assertEqual(len(control.calls), before)

    def test_right_episode_requires_ready_target_and_healthy_cameras(self) -> None:
        control = self.app.control
        self.app.start_right_button.invoke()
        self.assertTrue(self.warnings and "READY" in self.warnings[-1][0])

        self.app.right_ready.set(True)
        self.app.start_right_button.invoke()
        self.assertTrue(self.warnings and "目标" in self.warnings[-1][0])

        self.app.target_confirmed.set(True)
        control.value["camera_health"] = {"ok": False}
        before = len(control.calls)
        self.app.start_right_button.invoke()
        self.assertEqual(len(control.calls), before)
        self.assertTrue(self.warnings and "不能开始" in self.warnings[-1][0])

    def test_notice_survives_refresh_and_rejected_close_stays_open(self) -> None:
        control = self.app.control
        self.app.set_notice("测试提示", seconds=5)
        self.app._tick_once()
        self.assertIn("测试提示", self.app.message_var.get())

        # Model a manager rejecting session_close: the session id remains.
        control.value["session_id"] = "simulation"
        self.app.close_requested = True
        self.app._tick_once()
        self.assertFalse(self.app.closing)

    def test_normal_idle_window_close_ends_batch_but_does_not_touch_teleop(self) -> None:
        control = self.app.control
        control.value["session_id"] = "simulation"
        self.app.on_window_close()
        self.assertIn(("close_session_synchronously", {}), control.calls)
        self.assertNotIn(("abort_synchronously", {"reason": "collection_window_closed"}), control.calls)

    def test_json_status_parser_tolerates_ros_noise(self) -> None:
        value = ui.parse_json_output("ROS warning before status\n{\"state\": \"RUNNING\", \"fault_bits\": 0}\n")
        self.assertEqual(value["state"], "RUNNING")
        with self.assertRaises(ValueError):
            ui.parse_json_output("only warnings")

    def test_stale_preview_is_never_presented_as_live(self) -> None:
        self.app.preview.preview_age_s = 2.0
        self.app._tick_once()
        self.assertEqual(self.app.camera_metric_vars["chest"].get(), "预览断线")
        self.assertIn("中断", self.app.chest_image.cget("text"))


class PreviewRecoveryTest(unittest.TestCase):
    def test_bad_preview_packet_does_not_kill_following_live_frames(self) -> None:
        context = zmq.Context(); publisher = context.socket(zmq.PUB)
        port = publisher.bind_to_random_port("tcp://127.0.0.1")
        receiver = REAL_PREVIEW_RECEIVER(f"tcp://127.0.0.1:{port}")
        receiver.start(); time.sleep(0.2)
        publisher.send_string("{malformed-json")

        image = Image.new("RGB", (8, 8), (10, 20, 30)); encoded = io.BytesIO()
        image.save(encoded, format="JPEG")
        jpeg = base64.b64encode(encoded.getvalue()).decode()
        packet = json.dumps({
            "images": {role: jpeg for role in ui.ROLES},
            "timestamps": {role: time.time() for role in ui.ROLES},
        })
        latest = None
        for _ in range(20):
            publisher.send_string(packet); time.sleep(0.03)
            latest = receiver.latest()
            if latest is not None:
                break
        receiver.stop.set(); receiver.thread.join(timeout=2)
        publisher.close(0); context.term()
        self.assertIsNotNone(latest)
        self.assertFalse(receiver.thread.is_alive())
        self.assertEqual(latest[0]["chest"].size, (8, 8))


if __name__ == "__main__":
    unittest.main()

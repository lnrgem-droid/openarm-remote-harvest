#!/usr/bin/env python3
"""Hardware-free regression tests for the operator collection console."""
from __future__ import annotations

import importlib.util
import base64
import io
import json
import sys
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import MagicMock, patch
from pathlib import Path

import zmq
from PIL import Image


SCRIPT = Path(__file__).parents[1] / "scripts" / "mushroom_collection_console.py"
SPEC = importlib.util.spec_from_file_location("mushroom_collection_console", SCRIPT)
ui = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(ui)
REAL_PREVIEW_RECEIVER = ui.PreviewReceiver
REAL_TELEOP_MONITOR = ui.TeleopMonitor
REAL_SESSION_CONTROL = ui.SessionControl


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
            side = next(side for side, task in ui.TASKS.items() if task[0] == extra["task"])
            number = self.value["next_episode_by_task"][side]
            self.value.update(running=True, phase="recording", active_episode={
                "task": extra["task"], "task_group": side, "episode_number": number,
                "episode_id": f"{side}_episode_{number:04d}"})
        elif command == "episode_stop":
            active = self.value.get("active_episode") or {}
            if active.get("task_group"):
                self.value["next_episode_by_task"][active["task_group"]] = active["episode_number"] + 1
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
        self.value = {"state": "RUNNING", "fault_bits": 0, "last_running_age_s": 0.0,
                      "teleop_ready": True,
                      "collection": {"left_mode": "FOLLOW", "right_mode": "FOLLOW"}}
        self.pending = False
        self.request_error = ""
        self.results = ui.queue.Queue()
        self.calls = []

    def start(self) -> None:
        pass

    def snapshot(self):
        return dict(self.value)

    def request(self, command):
        self.calls.append(command)
        return True


class Args:
    jetson = "127.0.0.1"
    jetson_ssh = "unused"
    preview_port = 5556
    record_port = 5557
    automated_smoke_test = False


def master_ready_status():
    """A fresh actual left-capable ACK, plus all eight held-target axes."""
    goal = [0.] * 16
    goal[6] = .2
    return {'state': 'RUNNING', 'fault_bits': 0, 'teleop_ready': True, 'connected': True,
            'action_age_ms': 1., 'feedback_age_ms': 2., 'leader_axes': [0.] * 16, 'applied_axes': goal,
            'collection': {'left_mode': 'HOLD', 'right_mode': 'FOLLOW', 'recording': None,
                'left_align_active': False, 'left_align_phase': 'idle', 'left_align_supported': True,
                'left_alignment_error_rad': .2, 'transitioning_arms': [],
                'left_master_align_supported': True, 'left_master_align_active': False,
                'left_master_align_phase': 'idle', 'left_master_align_detail': {
                    'leader_ack': 16, 'leader_left_mode': 0, 'leader_right_mode': 0, 'servo_released': True}}}


def right_release_status():
    status = master_ready_status()
    collection = status['collection']
    collection.update(right_mode='HOLD', return_phase='hold',
                      right_pause_release_supported=True,
                      right_servo_release_required=True, right_servo_release_pending=False)
    collection['left_master_align_detail'].update(leader_ack=18, leader_right_mode=2)
    return status


def right_master_ready_status():
    status = master_ready_status()
    status['collection'].update(right_mode='HOLD', right_servo_release_required=False,
        right_servo_release_pending=False, right_master_align_supported=True,
        right_master_align_active=False, right_master_align_phase='idle',
        right_master_align_detail={'leader_ack':16,'leader_left_mode':0,'leader_right_mode':0,'servo_released':True})
    status['leader_axes'][14] = .0713
    return status


class ConsoleSimulationTest(unittest.TestCase):
    def right_auto_ready(self):
        self.app.teleop.value = right_master_ready_status()
        self.app._tick_once()

    def test_right_auto_cancel_and_confirm_send_only_the_explicit_start(self):
        self.right_auto_ready()
        self.assertEqual(self.app.motion_buttons['right_master_align'].cget('state'), 'normal')
        self.assertNotIn('right_follow', self.app.motion_buttons)
        with patch.object(ui.messagebox, 'askyesno', return_value=False):
            self.app.on_motion('right_master_align')
        self.assertFalse(self.app.teleop.calls)
        with patch.object(ui.messagebox, 'askyesno', return_value=True) as confirm:
            self.app.on_motion('right_master_align')
        self.assertIn('右主臂的 7 个关节和夹爪', confirm.call_args.args[1])
        self.assertIn('右从臂及其夹爪全程保持原目标不动', confirm.call_args.args[1])
        self.assertIn('停止右主臂对齐', confirm.call_args.args[1])
        self.assertEqual(self.app.teleop.calls, ['right_master_align'])
        self.assertFalse(any(c[0] == 'episode_start' for c in self.app.control.calls))

    def test_right_auto_rechecks_dialog_state_and_unsaved_recording(self):
        for change in [lambda: self.app.control.value.update(active_episode={'id':1}),
                       lambda: self.app.teleop.value.update(action_age_ms=150),
                       lambda: self.app.teleop.value['collection'].update(left_mode='FOLLOW'),
                       lambda: setattr(self.app, 'power_down_locked', True)]:
            self.app.power_down_locked = False
            self.app.control.value['active_episode'] = None
            self.right_auto_ready()
            def confirm(*_):
                change()
                return True
            with patch.object(ui.messagebox, 'askyesno', confirm):
                self.app.on_motion('right_master_align')
            self.assertFalse(self.app.teleop.calls)

    def test_right_auto_stop_available_during_outage_pending_start_and_close(self):
        self.right_auto_ready()
        self.app.right_master_align_may_be_active = True
        self.app.teleop.pending = True
        self.app.teleop.value.update(connected=False, teleop_ready=False, state='DISCONNECTED')
        self.app._tick_once()
        self.assertEqual(self.app.motion_buttons['right_master_pause'].cget('state'), 'normal')
        self.app.on_window_close()
        self.assertEqual(self.app.teleop.calls, ['right_master_pause'])
        self.assertTrue(self.root.winfo_exists())
        self.assertTrue(self.app.right_master_align_stop_requested)

    def test_right_auto_blocks_left_motion_capture_and_manual_right_follow(self):
        self.right_auto_ready()
        self.app.right_master_align_may_be_active = True
        self.app.right_ready.set(True); self.app.target_confirmed.set(True)
        self.app.on_start('right')
        self.app.on_motion('left_master_align')
        self.app.on_motion('right_follow')
        self.app.on_motion('right_return')
        self.assertFalse(self.app.teleop.calls)
        self.assertFalse(any(c[0] == 'episode_start' for c in self.app.control.calls))

    def test_right_auto_success_is_not_claimed_until_free_follow_and_fresh(self):
        self.right_auto_ready()
        c = self.app.teleop.value['collection']
        c.update(right_master_align_phase='completed', right_mode='FOLLOW', transitioning_arms=['right'])
        c['right_master_align_detail']['message'] = '正在恢复跟随'
        self.app._tick_once()
        self.assertNotIn('右臂已恢复跟随', self.app.motion_vars['right'].get())
        c['transitioning_arms'] = []
        self.app._tick_once()
        self.assertIn('右臂已恢复跟随', self.app.motion_vars['right'].get())
        self.app.teleop.value['feedback_age_ms'] = 101
        self.app._tick_once()
        self.assertNotIn('右臂已恢复跟随', self.app.motion_vars['right'].get())

    def test_right_auto_missing_capability_is_disabled_without_motion(self):
        self.right_auto_ready()
        self.app.teleop.value['collection'].pop('right_master_align_supported')
        self.app._tick_once()
        self.assertEqual(self.app.motion_buttons['right_master_align'].cget('state'), 'disabled')
        with patch.object(ui.messagebox, 'askyesno') as confirm:
            self.app.on_motion('right_master_align')
        confirm.assert_not_called()
        self.assertFalse(self.app.teleop.calls)

    def test_right_legacy_return_is_not_reported_as_auto_alignment(self):
        self.right_auto_ready()
        c=self.app.teleop.value['collection']
        c.update(right_mode='RETURNING', return_phase='moving')
        c['right_master_align_detail'].update(leader_right_mode=1, servo_released=False)
        self.app._tick_once()
        self.assertFalse(self.app.right_master_alignment_busy())
        self.assertEqual(self.app.motion_buttons['right_pause'].cget('text'), '停止双端回位')
        self.assertEqual(self.app.motion_buttons['right_master_pause'].cget('state'), 'disabled')
        self.app.on_motion('right_pause')
        self.assertEqual(self.app.teleop.calls, ['right_pause'])

    def test_right_recording_fault_latch_points_to_right_release_without_motion(self):
        self.right_release_ready()
        self.app.right_ready.set(True)
        self.app.target_confirmed.set(True)
        self.app.on_start('right')
        warning = self.warnings[-1][1]
        self.assertIn('左臂已经保持，无需恢复左臂跟随', warning)
        self.assertIn('右侧“解除右主臂回位伺服”', warning)
        self.assertEqual(self.app.motion_buttons['right_master_align'].cget('text'), '主臂自动对齐并恢复右臂跟随')
        self.assertFalse(self.app.teleop.calls)
        self.assertFalse(any(c[0] == 'episode_start' for c in self.app.control.calls))

    def test_right_recording_release_pending_is_not_reported_as_homing(self):
        self.right_release_ready()
        self.app.right_ready.set(True)
        self.app.target_confirmed.set(True)
        self.app.teleop.value['collection'].update(right_mode='RETURNING', right_servo_release_pending=True)
        self.app.on_start('right')
        self.assertIn('解除确认', self.warnings[-1][1])
        self.assertNotIn('仍在回位', self.warnings[-1][1])
        self.assertFalse(self.app.teleop.calls)
        self.assertFalse(any(c[0] == 'episode_start' for c in self.app.control.calls))

    def test_right_recording_guide_uses_right_gripper_and_requires_follow(self):
        self.app.teleop.value = right_master_ready_status()
        status = self.app.teleop.value
        collection = status['collection']
        collection.update(right_servo_release_required=False, right_ready_error_rad=.024, saved={})
        collection['left_master_align_detail'].update(leader_right_mode=0, leader_ack=16)
        status['leader_axes'][15] = .06065
        self.app.right_ready.set(True)
        self.app.target_confirmed.set(True)
        self.app.on_start('right')
        self.assertIn('无需手动对齐', self.warnings[-1][1])
        self.assertIn('右侧“主臂自动对齐并恢复右臂跟随”', self.warnings[-1][1])
        self.assertNotIn('J7', self.warnings[-1][1])  # Left J7 has a larger gap.
        self.app._tick_once()
        self.assertIn('主臂自动对齐并恢复右臂跟随', self.app.motion_vars['right'].get())
        status['leader_axes'][15] = .059
        self.app.on_start('right')
        self.assertIn('主臂自动对齐', self.warnings[-1][1])
        self.assertFalse(self.app.teleop.calls)
        self.assertFalse(any(c[0] == 'episode_start' for c in self.app.control.calls))
        collection['right_mode'] = 'FOLLOW'
        self.app.on_start('right')
        self.assertEqual(self.app.control.calls[-1][0], 'episode_start')
        self.assertFalse(self.app.teleop.calls)

    def test_right_recording_guide_waits_for_fresh_known_feedback(self):
        for change in [lambda s: s.update(action_age_ms=100),
                       lambda s: s.update(connected=False),
                       lambda s: s['collection']['right_master_align_detail'].update(leader_right_mode=None),
                       lambda s: s.update(leader_axes=[0.] * 15),
                       lambda s: s['leader_axes'].__setitem__(15, float('nan'))]:
            status = right_master_ready_status()
            status['collection']['right_servo_release_required'] = False
            status['collection']['left_master_align_detail']['leader_right_mode'] = 0
            change(status)
            reason = ui.right_recording_posture_reason(status)
            self.assertTrue(ui.right_master_start_reason(status))
            self.assertNotIn('请将右主臂', reason)
            self.assertNotIn('请点击右侧', reason)

    def test_right_recording_when_left_follows_only_requests_left_hold(self):
        self.app.right_ready.set(True)
        self.app.target_confirmed.set(True)
        self.app.on_start('right')
        self.assertIn('左侧“保持左臂及夹爪”', self.warnings[-1][1])
        self.assertNotIn('自动对齐', self.warnings[-1][1])
        self.assertFalse(self.app.teleop.calls)

    def right_release_ready(self):
        self.app.teleop.value = right_release_status()
        self.app.teleop.pending = False
        self.app._tick_once()

    def test_right_fault_release_explains_left_gate_and_waits_for_fresh_ack(self):
        self.right_release_ready()
        release = self.app.motion_buttons['right_pause']
        align = self.app.motion_buttons['left_master_align']
        self.assertEqual(release.cget('text'), '解除右主臂回位伺服')
        self.assertEqual(release.cget('state'), 'normal')
        self.assertEqual(align.cget('state'), 'disabled')
        self.assertIn('右主臂回位伺服尚未解除', self.app.motion_vars['left'].get())
        collection = self.app.teleop.value['collection']
        collection.update(right_mode='RETURNING', return_phase='releasing',
                          right_servo_release_required=False, right_servo_release_pending=True)
        self.app._tick_once()
        self.assertEqual(release.cget('state'), 'disabled')
        self.assertEqual(align.cget('state'), 'disabled')
        self.assertIn('解除确认', self.app.motion_vars['left'].get())
        self.assertIn('不会自动恢复跟随', self.app.motion_vars['right'].get())
        self.assertNotIn('已到位', self.app.motion_vars['right'].get())
        self.assertNotIn('恢复跟随中', self.app.motion_vars['right'].get())
        collection['right_servo_release_detail'] = {'message': '等待右主臂解除伺服确认超时，需再次显式请求'}
        collection.update(right_mode='HOLD', return_phase='hold', right_servo_release_pending=False,
                          right_servo_release_required=True)
        self.app._tick_once()
        self.assertIn('确认超时', self.app.motion_vars['right'].get())
        self.assertEqual(align.cget('state'), 'disabled')
        collection.update(right_mode='HOLD', return_phase='hold', right_servo_release_pending=False)
        collection['right_servo_release_required'] = False
        collection['right_servo_release_detail'] = {'message': '右主臂伺服已解除，右从臂保持'}
        collection['left_master_align_detail'].update(leader_ack=16, leader_right_mode=0)
        self.app._tick_once()
        self.assertEqual(align.cget('state'), 'normal')
        self.assertEqual(collection['left_alignment_error_rad'], .2)
        self.assertFalse(self.app.teleop.calls)  # Never chains either follow command.

    def test_right_release_old_backend_requires_reload_without_sending(self):
        self.right_release_ready()
        self.app.teleop.value['collection'].pop('right_pause_release_supported')
        self.app._tick_once()
        self.assertEqual(self.app.motion_buttons['right_pause'].cget('state'), 'disabled')
        self.assertIn('重新加载', self.app.motion_vars['right'].get())
        self.assertIn('右主臂回位伺服尚未解除', self.app.motion_vars['left'].get())
        with patch.object(ui.messagebox, 'askyesno') as confirm:
            self.app.on_motion('right_pause')
        confirm.assert_not_called()
        self.assertFalse(self.app.teleop.calls)

    def test_right_release_cancel_and_unconfirmed_alias_never_send(self):
        self.right_release_ready()
        with patch.object(ui.messagebox, 'askyesno', return_value=False) as confirm:
            self.app.on_motion('right_pause')
        self.assertIn('重力补偿', confirm.call_args.args[1])
        self.assertIn('右从臂及夹爪保持当前目标', confirm.call_args.args[1])
        self.assertIn('不会自动恢复跟随', confirm.call_args.args[1])
        self.app.on_motion('right_servo_release')
        self.assertFalse(self.app.teleop.calls)

    def test_right_release_confirm_sends_only_explicit_release_and_blocks_duplicate(self):
        self.right_release_ready()
        def send(command):
            self.app.teleop.calls.append(command)
            self.app.teleop.pending = True
            return True
        self.app.teleop.request = send
        with patch.object(ui.messagebox, 'askyesno', return_value=True) as confirm:
            self.app.on_motion('right_pause')
            self.assertIn('等待实时确认', self.app.operator_notice)
            self.app.on_motion('right_pause')
        self.assertEqual(confirm.call_count, 1)
        self.assertEqual(self.app.teleop.calls, ['right_servo_release'])
        self.assertIn('尚未确认', self.app.operator_notice)

    def test_right_release_rechecks_health_recording_and_state_after_confirmation(self):
        changes = [lambda: self.app.teleop.value.update(action_age_ms=101),
                   lambda: self.app.teleop.value['collection'].update(right_mode='RETURNING'),
                   lambda: self.app.teleop.value['collection']['left_master_align_detail'].update(leader_right_mode=None),
                   lambda: self.app.control.value.update(running=True, phase='recording'),
                   lambda: self.app.control.value.update(active_episode={'episode_number': 1}),
                   lambda: setattr(self.app.teleop, 'pending', True),
                   lambda: setattr(self.app, 'power_down_locked', True)]
        for index, change in enumerate(changes):
            with self.subTest(change=index):
                self.app.power_down_locked = False
                self.app.control.value.update(running=False, phase='idle', active_episode=None)
                self.right_release_ready()
                def confirm(*_args):
                    change()
                    return True
                with patch.object(ui.messagebox, 'askyesno', confirm):
                    self.app.on_motion('right_pause')
                self.assertFalse(self.app.teleop.calls)

    def test_right_release_unknown_stale_and_recording_disable_without_confirmation(self):
        changes = [lambda s: s.update(connected=False), lambda s: s.update(teleop_ready=False),
                   lambda s: s.update(action_age_ms=100), lambda s: s.update(feedback_age_ms=None),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_right_mode=False),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_right_mode=3),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_left_mode=None),
                   lambda s: s['collection'].update(left_align_active=True),
                   lambda s: s['collection'].update(transitioning_arms=['right']),
                   lambda s: s['collection'].update(recording={'token': 'episode'})]
        for index, change in enumerate(changes):
            with self.subTest(change=index):
                self.right_release_ready()
                change(self.app.teleop.value)
                self.app._tick_once()
                self.assertEqual(self.app.motion_buttons['right_pause'].cget('state'), 'disabled')
                with patch.object(ui.messagebox, 'askyesno') as confirm:
                    self.app.on_motion('right_pause')
                confirm.assert_not_called()
                self.assertFalse(self.app.teleop.calls)

    def test_ordinary_right_return_stop_remains_immediate_without_release_confirmation(self):
        self.app.teleop.value['collection'].update(right_mode='RETURNING', return_phase='moving')
        with patch.object(ui.messagebox, 'askyesno') as confirm:
            self.app.on_motion('right_pause')
        confirm.assert_not_called()
        self.assertEqual(self.app.teleop.calls, ['right_pause'])

    def master_alignment_ready(self):
        self.app.teleop.value = master_ready_status()
        self.app.left_master_align_may_be_active = self.app.left_master_align_stop_requested = False
        self.app.teleop.pending = False
        self.app._tick_once()

    def test_master_start_requires_explicit_concrete_movement_confirmation(self):
        self.master_alignment_ready()
        self.assertEqual(self.app.motion_buttons['left_master_align'].cget('state'), 'normal')
        with patch.object(ui.messagebox, 'askyesno', return_value=False) as confirm:
            self.app.on_motion('left_master_align')
        text = confirm.call_args.args[1]
        self.assertIn('左主臂的 7 个关节和夹爪将主动低速移动', text)
        self.assertIn('左从臂及其夹爪全程保持原目标不动', text)
        self.assertFalse(self.app.teleop.calls)
        with patch.object(ui.messagebox, 'askyesno', return_value=True):
            self.app.on_motion('left_master_align')
        self.assertEqual(self.app.teleop.calls, ['left_master_align'])
        self.assertTrue(self.app.left_master_align_may_be_active)
        self.app.on_motion('left_master_align')
        self.assertEqual(self.app.teleop.calls, ['left_master_align'])

    def test_master_confirmation_rechecks_recording_started_during_dialog(self):
        self.master_alignment_ready()
        def confirm(*_args):
            self.app.control.value['running'] = True
            return True
        with patch.object(ui.messagebox, 'askyesno', confirm):
            self.app.on_motion('left_master_align')
        self.assertFalse(self.app.teleop.calls)

    def test_master_unsealed_episode_blocks_before_and_after_confirmation(self):
        for during_dialog in (False, True):
            with self.subTest(during_dialog=during_dialog):
                self.master_alignment_ready()
                self.app.control.value.update(running=False, phase='awaiting_result', active_episode=None)
                def unsealed(*_args):
                    self.app.control.value['active_episode'] = {'episode_id': 'left_episode_0001'}
                    return True
                if not during_dialog:
                    unsealed()
                    self.app._tick_once()
                    self.assertEqual(self.app.motion_buttons['left_master_align'].cget('state'), 'disabled')
                with patch.object(ui.messagebox, 'askyesno', unsealed) as confirm:
                    self.app.on_motion('left_master_align')
                self.assertFalse(self.app.teleop.calls)
                self.assertIn('先结束并保存', self.app.message_var.get())

    def test_master_ui_capability_and_recording_gates(self):
        for change in ('old_controller', 'recording', 'right_return', 'stale', 'transition', 'follow'):
            with self.subTest(change=change):
                self.master_alignment_ready()
                status = self.app.teleop.value
                collection = status['collection']
                if change == 'old_controller': collection.pop('left_master_align_supported')
                elif change == 'recording': self.app.control.value['running'] = True
                elif change == 'right_return': collection['right_mode'] = 'RETURNING'
                elif change == 'stale': status['action_age_ms'] = 100
                elif change == 'transition': collection['transitioning_arms'] = ['right']
                else: collection['left_mode'] = 'FOLLOW'
                self.app._tick_once()
                self.assertEqual(self.app.motion_buttons['left_master_align'].cget('state'), 'disabled')
                self.app.on_motion('left_master_align')
                self.assertFalse(self.app.teleop.calls)
                self.app.control.value['running'] = False

    def test_master_pending_blocks_recording_right_motion_and_manual_adjustments(self):
        self.master_alignment_ready()
        self.app.on_motion('left_master_align')
        before = list(self.app.control.calls)
        self.app.on_start('right')
        self.app.on_motion('right_return')
        self.app.open_manual_left_guide()
        self.assertFalse(self.app.manual_left_check()['can_adjust'])
        self.assertFalse(self.app.manual_left_check()['can_resume'])
        self.assertEqual(self.app.control.calls, before)
        self.assertEqual(self.app.teleop.calls, ['left_master_align'])
        self.app._tick_once()
        self.assertEqual(self.app.start_right_button.cget('state'), 'disabled')
        self.assertEqual(self.app.motion_buttons['left_master_pause'].cget('state'), 'normal')

    def test_master_hold_and_close_request_pause_without_losing_window(self):
        for operation in ('hold', 'session', 'window'):
            with self.subTest(operation=operation):
                self.master_alignment_ready()
                self.app.teleop.calls.clear()
                self.app.left_master_align_may_be_active = True
                if operation == 'hold': self.app.on_motion('left_lock')
                elif operation == 'session': self.app.on_close_session()
                else: self.app.on_window_close()
                self.assertEqual(self.app.teleop.calls, ['left_master_pause'])
                self.assertTrue(self.app.left_master_align_stop_requested)
                self.assertTrue(self.root.winfo_exists())
                self.assertFalse(self.app.close_requested)

    def test_master_failed_release_never_claims_gravity_or_allows_close(self):
        self.master_alignment_ready()
        collection = self.app.teleop.value['collection']
        collection.update(left_master_align_active=True, left_master_align_phase='failed')
        collection['left_master_align_detail'].update(servo_released=False, leader_left_mode=2,
                                                     message='release timeout')
        self.app._tick_once()
        self.assertNotIn('伺服已解除', self.app.motion_vars['left'].get())
        self.assertNotIn('正在低速对齐', self.app.motion_vars['left'].get())
        self.assertIn('对齐失败，轨迹已停止', self.app.motion_vars['left'].get())
        self.app.on_window_close()
        self.assertEqual(self.app.teleop.calls, ['left_master_pause'])
        self.assertTrue(self.root.winfo_exists())

    def test_master_only_verified_power_down_allows_close_despite_cached_activity(self):
        self.master_alignment_ready()
        self.app.left_master_align_may_be_active = True
        self.app.power_down_locked = True
        self.app.power_down_succeeded = False  # Failed/partial/unknown disable is not an exception.
        self.app.on_close_session()
        self.app.on_window_close()
        self.assertFalse(self.app.close_requested)
        self.assertTrue(self.root.winfo_exists())
        self.app.power_down_succeeded = True  # Set only by the verified coordinator result.
        self.app.on_close_session()
        self.assertTrue(self.app.close_requested)
        self.app.on_window_close()
        self.assertFalse(self.app.teleop.calls)
        self.assertIn(('close_session_synchronously', {}), self.app.control.calls)

    def test_master_pause_display_requires_actual_free_ack_and_holds_follower(self):
        self.master_alignment_ready()
        collection = self.app.teleop.value['collection']
        collection.update(left_master_align_phase='paused')
        self.app._tick_once()
        self.assertIn('左从臂保持，左主臂伺服已解除', self.app.motion_vars['left'].get())
        self.app.teleop.value['feedback_age_ms'] = 150.
        self.app._tick_once()
        self.assertNotIn('主臂伺服已解除', self.app.motion_vars['left'].get())

    def test_master_success_needs_follow_and_completed_transition(self):
        self.master_alignment_ready()
        collection = self.app.teleop.value['collection']
        collection.update(left_master_align_phase='completed', left_mode='FOLLOW', transitioning_arms=['left'])
        self.app._tick_once()
        self.assertNotIn('左臂已恢复跟随', self.app.motion_vars['left'].get())
        collection['transitioning_arms'] = []
        self.app._tick_once()
        self.assertIn('左臂已恢复跟随', self.app.motion_vars['left'].get())
        self.app.teleop.value['teleop_ready'] = False
        self.app._tick_once()
        self.assertNotIn('左臂已恢复跟随', self.app.motion_vars['left'].get())

    def test_old_backend_can_close_without_new_master_command(self):
        self.app.on_close_session()
        self.assertTrue(self.app.close_requested)
        self.assertFalse(self.app.teleop.calls)

    def test_power_down_cancel_does_nothing(self):
        self.app.confirm_power_down = lambda: False
        calls = []
        self.app.power_down.start = lambda: calls.append("start")
        self.app.on_power_down()
        self.assertFalse(calls)
        self.assertFalse(self.app.power_down_locked)

    def test_power_down_locks_actions_and_window_until_result(self):
        self.app.confirm_power_down = lambda: True
        calls = []
        self.app.power_down.start = lambda: calls.append("start") or True
        self.app.on_power_down()
        self.app.on_power_down()
        self.assertEqual(calls, ["start"])
        previous = list(self.app.control.calls)
        self.app.on_start("left")
        self.app.on_motion("right_return")
        self.app.on_result("success")
        self.app.on_window_close()
        self.app.on_close_session()
        self.app._tick_once()
        self.assertTrue(self.root.winfo_exists())
        self.assertEqual(self.app.control.calls, previous)
        self.assertFalse(self.app.teleop.calls)
        self.assertEqual(self.app.start_left_button.cget("state"), "disabled")
        self.assertEqual(self.app.power_down_button.cget("state"), "disabled")
        self.app.power_down.events.put({"event": "complete", "ok": True,
            "message": "主从电机已确认失能", "devices": {
                side: {"ok": True, "verified": True} for side in ("host", "follower")}})
        self.app._tick_once()
        self.assertFalse(self.app.power_down_busy)
        self.assertTrue(self.app.power_down_succeeded)
        self.assertIn("已确认失能", self.app.message_var.get())
        self.assertTrue(self.root.winfo_exists())

    def test_failed_power_down_can_retry_but_never_resumes_motion(self):
        self.app.power_down_locked = self.app.power_down_busy = True
        self.app.power_down.events.put({"event": "complete", "ok": False,
                                      "message": "下电未完全确认", "devices": {}})
        self.app._tick_once()
        self.assertFalse(self.app.power_down_succeeded)
        self.assertEqual(self.app.power_down_button.cget("state"), "normal")
        self.assertEqual(self.app.start_left_button.cget("state"), "disabled")
        self.assertIn("未确认失能", self.app.power_down_detail.get())

    def test_power_down_does_not_overwrite_finalizing_result(self):
        self.app.power_down_locked = self.app.power_down_busy = True
        self.app.control.value.update(running=True, phase="stopping")
        self.app.teleop.value = {"state": "FAULT", "last_running_age_s": 10.}
        previous = list(self.app.control.calls)
        self.app._tick_once()
        self.assertEqual(self.app.control.calls, previous)

    def test_motion_buttons_and_recording_gate(self):
        self.app.motion_buttons["left_lock"].invoke()
        self.assertEqual(self.app.teleop.calls, ["left_lock"])
        self.app.control.value.update(running=True, phase="recording")
        self.app.on_motion("right_return")
        self.assertEqual(self.app.teleop.calls, ["left_lock"])
        self.app.control.value.update(running=False, phase="idle")
        self.app.right_ready.set(True); self.app.target_confirmed.set(True)
        self.app.on_start("right")
        self.assertNotEqual(self.app.control.calls[-1][0], "episode_start")
        self.app.teleop.value["collection"].update(left_mode="HOLD", saved={}, right_ready_error_rad=.02)
        self.app.teleop.value["collection"]["transitioning_arms"] = ["right"]
        self.app.on_start("right")
        self.assertNotEqual(self.app.control.calls[-1][0], "episode_start")
        self.assertIn("平滑", self.warnings[-1][1])
        self.app.teleop.value["collection"]["transitioning_arms"] = []
        self.app.on_start("right")
        self.assertEqual(self.app.control.calls[-1][0], "episode_start")

    def test_window_close_requests_return_pause_before_closing(self):
        self.app.teleop.value["collection"]["right_mode"] = "RETURNING"
        self.app.on_window_close()
        self.assertEqual(self.app.teleop.calls, ["right_pause"])
        self.assertTrue(self.root.winfo_exists())

    def manual_alignment_ready(self, error=.2):
        leader = [0.] * 16
        target = [0.] * 16
        target[6] = error
        self.app.teleop.value = {"state": "RUNNING", "fault_bits": 0, "teleop_ready": True,
            "connected": True, "action_age_ms": 1., "feedback_age_ms": 2.,
            "leader_axes": leader, "applied_axes": target,
            "collection": {"left_mode": "HOLD", "right_mode": "FOLLOW", "recording": None,
                "left_align_supported": True, "left_align_active": False, "left_align_phase": "idle",
                "left_align_detail": {}, "transitioning_arms": [], "left_alignment_error_rad": error}}
        self.app.left_align_may_be_active = self.app.left_align_stop_requested = False
        self.app.teleop.pending = False
        self.app._tick_once()

    def test_manual_guide_opens_updates_and_closes_without_any_commands(self):
        self.manual_alignment_ready()
        before = list(self.app.control.calls)
        self.assertNotIn("left_manual_guide", self.app.motion_buttons)
        self.app.on_motion("left_manual_guide")
        self.assertIsNone(self.app.manual_left_dialog)
        self.assertFalse(self.app.teleop.calls)
        self.assertIn("尚未加载", self.app.motion_vars["left"].get())
        self.assertEqual(self.app.motion_buttons["left_master_align"].cget("state"), "disabled")
        # The retained internal readout also supplies the automatic gate's
        # validated target data, but has no operator entry point.
        self.app.open_manual_left_guide()
        self.assertIsNotNone(self.app.manual_left_dialog)
        self.assertFalse(self.app.teleop.calls)
        self.assertEqual(self.app.control.calls, before)
        self.assertEqual(self.app.manual_left_table.get_children(), tuple([f"J{i}" for i in range(1, 8)] + ["夹爪"]))
        self.assertEqual(self.app.manual_left_table.item("J7", "values")[0], "★ J7")
        self.assertEqual(len(self.app.manual_left_table.get_children()), 8)
        self.assertIn("增大", self.app.manual_left_table.item("J7", "values")[-1])
        self.assertEqual(self.app.manual_left_confirm_button.cget("state"), "disabled")
        self.app.teleop.value["leader_axes"][6] = .18
        self.app.teleop.value["collection"]["left_alignment_error_rad"] = .02
        self.app._tick_once()
        self.assertEqual(self.app.manual_left_confirm_button.cget("state"), "normal")
        self.assertFalse(self.app.teleop.calls)
        self.app.close_manual_left_guide()
        self.assertIsNone(self.app.manual_left_dialog)
        self.assertFalse(self.app.teleop.calls)

    def test_manual_guide_uses_held_target_and_includes_gripper_direction(self):
        self.manual_alignment_ready(error=0.)
        self.app.teleop.value["left_actual_rad"] = [9.] * 7
        self.app.teleop.value["applied_axes"][7] = -.3
        self.app.teleop.value["collection"]["left_alignment_error_rad"] = .3
        self.app.open_manual_left_guide()
        self.assertEqual(self.app.manual_left_table.item("夹爪", "values")[0], "★ 夹爪")
        self.assertIn("张开", self.app.manual_left_table.item("夹爪", "values")[-1])
        self.assertIn("-17.19", self.app.manual_left_table.item("夹爪", "values")[2])
        self.app.confirm_manual_left_follow()
        self.assertFalse(self.app.teleop.calls)

    def test_manual_confirm_rechecks_alignment_health_and_complete_gripper_data(self):
        cases = [
            ("not_aligned", lambda s: None),
            ("health", lambda s: s.update(teleop_ready=False, readiness_reason="反馈过期")),
            ("stale", lambda s: s.update(action_age_ms=500)),
            ("missing_gripper", lambda s: s.update(leader_axes=s["leader_axes"][:7])),
            ("right_return", lambda s: s["collection"].update(right_mode="RETURNING")),
            ("auto_active", lambda s: s["collection"].update(left_align_active=True)),
            ("recording_lock", lambda s: s["collection"].update(recording={"token": "busy"})),
        ]
        for label, change in cases:
            with self.subTest(case=label):
                self.manual_alignment_ready(error=.2 if label == "not_aligned" else 0.)
                change(self.app.teleop.value)
                self.app.open_manual_left_guide()
                self.app.confirm_manual_left_follow()
                self.assertFalse(self.app.teleop.calls)
                self.assertEqual(self.app.manual_left_confirm_button.cget("state"), "disabled")
        self.assertNotEqual(self.app.manual_left_feedback.get(), "")

    def test_manual_guide_pauses_adjustment_when_state_is_stale_or_auto_target_moves(self):
        for mutation in ({"action_age_ms": 500}, {"teleop_ready": False}):
            self.manual_alignment_ready()
            self.app.teleop.value.update(mutation)
            self.app.open_manual_left_guide()
            self.assertIn("暂停调整", self.app.manual_left_summary.get())
            self.assertNotIn("优先调整", self.app.manual_left_summary.get())
            self.assertEqual(self.app.manual_left_table.item("J7", "values")[-1], "暂停调整，等待状态确认")
            self.assertEqual(self.app.manual_left_table.item("J7", "tags"), ("unknown",))
        self.manual_alignment_ready()
        self.app.teleop.value["collection"]["left_align_active"] = True
        self.app.update_manual_left_guide()
        self.assertIn("暂停调整", self.app.manual_left_summary.get())
        self.assertFalse(self.app.teleop.calls)

    def test_manual_confirm_only_explicitly_sends_existing_left_follow_once(self):
        self.manual_alignment_ready(error=0.)
        self.app.open_manual_left_guide()
        for _ in range(3): self.app._tick_once()
        self.assertFalse(self.app.teleop.calls)
        def request(command):
            self.app.teleop.calls.append(command)
            self.app.teleop.pending = True
            return True
        self.app.teleop.request = request
        self.app.manual_left_confirm_button.invoke()
        self.app.confirm_manual_left_follow()
        self.assertEqual(self.app.teleop.calls, ["left_follow"])
        self.assertIn("等待", self.app.manual_left_result)

    def test_manual_confirm_is_blocked_while_recorder_or_power_down_is_busy(self):
        self.manual_alignment_ready(error=0.)
        self.app.control.value.update(running=True, phase="recording")
        self.app.confirm_manual_left_follow()
        self.assertFalse(self.app.teleop.calls)
        self.app.control.value.update(running=False, phase="idle")
        self.app.power_down_locked = True
        self.app.confirm_manual_left_follow()
        self.assertFalse(self.app.teleop.calls)

    def test_manual_rejection_and_timeout_stay_visible_without_automatic_retry(self):
        for error in ("请对齐夹爪，差值过大", "请求超时，结果未确认"):
            self.manual_alignment_ready(error=0.)
            self.app.open_manual_left_guide()
            self.app.confirm_manual_left_follow()
            count = len(self.app.teleop.calls)
            self.app.teleop.results.put(("left_follow", {"error": error}))
            self.app._tick_once()
            self.app.operator_notice_until = 0
            self.app.teleop.results.put(("right_save", {"collection": {"note": "右臂保存完成"}}))
            self.app._tick_once()
            self.assertIn(error, self.app.manual_left_feedback.get())
            self.assertIn(error, self.app.motion_vars["left"].get())
            self.assertEqual(len(self.app.teleop.calls), count)

    def test_manual_success_requires_real_follow_and_completed_transition(self):
        self.manual_alignment_ready(error=0.)
        self.app.open_manual_left_guide()
        self.app.confirm_manual_left_follow()
        self.app.teleop.results.put(("left_follow", {"collection": {"note": "已对齐"}}))
        self.app._tick_once()
        self.assertNotIn("左臂已恢复跟随", self.app.manual_left_result)
        collection = self.app.teleop.value["collection"]
        collection.update(left_mode="FOLLOW", transitioning_arms=["left"], left_align_phase="idle")
        self.app._tick_once()
        self.assertNotIn("左臂已恢复跟随", self.app.manual_left_result)
        collection["transitioning_arms"] = []
        self.app._tick_once()
        self.assertIn("左臂已恢复跟随", self.app.manual_left_result)
        self.assertIn("可关闭指引", self.app.manual_left_summary.get())
        self.assertNotIn("先点击", self.app.manual_left_summary.get())
        self.assertEqual(self.app.manual_left_confirm_button.cget("state"), "disabled")

    def test_new_hold_and_reopened_guide_do_not_show_previous_follow_success(self):
        self.manual_alignment_ready(error=0.)
        self.app.open_manual_left_guide()
        self.app.confirm_manual_left_follow()
        self.app.teleop.value["collection"].update(left_mode="FOLLOW", transitioning_arms=[])
        self.app._tick_once()
        self.assertIn("左臂已恢复跟随", self.app.manual_left_result)
        self.app.close_manual_left_guide()
        self.app.on_motion("left_lock")
        self.app.teleop.value["collection"]["left_mode"] = "HOLD"
        self.app.open_manual_left_guide()
        self.assertFalse(self.app.manual_left_follow_attempted)
        self.assertNotIn("左臂已恢复跟随", self.app.manual_left_feedback.get())
        self.assertIn("尚未发送", self.app.manual_left_feedback.get())

    def test_previous_success_becomes_uncertain_after_disconnect_or_stale_feedback(self):
        for change in ({"state": "DISCONNECTED", "teleop_ready": False}, {"feedback_age_ms": 1000.}):
            with self.subTest(change=change):
                self.manual_alignment_ready(error=0.)
                self.app.open_manual_left_guide()
                self.app.confirm_manual_left_follow()
                self.app.teleop.value["collection"]["left_mode"] = "FOLLOW"
                self.app._tick_once()
                self.assertIn("左臂已恢复跟随", self.app.manual_left_feedback.get())
                self.app.teleop.value.update(change)
                self.app._tick_once()
                self.assertIn("当前状态不可确认", self.app.manual_left_feedback.get())
                self.assertNotIn("左臂已恢复跟随", self.app.motion_vars["left"].get())

    def test_read_only_guide_can_explain_follow_or_recording_without_adjustment_advice(self):
        for mode, recording in (("FOLLOW", False), ("HOLD", True)):
            with self.subTest(mode=mode, recording=recording):
                self.manual_alignment_ready()
                self.app.teleop.value["collection"]["left_mode"] = mode
                self.app.control.value["running"] = recording
                self.app.open_manual_left_guide()
                self.assertIn("暂停调整", self.app.manual_left_summary.get())
                self.assertNotIn("优先调整", self.app.manual_left_summary.get())
                self.assertEqual(self.app.manual_left_table.item("J7", "values")[-1], "暂停调整，等待状态确认")
                self.assertFalse(self.app.teleop.calls)

    def test_automatic_start_entry_is_absent_and_hidden_callback_cannot_send(self):
        self.manual_alignment_ready(error=0.)
        self.assertNotIn("left_align_follow", self.app.motion_buttons)
        self.app.on_motion("left_align_follow")
        self.assertFalse(self.app.teleop.calls)
        self.assertIn("入口已关闭", self.app.message_var.get())
        self.app.on_motion("left_follow")
        self.assertIsNone(self.app.manual_left_dialog)
        self.assertIn("手动对齐入口已移除", self.app.message_var.get())
        self.assertFalse(self.app.teleop.calls)

    def test_legacy_stop_only_shown_for_existing_active_or_uncertain_auto_motion(self):
        self.manual_alignment_ready()
        button = self.app.motion_buttons["left_align_pause"]
        self.assertFalse(button.winfo_manager())
        self.app.teleop.value["collection"].update(left_align_active=True, left_align_phase="aligning")
        self.app._tick_once()
        self.assertEqual(button.winfo_manager(), "grid")
        self.assertEqual(button.cget("state"), "normal")
        self.app.teleop.pending = True
        self.app.control.pending = True
        self.app.teleop.value.update(state="DISCONNECTED", teleop_ready=False)
        button.invoke()
        self.assertEqual(self.app.teleop.calls, ["left_align_pause"])

    def test_normal_close_still_preserves_legacy_stop_and_uncertain_request(self):
        for callback_name in ("on_window_close", "on_close_session"):
            for uncertain in (False, True):
                with self.subTest(callback=callback_name, uncertain=uncertain):
                    self.manual_alignment_ready()
                    self.app.left_align_may_be_active = uncertain
                    self.app.teleop.calls.clear()
                    self.app.teleop.value["collection"]["left_align_active"] = not uncertain
                    previous = list(self.app.control.calls)
                    getattr(self.app, callback_name)()
                    self.assertEqual(self.app.teleop.calls, ["left_align_pause"])
                    self.assertEqual(self.app.control.calls, previous)
                    self.assertTrue(self.root.winfo_exists())
                    self.assertFalse(self.app.close_requested)

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
        before = len(control.calls)
        self.app._tick_once()
        self.assertFalse(any(command == "episode_stop" for command, _ in control.calls[before:]))
        self.assertIn("录制继续", self.app.message_var.get())
        self.assertEqual(self.app.success_button.cget("state"), "normal")
        self.assertEqual(self.app.failure_button.cget("state"), "normal")

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

    def prepare_recording(self, side):
        self.app.control.value.update(running=False, phase="idle", active_episode=None)
        self.app.control.value["next_episode_by_task"][side] = 4
        self.app.teleop.value = {"state": "RUNNING", "fault_bits": 0, "last_running_age_s": 0.,
            "collection": {"left_mode": "HOLD" if side == "right" else "FOLLOW",
                "right_mode": "FOLLOW", "right_ready_error_rad": .02, "transitioning_arms": []}}
        self.app.left_ready.set(True); self.app.right_ready.set(True); self.app.target_confirmed.set(True)
        self.app.on_start(side)
        self.app._tick_once()
        self.assertTrue(self.app.control.value["running"])
        self.assertEqual(self.app.control.value["active_episode"]["episode_number"], 4)

    def test_diagnostic_outage_never_stops_recording_and_manual_result_sends_once(self):
        for side in ("left", "right"):
            for result in ("success", "failure"):
                for diagnostic in ("DISCONNECTED", "HEALTH_STALE", "FAULT", "REINITIALIZE_REQUIRED"):
                    with self.subTest(side=side, result=result, diagnostic=diagnostic):
                        self.prepare_recording(side)
                        before = len(self.app.control.calls)
                        for age in (4., 60., 3600.):
                            self.app.teleop.value = {"state": diagnostic, "fault_bits": 1 if diagnostic == "FAULT" else 0,
                                "last_running_age_s": age, "readiness_reason": "离线模拟诊断异常"}
                            self.app._tick_once()
                            self.assertTrue(self.app.control.value["running"])
                            self.assertIn("录制继续", self.app.message_var.get())
                            self.assertEqual(self.app.success_button.cget("state"), "normal")
                            self.assertEqual(self.app.failure_button.cget("state"), "normal")
                            for button in (self.app.start_left_button, self.app.start_right_button):
                                self.assertEqual(button.cget("state"), "disabled")
                        self.assertFalse(any(command == "episode_stop" for command, _ in self.app.control.calls[before:]))
                        button = self.app.success_button if result == "success" else self.app.failure_button
                        button.invoke(); button.invoke()
                        stops = [(command, payload) for command, payload in self.app.control.calls[before:]
                                 if command == "episode_stop"]
                        self.assertEqual(stops, [("episode_stop", {"result": result,
                            "failure_code": "operator_marked_failure" if result == "failure" else ""})])

    def test_active_episode_number_stays_visible_until_operator_saves(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                self.prepare_recording(side)
                arm = "左" if side == "left" else "右"
                button = getattr(self.app, "start_" + side + "_button")
                # Even an older manager's advanced counter cannot replace the
                # current episode shown while that episode remains open.
                self.app.control.value["next_episode_by_task"][side] = 5
                self.app._tick_once()
                self.assertEqual(button.cget("text"), f"正在录制{arm}臂｜第0004条")
                self.assertIn("当前第 0004 条", self.app.count_vars[side].get())
                self.assertIn("episode_0004", self.app.storage_vars[side].get())
                self.assertNotIn("下一条", self.app.count_vars[side].get())
                self.app.on_result("success"); self.app._tick_once()
                self.assertIn("开始录制", button.cget("text"))
                self.assertIn("0005", button.cget("text"))
                self.assertIn("下一条", self.app.count_vars[side].get())

    def test_interrupted_episode_waits_for_manual_result_without_false_recording(self):
        for side in ("left", "right"):
            for result in ("success", "failure"):
                with self.subTest(side=side, result=result):
                    self.prepare_recording(side)
                    self.app.control.value.update(phase="awaiting_result", recording_active=False,
                        capture_interrupted="camera_stream_stale")
                    self.app.teleop.value.update(state="DISCONNECTED", last_running_age_s=600.)
                    before = len(self.app.control.calls)
                    self.app._tick_once()
                    self.assertTrue(self.app.control.value["running"])
                    self.assertIn("已中断，等待保存", self.app.status_var.get())
                    self.assertNotIn("正在录制", self.app.status_var.get())
                    self.assertIn("camera_stream_stale", self.app.message_var.get())
                    self.assertIn(f"{side}_episode_0004", self.app.message_var.get())
                    self.assertIn("中断数据不会标为有效", self.app.message_var.get())
                    button = getattr(self.app, "start_" + side + "_button")
                    self.assertIn("本条已中断，等待保存", button.cget("text"))
                    self.assertIn("0004", button.cget("text"))
                    for start in (self.app.start_left_button, self.app.start_right_button):
                        self.assertEqual(start.cget("state"), "disabled")
                    self.assertEqual(self.app.success_button.cget("state"), "normal")
                    self.assertEqual(self.app.failure_button.cget("state"), "normal")
                    self.assertFalse(any(command == "episode_stop" for command, _ in self.app.control.calls[before:]))
                    self.app.on_result(result); self.app.on_result(result)
                    stops = [payload for command, payload in self.app.control.calls[before:] if command == "episode_stop"]
                    self.assertEqual(len(stops), 1)
                    self.assertEqual(stops[0]["result"], result)


class StatusPollingTest(unittest.TestCase):
    def right_master_monitor(self):
        monitor = self.master_monitor()
        monitor._value = right_master_ready_status()
        return monitor

    def test_right_master_start_timeout_keeps_uncertainty_and_never_replays(self):
        monitor = self.right_master_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_master_align'))
        calls = self.master_poll(monitor, timeout=True)
        self.assertIn('"command":"right_master_align"', calls[0])
        self.assertEqual(len(calls), 1)
        self.assertTrue(monitor.requests.empty())
        self.assertTrue(monitor.right_master_align_may_be_active)
        self.assertEqual(monitor._value['state'], 'DISCONNECTED')
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertFalse(monitor.request('right_return'))
            self.assertFalse(monitor.request('left_follow'))
            self.assertTrue(monitor.request('right_master_pause'))

    def test_right_master_unsent_start_replaced_by_pause_and_idle_free_ack_confirms(self):
        monitor = self.right_master_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_master_align'))
            self.assertTrue(monitor.request('right_master_pause'))
            self.assertFalse(monitor.request('right_master_pause'))
        calls = self.master_poll(monitor, right_master_ready_status())
        self.assertIn('"command":"right_master_pause"', calls[0])
        self.assertFalse(monitor.right_master_align_may_be_active)
        self.assertFalse(monitor.right_master_align_stop_requested)

    def test_right_master_stop_timeout_keeps_intent_until_fresh_hold_and_free_ack(self):
        monitor = self.right_master_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_master_pause'))
        self.master_poll(monitor, timeout=True)
        changes = [lambda s: s['collection'].update(right_mode='FOLLOW', right_master_align_phase='completed'),
                   lambda s: s['collection']['right_master_align_detail'].update(servo_released=False),
                   lambda s: s['collection']['right_master_align_detail'].update(leader_right_mode=2),
                   lambda s: s.update(action_age_ms=150),
                   lambda s: s['collection'].update(right_master_align_supported=False)]
        for change in changes:
            reply = right_master_ready_status()
            reply['collection']['right_master_align_phase'] = 'paused'
            change(reply)
            self.master_poll(monitor, reply)
            self.assertTrue(monitor.right_master_align_may_be_active)
            self.assertTrue(monitor.right_master_align_stop_requested)
        reply = right_master_ready_status()
        reply['collection']['right_master_align_phase'] = 'paused'
        reply['collection']['right_master_align_detail']['leader_left_mode'] = 2  # Stop right does not release left.
        self.master_poll(monitor, reply)
        self.assertFalse(monitor.right_master_align_may_be_active)
        self.assertFalse(monitor.right_master_align_stop_requested)

    def test_right_master_old_poll_cannot_clear_new_start_or_queued_stop(self):
        monitor = self.right_master_monitor()
        old = right_master_ready_status()
        old['collection']['right_master_align_phase'] = 'paused'
        def enqueue():
            self.assertTrue(monitor.request('right_master_align'))
        self.master_poll(monitor, old, during_poll=enqueue)
        self.assertTrue(monitor.right_master_align_may_be_active)
        self.assertEqual(monitor.requests.get_nowait(), 'right_master_align')
        # Emulate that start is in flight and a stop is queued; its terminal
        # response must not consume the subsequently requested stop intent.
        monitor.requests.put_nowait('right_master_align')
        def enqueue_stop():
            self.assertTrue(monitor.request('right_master_pause'))
        self.master_poll(monitor, old, during_poll=enqueue_stop)
        self.assertTrue(monitor.right_master_align_may_be_active)
        self.assertTrue(monitor.right_master_align_stop_requested)
        self.assertEqual(monitor.requests.get_nowait(), 'right_master_pause')

    def test_right_master_rejected_start_is_distinct_from_transport_uncertainty(self):
        monitor = self.right_master_monitor()
        def rejected(argv, **kwargs):
            monitor.stop.set()
            raise ui.subprocess.CalledProcessError(2, argv, output='{"error":"recording active"}')
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)), patch.object(ui.subprocess, 'run', rejected):
            self.assertTrue(monitor.request('right_master_align'))
            monitor._run()
        self.assertFalse(monitor.right_master_align_may_be_active)
        self.assertEqual(monitor._value['error'], 'recording active')
        self.assertTrue(monitor._value['connected'])


    def right_release_monitor(self):
        monitor = REAL_TELEOP_MONITOR('offline-peer')
        monitor._value = right_release_status()
        return monitor

    def test_right_release_transport_requires_fresh_complete_capability_and_idle_motion(self):
        changes = [lambda s: s['collection'].pop('right_pause_release_supported'),
                   lambda s: s['collection'].update(right_servo_release_required=False),
                   lambda s: s['collection'].update(right_servo_release_pending=True),
                   lambda s: s['collection'].update(right_mode='RETURNING'),
                   lambda s: s['collection'].update(recording={'token': 'episode'}),
                   lambda s: s['collection'].update(transitioning_arms=['right']),
                   lambda s: s['collection'].update(left_align_active=True),
                   lambda s: s['collection'].update(left_master_align_active=True),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_left_mode=1),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_right_mode=None),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_right_mode=False),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_right_mode=3),
                   lambda s: s.update(connected=False), lambda s: s.update(teleop_ready=False),
                   lambda s: s.update(state='FAULT', fault_bits=1),
                   lambda s: s.update(action_age_ms=100), lambda s: s.update(feedback_age_ms=float('nan'))]
        for index, change in enumerate(changes):
            with self.subTest(change=index):
                monitor = self.right_release_monitor()
                change(monitor._value)
                with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
                    self.assertFalse(monitor.request('right_servo_release'))
                self.assertTrue(monitor.requests.empty())
                self.assertTrue(monitor.request_error)

    def test_right_release_transport_encodes_explicit_payload_once(self):
        monitor = self.right_release_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_servo_release'))
            self.assertFalse(monitor.request('right_servo_release'))
        calls = self.master_poll(monitor, reply=right_release_status())
        self.assertEqual(len(calls), 1)
        self.assertIn('"command":"right_pause","release_servo":true', calls[0])
        self.assertNotIn('left_master_align', calls[0])
        self.assertNotIn('right_follow', calls[0])
        self.assertTrue(monitor.requests.empty())

    def test_ordinary_stop_never_acquires_release_flag_even_if_delayed_until_hold(self):
        monitor = self.right_release_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_pause'))
        calls = self.master_poll(monitor, reply=right_release_status())
        self.assertIn('"command":"right_pause"', calls[0])
        self.assertNotIn('release_servo', calls[0])

    def test_right_release_rechecks_before_wire_and_never_replays_local_rejection(self):
        monitor = self.right_release_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_servo_release'))
        monitor._value['action_age_ms'] = 100
        calls = self.master_poll(monitor, reply=right_release_status())
        self.assertEqual(len(calls), 1)
        self.assertIn('"command":"status"', calls[0])
        self.assertNotIn('release_servo', calls[0])
        command, reply = monitor.results.get_nowait()
        self.assertEqual(command, 'right_servo_release')
        self.assertIn('已过期', reply['error'])
        self.assertTrue(monitor.requests.empty())

    def test_right_release_timeout_never_replays_or_claims_servo_is_free(self):
        monitor = self.right_release_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('right_servo_release'))
        calls = self.master_poll(monitor, timeout=True)
        self.assertEqual(len(calls), 1)
        self.assertEqual(monitor._value['state'], 'DISCONNECTED')
        self.assertEqual(monitor._value['collection']['left_master_align_detail']['leader_right_mode'], 2)
        self.assertTrue(monitor.requests.empty())
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertFalse(monitor.request('right_servo_release'))

    def master_monitor(self):
        monitor = REAL_TELEOP_MONITOR('offline-peer')
        monitor._value = master_ready_status()
        return monitor

    def master_poll(self, monitor, reply=None, timeout=False, during_poll=None):
        monitor.stop.clear()
        calls = []
        def run(argv, **kwargs):
            calls.append(kwargs.get('input'))
            if during_poll: during_poll()
            monitor.stop.set()
            if timeout:
                raise ui.subprocess.TimeoutExpired(argv, kwargs['timeout'])
            return ui.subprocess.CompletedProcess(argv, 0, stdout=json.dumps(reply), stderr='')
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)), patch.object(ui.subprocess, 'run', run):
            monitor._run()
        return calls

    def test_master_monitor_start_requires_physical_capability_and_all_gates(self):
        changes = [lambda s: s['collection'].update(left_master_align_supported=False),
                   lambda s: s['collection'].update(left_mode='FOLLOW'),
                   lambda s: s['collection'].update(recording='episode'),
                   lambda s: s['collection'].update(right_mode='RETURNING'),
                   lambda s: s['collection'].update(transitioning_arms=['right']),
                   lambda s: s.update(teleop_ready=False), lambda s: s.update(feedback_age_ms=101),
                   lambda s: s.update(leader_axes=[0.] * 15),
                   lambda s: s['collection'].update(left_alignment_error_rad=.3),
                   lambda s: s['collection'].update(left_master_align_active=True),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_right_mode=1),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_left_mode=False),
                   lambda s: s['collection']['left_master_align_detail'].update(servo_released=False)]
        for index, change in enumerate(changes):
            with self.subTest(case=index):
                monitor = self.master_monitor()
                change(monitor._value)
                with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
                    self.assertFalse(monitor.request('left_master_align'))
                self.assertTrue(monitor.requests.empty())
                self.assertTrue(monitor.request_error)

    def test_master_start_timeout_keeps_uncertainty_and_never_replays(self):
        monitor = self.master_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('left_master_align'))
        calls = self.master_poll(monitor, timeout=True)
        self.assertIn('"command":"left_master_align"', calls[0])
        self.assertEqual(len(calls), 1)
        self.assertTrue(monitor.requests.empty())
        self.assertTrue(monitor.left_master_align_may_be_active)
        self.assertEqual(monitor._value['state'], 'DISCONNECTED')
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertFalse(monitor.request('right_return'))
            self.assertFalse(monitor.request('left_follow'))
            self.assertTrue(monitor.request('left_master_pause'))

    def test_master_unsent_start_replaced_by_pause_and_idle_free_ack_confirms(self):
        monitor = self.master_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('left_master_align'))
            self.assertTrue(monitor.request('left_master_pause'))
            self.assertFalse(monitor.request('left_master_pause'))
        calls = self.master_poll(monitor, master_ready_status())
        self.assertIn('"command":"left_master_pause"', calls[0])
        self.assertFalse(monitor.left_master_align_may_be_active)
        self.assertFalse(monitor.left_master_align_stop_requested)

    def test_master_stop_timeout_keeps_intent_until_fresh_hold_and_free_ack(self):
        monitor = self.master_monitor()
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('left_master_pause'))
        self.master_poll(monitor, timeout=True)
        changes = [lambda s: s['collection'].update(left_mode='FOLLOW', left_master_align_phase='completed'),
                   lambda s: s['collection']['left_master_align_detail'].update(servo_released=False),
                   lambda s: s['collection']['left_master_align_detail'].update(leader_left_mode=2),
                   lambda s: s.update(action_age_ms=150),
                   lambda s: s['collection'].update(left_master_align_supported=False)]
        for change in changes:
            reply = master_ready_status()
            reply['collection']['left_master_align_phase'] = 'paused'
            change(reply)
            self.master_poll(monitor, reply)
            self.assertTrue(monitor.left_master_align_may_be_active)
            self.assertTrue(monitor.left_master_align_stop_requested)
        reply = master_ready_status()
        reply['collection']['left_master_align_phase'] = 'paused'
        reply['collection']['left_master_align_detail']['leader_right_mode'] = 2  # Never release right to stop left.
        self.master_poll(monitor, reply)
        self.assertFalse(monitor.left_master_align_may_be_active)
        self.assertFalse(monitor.left_master_align_stop_requested)

    def test_master_old_poll_cannot_clear_new_start_or_queued_stop(self):
        monitor = self.master_monitor()
        old = master_ready_status()
        old['collection']['left_master_align_phase'] = 'paused'
        def enqueue():
            self.assertTrue(monitor.request('left_master_align'))
        self.master_poll(monitor, old, during_poll=enqueue)
        self.assertTrue(monitor.left_master_align_may_be_active)
        self.assertEqual(monitor.requests.get_nowait(), 'left_master_align')
        # Emulate that start is in flight and a stop is queued; its terminal
        # response must not consume the subsequently requested stop intent.
        monitor.requests.put_nowait('left_master_align')
        def enqueue_stop():
            self.assertTrue(monitor.request('left_master_pause'))
        self.master_poll(monitor, old, during_poll=enqueue_stop)
        self.assertTrue(monitor.left_master_align_may_be_active)
        self.assertTrue(monitor.left_master_align_stop_requested)
        self.assertEqual(monitor.requests.get_nowait(), 'left_master_pause')

    def test_master_rejected_start_is_distinct_from_transport_uncertainty(self):
        monitor = self.master_monitor()
        def rejected(argv, **kwargs):
            monitor.stop.set()
            raise ui.subprocess.CalledProcessError(2, argv, output='{"error":"recording active"}')
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)), patch.object(ui.subprocess, 'run', rejected):
            self.assertTrue(monitor.request('left_master_align'))
            monitor._run()
        self.assertFalse(monitor.left_master_align_may_be_active)
        self.assertEqual(monitor._value['error'], 'recording active')
        self.assertTrue(monitor._value['connected'])

    def test_master_physical_activity_after_reopen_blocks_other_motion(self):
        monitor = self.master_monitor()
        monitor._value['collection'].update(left_master_align_phase='failed', left_master_align_active=True)
        monitor._value['collection']['left_master_align_detail'].update(leader_left_mode=2, servo_released=False)
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertFalse(monitor.request('left_follow'))
            self.assertFalse(monitor.request('right_return'))
            self.assertTrue(monitor.request('left_lock'))
        self.assertEqual(monitor.requests.get_nowait(), 'left_master_pause')

    def test_master_unknown_capability_during_outage_does_not_remove_stop(self):
        monitor = self.master_monitor()
        monitor.left_master_align_may_be_active = True
        monitor._value.update(connected=False, teleop_ready=False)
        monitor._value['collection']['left_master_align_supported'] = False
        with patch.object(ui, 'apply_current_report', lambda v: dict(v)):
            self.assertTrue(monitor.request('left_master_pause'))

    def alignment_monitor(self):
        monitor = REAL_TELEOP_MONITOR("offline-peer")
        monitor._value = {"state": "RUNNING", "fault_bits": 0, "teleop_ready": True,
            "collection": {"left_mode": "HOLD", "right_mode": "FOLLOW", "left_align_supported": True,
                "left_align_active": False, "left_align_phase": "idle", "transitioning_arms": []}}
        return monitor

    def test_automatic_start_is_rejected_even_when_controller_supports_it(self):
        monitor = self.alignment_monitor()
        with patch.object(ui, "apply_current_report", lambda value: dict(value)):
            self.assertFalse(monitor.request("left_align_follow"))
        self.assertTrue(monitor.requests.empty())
        self.assertIn("入口已关闭", monitor.request_error)

    def test_manual_follow_uses_fast_socket_and_timeout_never_retries(self):
        for timeout in (False, True):
            with self.subTest(timeout=timeout):
                monitor = self.alignment_monitor()
                calls = []
                def run(argv, **kwargs):
                    calls.append((argv, kwargs))
                    monitor.stop.set()
                    if timeout: raise ui.subprocess.TimeoutExpired(argv, kwargs["timeout"])
                    return ui.subprocess.CompletedProcess(argv, 0, stdout=json.dumps(monitor._value), stderr="")
                with patch.object(ui, "apply_current_report", lambda value: dict(value)), patch.object(ui.subprocess, "run", run):
                    self.assertTrue(monitor.request("left_follow"))
                    monitor._run()
                self.assertEqual(len(calls), 1)
                self.assertEqual(calls[0][0][-1], "/usr/bin/python3 -")
                self.assertIn('"command":"left_follow"', calls[0][1]["input"])
                self.assertEqual(calls[0][1]["timeout"], 3)
                self.assertTrue(monitor.requests.empty())

    def test_manual_follow_health_rejection_has_actual_reason(self):
        monitor = self.alignment_monitor()
        monitor._value.update(teleop_ready=False, readiness_reason="反馈过期")
        with patch.object(ui, "apply_current_report", lambda value: dict(value)):
            self.assertFalse(monitor.request("left_follow"))
        self.assertIn("反馈过期", monitor.request_error)
        self.assertNotIn("前一个", monitor.request_error)
        self.assertTrue(monitor.requests.empty())

    def test_legacy_unsent_auto_start_can_only_be_replaced_with_stop(self):
        monitor = self.alignment_monitor()
        # Compatibility with an already queued request from the previous UI;
        # new UI requests are never allowed to enqueue this command.
        monitor.requests.put_nowait("left_align_follow")
        monitor.pending = monitor.left_align_may_be_active = True
        with patch.object(ui, "apply_current_report", lambda value: dict(value)):
            self.assertTrue(monitor.request("left_align_pause"))
            self.assertFalse(monitor.request("left_align_pause"))
        self.assertEqual(monitor.requests.get_nowait(), "left_align_pause")
        self.assertTrue(monitor.requests.empty())
        self.assertTrue(monitor.left_align_stop_requested)

    def test_legacy_stop_timeout_preserves_intent_until_confirmed_hold(self):
        monitor = self.alignment_monitor()
        calls = []
        def timeout(argv, **kwargs):
            calls.append(kwargs["input"])
            monitor.stop.set()
            raise ui.subprocess.TimeoutExpired(argv, kwargs["timeout"])
        with patch.object(ui, "apply_current_report", lambda value: dict(value)), patch.object(ui.subprocess, "run", timeout):
            self.assertTrue(monitor.request("left_align_pause"))
            monitor._run()
        for mode, phase, expected in (("FOLLOW", "completed", True), ("HOLD", "paused", False)):
            monitor.stop.clear()
            reply = {"state": "RUNNING", "teleop_ready": True, "collection": {
                "left_mode": mode, "left_align_phase": phase, "left_align_active": False,
                "left_align_supported": True, "transitioning_arms": []}}
            def status(argv, **kwargs):
                calls.append(kwargs["input"])
                monitor.stop.set()
                return ui.subprocess.CompletedProcess(argv, 0, stdout=json.dumps(reply), stderr="")
            with patch.object(ui, "apply_current_report", lambda value: dict(value)), patch.object(ui.subprocess, "run", status):
                monitor._run()
            self.assertEqual(monitor.left_align_may_be_active, expected)
            self.assertEqual(monitor.left_align_stop_requested, expected)
        self.assertEqual(len(calls), 3)
        self.assertIn('"command":"left_align_pause"', calls[0])
        self.assertTrue(all('"command":"status"' in item for item in calls[1:]))

    def test_reopening_interrupted_episode_enters_current_batch_without_new_start(self):
        catalog = {"ok": True, "running": True, "phase": "awaiting_result", "session_id": "existing-batch",
                   "active_episode": {"episode_id": "right_episode_0004"}}
        with patch.object(ui, "recorder_request", return_value=catalog) as request:
            self.assertTrue(ui.choose_collection_session(object(), "tcp://offline"))
        request.assert_called_once_with("tcp://offline", {"command": "session_catalog"})

    def test_interrupted_service_response_is_never_described_as_recording(self):
        class InlineThread:
            def __init__(self, target, **_kwargs): self.target = target
            def start(self): self.target()
        context = MagicMock()
        context.socket.return_value.recv_json.return_value = {
            "ok": True, "running": True, "phase": "awaiting_result",
            "capture_interrupted": "recorder_exit_1", "active_episode": {"episode_id": "right_episode_0004"}}
        control = REAL_SESSION_CONTROL("offline", 5557)
        with patch.object(ui.zmq, "Context", return_value=context), patch.object(ui.threading, "Thread", InlineThread):
            self.assertTrue(control.request("status"))
        state, message, pending = control.snapshot()
        self.assertFalse(pending)
        self.assertEqual(state["phase"], "awaiting_result")
        self.assertIn("recorder_exit_1", message)
        self.assertIn("等待保存", message)
        self.assertNotIn("正在录制", message)

    def test_status_poll_uses_bounded_read_only_socket_request_without_ros_startup(self):
        monitor = REAL_TELEOP_MONITOR("offline-peer")
        calls = []
        def run(argv, **kwargs):
            calls.append((argv, kwargs))
            monitor.stop.set()
            return ui.subprocess.CompletedProcess(argv, 0, stdout='{"state":"RUNNING","fault_bits":0}', stderr="")
        with patch.object(ui.subprocess, "run", run), patch.object(ui, "apply_current_report",
                lambda value: {**value, "teleop_ready": True}):
            monitor._run()
        self.assertEqual(len(calls), 1)
        argv, kwargs = calls[0]
        self.assertEqual(argv[-2:], ["offline-peer", "/usr/bin/python3 -"])
        self.assertEqual(kwargs["timeout"], 3)
        self.assertIn('"command":"status"', kwargs["input"])
        self.assertNotIn("remote-teleop-control", kwargs["input"])
        self.assertNotIn("episode_stop", kwargs["input"])
        self.assertTrue(monitor._value["connected"])


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


class PowerDownTransportTest(unittest.TestCase):
    def run_client(self, source):
        popen = ui.subprocess.Popen
        client = ui.PowerDownClient(Args())
        def fake_coordinator(argv, **kwargs):
            self.assertIn("--confirm", argv)
            return popen([sys.executable, "-u", "-c", source], **kwargs)
        with patch.object(ui.subprocess, "Popen", fake_coordinator):
            self.assertTrue(client.start())
            self.assertFalse(client.start())
            deadline = time.monotonic() + 4
            while client.pending and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(client.pending)
        events = []
        while not client.events.empty():
            events.append(client.events.get_nowait())
        return events

    def test_progress_and_success_travel_through_real_process_pipe(self):
        events = self.run_client('import json,time\ntime.sleep(.05)\n'
            'print(json.dumps({"event":"progress","message":"保存数据"}))\n'
            'print(json.dumps({"event":"complete","ok":True,"message":"已确认失能"}))')
        self.assertEqual([event["event"] for event in events], ["progress", "complete"])
        self.assertTrue(events[-1]["ok"])

    def test_unexpected_process_exit_cannot_report_success(self):
        events = self.run_client('import time\ntime.sleep(.05)\nprint("broken coordinator")')
        self.assertFalse(events[-1]["ok"])
        self.assertIn("未确认", events[-1]["message"])


if __name__ == "__main__":
    unittest.main()

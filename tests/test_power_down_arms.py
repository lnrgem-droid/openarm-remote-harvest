"""No ROS, SSH, systemd or CAN calls: all shutdown effects are injected."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest


spec = importlib.util.spec_from_file_location("power_down", Path(__file__).parents[1] / "scripts/power_down_arms.py")
shutdown = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shutdown)


def idle(**values):
    return {"ok": True, "running": False, "phase": "idle", "session_root": "/batch", **values}


def saved(**values):
    return {"episode_root": "/batch/one", "lerobot_root": "/batch/one/data", "ended_unix_s": 20,
            "recorder_returncode": 0, "result": "aborted", "valid": False,
            "rgbd_writer_receipt": {"complete": True, "dataset_root": "/batch/one/data"}, **values}


class FakeRecorder:
    def __init__(self, states):
        self.states = list(states)
        self.calls = []

    def __call__(self, command, **fields):
        self.calls.append((command, fields))
        if command == "episode_stop":
            return {"ok": True}
        if len(self.states) > 1:
            return self.states.pop(0)
        return self.states[0]


class SaveTest(unittest.TestCase):
    def test_recording_is_aborted_and_waited_until_complete(self):
        recorder = FakeRecorder([idle(running=True, phase="recording", active_episode=saved()),
                                 idle(last_episode=saved())])
        shutdown.seal_recording(recorder)
        self.assertEqual(recorder.calls[1], ("episode_stop", {
            "result": "aborted", "failure_code": "operator_power_down"}))
        self.assertEqual(recorder.calls[-1][0], "status")

    def test_existing_finalization_preserves_success_result(self):
        recorder = FakeRecorder([idle(running=True, phase="stopping", active_episode=saved()),
                                 idle(last_episode=saved(result="success"))])
        shutdown.seal_recording(recorder)
        self.assertTrue(all(call[0] == "status" for call in recorder.calls))

    def test_incomplete_save_or_session_swap_is_rejected(self):
        for final in [idle(last_episode=saved(recorder_returncode=1)),
                      idle(last_episode=saved(rgbd_writer_receipt={"complete": False})),
                      idle(last_episode=saved(episode_root="/different")),
                      idle(last_episode=saved(), session_root="/different"),
                      idle(last_episode=None), idle(phase="error", last_episode=saved())]:
            with self.subTest(final=final):
                recorder = FakeRecorder([idle(running=True, phase="stopping", active_episode=saved()), final])
                with self.assertRaises(shutdown.ShutdownError):
                    shutdown.seal_recording(recorder)

    def test_retry_cannot_bypass_failed_previous_save(self):
        with self.assertRaises(shutdown.ShutdownError):
            shutdown.seal_recording(FakeRecorder([idle(last_episode=saved(recorder_returncode=1))]))

    def test_timeout_is_bounded(self):
        times = iter([0, 1, 50])
        recorder = FakeRecorder([idle(running=True, phase="stopping", active_episode=saved())])
        with self.assertRaisesRegex(shutdown.ShutdownError, "超时"):
            shutdown.seal_recording(recorder, clock=lambda: next(times), sleep=lambda _: None)

    def test_unknown_or_unacknowledged_status_rejected(self):
        for state in [{}, {"ok": False}, idle(running=None), idle(phase="starting")]:
            with self.subTest(state=state), self.assertRaises(shutdown.ShutdownError):
                shutdown.seal_recording(FakeRecorder([state]))


class EvidenceTest(unittest.TestCase):
    def test_failed_systemd_unit_is_stopped_only_when_no_processes_remain(self):
        self.assertTrue(shutdown.service_stopped("ActiveState=failed\nMainPID=0\nControlGroup=\n"))
        self.assertTrue(shutdown.service_stopped("ActiveState=inactive\nMainPID=0\nControlGroup=\n"))
        self.assertFalse(shutdown.service_stopped("ActiveState=failed\nMainPID=123\nControlGroup=\n"))
        self.assertFalse(shutdown.service_stopped("ActiveState=failed\nMainPID=0\nControlGroup=/remaining\n"))
        self.assertFalse(shutdown.service_stopped("ActiveState=deactivating\nMainPID=0\nControlGroup=\n"))
        self.assertFalse(shutdown.service_stopped(""))

    def test_hold_waits_for_authoritative_status_after_stale_ack(self):
        commands = []
        replies = iter([{"state": "RUNNING"}, {"state": "RUNNING"},
                        {"state": "RUNNING"}, {"state": "READY"}])
        def request(command):
            commands.append(command)
            return next(replies)
        self.assertEqual(shutdown.confirm_hold(request, sleep=lambda _: None)["state"], "READY")
        self.assertEqual(commands, ["status", "hold", "status", "status"])

    def test_only_all_sixteen_post_command_disabled_motors_pass(self):
        evidence = shutdown.DisableEvidence(["can0", "can1"])
        evidence.begin(100.)
        for iface in ["can0", "can1"]:
            for motor in range(1, 9):
                for _ in range(3):
                    evidence.observe(iface, (motor, 0), 99.)
        self.assertFalse(evidence.report()["verified"])
        for iface in ["can0", "can1"]:
            for motor in range(1, 9):
                for _ in range(3):
                    evidence.observe(iface, (motor, 0), 101.)
        self.assertTrue(evidence.report()["verified"])
        evidence.observe("can1", (8, 1), 102.)
        self.assertFalse(evidence.report()["verified"])

    def test_no_feedback_and_partial_feedback_never_prove_disable(self):
        evidence = shutdown.DisableEvidence(["can0", "can1"])
        evidence.begin(1.)
        self.assertFalse(evidence.report()["verified"])
        for _ in range(3):
            evidence.observe("can0", (1, 0), 2.)
        self.assertFalse(evidence.report()["verified"])


class CoordinatorTest(unittest.TestCase):
    def run_flow(self, *, recorder=None, arm_failure=None, failed_side=None, hold_error=False, stop_error=False):
        calls = []
        reports = []
        class Device:
            def __init__(self, side): self.side = side
            def arm(self):
                calls.append("arm:" + self.side)
                if self.side == arm_failure: raise RuntimeError("SSH unreachable")
            def go(self):
                self_test.assertIn("arm:host", calls)
                self_test.assertIn("arm:follower", calls)
                calls.append("disable:" + self.side)
            def result(self):
                return {"ok": self.side != failed_side, "verified": self.side != failed_side}
            def close(self): calls.append("close:" + self.side)
        def hold():
            calls.append("hold")
            if hold_error: raise RuntimeError("hold failed")
        def stop():
            calls.append("stop_service")
            if stop_error: raise RuntimeError("systemctl timed out")
        self_test = self
        result = shutdown.coordinate(recorder or FakeRecorder([idle()]), hold, Device, stop, reports.append)
        return result, calls, reports

    def test_success_requires_both_devices_then_stops_service(self):
        result, calls, reports = self.run_flow()
        self.assertTrue(result["ok"])
        self.assertLess(calls.index("disable:host"), calls.index("stop_service"))
        self.assertEqual(reports[-1]["event"], "complete")

    def test_no_disable_if_either_listener_failed_to_arm(self):
        result, calls, _ = self.run_flow(arm_failure="follower")
        self.assertFalse(result["ok"])
        self.assertFalse(any(call.startswith("disable:") for call in calls))
        self.assertNotIn("stop_service", calls)

    def test_partial_disable_keeps_service_and_reports_each_endpoint(self):
        result, calls, _ = self.run_flow(failed_side="follower")
        self.assertFalse(result["ok"])
        self.assertTrue(result["devices"]["host"]["verified"])
        self.assertFalse(result["devices"]["follower"]["verified"])
        self.assertNotIn("stop_service", calls)

    def test_save_or_hold_failure_never_sends_disable(self):
        for kwargs in [{"recorder": FakeRecorder([idle(phase="error")])}, {"hold_error": True}]:
            result, calls, _ = self.run_flow(**kwargs)
            self.assertFalse(result["ok"])
            self.assertFalse(any(call.startswith("disable:") for call in calls))

    def test_service_stop_failure_is_not_success(self):
        result, _, _ = self.run_flow(stop_error=True)
        self.assertFalse(result["ok"])
        self.assertEqual(result["stage"], "stopping_service")


class DeviceTransportTest(unittest.TestCase):
    def test_two_phase_helper_pipe_requires_go(self):
        device = shutdown.DeviceProcess.__new__(shutdown.DeviceProcess)
        device.buffer = b""
        source = ('import json,sys\nprint(json.dumps({"event":"armed"}),flush=True)\n'
                  'line=sys.stdin.readline()\n'
                  'print(json.dumps({"event":"device_result","ok":line=="disable\\n"}),flush=True)')
        device.process = subprocess.Popen([sys.executable, "-u", "-c", source],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
        try:
            self.assertEqual(device.arm()["event"], "armed")
            device.go()
            self.assertTrue(device.result()["ok"])
        finally:
            device.close()


if __name__ == "__main__":
    unittest.main()

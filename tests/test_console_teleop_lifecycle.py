"""No hardware: verify start admission, profile isolation and service selection."""
import copy
import importlib.util
import os
from pathlib import Path
import sys
import json
import shlex
import subprocess
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import start_teleop_from_console as start
import inspect_teleop_recovery as recovery
import power_down_arms as stop


def cold():
    parts = {side: {"result": "UNKNOWN", "motors": {
        f"{bus}/{motor}": {"states": []} for bus in interfaces for motor in range(1, 9)}}
        for side, interfaces in recovery.buses_by_side()}
    return {"reason": "unexplained empty stack", "evidence": {
        "ssh_ok": True, "host_can_ok": True, "local_control_processes": [],
        "host_unit": {"LoadState": "loaded", "ActiveState": "inactive", "MainPID": "0", "ControlGroup": ""},
        "remote": {"can_ok": True, "recorder_idle": True, "processes": [],
                   "runtime_socket_exists": False, "status": None},
        "motor_report": {"result": "UNKNOWN", **parts}}}


class StartAdmissionTest(unittest.TestCase):
    def test_empty_stack_can_start_only_with_complete_passive_evidence(self):
        self.assertEqual(start.start_decision(cold()), "cold")
        for key in ("local_control_processes", "host_unit", "motor_report", "ssh_ok", "host_can_ok"):
            value = cold()
            value["evidence"].pop(key)
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                start.start_decision(value)

    def test_never_reinitialize_existing_unknown_or_faulted_stack(self):
        for change in ({"processes": [123]}, {"runtime_socket_exists": True},
                       {"status": {"state": "FAULT"}}, {"status": {"state": "RUNNING"}},
                       {"recorder_idle": False}, {"can_ok": False}):
            value = cold()
            value["evidence"]["remote"].update(change)
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                start.start_decision(value)
        value = cold()
        value["evidence"]["local_control_processes"] = [123]
        with self.assertRaises(RuntimeError):
            start.start_decision(value)

    def test_positive_enabled_feedback_blocks_cold_start(self):
        value = cold()
        value["evidence"]["motor_report"]["host"]["motors"]["can0/1"]["states"] = [1]
        with self.assertRaises(RuntimeError):
            start.start_decision(value)

    def test_verified_stop_requires_explicit_recovery_classification(self):
        value = cold()
        value["evidence"]["remote"]["processes"] = [123]
        value.update(code="OPERATOR_POWER_DOWN_CONFIRM", confirmation_required=True)
        self.assertEqual(start.start_decision(value), "restart")
        value["confirmation_required"] = False
        with self.assertRaises(RuntimeError):
            start.start_decision(value)

    def test_live_healthy_session_is_reused_without_service_mutation(self):
        value = cold()
        value["evidence"]["host_unit"]["ActiveState"] = "active"
        value["evidence"]["health_report_fresh"] = True
        with patch.object(start, "apply_report", return_value={"teleop_ready": True}), \
             patch.object(start.subprocess, "run") as run, patch.object(start, "emit"):
            result = start.start(recovery.connection_config(), inspect_fn=lambda _: value)
        self.assertTrue(result["ok"])
        run.assert_not_called()

    def test_changed_identity_aborts_before_restart(self):
        first, second = cold(), cold()
        second["evidence"]["remote"]["boot_id"] = "changed"
        reads = iter([first, second])
        with patch.object(start, "emit"), patch.object(start.subprocess, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "变化"):
                start.start(recovery.connection_config(), inspect_fn=lambda _: next(reads))
        run.assert_not_called()


class WiringProfileTest(unittest.TestCase):
    def test_profiles_select_correct_service_and_follower_buses(self):
        for profile, service, buses in [
                ("jetson", "openarm-remote-teleop.service", ["can1", "can2"]),
                ("host-test", "openarm-host-test-teleop.service", ["can0", "can1"])]:
            with patch.dict(os.environ, OPENARM_TELEOP_PROFILE=profile):
                cfg = recovery.connection_config()
                self.assertEqual(cfg["service"], service)
                self.assertEqual(cfg["follower_interfaces"], buses)
                reports = {}
                for side, interfaces in recovery.buses_by_side():
                    reports[side] = {"ok": True, "verified": True, "acknowledged": True,
                        "motors": {f"{bus}/{motor}": {"disabled": True, "states": [0, 0, 0]}
                                   for bus in interfaces for motor in range(1, 9)}}
                self.assertTrue(recovery.verified_disable_reports(reports))
                reports["follower"]["motors"].pop(next(iter(reports["follower"]["motors"])))
                self.assertFalse(recovery.verified_disable_reports(reports))

    def test_stop_targets_selected_service(self):
        reply = type("Reply", (), {"stdout": "ActiveState=inactive\nMainPID=0\nControlGroup=\n"})()
        with patch.object(stop.subprocess, "run", return_value=reply) as run:
            stop.stop_service("openarm-host-test-teleop.service")
        self.assertEqual(run.call_args_list[0].args[0],
                         ["systemctl", "--user", "stop", "openarm-host-test-teleop.service"])

    def test_unknown_profile_is_rejected(self):
        with patch.dict(os.environ, OPENARM_TELEOP_PROFILE="other"):
            with self.assertRaises(ValueError):
                recovery.connection_config()

    def test_real_console_launcher_exits_before_motor_start_and_releases_lock(self):
        source = (Path(__file__).parents[1] / "scripts/daily_start_teleop_rgbd.sh").read_text()
        begin = source.index('\nif [[ "$(basename "$COLLECTION_CONSOLE")"')
        branch = source[begin:source.index('\nstatus="$(teleop_status)"', begin)]
        for mode, profile in (("true", "host-test"), ("false", "jetson")):
            with self.subTest(profile=profile), tempfile.TemporaryDirectory() as d:
                root = Path(d)
                console = root / "mushroom_collection_console.py"
                console.write_text("import sys,json,fcntl\nfrom pathlib import Path\n"
                    "with open(Path(__file__).with_name('lock'),'a') as f:\n"
                    " fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)\n"
                    "Path(__file__).with_name('args.json').write_text(json.dumps(sys.argv[1:]))\n")
                variables = {"COLLECTION_CONSOLE": str(console), "COLLECTION_PYTHON": sys.executable,
                             "TEST_HOST_MODE": mode, "COLLECTION_CONSOLE_EXTRA_ARGS": "",
                             "PEER_IP": "192.168.50.3" if mode == "true" else "192.168.50.2",
                             "JETSON_HOST": "openarm-follower-test" if mode == "true" else "openarm-jetson"}
                shell = "set -euo pipefail\n" + "\n".join(
                    f"{k}={shlex.quote(v)}" for k, v in variables.items())
                shell += f"\nexec 9>{shlex.quote(str(root / 'lock'))}\nflock -n 9\n"
                shell += branch + "\necho ERROR_UNEXPECTED_MOTOR_START\nexit 99\n"
                result = subprocess.run(["bash", "-c", shell], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertNotIn("ERROR_UNEXPECTED_MOTOR_START", result.stdout)
                args = json.loads((root / "args.json").read_text())
                self.assertEqual(args[args.index("--teleop-profile") + 1], profile)
                self.assertEqual(args[args.index("--jetson-ssh") + 1], variables["JETSON_HOST"])


class StartupFailureMessageTest(unittest.TestCase):
    def test_alignment_failure_reports_current_attempt_not_old_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.log'
            path.write_text('ERROR: old fault\n')
            cursor = start.startup_log_cursor({'StandardOutput': 'append:' + str(path)})
            with path.open('a') as stream:
                stream.write('  自动对齐验收：未通过，不允许开始遥操\n'
                             '  - 控制状态必须为 RUNNING 且无故障\n')
            message = start.startup_exit_reason(cursor)
            self.assertIn('启动验收失败', message)
            self.assertIn('RUNNING', message)
            self.assertNotIn('old fault', message)
            with path.open('a') as stream:
                stream.write('ERROR: 启动验收失败：控制故障：主臂指令接收超时\n')
            self.assertIn('主臂指令接收超时', start.startup_exit_reason(cursor))

    def test_systemd_append_mode_resolves_effective_unit_log(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.log"
            path.write_text("ERROR: old failure\n")
            reply = subprocess.CompletedProcess([], 0, stdout=(
                "# main unit\n[Service]\nStandardOutput=append:/wrong.log\n"
                "# drop-in\n[Service]\nStandardOutput=append:" + str(path) + "\n"))
            with patch.object(start.subprocess, "run", return_value=reply) as run:
                cursor = start.startup_log_cursor({"StandardOutput": "append"}, "teleop.service")
            self.assertEqual(run.call_args.args[0],
                             ["systemctl", "--user", "cat", "teleop.service"])
            with path.open("a") as stream:
                stream.write("ERROR: 从臂未实际到达初始位\n"
                    "[follower_gravity_pd.can1]: Startup homing timed out: J4 target=0.6283 actual=0.1913 error=0.4370 rad, feedback_enabled_fresh=0\n")
            message = start.startup_exit_reason(cursor)
            self.assertNotIn("36.0°", message)
            self.assertNotIn("11.0°", message)
            self.assertIn("日志角度不能作为已核验的实测位置", message)
            self.assertIn("电机使能或反馈未通过核验", message)
            self.assertNotIn("old failure", message)

    def test_unavailable_effective_unit_does_not_guess_log_path(self):
        with patch.object(start.subprocess, "run", side_effect=subprocess.TimeoutExpired("systemctl", 5)):
            self.assertIsNone(start.startup_log_cursor({"StandardOutput": "append"}, "teleop.service"))
        for output in ("journal", "append:/tmp/%n.log", "append:/tmp/escaped\\x20.log"):
            reply = subprocess.CompletedProcess([], 0, stdout="[Service]\nStandardOutput=" + output)
            with self.subTest(output=output), patch.object(start.subprocess, "run", return_value=reply):
                self.assertIsNone(start.startup_log_cursor({"StandardOutput": "append"}, "teleop.service"))

    def test_only_this_attempt_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.log"
            path.write_text("ERROR: old unrelated failure\n")
            cursor = start.startup_log_cursor({"StandardOutput": "append:" + str(path)})
            self.assertNotIn("old unrelated", start.startup_exit_reason(cursor))
            with path.open("a") as stream:
                stream.write("ERROR: 从臂未实际到达初始位\n"
                    "[follower_gravity_pd.can1]: Startup homing timed out: J1 target=0.0000 actual=0.4568 error=0.4568 rad\n"
                    "[follower_gravity_pd.can2]: Startup homing timed out: J1 target=0.0000 actual=-0.8394 error=0.8394 rad\n")
            message = start.startup_exit_reason(cursor)
            self.assertIn("从臂归位超时", message)
            self.assertIn("26.2°", message)
            self.assertIn("-48.1°", message)
            self.assertIn("主臂尚未启动", message)
            self.assertNotIn("被替换", message)

    def test_leader_missing_feedback_explains_power_and_recovery_without_cached_angles(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.log'
            path.write_text('ERROR: 从臂未实际到达初始位\n')
            cursor = start.startup_log_cursor({'StandardOutput': 'append:' + str(path)})
            with path.open('a') as stream:
                stream.write('ERROR: 主臂未实际到达初始位，停止自动启动\n'
                    '[leader_gravity_pd.can0]: Startup homing timed out: J4 target=0.6283 actual=0.0000 feedback_enabled_fresh=0\n'
                    '[leader_gravity_pd.can1]: Startup homing timed out: J4 target=0.6283 actual=0.0000 feedback_enabled_fresh=0\n')
            message = start.startup_exit_reason(cursor)
            self.assertIn('右主臂（笔记本 can0）', message)
            self.assertIn('左主臂（笔记本 can1）', message)
            self.assertIn('USB 盒亮灯不代表电机已通电', message)
            self.assertIn('停止遥操并失能', message)
            self.assertIn('从臂可能仍使能', message)
            self.assertNotIn('36.0°', message)
            self.assertNotIn('从臂归位超时', message)

    def test_leader_verified_feedback_timeout_does_not_claim_power_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.log'
            cursor = start.startup_log_cursor({'StandardOutput': 'append:' + str(path)})
            path.write_text('ERROR: 主臂未实际到达初始位\n'
                '[leader_gravity_pd.can0]: Startup homing timed out: J4 target=0.6283 actual=0.5000 feedback_enabled_fresh=1\n')
            message = start.startup_exit_reason(cursor)
            self.assertIn('机械阻挡', message)
            self.assertNotIn('电机使能或反馈未通过核验', message)

    def test_missing_rotated_or_truncated_log_uses_generic_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.log"
            path.write_text("old log data\n" * 100)
            cursor = start.startup_log_cursor({"StandardOutput": "append:" + str(path)})
            path.write_text("ERROR: stale\n")
            self.assertNotIn("stale", start.startup_exit_reason(cursor))
            path.rename(path.with_suffix(".old"))
            path.write_text("ERROR: foreign log\n")
            self.assertNotIn("foreign", start.startup_exit_reason(cursor))
            path.unlink()
            self.assertIn("启动服务已退出", start.startup_exit_reason(cursor))

    def test_new_log_and_other_startup_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teleop.log"
            cursor = start.startup_log_cursor({"StandardOutput": "append:" + str(path)})
            path.write_text("ERROR: 控制器缺少动态库\n")
            self.assertEqual(start.startup_exit_reason(cursor), "ERROR: 控制器缺少动态库")
        self.assertIsNone(start.startup_log_cursor({"StandardOutput": "journal"}))
        self.assertIn("启动服务已退出", start.startup_exit_reason(None))


if __name__ == "__main__":
    unittest.main()

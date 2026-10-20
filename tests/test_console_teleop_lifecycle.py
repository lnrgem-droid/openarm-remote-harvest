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


if __name__ == "__main__":
    unittest.main()

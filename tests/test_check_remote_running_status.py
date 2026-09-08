#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "check_remote_running_status.py"
SPEC = importlib.util.spec_from_file_location("check_remote_running_status", SCRIPT)
check = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(check)


class RunningContractTest(unittest.TestCase):
    def healthy(self):
        return {
            "state": "RUNNING",
            "fault_bits": 0,
            "relative_follow_reference_captured": True,
            "enabled_arms": ["right", "left"],
            "leader_session_id": 42,
            "feedback_fresh_for_control": True,
            "action_age_ms": 2.5,
        }

    def test_complete_contract_passes(self):
        self.assertTrue(check.is_healthy_running(self.healthy()))

    def test_each_motion_gate_is_mandatory(self):
        changes = {
            "state": "READY",
            "fault_bits": 1,
            "relative_follow_reference_captured": False,
            "enabled_arms": ["right"],
            "leader_session_id": 0,
            "feedback_fresh_for_control": False,
            "action_age_ms": 101.0,
        }
        for field, bad_value in changes.items():
            with self.subTest(field=field):
                status = self.healthy(); status[field] = bad_value
                self.assertFalse(check.is_healthy_running(status))

    def test_parser_tolerates_ros_noise_but_rejects_missing_json(self):
        value = check.parse_status("warning\n{\"state\":\"RUNNING\"}\n")
        self.assertEqual(value["state"], "RUNNING")
        with self.assertRaises(ValueError):
            check.parse_status("warning only")


if __name__ == "__main__":
    unittest.main()

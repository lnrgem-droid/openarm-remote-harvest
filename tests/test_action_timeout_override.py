"""No hardware: bounded launch override and real watchdog timeout behavior."""
import ast
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).parents[1]
for package in ('remote_teleop_protocol', 'remote_teleop_follower_safety'):
    sys.path.insert(0, str(ROOT / 'ros2_robot/src' / package))
from remote_teleop_protocol import ControlState, FaultBits
from remote_teleop_follower_safety.state_machine import SafetyReaction, SafetyStateMachine
from remote_teleop_follower_safety.watchdog import ControllerHeartbeat, WatchdogConfig, WatchdogSupervisor


def launch_arguments(path, value):
    tree = ast.parse(path.read_text())
    action = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_launch_actions')
    class Config:
        def __init__(self, key): self.key = key
        def perform(self, context): return context[self.key]
    namespace = {'LaunchConfiguration': Config, 'Node': lambda **kw: kw,
                 'TimerAction': lambda **kw: kw, 'OpaqueFunction': lambda **kw: kw,
                 '_start_control_stack': lambda _: None}
    exec(compile(ast.Module(body=[action], type_ignores=[]), str(path), 'exec'), namespace)
    actions = namespace['_launch_actions']({'reaction_verified': 'true', 'startup_home': 'true',
                                           'network_action_timeout_ms': value})
    return actions[0]['arguments']


class ActionTimeoutOverrideTest(unittest.TestCase):
    def test_launch_passes_only_the_two_supported_finite_values(self):
        path = ROOT / 'ros2_robot/src/remote_teleop_runtime/launch/bimanual_follower.launch.py'
        for value in ('150', '300'):
            args = launch_arguments(path, value)
            self.assertEqual(args[args.index('--network-action-timeout-ms')+1], value)
            self.assertIn('position_hold', args)
            self.assertEqual(args[args.index('--startup-grace-s')+1], '35')
        for value in ('0', '-1', 'inf', 'nan', '301', ''):
            with self.subTest(value=value), self.assertRaises(ValueError):
                launch_arguments(path, value)

    def test_invalid_shell_override_exits_before_ros_or_hardware(self):
        for value in ('0', '-1', 'inf', '301'):
            result = subprocess.run(['bash', str(ROOT / 'scripts/run_bimanual_remote_feedback.sh')],
                env={**os.environ, 'OPENARM_NETWORK_ACTION_TIMEOUT_MS': value},
                text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 64)
            self.assertIn('must be 150 or 300', result.stderr)

    def test_300ms_tolerates_short_gap_but_still_latches_long_gap(self):
        base = 10_000_000_000
        for limit in (150, 300):
            machine = SafetyStateMachine(SafetyReaction.POSITION_HOLD, True)
            supervisor = WatchdogSupervisor(machine, WatchdogConfig(network_action_timeout_ns=limit*1_000_000))
            supervisor.boot(base)
            machine.observe_leader_session(22)
            machine.alignment_complete(22)
            machine.request_run(22)
            for sequence, age_ms in enumerate((155, 299, 301), 1):
                now = base + age_ms*1_000_000
                supervisor.receive_heartbeat(ControllerHeartbeat(11, sequence, now, now, base, 22, True, False, 0), now)
                supervisor.check(now)
                self.assertEqual(machine.state, ControlState.FAULT if age_ms > limit else ControlState.RUNNING)
            self.assertTrue(machine.snapshot().fault_bits & FaultBits.NETWORK_TIMEOUT)
            # A newly received action cannot erase the latched fault.
            now = base + 302_000_000
            supervisor.receive_heartbeat(ControllerHeartbeat(11, 4, now, now, now, 22, True, False, 0), now)
            self.assertEqual(machine.state, ControlState.FAULT)


if __name__ == '__main__':
    unittest.main()

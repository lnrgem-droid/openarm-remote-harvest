"""Run the merged preflight against fake overlays, never robot processes."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).parents[1]


class MergedPreflightTest(unittest.TestCase):
    def run_preflight(self, failure=None):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            setup = root / "ros-setup.sh"
            modules = root / "modules"
            modules.mkdir()
            setup.write_text('return 7\n' if failure == "setup" else
                             'export PYTHONPATH="' + str(modules) + '"\n')
            config = root / "config"
            config.mkdir()
            (config / "bimanual_follower.yaml").write_text(
                '/**:\n  ros__parameters:\n    startup_home_target: ' +
                ('[0, 0, 0, 0.628, 0, 0, 0]' if failure == "home_pose" else '[0, 0, 0, 0, 0, 0, 0]') + '\n')
            packages = {
                "ament_index_python/packages.py":
                    'def get_package_share_directory(name): return ' + repr(str(root)) + '\n',
                "remote_teleop_protocol/__init__.py":
                    "from enum import IntEnum, IntFlag\n"
                    "class ControlState(IntEnum):\n INIT=0; ALIGNING=1; READY=2; RUNNING=3; FAULT=4; E_STOP=5\n"
                    "class FaultBits(IntFlag):\n NONE=0; CAN_ERROR=2; INVALID_COMMAND=8; E_STOP_ACTIVE=32\n",
                "remote_teleop_protocol/protocol.py":
                    "def decode_collection_ack(value): return (0, 0)\nCOLLECTION_STATE_FLAGS={9,17,19,23}\n",
                "remote_teleop_runtime/collection_motion.py":
                    'COMMANDS={"left_master_align","left_lock_both","left_hold_release","left_home","right_home","left_home_stop","right_home_stop"}\n',
                "remote_teleop_runtime/leader.py":
                    'class LeaderGateway:\n collection_acknowledgement=True\n',
                "remote_teleop_runtime/follower.py": 'class FollowerGateway: pass\n',
                "remote_teleop_runtime/follower_io.py":
                    'class FollowerIOWorker: pass\nIO_PROTOCOL_VERSION=1\n',
                "remote_teleop_follower_safety/service.py": 'WATCHDOG_IO_PROTOCOL_VERSION=1\n',
                "remote_teleop_follower_safety/state_machine.py": (ROOT /
                    'ros2_robot/src/remote_teleop_follower_safety/remote_teleop_follower_safety/state_machine.py').read_text(),
            }
            if failure == "fault_hold":
                packages['remote_teleop_follower_safety/state_machine.py'] = packages[
                    'remote_teleop_follower_safety/state_machine.py'].replace(
                        'if self._state in (ControlState.FAULT, ControlState.E_STOP):\n            return\n',
                        'if self._state in (ControlState.FAULT, ControlState.E_STOP):\n            raise TransitionError("old HOLD behavior")\n')
            if failure == "protocol":
                packages["remote_teleop_follower_safety/service.py"] = 'WATCHDOG_IO_PROTOCOL_VERSION=99\n'
            if failure == "bilateral_hold":
                packages["remote_teleop_runtime/collection_motion.py"] = 'COMMANDS={"left_master_align"}\n'
            for name, content in packages.items():
                p = modules / name
                p.parent.mkdir(exist_ok=True)
                p.write_text(content)
            for overlay in ("install", "install_bimanual", "install_gripper_candidate", "install_gripper_spring_candidate"):
                p = root / "ros2_robot" / overlay
                p.mkdir(parents=True)
                (p / "setup.bash").write_text(":\n")
                (p / "local_setup.bash").write_text(":\n")
            node = root / "ros2_robot/install_gripper_spring_candidate/openarm_gravity_pd_control/lib/openarm_gravity_pd_control/openarm_gravity_pd_node"
            node.parent.mkdir(parents=True)
            node.write_text("bounded tracking assist\n" + ("" if failure == "home_marker" else
                            "Startup homing to upstream OpenArm INITIAL_POSITION\n"))
            node.chmod(0o600 if failure == "not_executable" else 0o700)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            (bin_dir / "ldd").write_text('#!/bin/sh\necho "' +
                ('libmissing.so => not found' if failure == "dependency" else 'libok.so => /fake/libok.so') + '"\n')
            (bin_dir / "taskset").write_text('#!/bin/sh\nexit ' + ('1' if failure == "cpuset" else '0') + '\n')
            for p in bin_dir.iterdir():
                p.chmod(0o700)
            script = (ROOT / "scripts/check_remote_startup_build.sh").read_text()
            script = script.replace("/opt/ros/humble/setup.bash", str(setup))
            return subprocess.run(["bash", "-s", "--", str(root), "install_gripper_spring_candidate",
                                   "true", "0-2", "150"], input=script, text=True, capture_output=True,
                                  timeout=5, env={**os.environ, "PATH": str(bin_dir)+os.pathsep+os.environ["PATH"]})

    def test_valid_build_passes(self):
        result = self.run_preflight()
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_every_required_gate_still_blocks_incompatible_build(self):
        for failure in ("setup", "dependency", "home_marker", "not_executable", "cpuset", "protocol", "bilateral_hold", "fault_hold", "home_pose"):
            with self.subTest(failure=failure):
                result = self.run_preflight(failure)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()

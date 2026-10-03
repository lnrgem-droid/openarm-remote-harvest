"""Compile and exercise the production eight-axis local servo guard offline."""
from pathlib import Path
import subprocess


def test_local_servo_guard(tmp_path):
    root = Path(__file__).resolve().parents[1]
    binary = tmp_path / "collection_guard"
    subprocess.run([
        "/usr/bin/c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-I", str(root / "ros2_robot/src/openarm_gravity_pd_control/include"),
        str(root / "tests/collection_tracking_guard_test.cpp"), "-o", str(binary),
    ], check=True)
    subprocess.run([str(binary)], check=True)

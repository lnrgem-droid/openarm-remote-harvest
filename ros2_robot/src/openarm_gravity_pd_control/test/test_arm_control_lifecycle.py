"""Compile/run the actual lifecycle helper without ROS, CAN, or robot nodes."""
from pathlib import Path
import subprocess


def test_lifecycle_under_slow_homing_failure_and_shutdown(tmp_path):
    package = Path(__file__).resolve().parents[1]
    binary = tmp_path / "lifecycle_test"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
        "-I", str(package / "include"),
        str(package / "test" / "arm_control_lifecycle_test.cpp"), "-o", str(binary),
    ], check=True, timeout=30)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=15)
    assert "10 lifecycle cases passed" in result.stdout


def test_ros_feedback_publication_is_gated_by_lifecycle_stop():
    # Bind the tested helper's stop condition to the actual ROS publication path;
    # constructing the real node would enable motors and is forbidden here.
    package = Path(__file__).resolve().parents[1]
    source = (package / "src" / "openarm_gravity_pd_node.cpp").read_text()
    publication = source.split("void publishJointStates()", 1)[1].split(
        "std::unique_ptr<ArmController>", 1)[0]
    gate = publication.index("control_lifecycle_->stopping()")
    early_return = publication.index("return;", gate)
    assert gate < early_return < publication.index("getJointStateSnapshot")
    assert early_return < publication.index("msg.header.stamp")

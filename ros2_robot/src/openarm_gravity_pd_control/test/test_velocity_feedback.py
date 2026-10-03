"""Pure codec/calibration regression; no node, socket, or motor connection."""
from pathlib import Path
import os
import re
import subprocess


def test_actual_library_feedback_calibration(tmp_path):
    package = Path(__file__).resolve().parents[1]
    # Set OPENARM_CAN_PREFIX for another host/install. Link only pure codec symbols;
    # the test never constructs a CAN socket or ROS/ArmController instance.
    prefix = Path(os.environ.get(
        "OPENARM_CAN_PREFIX", "/home/openarm/openarm_robot/ros2_robot/install/openarm_can"))
    archive = prefix / "lib" / "libopenarm_can.a"
    assert archive.is_file(), "Set OPENARM_CAN_PREFIX to the installed OpenArmCAN prefix"
    binary = tmp_path / "velocity_feedback_test"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-I", str(package / "include"), "-I", str(prefix / "include"),
        str(package / "test" / "velocity_feedback_test.cpp"),
        str(archive), "-o", str(binary),
    ], check=True, timeout=30)
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=10)
    assert "4 velocity calibration cases passed (16 axes x 4096 decoder codes)" in result.stdout


def test_all_feedback_reads_apply_calibration_once_at_ingress():
    package = Path(__file__).resolve().parents[1]
    source = (package / "src" / "arm_controller.cpp").read_text()
    # Bind the pure tested calibration to all three real feedback consumers.
    # Node construction would open hardware and is deliberately not used.
    expected = (
        r"calibratedFeedbackVelocity\(\s*motors\[i\]\.get_velocity\(\),\s*params_\.velocity_feedback_scale, i\)",
        r"calibratedFeedbackVelocity\(\s*arm_motors\[i\]\.get_velocity\(\),\s*params_\.velocity_feedback_scale, i\)",
        r"calibratedFeedbackVelocity\(motor\.get_velocity\(\),\s*params_\.velocity_feedback_scale, ARM_DOF\)",
    )
    assert source.count(".get_velocity()") == 3
    assert source.count("calibratedFeedbackVelocity(") == 3
    for pattern in expected:
        assert len(re.findall(pattern, source)) == 1
    assert "latest_state_.velocity = dq_act;" in source
    assert "trackingAssist(command_positions_[i]-q_act[i], dq_act[i]" in source
    assert "gripper_haptic=gripper_contact_.step(motor.get_position()," in source
    assert "arm_cmds.push_back({kp, kd, command_positions_[i], 0.0, tau_grav[i] + tau_haptic[i] + assist});" in source
    assert "{{gripper_kp, gripper_kd, gripper_rad, 0.0, gripper_haptic}}" in source


def test_both_scales_validated_before_any_controller_or_can_setup():
    package = Path(__file__).resolve().parents[1]
    node = (package / "src" / "openarm_gravity_pd_node.cpp").read_text()
    for side in ("left", "right"):
        assert f'declare_parameter("{side}_velocity_feedback_scale", std::vector<double>{{}});' in node
    loop = node[node.index("auto left_params = params;"):node.index("// ── Create arm controllers")]
    assert 'std::make_pair("left", &left_params)' in loop
    assert 'std::make_pair("right", &right_params)' in loop
    assert 'get_parameter(prefix + "_velocity_feedback_scale").as_double_array()' in loop
    assert "entry.second->velocity_feedback_scale =" in loop
    assert node.index("resolveVelocityFeedbackScale(") < node.index("std::make_unique<ArmController>")
    controller = (package / "src" / "arm_controller.cpp").read_text()
    ctor = controller.split("ArmController::ArmController(",1)[1].split("ArmController::~ArmController()",1)[0]
    assert ctor.index("validateVelocityFeedbackScale(") < ctor.index("make_unique<ArmDynamics>")
    assert ctor.index("validateVelocityFeedbackScale(") < ctor.index("drive_feedback_guard_.open(")

import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).parents[1]


def test_production_gripper_contact_cpp(tmp_path):
    binary=tmp_path/'gripper-contact'
    subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',
        str(ROOT/'ros2_robot/src/openarm_gravity_pd_control/include'),
        str(ROOT/'tests/gripper_contact_sim.cpp'),'-o',str(binary)],check=True)
    data=json.loads(subprocess.check_output([str(binary)],text=True))
    assert data['new_torque_nm'] < data['old_torque_nm']
    assert data['new_torque_nm'] >= -data['cap_nm']


def test_new_contact_is_leader_only_and_opt_in():
    leader=(ROOT/'ros2_robot/src/remote_teleop_runtime/launch/bimanual_leader.launch.py').read_text()
    follower=(ROOT/'ros2_robot/src/remote_teleop_runtime/launch/bimanual_follower.launch.py').read_text()
    assert 'DeclareLaunchArgument("gripper_contact_feedback", default_value="false"' in leader
    assert '"gripper_contact_feedback"' not in follower


def test_controller_wiring_keeps_arm_feedback_and_gates_gripper():
    cpp=(ROOT/'ros2_robot/src/openarm_gravity_pd_control/src/arm_controller.cpp').read_text()
    assert 'fresh && remote_gripper_valid_ && !direct_target && !collection_servo' in cpp
    assert 'drive_feedback_guard_.gripperHealthy(' in cpp
    assert 'remote_gripper_valid_ = remote_gripper.size()==1' in cpp

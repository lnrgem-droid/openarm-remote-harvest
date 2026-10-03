"""Execute actual C++ correction and validate its output with startup gates."""
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import math

import pytest

ROOT=Path(__file__).parents[1]


@pytest.fixture(scope='module')
def results(tmp_path_factory):
    compiler=shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler unavailable')
    binary=tmp_path_factory.mktemp('tracking-sim')/'simulate'
    subprocess.run([compiler,'-std=c++17','-O2','-Wall','-Wextra','-Werror',
                    '-I',str(ROOT/'ros2_robot/src/openarm_gravity_pd_control/include'),
                    str(ROOT/'tests/tracking_assist_sim.cpp'),'-o',str(binary)],check=True)
    result=subprocess.run([str(binary)],capture_output=True,text=True,check=True)
    return json.loads(result.stdout)


def test_reproduces_measured_mismatch_then_reduces_it(results):
    assert results['old_j4_rad']==pytest.approx(.114,abs=.002)
    assert results['old_j7_rad']==pytest.approx(.121,abs=.002)
    assert abs(results['new_j4_rad'])<.05
    assert abs(results['new_j7_rad'])<.035
    print('\nTracking simulation:',json.dumps(results))


def test_varied_starts_inertia_friction_and_load(results):
    assert results['sweep_cases']==72
    assert results['bounded_contact_error_rad']>.07
    assert results['overload_error_rad']>.07


def test_production_alignment_gate_accepts_correction_not_blockage(results):
    spec=importlib.util.spec_from_file_location('startup',ROOT/'scripts/verify_startup_alignment.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    policy=json.loads((ROOT/'scripts/startup_alignment_policy.json').read_text())
    s={'state':'RUNNING','fault_bits':0,'feedback_fresh_for_control':True,
       'action_age_ms':1.,'feedback_age_ms':1.,'leader_session_id':123,
       'enabled_arms':['left','right'],'relative_follow_reference_captured':True,
       'collection':{'left_mode':'FOLLOW','right_mode':'FOLLOW','recording':None,
                     'return_phase':'idle','transitioning_arms':[]}}
    for side in ('left','right'):
        for key in ('leader_'+side+'_rad',side+'_actual_rad',side+'_target_rad'):
            s[key]=[0.,0.,0.,math.pi/5,0.,0.,0.]
    for prefix,accepted in (('old',False),('new',True)):
        s['left_actual_rad'][3]=math.pi/5-results[prefix+'_j4_rad']
        s['left_actual_rad'][6]=-results[prefix+'_j7_rad']
        assert module.assess(s,'following',policy)['ok']==accepted
    s['left_actual_rad'][3]=math.pi/5-results['bounded_contact_error_rad']
    assert not module.assess(s,'following',policy)['ok']


def test_candidate_does_not_change_default_launch_or_gripper():
    for role in ('leader','follower'):
        source=(ROOT/f'ros2_robot/src/remote_teleop_runtime/launch/bimanual_{role}.launch.py').read_text()
        assert 'DeclareLaunchArgument("tracking_candidate", default_value="false"' in source
    cpp=(ROOT/'ros2_robot/src/openarm_gravity_pd_control/src/arm_controller.cpp').read_text()
    assert 'collection_servo ? collectionReturnAssistAllowed' in cpp
    assert 'collection_faulted, params_.startup_tracking_assist_enabled' in cpp
    assert 'params_.tracking_assist_enabled && command_fresh' in cpp


def test_homing_failure_latches_measured_pose_and_cannot_release_one_arm():
    cpp=(ROOT/'ros2_robot/src/openarm_gravity_pd_control/src/arm_controller.cpp').read_text()
    node=(ROOT/'ros2_robot/src/openarm_gravity_pd_control/src/openarm_gravity_pd_node.cpp').read_text()
    header=(ROOT/'ros2_robot/src/openarm_gravity_pd_control/include/openarm_gravity_pd_control/arm_controller.hpp').read_text()
    failure=cpp[cpp.index('if (elapsed >= timeout_s)'):]
    assert 'startup_home_failed_.store(true)' in failure
    assert 'home_target[i]=current_motors[i].get_position()' in failure
    assert 'command_positions_=home_target' in failure
    assert '!startup_home_failed_.load()' in cpp
    assert 'if (!enabled && startup_home_failed_.load()) return false' in header
    service=node[node.index('startup_hold_service_ ='):]
    assert service.index('startupHomingFailed()') < service.index('setStartupHold(')


def test_runtime_dependencies_checked_before_any_homing():
    script=(ROOT/'scripts/run_bimanual_remote_feedback.sh').read_text()
    check=script[script.index('verify_runtime_builds() {'):script.index('\ncleanup() {')]
    assert 'ldd "$HOST_CONTROL_NODE"' in check
    assert "ldd '$JETSON_CONTROL_NODE'" in check
    assert "grep -q 'not found'" in check
    main=script[script.index("echo '============================================================'"):]
    assert main.index('verify_runtime_builds') < main.index('show_stage 1')
    leader_wait=script[script.index('for attempt in $(seq 1 50); do'):]
    assert "grep -q 'process has died'" in leader_wait

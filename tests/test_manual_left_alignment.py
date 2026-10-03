"""Hardware-free guidance checks, including the observed J7 hold mismatch."""
import copy
import importlib.util
import math
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('manual_left_alignment', Path(__file__).parents[1]/'scripts/manual_left_alignment.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
guide = module.manual_left_alignment


def status():
    q = [0., 0., 0., .6, 0., 0., 0., -.4]*2
    return dict(state='RUNNING', fault_bits=0, teleop_ready=True, connected=True,
                action_age_ms=1., feedback_age_ms=2., leader_axes=q[:], applied_axes=q[:],
                collection=dict(left_mode='HOLD', right_mode='FOLLOW', left_alignment_error_rad=0.,
                                transitioning_arms=[], recording=None, left_align_active=False))


def test_all_eight_axes_must_align_and_input_is_never_mutated():
    value=status(); original=copy.deepcopy(value)
    result=guide(value)
    assert result['can_resume'] and result['aligned'] and len(result['rows']) == 8
    assert value == original


def test_observed_j7_mismatch_gives_correct_signed_master_adjustment():
    value=status(); value['leader_axes'][6]=-.3301670863; value['applied_axes'][6]=-.1405737392
    value['collection']['left_alignment_error_rad']=.1895933471
    result=guide(value)
    assert result['can_adjust'] and not result['can_resume'] and result['worst_axis'] == 'J7'
    assert result['rows'][6]['delta_deg'] == pytest.approx(10.86289861)
    assert '增大' in result['rows'][6]['instruction']


@pytest.mark.parametrize('target,word',[(-.6,'张开'),(-.2,'合拢')])
def test_gripper_direction_uses_calibrated_opening_sign(target,word):
    value=status(); value['applied_axes'][7]=target
    value['collection']['left_alignment_error_rad']=abs(target+.4)
    result=guide(value)
    assert result['worst_axis']=='夹爪' and not result['can_resume']
    assert word in result['rows'][7]['instruction']


@pytest.mark.parametrize('error,ready',[(.06,True),(.0600001,False)])
def test_threshold_is_not_relaxed_or_rounded(error,ready):
    value=status(); value['leader_axes'][0]=error; value['collection']['left_alignment_error_rad']=error
    assert guide(value)['can_resume'] is ready


@pytest.mark.parametrize('key,bad',[('leader_axes',None),('leader_axes',[0.]*7),
                                    ('applied_axes',[0.]*8),('leader_axes',[math.nan]*16),
                                    ('leader_axes',[math.inf]*16),('applied_axes',[True]*16)])
def test_missing_or_nonfinite_axes_cannot_be_treated_as_aligned(key,bad):
    value=status();value[key]=bad;result=guide(value)
    assert not result['available'] and not result['can_resume']


@pytest.mark.parametrize('key,bad',[('teleop_ready',False),('state','DISCONNECTED'),('fault_bits',1),
                                    ('connected',False),('action_age_ms',100),('feedback_age_ms',None),
                                    ('left_align_may_be_active',True),('left_align_stop_requested',True)])
def test_unhealthy_or_uncertain_status_disables_confirmation(key,bad):
    value=status();value[key]=bad
    result=guide(value)
    assert not result['can_resume'] and not result['can_adjust'] and not result['aligned']


@pytest.mark.parametrize('key,bad',[('left_mode','FOLLOW'),('right_mode','RETURNING'),
                                    ('left_align_active',True),('recording',{'side':'right'}),
                                    ('transitioning_arms',['left']),('transitioning_arms',None),
                                    ('left_alignment_error_rad',None),('left_alignment_error_rad',.1)])
def test_controller_gates_and_reported_error_are_authoritative(key,bad):
    value=status();value['collection'][key]=bad
    result=guide(value)
    assert not result['can_resume'] and not result['can_adjust']


def test_target_is_held_command_not_measured_follower_pose():
    value=status();value['left_actual_rad']=[.3]*7
    result=guide(value)
    assert result['can_resume'] and result['rows'][0]['target_deg'] == 0

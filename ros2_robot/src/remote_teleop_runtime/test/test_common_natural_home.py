"""Common low initial pose configuration; no ROS nodes or motor access."""
import json
import pytest
from remote_teleop_runtime.left_only_launch import load_common_home, controller_parameters

Q=[.1997,-.012,.007,.355,.0097,.0021,-.1864]


def test_both_roles_receive_identical_saved_target(tmp_path):
    path=tmp_path/'pose.json'
    path.write_text(json.dumps(dict(startup_mode='common_natural',reference_arm='leader',home_rad=Q)))
    q=load_common_home(path)
    params=[controller_parameters(role,'can1','a','b','c',q)[-1] for role in ['leader','follower']]
    assert params[0]['startup_home_target']==params[1]['startup_home_target']==Q
    for p in params:
        assert p['startup_home'] and not p['startup_hold_current']
        assert p['startup_home_duration_s']==4.
        assert p['startup_home_tolerance_rad']<=.015
        assert p['enable_left'] and not p['enable_right']


@pytest.mark.parametrize('field,value',[('startup_mode','measured'),('reference_arm','follower'),
 ('home_rad',[0]*6),('home_rad',[float('nan')]*7),('home_rad',[99]*7)])
def test_bad_saved_pose_never_launches(tmp_path,field,value):
    data=dict(startup_mode='common_natural',reference_arm='leader',home_rad=Q);data[field]=value
    path=tmp_path/'pose.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError):load_common_home(path)


def test_missing_pose_does_not_fallback_to_old_raised_pose():
    with pytest.raises(ValueError):controller_parameters('leader','can1','a','b','c')

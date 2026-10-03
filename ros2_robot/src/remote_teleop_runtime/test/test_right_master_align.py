"""Right master alignment with synthetic feedback only; never accesses hardware."""
import pytest
from remote_teleop_runtime.collection_motion import CollectionMotion, RIGHT_MASTER_ALIGN_SPEED_RAD_S
Q=[0.,0.,0.,.6,0.,0.,0.,-.4]*2
FREE=0x10
SERVO=0x11
FAULT=0x12

def command(m,name,master=None,now=1.,ack=FREE,healthy=True,**request):
    m.command(name,request,Q,Q,Q if master is None else master,[0.]*16,now,healthy,leader_ack=ack)

def begin(tmp_path,axis=6,change=.0713):
    m=CollectionMotion(tmp_path/'pose.json');command(m,'left_lock')
    m.right=tuple(Q[8:]);m.phase='hold'
    master=Q[:];master[8+axis]+=change
    command(m,'right_master_align',master)
    return m,master

def step(m,master,now,ack=None,track=True,healthy=True,actual=None):
    ack=(SERVO if m.leader_target is not None else FREE) if ack is None else ack
    output=m.update(Q if actual is None else actual,master,now,healthy,master,ack)
    if track and m.leader_target is not None:master=master[:8]+list(m.leader_target)
    return output,master

def moving(m,master):
    now=1.
    while m.right_master_align_phase!='moving':
        now+=.01;_,master=step(m,master,now);assert now<2.
    return now,master

@pytest.mark.parametrize('axis',range(8))
def test_all_axes_speed_bounded_followers_held_until_fresh_release(tmp_path,axis):
    m,master=begin(tmp_path,axis,-.5 if axis==7 else .5)
    left=m.left;right=m.right;now,master=moving(m,master);peak=0.;phases=set()
    for _ in range(1500):
        before=master[:];now+=.01;out,master=step(m,master,now)
        peak=max(peak,abs(master[axis+8]-before[axis+8])/.01)
        phases.add(m.right_master_align_phase)
        assert m.left==left and out[:8]==list(left)
        assert m.left_master_target is None and not m.left_master_align_active
        if m.right_master_align_phase=='completed':break
        assert m.right==right and out[8:]==list(right)
        if m.leader_target is not None:assert m.flags==7
    assert {'moving','settling','releasing','completed'}<=phases
    assert .99*RIGHT_MASTER_ALIGN_SPEED_RAD_S[axis]<=peak<=RIGHT_MASTER_ALIGN_SPEED_RAD_S[axis]+1e-9
    assert master[8:]==pytest.approx(Q[8:])
    assert m.right is None and not m.right_master_align_active
    assert m.status(Q,master)['right_master_align_detail']['servo_released']

@pytest.mark.parametrize('ack',[0,1,2,3,0x11,0x12,0x14,0x18,0x13,0x20])
def test_stale_capability_and_nonfree_servo_cannot_start(tmp_path,ack):
    m=CollectionMotion(tmp_path/'pose.json');command(m,'left_lock');m.right=tuple(Q[8:])
    with pytest.raises(ValueError):command(m,'right_master_align',ack=ack)
    assert not m.right_master_align_active and m.leader_target is None

@pytest.mark.parametrize('axis,value',[(0,-1.5),(1,-.3),(7,.01),(6,2.),(3,float('nan'))])
def test_correct_right_limits_finite_poses(tmp_path,axis,value):
    m=CollectionMotion(tmp_path/'pose.json');command(m,'left_lock');m.right=tuple(Q[8:]);master=Q[:];master[8+axis]=value
    with pytest.raises(ValueError):command(m,'right_master_align',master)
    assert not m.right_master_align_active

@pytest.mark.parametrize('mutation',[lambda m:setattr(m,'left',None),lambda m:setattr(m,'right',None),lambda m:setattr(m,'returning',True),lambda m:setattr(m,'left_align_active',True),lambda m:m.transition.update(left=list(Q[:8])),lambda m:setattr(m,'recording',{'token':'test'})])
def test_conflicting_state_rejects_start(tmp_path,mutation):
    m=CollectionMotion(tmp_path/'pose.json');command(m,'left_lock');m.right=tuple(Q[8:]);mutation(m)
    with pytest.raises(ValueError):command(m,'right_master_align')
    assert not m.right_master_align_active

@pytest.mark.parametrize('name',['collection_begin','left_follow','left_lock','left_master_align','left_align_follow','right_return','right_save','right_follow'])
def test_active_alignment_blocks_other_motion_and_recording(tmp_path,name):
    m,master=begin(tmp_path)
    with pytest.raises(ValueError):command(m,name,master,side='right',token='test')
    assert m.right==tuple(Q[8:]) and m.left==tuple(Q[:8])

def test_duplicate_does_not_restart_or_recapture_goal(tmp_path):
    m,master=begin(tmp_path);now,master=moving(m,master)
    before=(m.right_master_goal,m.right_master_start,m.right_master_elapsed)
    command(m,'right_master_align',master,now+.1,SERVO)
    assert before==(m.right_master_goal,m.right_master_start,m.right_master_elapsed)

@pytest.mark.parametrize('ack',[FREE,FAULT,0x14,0,0x13])
def test_bad_ack_stops_without_follow(tmp_path,ack):
    m,master=begin(tmp_path);now,master=moving(m,master)
    step(m,master,now+.01,ack=ack)
    assert m.right_master_align_phase=='failed' and m.right_master_align_active
    assert m.right==tuple(Q[8:]) and m.leader_target==tuple(master[8:])

@pytest.mark.parametrize('axis,change',[(6,.5),(7,-.5)])
def test_stuck_master_and_gripper_tracking_failure_never_resume(tmp_path,axis,change):
    m,master=begin(tmp_path,axis,change);now,master=moving(m,master)
    for _ in range(1800):
        now+=.01;out,master=step(m,master,now,track=False)
        if m.right_master_align_phase=='failed':break
    assert m.right_master_align_phase=='failed' and m.right_master_align_active
    assert m.right==tuple(Q[8:]) and out[8:]==Q[8:]
    held=m.leader_target;step(m,master,now+.1)
    assert m.leader_target==held and m.right_master_align_phase=='failed'

def test_small_unreachable_gripper_gap_times_out_without_looser_tolerance(tmp_path):
    m,master=begin(tmp_path,7,-.0713);now,master=moving(m,master)
    for _ in range(1300):
        now+=.01;_,master=step(m,master,now,track=False)
        if m.right_master_align_phase=='failed':break
    assert m.right_master_align_phase=='failed'
    assert '超时' in m.right_master_align_detail['message'] and m.right is not None

@pytest.mark.parametrize('phase',['moving','releasing'])
def test_pause_waits_free_ack_and_does_not_resume(tmp_path,phase):
    m,master=begin(tmp_path);now,master=moving(m,master)
    if phase=='releasing':
        for _ in range(400):
            now+=.01;_,master=step(m,master,now)
            if m.right_master_align_phase=='releasing':break
    command(m,'right_master_pause',master,now,FREE if m.leader_target is None else SERVO)
    for _ in range(35):now+=.01;_,master=step(m,master,now,ack=SERVO)
    assert m.right_master_align_active and m.right is not None
    for _ in range(20):now+=.01;_,master=step(m,master,now,ack=FREE)
    assert m.right_master_align_phase=='paused' and not m.right_master_align_active
    assert m.right==tuple(Q[8:]) and m.left==tuple(Q[:8]) and m.leader_target is None

def test_health_loss_keeps_hold_and_requires_explicit_stop(tmp_path):
    m,master=begin(tmp_path);now,master=moving(m,master)
    step(m,master,now+.01,healthy=False)
    step(m,master,now+.02)
    assert m.right_master_align_phase=='failed' and m.right_master_align_active
    command(m,'right_master_pause',master,now+.03,FAULT)
    for i in range(20):step(m,master,now+.04+i*.01,ack=FREE)
    assert m.right_master_align_phase=='paused' and m.right is not None

@pytest.mark.parametrize('axis',[11,15])
def test_follower_drift_cancels_alignment(tmp_path,axis):
    m,master=begin(tmp_path);now,master=moving(m,master);actual=Q[:];actual[axis]+=.11
    out,_=step(m,master,now+.01,actual=actual)
    assert m.right_master_align_phase=='failed' and out[8:]==Q[8:]

def test_release_drift_does_not_resume_and_delayed_stop_latches_latest_target(tmp_path):
    m,master=begin(tmp_path);now,master=moving(m,master)
    for _ in range(500):
        now+=.01;_,master=step(m,master,now)
        if m.right_master_align_phase=='releasing':break
    master[14]+=.07
    for _ in range(20):now+=.01;_,master=step(m,master,now,ack=FREE)
    assert m.right_master_align_phase=='failed' and m.right is not None
    assert not m.right_master_align_active

def test_late_stop_after_completion_holds_latest_applied_pose(tmp_path):
    m,master=begin(tmp_path);now,master=moving(m,master)
    for _ in range(500):
        now+=.01;_,master=step(m,master,now)
        if m.right_master_align_phase=='completed':break
    latest=Q[:];latest[14]=.12
    m.command('right_master_pause',{},Q,latest,latest,[0.]*16,now,True,leader_ack=FREE)
    assert m.right==tuple(latest[8:]) and m.transition['right'] is None
    for i in range(20):step(m,latest,now+.01+i*.01,ack=FREE)
    assert m.right_master_align_phase=='paused' and m.right==tuple(latest[8:])

def test_auto_stop_during_legacy_return_cannot_start_parallel_coordinator(tmp_path):
    m=CollectionMotion(tmp_path/'pose.json');command(m,'left_lock');command(m,'right_save')
    command(m,'right_return')
    step(m,Q[:],1.01)
    command(m,'right_master_pause',now=1.02,ack=SERVO)
    assert not m.returning and not m.right_master_align_active
    assert m.right==tuple(Q[8:])

def test_legacy_return_resets_completed_auto_status(tmp_path):
    m=CollectionMotion(tmp_path/'pose.json');command(m,'left_lock');command(m,'right_save')
    m.right_master_align_phase='completed'
    command(m,'right_return')
    assert m.right_master_align_phase=='idle' and m.right_master_align_detail=={}

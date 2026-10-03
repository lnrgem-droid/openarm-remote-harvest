"""No hardware commands: reproduce false RUNNING and cached UI/reuse bugs."""
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import pytest

ROOT=Path(__file__).parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from teleop_readiness import evaluate, motor_health, apply_report, read_report


def status():
    return dict(state='RUNNING',fault_bits=0,leader_session_id=123,
                action_age_ms=1.,feedback_age_ms=2.,feedback_fresh_for_control=True,
                relative_follow_reference_captured=True,enabled_arms=['left','right'],
                collection=dict(left_mode='FOLLOW',right_mode='FOLLOW',transitioning_arms=[]))


def motors(value=1):
    r={'result':'ENABLED'} # intentionally keep a false summary in fault tests
    for role,buses in [('host',['can0','can1']),('follower',['can1','can2'])]:
        r[role]={'result':'ENABLED','motors':{f'{b}/{j}':dict(state=value,states=[value],fresh=True,samples=50)
                  for b in buses for j in range(1,9)}}
    return r


def report(s=None,m=None,latch=None):
    return dict(schema_version=1,host_boot_id='test-boot',probe_started_monotonic_ns=10**9,
                status=s or status(),motors=motors() if m is None else m,invalidated_session=latch)


def test_running_and_zero_tracking_error_does_not_prove_enable():
    s=status();s.update(max_tracking_error_rad=0.,left_max_tracking_error_rad=0.)
    r=evaluate(s,motors(0))
    assert not r['teleop_ready'] and r['diagnostic']=='MOTORS_DISABLED'
    assert r['invalidated_session']==123
    assert apply_report(s,report(m=motors(0)))['state']=='MOTORS_DISABLED'


@pytest.mark.parametrize('role,bus',[('host','can0'),('host','can1'),('follower','can1'),('follower','can2')])
@pytest.mark.parametrize('motor',[1,7,8])
def test_each_arm_and_gripper_is_required(role,bus,motor):
    m=motors();m[role]['motors'][f'{bus}/{motor}'].update(state=0,states=[0])
    assert not evaluate(status(),m)['teleop_ready']


@pytest.mark.parametrize('field,value',[('state',None),('state',True),('fresh',False),('samples',2)])
def test_uncertain_motor_cannot_pass(field,value):
    m=motors();m['host']['motors']['can1/8'][field]=value
    assert motor_health(m)[0]=='MOTOR_UNKNOWN'


def test_motor_fault_is_not_hidden_by_other_enabled_drives():
    m=motors();m['follower']['motors']['can2/8'].update(state=12,states=[12])
    assert motor_health(m)[0]=='MOTOR_FAULT'
    assert '0xC' in motor_health(m)[1]


def test_missing_entries_and_summary_only_fail_closed():
    for m in (None,{}, {'result':'ENABLED'},[],{'host':None}):
        assert not evaluate(status(),m)['teleop_ready']


def test_restoring_enable_does_not_auto_resume_same_session():
    r=evaluate(status(),motors(0))
    r=evaluate(status(),motors(),r['invalidated_session'])
    assert r['diagnostic']=='REINITIALIZE_REQUIRED'
    s=status();s['leader_session_id']=124
    assert evaluate(s,motors(),r['invalidated_session'])['teleop_ready']


@pytest.mark.parametrize('field,value',[
    ('state','FAULT'),('state','E_STOP'),('fault_bits',1),('fault_bits',False),
    ('leader_session_id',None),('leader_session_id',True),('action_age_ms',float('nan')),
    ('feedback_age_ms',100),('feedback_age_ms',-1),('feedback_fresh_for_control',False),
    ('relative_follow_reference_captured',False),('enabled_arms',None),
    ('enabled_arms',['left',None]),('collection',None)])
def test_software_contract_is_still_required(field,value):
    s=status();s[field]=value
    assert not evaluate(s,motors())['teleop_ready']


def test_hold_and_transition_are_not_mislabelled_follow():
    s=status();s['collection']['left_mode']='HOLD'
    r=evaluate(s,motors())
    assert r['teleop_ready'] and r['follow_enabled']=={'left':False,'right':True}
    s['collection']['transitioning_arms']=['right']
    assert not any(evaluate(s,motors())['follow_enabled'].values())


def test_report_age_uses_local_monotonic_acquisition_start(tmp_path):
    p=tmp_path/'health.json';p.write_text(json.dumps(report()))
    assert read_report(p,now_ns=2*10**9,boot_id='test-boot')
    for now,boot in [(6*10**9,'test-boot'),(0,'test-boot'),(2*10**9,'other-boot')]:
        with pytest.raises(ValueError):read_report(p,now_ns=now,boot_id=boot)


def test_old_good_report_cannot_approve_a_new_session():
    s=status();s['leader_session_id']=124
    r=apply_report(s,report())
    assert not r['teleop_ready'] and r['state']=='CHECKING' and r['control_state']=='RUNNING'


def test_ui_snapshot_and_buttons_cannot_use_raw_running(monkeypatch):
    path=ROOT.parent/'openarm-rgbd-preview/scripts/mushroom_collection_console.py'
    spec=importlib.util.spec_from_file_location('readiness_ui_test',path)
    ui=importlib.util.module_from_spec(spec);spec.loader.exec_module(ui)
    monitor=ui.TeleopMonitor('unused')
    monitor._value=status();monitor._last_running=time.monotonic()-10
    monkeypatch.setattr(ui,'apply_current_report',lambda s:apply_report(s,report(m=motors(0))))
    value=monitor.snapshot()
    assert value['state']=='MOTORS_DISABLED' and value['last_running_age_s']>3
    assert not monitor.request('right_return')
    assert not monitor.request('left_follow')
    assert monitor.request('right_pause') # explicit stop remains available


def test_reuse_cli_requires_fresh_physical_proof(tmp_path):
    p=tmp_path/'health.json'
    r=report();r.update(host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                       probe_started_monotonic_ns=time.monotonic_ns())
    p.write_text(json.dumps(r))
    cmd=[sys.executable,str(ROOT/'scripts/check_remote_running_status.py'),'--health-report',str(p)]
    assert subprocess.run(cmd,input=json.dumps(status()),text=True).returncode==0
    r['motors']=motors(0);p.write_text(json.dumps(r))
    assert subprocess.run(cmd,input=json.dumps(status()),text=True).returncode==1


def test_monitor_implementation_does_not_send_recovery_or_motor_commands():
    code=(ROOT/'scripts/watch_teleop_link.py').read_text()
    assert 'check_motor_enable.py' in code and 'probe_started_monotonic_ns' in code
    assert 'systemctl' not in code and 'cansend' not in code
    assert 'request_run' not in code and '"command":"status"' in code

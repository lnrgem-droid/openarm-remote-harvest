"""Recovery contract with synthetic motor evidence. No physical commands."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
import left_only_session as session
import left_only_device as device
import left_only_desktop as desktop


@pytest.fixture
def setup(monkeypatch):
    calls=[]
    states={r:{'ActiveState':'active','MainPID':str(i+1)} for i,r in enumerate(session.UNITS)}
    status={'mode':'left_only','enabled_arms':['left'],'state':'FAULT','fault_bits':2}
    monkeypatch.setattr(session,'unit_state',lambda c,r: states[r].copy())
    monkeypatch.setattr(session,'control',lambda c,cmd: status.copy())
    monkeypatch.setattr(session,'disabled_pair',lambda c: True)
    def stop(args):
        calls.append(args)
        for role,unit in session.UNITS.items():
            if unit in args: states[role]={'ActiveState':'inactive','MainPID':'0'}
        return ''
    monkeypatch.setattr(session,'run',lambda args,*a: stop(args))
    monkeypatch.setattr(session,'remote',lambda c,args,*a: stop(args))
    monkeypatch.setattr(session,'disable',lambda c: pytest.fail('unconfirmed disable'))
    return calls,states,status


def test_idle_has_no_status_probe_or_stop(setup,monkeypatch):
    calls,states,_=setup
    for r in states:states[r]={'ActiveState':'inactive','MainPID':'0'}
    monkeypatch.setattr(session,'control',lambda *a:pytest.fail('idle status unnecessary'))
    assert session.prepare_start({}) is None
    assert not calls


def test_disabled_fault_session_is_cleaned_before_preflight(setup):
    calls,states,_=setup
    assert session.prepare_start({}) is None
    assert calls==[['systemctl','--user','stop',session.UNITS[r]] for r in ('leader','follower')]
    assert all(session.is_idle(s) for s in states.values())


def test_enabled_fault_never_stopped_without_support(setup,monkeypatch):
    calls,_,_=setup
    monkeypatch.setattr(session,'disabled_pair',lambda c:False)
    with pytest.raises(session.RecoveryNeedsSupport):session.prepare_start({})
    assert not calls


def test_supported_restart_disables_before_start(setup,monkeypatch):
    calls,states,_=setup
    monkeypatch.setattr(session,'disabled_pair',lambda c:False)
    def disable(c):
        calls.append('disable')
        for r in states:states[r]={'ActiveState':'inactive','MainPID':'0'}
    monkeypatch.setattr(session,'disable',disable)
    assert session.prepare_start({},True) is None
    assert calls==['disable']


def test_healthy_repeat_start_is_noop(setup,monkeypatch):
    calls,_,status=setup
    status.update(state='RUNNING',fault_bits=0,feedback_fresh_for_control=True,
                  motor_health={'healthy':True},action_age_ms=1.)
    monkeypatch.setattr(session,'disabled_pair',lambda *a:pytest.fail('no CAN probe for healthy session'))
    assert session.prepare_start({})['state']=='RUNNING'
    assert not calls


@pytest.mark.parametrize('fault',['unknown','motor_error','disconnected'])
def test_incomplete_motor_evidence_never_stops(setup,monkeypatch,fault):
    calls,_,_=setup
    def probe(c):raise RuntimeError(fault)
    monkeypatch.setattr(session,'disabled_pair',probe)
    with pytest.raises(RuntimeError,match=fault):session.prepare_start({},True)
    assert not calls


def test_initializing_disabled_session_not_auto_stopped(setup):
    calls,_,status=setup;status.update(state='ALIGNING',fault_bits=0)
    with pytest.raises(session.RecoveryNeedsSupport):session.prepare_start({})
    assert not calls


def test_second_probe_changes_no_stop(setup,monkeypatch):
    calls,_,_=setup
    replies=iter([True,False]);monkeypatch.setattr(session,'disabled_pair',lambda c:next(replies))
    with pytest.raises(session.RecoveryNeedsSupport):session.prepare_start({})
    assert not calls


def test_version_failure_cannot_cleanup_or_start(setup,monkeypatch):
    calls,_,_=setup
    def verify(c):raise RuntimeError('bad version')
    monkeypatch.setattr(session,'verify_versions',verify)
    with pytest.raises(RuntimeError,match='bad version'):session.start({})
    assert not calls


def test_drive_probe_never_reads_registers(monkeypatch):
    calls=[]
    monkeypatch.setattr(device,'validate_bus',lambda *a:('usb',3))
    def query(bus,motor,command):
        calls.append((bus,motor,command));return bytes([motor])+bytes(7)
    monkeypatch.setattr(device,'query',query)
    result=device.drive_states('follower','can2')
    assert result['all_disabled'] and len(result['motors'])==8
    assert len(calls)==24 and all(c[2]==0xcc for c in calls)


@pytest.mark.parametrize('answer,expected_calls',[('取消',1),('已支撑',2)])
def test_terminal_retries_only_after_support(monkeypatch,answer,expected_calls):
    calls=[]
    monkeypatch.setattr('builtins.input',lambda text:answer)
    def run(args,**kw):calls.append(args);return NS(returncode=3 if len(calls)==1 else 0)
    monkeypatch.setattr(desktop.subprocess,'run',run)
    desktop.execute(Path('/release'),['start','--workspace-clear'])
    assert len(calls)==expected_calls
    assert '--arms-supported' not in calls[0]
    if expected_calls==2:assert '--arms-supported' in calls[1]


def test_parallel_operations_do_not_overlap(tmp_path,monkeypatch):
    monkeypatch.setattr(session.Path,'home',lambda:tmp_path)
    with session.operation_lock():
        with pytest.raises(RuntimeError,match='正在执行'):
            with session.operation_lock():pytest.fail('must be locked')


def test_common_home_verifier_uses_left_specific_policy(monkeypatch):
    calls=[]
    def remote(c,args,*a):calls.append(args);return '{"accepted": true}'
    monkeypatch.setattr(session,'remote',remote)
    session.verify_alignment({'peer_root':'/release'},'aligning')
    assert '--measured-start' not in calls[0]
    assert calls[0][calls[0].index('--policy')+1]=='/release/scripts/left_startup_alignment_policy.json'

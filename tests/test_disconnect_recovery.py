import json
import copy
import importlib.util
from pathlib import Path
import subprocess
import pytest

ROOT=Path(__file__).parents[1]


def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts'/f'{name}.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


recovery=load('inspect_teleop_recovery')
watch=load('watch_teleop_link')


def reboot():
    return dict(ssh_ok=True,host_motors='DISABLED',previous_boot_id='11111111-1111-1111-1111-111111111111',
        host_service_age_s=500,remote=dict(boot_id='22222222-2222-2222-2222-222222222222',
        uptime_s=100,processes=[],runtime_socket_exists=False,status=None,can_ok=True,recorder_idle=True))


def disabled_timeout():
    e=reboot();e['host_can_ok']=True; e['host_service_pid']=123
    e['motor_report']={'result':'DISABLED'}
    for role,buses in (('host',('can0','can1')),('follower',('can1','can2'))):
        e['motor_report'][role]={'result':'DISABLED','motors':{
            f'{bus}/{j}':dict(state=0,states=[0],fresh=True,samples=50)
            for bus in buses for j in range(1,9)}}
    e['remote'].update(processes=[234,235,236],runtime_socket_exists=True,status={
        'state':'FAULT','fault_bits':1,'reason':'leader action receive timeout',
        'leader_session_id':45,'action_age_ms':1.,'feedback_age_ms':2.,
        'feedback_fresh_for_control':True,'enabled_arms':['left','right'],
        'collection':dict(left_mode='HOLD',right_mode='FOLLOW',recording=None,
                          return_phase='idle',transitioning_arms=[])})
    return e


def normal_power_down():
    e = disabled_timeout()
    e['host_service_pid'] = 0
    e['host_boot_id'] = 'host-boot'
    e['host_unit'] = dict(MainPID='0', ControlGroup='', ActiveState='failed', InvocationID='unit-instance')
    e['remote']['process_identities'] = {'234': {'start_ticks': 1000, 'argv': ['follower']}}
    e['remote']['status'].update(fault_bits=2, reason='controller reports CAN failure',
                                feedback_fresh_for_control=False, action_age_ms=400000, feedback_age_ms=400000)
    devices = {}
    for role, buses in (('host', ('can0', 'can1')), ('follower', ('can1', 'can2'))):
        devices[role] = dict(ok=True, verified=True, acknowledged=True, motors={
            f'{bus}/{motor}': dict(disabled=True, states=[0, 0, 0])
            for bus in buses for motor in range(1, 9)})
        e['motor_report'][role] = dict(result='UNKNOWN', motors={
            f'{bus}/{motor}': dict(state=None, states=[], fresh=False, samples=0)
            for bus in buses for motor in range(1, 9)})
    e['motor_report']['result'] = 'UNKNOWN'
    e['power_down_receipt'] = dict(schema_version=1, shutdown_id='proof-1', consumed=False,
        host_boot_id=e['host_boot_id'], remote_boot_id=e['remote']['boot_id'],
        service_invocation_id='unit-instance', leader_session_id=e['remote']['status']['leader_session_id'],
        remote_process_identities=copy.deepcopy(e['remote']['process_identities']), devices=devices)
    return e


def test_known_normal_power_down_offers_confirmed_recovery_when_feedback_is_silent():
    report = recovery.classify(normal_power_down())
    assert report['code'] == 'OPERATOR_POWER_DOWN_CONFIRM'
    assert report['confirmation_required'] and not report['automatic_restart_allowed']


@pytest.mark.parametrize('field,value', [('consumed', True), ('shutdown_id', None),
    ('host_boot_id', 'other'), ('remote_boot_id', 'other'), ('service_invocation_id', 'other'),
    ('leader_session_id', 1234), ('remote_process_identities', {}), ('devices', {}), ('schema_version', 9)])
def test_old_or_incomplete_shutdown_receipt_cannot_bypass_fault(field, value):
    e = normal_power_down(); e['power_down_receipt'][field] = value
    assert not recovery.classify(e)['confirmation_required']


@pytest.mark.parametrize('field,value', [('MainPID', '123'), ('ControlGroup', '/live'),
    ('ActiveState', 'active'), ('InvocationID', 'new-instance')])
def test_restart_receipt_requires_exact_stopped_host_service(field, value):
    e = normal_power_down(); e['host_unit'][field] = value
    assert not recovery.classify(e)['confirmation_required']


def test_new_remote_process_or_session_invalidates_normal_shutdown():
    e = normal_power_down(); e['remote']['process_identities']['234']['start_ticks'] += 1
    assert not recovery.classify(e)['confirmation_required']
    e = normal_power_down(); e['remote']['status']['leader_session_id'] += 1
    assert not recovery.classify(e)['confirmation_required']


@pytest.mark.parametrize('field,value', [('can_ok', False), ('recorder_idle', False), ('boot_id', 'new-boot')])
def test_normal_shutdown_does_not_ignore_recorder_or_can_failure(field, value):
    e = normal_power_down(); e['remote'][field] = value
    assert not recovery.classify(e)['confirmation_required']


def test_unknown_without_receipt_is_not_treated_as_disabled():
    e = normal_power_down(); e.pop('power_down_receipt')
    assert recovery.classify(e)['code'] == 'LATCHED_FAULT'


def test_real_drive_fault_or_enabled_feedback_invalidates_receipt():
    for state in [1, 8]:
        e = normal_power_down(); e['motor_report']['follower']['motors']['can2/8']['states'] = [state]
        assert not recovery.classify(e)['confirmation_required']
    for field, value in [('state', 'E_STOP'), ('fault_bits', 8), ('reason', 'another CAN problem')]:
        e = normal_power_down(); e['remote']['status'][field] = value
        assert not recovery.classify(e)['confirmation_required']


def test_receipt_roundtrip_and_one_time_consumption(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', tmp_path / 'last-power-down.json')
    e = normal_power_down()
    receipt = recovery.save_power_down_receipt(e, e, e['power_down_receipt']['devices'])
    assert recovery.read_power_down_receipt() == receipt
    recovery.consume_power_down_receipt(receipt['shutdown_id'])
    e['power_down_receipt'] = recovery.read_power_down_receipt()
    assert not recovery.classify(e)['confirmation_required']
    with pytest.raises(RuntimeError):
        recovery.consume_power_down_receipt(receipt['shutdown_id'])


def test_receipt_not_written_for_changed_session_or_unverified_motor(tmp_path, monkeypatch):
    path = tmp_path / 'last-power-down.json'
    monkeypatch.setattr(recovery, 'POWER_DOWN_RECEIPT', path)
    e = normal_power_down(); after = copy.deepcopy(e)
    after['remote']['status']['leader_session_id'] += 1
    with pytest.raises(RuntimeError):
        recovery.save_power_down_receipt(e, after, e['power_down_receipt']['devices'])
    e['power_down_receipt']['devices']['host']['motors']['can0/1']['states'] = []
    with pytest.raises(RuntimeError):
        recovery.save_power_down_receipt(e, e, e['power_down_receipt']['devices'])
    assert not path.exists()


def test_all_disabled_old_timeout_offers_confirmation_not_automatic_motion():
    e=disabled_timeout();r=recovery.classify(e)
    assert r['code']=='DISABLED_TIMEOUT_CONFIRM' and r['confirmation_required']
    assert r['automatic_restart_allowed'] is False
    e['remote']['status']['collection']['right_mode']='HOLD'
    e['remote']['status']['collection']['return_phase']='hold'
    assert recovery.classify(e)['confirmation_required']


@pytest.mark.parametrize('field,value',[
    ('state','E_STOP'),('fault_bits',2),('fault_bits',3),('fault_bits',True),
    ('reason','controller reports CAN failure'),('leader_session_id',None),
    ('action_age_ms',100),('action_age_ms',float('nan')),('action_age_ms',-1),
    ('feedback_age_ms',float('inf')),('feedback_fresh_for_control',False),
    ('enabled_arms',['right']),('collection',{}),('collection','bad')])
def test_old_timeout_recovery_rejects_other_faults_or_stale_data(field,value):
    e=disabled_timeout(); e['remote']['status'][field]=value
    assert not recovery.classify(e)['confirmation_required']


@pytest.mark.parametrize('field,value',[
    ('recording',{}),('return_phase','moving'),('return_phase','releasing'),
    ('right_mode','RETURNING'),('transitioning_arms',['right'])])
def test_timeout_recovery_rejects_active_collection(field,value):
    e=disabled_timeout(); e['remote']['status']['collection'][field]=value
    assert not recovery.classify(e)['confirmation_required']


@pytest.mark.parametrize('field,value',[
    ('can_ok',False),('recorder_idle',False),('runtime_socket_exists',False),
    ('processes',[]),('boot_id','bad')])
def test_timeout_recovery_requires_verified_peer(field,value):
    e=disabled_timeout();e['remote'][field]=value
    assert not recovery.classify(e)['confirmation_required']


@pytest.mark.parametrize('field,value',[
    ('state',1),('state',None),('states',[0,1]),('fresh',False),('samples',2)])
def test_every_motor_must_be_fresh_consistently_disabled(field,value):
    for role,bus in [('host','can0'),('follower','can2')]:
        e=disabled_timeout();e['motor_report'][role]['motors'][bus+'/8'][field]=value
        assert not recovery.classify(e)['confirmation_required']


def test_timeout_recovery_cannot_trust_only_summary_or_skip_host_can():
    e=disabled_timeout();e['motor_report']={'result':'DISABLED'}
    assert not recovery.classify(e)['confirmation_required']
    e=disabled_timeout();e['host_can_ok']=False
    assert not recovery.classify(e)['confirmation_required']


def test_reboot_requires_operator_confirmation_never_automatic():
    r=recovery.classify(reboot())
    assert r['code']=='PEER_REBOOT_CONFIRM' and r['confirmation_required']
    assert not r['automatic_restart_allowed']


@pytest.mark.parametrize('field,value,code',[
    ('ssh_ok',False,'NETWORK_OR_SSH'),('host_motors','ENABLED','HOST_MOTORS_NOT_DISABLED'),
    ('host_motors','MIXED','HOST_MOTORS_NOT_DISABLED'),('host_motors','UNKNOWN','HOST_MOTORS_NOT_DISABLED')])
def test_network_and_uncertain_motor_states_block(field,value,code):
    e=reboot();e[field]=value;r=recovery.classify(e)
    assert r['code']==code and not r['confirmation_required']


@pytest.mark.parametrize('field,value',[
    ('can_ok',False),('recorder_idle',False),('processes',[123]),('runtime_socket_exists',True),
    ('status',{'state':'FAULT','fault_bits':2}),('status',{'state':'E_STOP','fault_bits':0}),
    ('status',{'state':'RUNNING','fault_bits':0,'collection':{'left_mode':'HOLD'}}),
    ('status',{'state':'RUNNING','fault_bits':0,'collection':{'recording':{'token':'live'}}}),
    ('status',{'state':'RUNNING','fault_bits':0,'collection':{'return_phase':'moving'}}),
    ('boot_id','invalid'),('processes',None),('runtime_socket_exists',None)])
def test_recording_holding_faults_can_and_partial_stacks_block(field,value):
    e=reboot();e['remote'][field]=value
    assert not recovery.classify(e)['confirmation_required']


def test_same_boot_process_crash_is_not_reboot():
    e=reboot();e['previous_boot_id']=e['remote']['boot_id']
    assert recovery.classify(e)['code']=='UNEXPLAINED_STACK_LOSS'


def test_legacy_session_uses_uptime_durations_not_cross_host_wall_clock():
    e=reboot();e['previous_boot_id']=None
    assert recovery.classify(e)['confirmation_required']
    for age in (None,0,100,float('nan'),float('inf')):
        e['host_service_age_s']=age
        assert not recovery.classify(e)['confirmation_required']


def test_network_return_does_not_clear_latched_fault():
    assert watch.describe(None)[0]=='UNAVAILABLE'
    s={'state':'FAULT','fault_bits':1,'reason':'network timeout'}
    assert watch.describe(s)[0]=='FAULT'
    s={'state':'RUNNING','fault_bits':0,'feedback_fresh_for_control':True,'action_age_ms':1.}
    # A software RUNNING without physical drive proof is never ready.
    assert watch.describe(s)[0]=='MOTOR_UNKNOWN'
    s['action_age_ms']=120
    assert watch.describe(s)[0]=='MOTOR_UNKNOWN'


def test_recovery_confirmation_and_second_probe_are_in_actual_launcher():
    script=(ROOT/'scripts/daily_start_teleop_rgbd.sh').read_text()
    f=script[script.index('recover_restarted_peer() {'):script.index('\nensure_host_can()')]
    assert f.count('inspect_teleop_recovery.py')==2
    assert 'read -r -t 60 confirmation' in f
    assert 'r|R|恢复)' in f
    assert '[[ "$boot_before" == "$boot_after" ]]' in f
    assert 'systemctl' not in f # classifier never restarts the service itself


def test_no_can_send_no_recovery_commands_in_diagnostic_tools():
    for filename in ('inspect_teleop_recovery.py','watch_teleop_link.py'):
        source=(ROOT/'scripts'/filename).read_text()
        for forbidden in ('"command":"run"','"command":"reset"','"command":"disable"',
                          "'restart'",'socket.AF_CAN'):
            assert forbidden not in source


def confirmation_fixture(tmp_path,second_ok=True,same_boot=True,evidence=None,changed_session=False):
    script=(ROOT/'scripts/daily_start_teleop_rgbd.sh').read_text()
    function=script[script.index('recover_restarted_peer() {'):script.index('\nensure_host_can()')]
    invocation='/usr/bin/python3 "$TELEOP_ROOT/scripts/inspect_teleop_recovery.py" \\\n    --jetson "$JETSON_HOST" --output "$report"'
    assert function.count(invocation)==2
    function=function.replace(invocation,'probe_fixture "$report"')
    evidence=reboot() if evidence is None else evidence
    first=dict(recovery.classify(evidence), evidence=evidence)
    second=json.loads(json.dumps(first)); second['confirmation_required']=second_ok
    if not same_boot: second['evidence']['remote']['boot_id']='33333333-3333-3333-3333-333333333333'
    if changed_session: second['evidence']['remote']['status']['leader_session_id']+=1
    for name,value in [('first.json',first),('second.json',second)]:
        (tmp_path/name).write_text(json.dumps(value))
    setup='''set -euo pipefail
LOG_DIR="$1"; JETSON_HOST=mock; TELEOP_ROOT=mock; COUNT=0
probe_fixture() {
 COUNT=$((COUNT+1))
 if [[ "$COUNT" == 1 ]]; then cp "$LOG_DIR/first.json" "$1"; else cp "$LOG_DIR/second.json" "$1"; fi
}
'''
    return setup+function+'\nif recover_restarted_peer; then exit 0; else exit $?; fi'


@pytest.mark.parametrize('answer,changed_session,expected', [
    ('恢复\n', False, 0), ('取消\n', False, 2), ('', False, 2), ('r\n', True, 3)])
def test_normal_shutdown_shell_consumes_receipt_only_after_confirmation(tmp_path, answer, changed_session, expected):
    evidence = normal_power_down()
    path = tmp_path / 'home/.local/state/openarm/last-power-down.json'
    path.parent.mkdir(parents=True); path.write_text(json.dumps(evidence['power_down_receipt']))
    script = confirmation_fixture(tmp_path, evidence=evidence, changed_session=changed_session)
    script = script.replace('TELEOP_ROOT=mock', 'TELEOP_ROOT="$2"; export HOME="$LOG_DIR/home"')
    result = subprocess.run(['bash', '-c', script, 'test', str(tmp_path), str(ROOT)],
        input=answer, text=True, capture_output=True, timeout=5)
    assert result.returncode == expected, result.stdout + result.stderr
    assert '正常失能记录' in result.stdout
    assert json.loads(path.read_text())['consumed'] is (expected == 0)


def test_stopped_service_with_receipt_enters_recovery_branch(tmp_path):
    path = tmp_path / '.local/state/openarm/last-power-down.json'
    path.parent.mkdir(parents=True); path.write_text('{}')
    source = (ROOT / 'scripts/daily_start_teleop_rgbd.sh').read_text()
    start = source.index('if [[ "$recover_power_cycle" != true ]] && {')
    end = source.index('if [[ "$recover_power_cycle" != true && "$motors_enabled" == true ]] && refresh_healthy_running_status;', start)
    setup = '''set -euo pipefail
export HOME="$1"
recover_power_cycle=false; motors_enabled=false; TELEOP_SERVICE=mock
systemctl() { return 3; }
recover_restarted_peer() { echo RECOVERY_VISITED; return 0; }
'''
    result = subprocess.run(['bash', '-c', setup + source[start:end] + '\n[[ "$recover_power_cycle" == true ]]',
        'test', str(tmp_path)], text=True, capture_output=True, timeout=5)
    assert result.returncode == 0
    assert 'RECOVERY_VISITED' in result.stdout


@pytest.mark.parametrize('answer,second_ok,same_boot,expected,message',[
    ('恢复\n',True,True,0,'二次检查通过'),('r\n',True,True,0,'二次检查通过'),
    ('R\n',True,True,0,'二次检查通过'),('取消\n',True,True,2,'输入未匹配'),
    ('\n',True,True,2,'输入未匹配'),('',True,True,2,'EOF'),
    ('恢复\n',False,True,3,'恢复条件发生变化'),('恢复\n',True,False,3,'再次重启')])
def test_real_shell_confirmation_never_ignores_state_change(tmp_path,answer,second_ok,same_boot,expected,message):
    script=confirmation_fixture(tmp_path,second_ok,same_boot)
    result=subprocess.run(['bash','-c',script,
        'test',str(tmp_path)],input=answer,text=True,capture_output=True,timeout=5)
    assert result.returncode==expected,result.stdout+result.stderr
    assert message in result.stdout+result.stderr


def test_confirmation_timeout_is_reported_without_waiting_a_minute(tmp_path):
    script=confirmation_fixture(tmp_path)
    # Mock only Bash read: no CAN, SSH or systemd commands in the fixture.
    result=subprocess.run(['bash','-c','read() { return 142; }\n'+script,'test',str(tmp_path)],
                          text=True,capture_output=True,timeout=5)
    assert result.returncode==2
    assert '60秒内未完成确认' in result.stdout
    assert '二次检查通过' not in result.stdout


@pytest.mark.parametrize('answer,second_ok,changed_session,expected',[
    ('r\n',True,False,0),('',True,False,2),('取消\n',True,False,2),
    ('r\n',False,False,3),('r\n',True,True,3)])
def test_disabled_timeout_uses_confirmed_same_session_shell_gate(tmp_path,answer,second_ok,changed_session,expected):
    script=confirmation_fixture(tmp_path,second_ok=second_ok,evidence=disabled_timeout(),changed_session=changed_session)
    result=subprocess.run(['bash','-c',script,'test',str(tmp_path)],input=answer,
                          text=True,capture_output=True,timeout=5)
    assert result.returncode==expected,result.stdout+result.stderr
    assert '清理旧会话及左/右保持任务' in result.stdout
    assert 'Jetson已经重启' not in result.stdout
    assert ('二次检查通过' in result.stdout)==(expected==0)


def test_terminal_input_survives_launcher_tee_pipeline(tmp_path):
    import os
    import pty
    import select
    import time
    script=tmp_path/'mock-recovery.sh'
    script.write_text(confirmation_fixture(tmp_path))
    master,slave=pty.openpty()
    # Same stdout pipeline as the desktop wrapper, using only fixture probes.
    process=subprocess.Popen(['bash','-c',
        'set -o pipefail; bash "$1" "$2" 2>&1 | tee "$2/output.log"',
        'test',str(script),str(tmp_path)],stdin=slave,stdout=slave,stderr=slave)
    os.close(slave)
    output=b''
    try:
        deadline=time.monotonic()+5
        while '等待你的确认'.encode() not in output:
            assert time.monotonic()<deadline,output.decode(errors='replace')
            if select.select([master],[],[],0.1)[0]:
                output+=os.read(master,65536)
        os.write(master,b'r\n')
        assert process.wait(timeout=5)==0
        assert '二次检查通过' in (tmp_path/'output.log').read_text()
    finally:
        if process.poll() is None:
            process.kill(); process.wait()
        os.close(master)


@pytest.mark.parametrize('code',[2,3])
def test_cancel_or_changed_evidence_exits_before_generic_fault(tmp_path,code):
    source=(ROOT/'scripts/daily_start_teleop_rgbd.sh').read_text()
    start=source.index('if [[ "$recover_power_cycle" != true ]] && {')
    end=source.index('if [[ "$recover_power_cycle" != true && "$motors_enabled" == true ]] && refresh_healthy_running_status;',start)
    setup=f'''set -euo pipefail
recover_power_cycle=false; motors_enabled=false; TELEOP_SERVICE=mock
systemctl() {{ return 0; }}
recover_restarted_peer() {{ return {code}; }}
'''
    result=subprocess.run(['bash','-c',setup+source[start:end]+'echo UNEXPECTED_FALLTHROUGH'],
                          text=True,capture_output=True,timeout=5)
    assert result.returncode==2
    assert 'UNEXPECTED_FALLTHROUGH' not in result.stdout
    assert '重新打开桌面启动器' in result.stdout

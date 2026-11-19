"""Offline deployment and read-only preflight tests. Never connects to robot."""
import importlib.util
from pathlib import Path
import sys
import json
from types import SimpleNamespace
import pytest
ROOT = Path(__file__).parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import left_only_device as device
import left_only_session as session
import verify_left_release as release
import install_left_only as installer
from left_only_alignment_fixture import status, POLICY, Clock, alignment


@pytest.fixture
def bus(monkeypatch):
    calls = []
    monkeypatch.setattr(device,'controller_owners',lambda: [])
    monkeypatch.setattr(device,'validate_bus',lambda role,bus: ('usb-fixed',4))
    def query(bus,motor,command,rid=0):
        calls.append((bus,motor,command,rid))
        if command==0x33: return bytes([motor,0,command,rid])+bytes.fromhex(device.FOLLOWER_LEFT_J1[rid])
        return bytes([motor])+b'\0'*7
    monkeypatch.setattr(device,'query',query)
    return calls


def test_preflight_only_queries_selected_left_bus(bus):
    assert device.check('follower','can2')['ok']
    assert len(bus)==28 and all(c[0]=='can2' and c[2] in (0x33,0xcc) for c in bus)
    assert {c[1] for c in bus}==set(range(1,9))


def test_existing_controller_prevents_any_can_query(bus,monkeypatch):
    monkeypatch.setattr(device,'controller_owners',lambda: [{'pid':123}])
    with pytest.raises(RuntimeError,match='控制进程'): device.check('leader','can1')
    assert not bus


def test_wrong_physical_arm_refused_before_status_queries(bus,monkeypatch):
    monkeypatch.setattr(device,'query',lambda *args: bytes(8))
    with pytest.raises(RuntimeError,match='健康左从臂'): device.check('follower','can1')


def test_status4_not_bypassed(bus,monkeypatch):
    monkeypatch.setattr(device,'query',lambda *args: b'\x41'+b'\0'*7)
    with pytest.raises(RuntimeError,match='状态4'): device.check('leader','can1')


@pytest.mark.parametrize('phase',['aligning','following'])
def test_left_alignment_requires_healthy_left_not_right(phase):
    s = status(phase); s.update(mode='left_only',enabled_arms=['left'],motor_health={'healthy':True})
    s['collection']['right_mode']='DISABLED'
    for key in list(s):
        if 'right' in key: s[key]=None
    c=Clock()
    result=alignment.verify(lambda:s,phase,POLICY,c.time,c.sleep,arms=('left',))
    assert result['accepted'] and len(result['joints'])==7
    s['motor_health']['healthy']=False
    assert not alignment.assess(s,phase,POLICY,('left',))['ok']


def test_launch_failure_requests_hold_never_disable_or_release(monkeypatch):
    monkeypatch.setattr(session,'verify_versions',lambda c: None)
    monkeypatch.setattr(session,'prepare_start',lambda c, supported=False: None)
    calls=[]
    monkeypatch.setattr(session,'preflight',lambda c: {})
    monkeypatch.setattr(session,'remote',lambda c,args,*a: calls.append(args) or '')
    monkeypatch.setattr(session,'run',lambda args,*a: calls.append(args) or '')
    monkeypatch.setattr(session,'await_startup_hold',lambda c: (_ for _ in ()).throw(RuntimeError('homing failed')))
    monkeypatch.setattr(session,'control',lambda c,command: calls.append(command) or {})
    monkeypatch.setattr(session,'ros_call',lambda *a: pytest.fail('no release/disable on failed startup'))
    with pytest.raises(RuntimeError,match='homing failed'): session.start({})
    assert 'hold' in calls and 'disable' not in calls


def test_wrong_version_starts_nothing(monkeypatch):
    monkeypatch.setattr(session,'verify_versions',lambda c: None)
    monkeypatch.setattr(session,'prepare_start',lambda c, supported=False: None)
    monkeypatch.setattr(session,'preflight',lambda c: (_ for _ in ()).throw(RuntimeError('version mismatch')))
    monkeypatch.setattr(session,'remote',lambda *a: pytest.fail('no service action'))
    monkeypatch.setattr(session,'run',lambda *a: pytest.fail('no service action'))
    with pytest.raises(RuntimeError): session.start({})


def test_manifest_detects_source_and_binary_edits(tmp_path):
    src=tmp_path/'source.py'; src.write_text('source')
    binary=tmp_path/'binary'; binary.write_text('built')
    (tmp_path/'release.sha256').write_text(release.digest(src)+'  source.py\n')
    (tmp_path/'installed.sha256').write_text(release.digest(binary)+'  binary\n')
    (tmp_path/'built_from.sha256').write_text(release.digest(tmp_path/'release.sha256'))
    release.verify(tmp_path)
    binary.write_text('edited')
    with pytest.raises(RuntimeError): release.verify(tmp_path)
    release.verify(tmp_path,built=False)
    src.write_text('edited')
    with pytest.raises(RuntimeError): release.verify(tmp_path,built=False)


def test_generated_services_do_not_restart_or_autostart():
    for role,bus in [('leader','can1'),('follower','can2')]:
        unit=installer.unit_text(Path('/tmp/update'),role,bus,'192.168.50.2')
        assert f'{role} {bus}' in unit and 'Restart=no' in unit
        assert '[Install]' not in unit and 'WantedBy' not in unit


@pytest.mark.parametrize('role,bus',[('leader','can1'),('follower','can2')])
def test_systemd_accepts_generated_service(tmp_path,role,bus):
    import subprocess
    directory=tmp_path/'path with spaces'
    directory.mkdir()
    unit=tmp_path/('openarm-left-'+role+'.service')
    unit.write_text(installer.unit_text(directory,role,bus,'192.168.50.2'))
    result=subprocess.run(['systemd-analyze','--user','verify',str(unit)],capture_output=True,text=True)
    assert result.returncode==0, result.stderr

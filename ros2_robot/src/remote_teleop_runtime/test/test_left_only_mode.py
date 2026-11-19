"""Left-only behavior: real gateway methods, synthetic feedback, no hardware."""
from types import SimpleNamespace
import struct
import threading
import pytest
from sensor_msgs.msg import JointState
from test_follower_hold_continuity import gateway as follower_gateway
from test_leader_collection_routing import gateway, state
from remote_teleop_protocol import decode_message
from remote_teleop_runtime.left_only_support import MotorEvidence, decode_drive, LEFT_ACTION_PORT


def left_follower(tmp_path):
    n = follower_gateway(tmp_path)
    n.left_only = True
    n.have_right_feedback = False
    n.run_leader_right = None
    n.motor_health = SimpleNamespace(healthy=lambda: True)
    n.motor_health.report = lambda: {"healthy": n.motor_health.healthy(), "armed": True, "fault": None}
    n.right_publisher = SimpleNamespace(publish=lambda _: pytest.fail('right output'))
    return n


def test_follower_tracks_without_any_right_state_and_holds_on_fault(tmp_path):
    n = left_follower(tmp_path)
    assert n.have_feedback and n.run_reference_ready()
    assert n.publish_target(1_000_000_000)
    assert n.command_was_running and n.left_messages
    before = n.applied_axes
    assert before[8:] == (0.,)*8 and n.last_target_right is None and n.hold_right is None
    n.motor_health.healthy = lambda: False
    n.positions[3] -= .1
    n.latest_action.left_arm = (.5,)*7
    assert n.publish_target(1_001_000_000)
    assert not n.command_was_running
    assert n.applied_axes == before


def test_follower_ignores_right_only_or_incomplete_joint_messages(tmp_path):
    n = left_follower(tmp_path); n.lock = threading.Lock()
    n.have_left_feedback = False
    n.last_left_feedback_ns = n.last_feedback_ns = 0
    msg = JointState()
    msg.name = [f'openarm_right_joint{i}' for i in range(1,8)]+['openarm_right_finger_joint1']
    msg.position = msg.velocity = msg.effort = [0.]*8
    n.on_joint_state(msg)
    assert not n.have_feedback and n.last_feedback_ns == 0
    msg.name = [f'openarm_left_joint{i}' for i in range(1,8)]
    msg.position = msg.velocity = msg.effort = [0.]*7
    n.on_joint_state(msg)
    assert not n.have_feedback


def test_leader_uses_only_left_and_isolated_port(gateway):
    n = gateway; n.left_only = True; n.have_right = False; n.right_rx = 0
    n.action_port = LEFT_ACTION_PORT
    n.motor_health = SimpleNamespace(healthy=lambda: True)
    n.motor_health.report = lambda: {"healthy": n.motor_health.healthy(), "armed": True, "fault": None}
    n.tick()
    assert n.sent == 1
    packet, address = n.sock.sent[0]
    assert address[1] == 50110 and decode_message(packet).collection_ack == 0
    n.publish_return(state(23))
    assert not n.return_pub.messages and not n.left_return_pub.messages
    n.motor_health.healthy = lambda: False
    n.tick()
    assert n.sent == 1


def fill(e, at, state=1):
    for _ in range(3):
        for motor in range(1,9): e.observe(motor,state,at)


@pytest.mark.parametrize('failure', ['disabled', 'state4', 'stale', 'missing'])
def test_motor_health_is_latched_and_never_auto_recovers(failure):
    e = MotorEvidence(); fill(e,10.); assert e.healthy(10.01)
    if failure == 'disabled': e.observe(1,0,10.02)
    elif failure == 'state4': e.observe(1,4,10.02)
    elif failure == 'missing': e.samples[8].clear()
    now = 10.3 if failure == 'stale' else 10.02
    assert not e.healthy(now)
    fill(e,11.)
    assert not e.healthy(11.) and e.fault


def test_monitor_requires_all_eight_enabled_with_three_fresh_samples():
    e = MotorEvidence(); fill(e,1.,0); assert not e.healthy(1.)
    for i in range(1,8):
        for _ in range(3): e.observe(i,1,2.)
    assert not e.healthy(2.)
    for _ in range(3): e.observe(8,1,2.)
    assert e.healthy(2.)


@pytest.mark.parametrize('cid,first,expected', [(0x11,0x41,(1,4)),(0x18,0x18,(8,1)),
    (0x11,0x12,None),(0x19,0x19,None),(0x80000011,0x11,None)])
def test_decode_exact_motor_identity(cid,first,expected):
    packet = struct.pack('=IBBBB8s',cid,8,0,0,0,bytes([first])+b'\0'*7)
    assert decode_drive(packet) == expected


def test_follower_reports_can_failure_even_with_fresh_ros_feedback(tmp_path):
    n=left_follower(tmp_path)
    n.align_since_ns=0
    n.heartbeat=lambda _: None
    n.send_state=lambda _: None
    n.motor_health.healthy=lambda: False
    reports=[]
    n.io=SimpleNamespace(control_completed=lambda *args: reports.append(args))
    n.tick()
    assert reports and reports[0][2]==0 and reports[0][3] is True


@pytest.mark.parametrize('role,bus',[('leader','can1'),('follower','can2'),('follower','can1')])
def test_launch_keeps_left_kinematics_and_disables_right(role,bus):
    from remote_teleop_runtime.left_only_launch import controller_parameters
    p=controller_parameters(role,bus,'robot.urdf','limits.yaml','role.yaml',[0.,0.,0.,.35,0.,0.,-.18])[-1]
    assert p['enable_left'] is True and p['enable_right'] is False
    assert p['left_arm_can']==bus and p['right_arm_can']=='__RIGHT_DISABLED__'
    assert p['startup_home'] is True
    assert p['startup_hold_current'] is False
    assert p['startup_home_target']==[0.,0.,0.,.35,0.,0.,-.18]
    assert p['force_feedback_enabled'] is (role=='leader')


def test_faulted_watchdog_prevents_any_controller_launch(monkeypatch):
    from launch import LaunchContext
    from remote_teleop_runtime import left_only_launch as launch
    c=LaunchContext(); c.launch_configurations['left_can']='can2'
    monkeypatch.setattr(launch,'UnixDatagramClient',lambda _: SimpleNamespace(close=lambda:None))
    monkeypatch.setattr(launch,'safety_command',lambda *a: {'state':'FAULT','fault_bits':1})
    monkeypatch.setattr(launch,'Node',lambda **k: pytest.fail('fault must not start controllers'))
    with pytest.raises(RuntimeError,match='not ALIGNING'): launch.start_nodes(c,'follower')


def test_left_leader_rejects_right_can_before_any_nodes(monkeypatch):
    from launch import LaunchContext
    from remote_teleop_runtime import left_only_launch as launch
    c=LaunchContext(); c.launch_configurations['left_can']='can0'
    monkeypatch.setattr(launch,'Node',lambda **k: pytest.fail('wrong CAN must not start'))
    with pytest.raises(RuntimeError,match='unsupported'): launch.start_nodes(c,'leader')


def test_initial_health_sampling_preserves_grace_but_real_fault_does_not(tmp_path):
    n=left_follower(tmp_path); n.align_since_ns=0
    n.heartbeat=lambda _:None; n.send_state=lambda _:None
    evidence={'healthy':False,'armed':False,'fault':None}
    n.motor_health.report=lambda:evidence
    n.motor_health.healthy=lambda:evidence['healthy']
    reports=[]
    n.io=SimpleNamespace(control_completed=lambda *a: reports.append(a))
    n.tick()
    assert not reports  # No heartbeat asserting false health or early CAN fault.
    evidence['fault']='motor 1 abnormal state 4'
    n.tick()
    assert reports[0][2]==0

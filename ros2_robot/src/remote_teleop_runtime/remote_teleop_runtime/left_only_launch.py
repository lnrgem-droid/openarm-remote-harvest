"""Launch only physical LEFT controllers; right hardware is never constructed."""
import os
import json
import math
import tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .common import UnixDatagramClient, safety_command
from .left_only_support import LEFT_WATCHDOG_SOCKET
from .collection_motion import LEFT_LIMITS


def load_common_home(path):
    with open(path) as file:
        policy = json.load(file)
    home = policy.get('home_rad')
    if (policy.get('startup_mode') != 'common_natural' or policy.get('reference_arm') != 'leader'
            or not isinstance(home, list) or len(home) != 7
            or not all(type(q) in (int,float) and math.isfinite(q) and low <= q <= high
                       for q,(low,high) in zip(home,LEFT_LIMITS[:7]))):
        raise ValueError('单左臂共同垂落初始位未配置或超出关节限位，未启动')
    return home


def controller_parameters(role, bus, urdf, limits, config, home=None):
    if home is None:
        raise ValueError('缺少已保存的共同垂落初始位，未启动')
    return [config, {'enable_right': False, 'enable_left': True,
        'right_arm_can': '__RIGHT_DISABLED__', 'left_arm_can': bus,
        'urdf_path': urdf, 'joint_limits_path': limits,
        'startup_home': True, 'startup_hold_current': False,
        'startup_home_target': list(home), 'startup_home_duration_s': 4.0,
        'startup_home_timeout_s': 20.0, 'startup_home_tolerance_rad': 0.015,
        'startup_tracking_assist_enabled': True,
        'startup_use_tracking_gains': role == 'follower',
        'tracking_assist_enabled': role == 'follower',
        'force_feedback_enabled': role == 'leader',
        'gripper_contact_feedback': role == 'leader',
        'force_feedback_scale': 1.0, 'force_feedback_filter_alpha': .12,
        'bilateral_position_feedback_enabled': False,
        # Explicit names preserve left kinematics and dataset semantics.
        'collection_left_return_topic': '/leader/left_arm/collection_return'}]


def start_nodes(context, role):
    bus = LaunchConfiguration('left_can').perform(context)
    if bus not in ({'can1'} if role == 'leader' else {'can1', 'can2'}):
        raise RuntimeError('unsupported physical left CAN interface')
    if role == 'follower':
        probe = UnixDatagramClient(LEFT_WATCHDOG_SOCKET)
        try: status = safety_command(probe, 'status')
        finally: probe.close()
        if not status or status.get('state') != 'ALIGNING' or status.get('fault_bits') != 0:
            raise RuntimeError('fresh isolated watchdog is not ALIGNING; no controller started')
    home = load_common_home(os.environ['OPENARM_LEFT_STARTUP_POLICY'])
    desc = get_package_share_directory('openarm_description')
    rt = get_package_share_directory('remote_teleop_runtime')
    pd = get_package_share_directory('openarm_gravity_pd_control')
    doc = xacro.process_file(os.path.join(desc, 'urdf/robot/v10.urdf.xacro'), mappings={'bimanual': 'true'})
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', prefix='openarm_left_', delete=False) as f:
        f.write(doc.toxml()); urdf = f.name
    limits = os.path.join(desc, 'config/arm/v10/joint_limits.yaml')
    args = ['--left-only', '--health-bus', bus, '--rate', '250']
    if role == 'leader': args += ['--peer', LaunchConfiguration('peer')]
    return [
        Node(package='openarm_gravity_pd_control', executable='openarm_gravity_pd_node',
             parameters=[os.path.join(pd, "config/control_params.yaml")] + controller_parameters(role, bus, urdf, limits,
                 os.path.join(rt, 'config', f'bimanual_{role}.yaml'), home),
             name=f'{role}_gravity_pd', output='screen'),
        Node(package='remote_teleop_runtime', executable=f'remote-teleop-{role}',
             arguments=args, name=f'left_{role}_gateway', output='screen'),
    ]


def description(role):
    actions = [SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '71'),
        DeclareLaunchArgument('left_can', default_value='can1' if role == 'leader' else 'can2'),
        DeclareLaunchArgument('peer', default_value='192.168.50.2')]
    if role == 'follower':
        actions += [Node(package='remote_teleop_follower_safety',
            executable='remote-teleop-follower-watchdog', output='screen',
            arguments=['--socket', LEFT_WATCHDOG_SOCKET, '--verified-reaction', 'position_hold',
                       '--startup-grace-s', '35', '--network-action-timeout-ms', '150']),
            TimerAction(period=1., actions=[OpaqueFunction(function=lambda c: start_nodes(c, role))])]
    else: actions += [OpaqueFunction(function=lambda c: start_nodes(c, role))]
    return LaunchDescription(actions)

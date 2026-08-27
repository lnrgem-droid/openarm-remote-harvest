import os, tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _urdf():
    share = get_package_share_directory("openarm_description")
    doc = xacro.process_file(os.path.join(share, "urdf", "robot", "v10.urdf.xacro"), mappings={"bimanual":"true"})
    f = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".urdf", prefix="openarm_remote_")
    f.write(doc.toxml()); f.close()
    return f.name, os.path.join(share, "config", "arm", "v10", "joint_limits.yaml")

def _nodes(context):
    urdf, limits = _urdf()
    pd_share = get_package_share_directory("openarm_gravity_pd_control")
    rt_share = get_package_share_directory("remote_teleop_runtime")
    verified = LaunchConfiguration("reaction_verified").perform(context).lower() == "true"
    debug_controller = LaunchConfiguration("debug_controller").perform(context).lower() == "true"
    watchdog_args = []
    if verified: watchdog_args += ["--verified-reaction", "position_hold"]
    controller_prefix = None
    if debug_controller:
        controller_prefix = [
            "gdb", "-q", "--batch",
            "-ex", "run",
            "-ex", "thread apply all bt",
            "--args",
        ]
    return [
        Node(package="remote_teleop_follower_safety",
             executable="remote-teleop-follower-watchdog",
             arguments=watchdog_args, output="screen"),
        Node(package="openarm_gravity_pd_control", executable="openarm_gravity_pd_node",
             parameters=[os.path.join(pd_share,"config","control_params.yaml"),
                         os.path.join(rt_share,"config","right_follower.yaml"),
                         {"urdf_path":urdf,"joint_limits_path":limits}],
             prefix=controller_prefix, output="screen"),
        Node(package="remote_teleop_runtime", executable="remote-teleop-follower", output="screen"),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("reaction_verified", default_value="false",
            description="Set true only after the supervised command-refresh hold test passes"),
        DeclareLaunchArgument("debug_controller", default_value="false",
            description="Run the controller under gdb and print a backtrace if it exits"),
        OpaqueFunction(function=_nodes),
    ])

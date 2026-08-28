import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, Shutdown, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from remote_teleop_runtime.common import UnixDatagramClient, WATCHDOG_SOCKET, safety_command


def _urdf():
    share = get_package_share_directory("openarm_description")
    doc = xacro.process_file(
        os.path.join(share, "urdf", "robot", "v10.urdf.xacro"), mappings={"bimanual": "true"})
    output = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".urdf", prefix="openarm_remote_")
    output.write(doc.toxml())
    output.close()
    return output.name, os.path.join(share, "config", "arm", "v10", "joint_limits.yaml")


def _start_control_stack(context):
    urdf, limits = _urdf()
    pd_share = get_package_share_directory("openarm_gravity_pd_control")
    rt_share = get_package_share_directory("remote_teleop_runtime")
    debug_controller = LaunchConfiguration("debug_controller").perform(context).lower() == "true"
    probe = UnixDatagramClient(WATCHDOG_SOCKET)
    try:
        status = safety_command(probe, "status")
    finally:
        probe.close()
    if not status or status.get("state") not in {"INIT", "ALIGNING", "READY", "RUNNING", "FAULT", "E_STOP"}:
        return [
            LogInfo(msg="ERROR: follower watchdog did not return a valid health response; control stack was not started."),
            Shutdown(reason="follower watchdog health gate failed"),
        ]
    controller_prefix = "gdb -q --batch -ex run -ex 'thread apply all bt' --args" if debug_controller else None
    return [
        Node(package="openarm_gravity_pd_control", executable="openarm_gravity_pd_node",
             parameters=[os.path.join(pd_share, "config", "control_params.yaml"),
                         os.path.join(rt_share, "config", "bimanual_follower.yaml"),
                         {"urdf_path": urdf, "joint_limits_path": limits}],
             name="follower_gravity_pd", prefix=controller_prefix, output="screen"),
        Node(package="remote_teleop_runtime", executable="remote-teleop-follower",
             arguments=["--enable-left"], name="follower_gateway", output="screen"),
    ]


def _launch_actions(context):
    verified = LaunchConfiguration("reaction_verified").perform(context).lower() == "true"
    watchdog_args = ["--verified-reaction", "position_hold"] if verified else []
    return [
        Node(package="remote_teleop_follower_safety",
             executable="remote-teleop-follower-watchdog",
             arguments=watchdog_args, output="screen"),
        TimerAction(period=1.0, actions=[OpaqueFunction(function=_start_control_stack)]),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("reaction_verified", default_value="false",
            description="Set true only after the supervised command-refresh hold test passes"),
        DeclareLaunchArgument("debug_controller", default_value="false",
            description="Run the controller under gdb and print a backtrace if it exits"),
        OpaqueFunction(function=_launch_actions),
    ])

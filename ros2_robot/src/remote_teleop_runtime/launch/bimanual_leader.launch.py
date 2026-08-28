import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _urdf():
    share = get_package_share_directory("openarm_description")
    doc = xacro.process_file(
        os.path.join(share, "urdf", "robot", "v10.urdf.xacro"), mappings={"bimanual": "true"})
    output = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".urdf", prefix="openarm_remote_")
    output.write(doc.toxml())
    output.close()
    return output.name, os.path.join(share, "config", "arm", "v10", "joint_limits.yaml")


def generate_launch_description():
    urdf, limits = _urdf()
    pd_share = get_package_share_directory("openarm_gravity_pd_control")
    rt_share = get_package_share_directory("remote_teleop_runtime")
    peer = LaunchConfiguration("peer")
    startup_home = LaunchConfiguration("startup_home")
    force_feedback = LaunchConfiguration("force_feedback")
    return LaunchDescription([
        # UDP is the sole cross-host data transport; ROS control remains local.
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("peer", default_value="192.168.50.2"),
        DeclareLaunchArgument("startup_home", default_value="true",
            description="Explicitly move both leader arms to existing encoder q=0 at startup"),
        DeclareLaunchArgument("force_feedback", default_value="false",
            description="Enable bounded bilateral virtual-force feedback from follower tracking error"),
        Node(package="openarm_gravity_pd_control", executable="openarm_gravity_pd_node",
             parameters=[os.path.join(pd_share, "config", "control_params.yaml"),
                         os.path.join(rt_share, "config", "bimanual_leader.yaml"),
                         {"urdf_path": urdf, "joint_limits_path": limits,
                          "startup_home": startup_home,
                          "force_feedback_enabled": force_feedback,
                          "force_feedback_scale": 1.0,
                          "force_feedback_filter_alpha": 0.12,
                          "bilateral_position_feedback_enabled": False}],
             name="leader_gravity_pd", output="screen"),
        Node(package="remote_teleop_runtime", executable="remote-teleop-leader",
             arguments=["--peer", peer, "--rate", "250", "--enable-left"],
             name="leader_gateway", output="screen"),
    ])

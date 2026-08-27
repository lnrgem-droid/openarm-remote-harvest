import os, tempfile
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _urdf():
    share = get_package_share_directory("openarm_description")
    doc = xacro.process_file(os.path.join(share, "urdf", "robot", "v10.urdf.xacro"), mappings={"bimanual":"true"})
    f = tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".urdf", prefix="openarm_remote_")
    f.write(doc.toxml()); f.close()
    return f.name, os.path.join(share, "config", "arm", "v10", "joint_limits.yaml")

def generate_launch_description():
    urdf, limits = _urdf()
    pd_share = get_package_share_directory("openarm_gravity_pd_control")
    rt_share = get_package_share_directory("remote_teleop_runtime")
    peer = LaunchConfiguration("peer")
    return LaunchDescription([
        # UDP is the only cross-machine transport.  Keeping ROS discovery local
        # prevents this leader from ever consuming Jetson feedback or commands.
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("peer", default_value="192.168.50.2"),
        Node(package="openarm_gravity_pd_control", executable="openarm_gravity_pd_node",
             parameters=[os.path.join(pd_share,"config","control_params.yaml"),
                         os.path.join(rt_share,"config","right_leader.yaml"),
                         {"urdf_path":urdf,"joint_limits_path":limits}],
             name="leader_gravity_pd", output="screen"),
        Node(package="remote_teleop_runtime", executable="remote-teleop-leader",
             arguments=["--peer", peer, "--rate", "100"],
             name="leader_gateway", output="screen"),
    ])

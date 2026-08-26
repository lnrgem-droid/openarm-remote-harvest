"""启动 robot_bridge 桥接节点。

所有参数（接口名 / launch / 限位 / host / port / 频率 / ttl / arm_mode）
统一来自 config_file 指向的 YAML。要改行为优先改 YAML；
临时覆盖单个服务器参数可用：
    ros2 launch robot_bridge bridge.launch.py config_file:=/path/other.yaml
    ros2 run robot_bridge bridge_node --ros-args -p port:=9001
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("robot_bridge"), "config", "bridge_config.yaml")

    config_arg = DeclareLaunchArgument(
        "config_file", default_value=default_config,
        description="集中配置文件路径（见 config/bridge_config.yaml）")
    arm_mode_arg = DeclareLaunchArgument(
        "arm_mode", default_value="trajectory",
        description="双臂控制模式",
        choices=["trajectory", "gravity_pd", "cartesian"])

    node = Node(
        package="robot_bridge",
        executable="bridge_node",
        name="robot_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "arm_mode": LaunchConfiguration("arm_mode"),
        }],
    )

    return LaunchDescription([config_arg, arm_mode_arg, node])

#!/usr/bin/env python3
"""急停节点 launch 文件"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_arg = DeclareLaunchArgument(
        'can_interfaces',
        default_value='["can0", "can1", "can2", "can3"]',
        description='CAN 接口列表（逗号分隔）',
    )

    emergency_stop_node = Node(
        package='emergency_stop',
        executable='emergency_stop_node',
        name='emergency_stop_node',
        output='screen',
        emulate_tty=True,  # 使键盘输入可用
        parameters=[{
            'can_interfaces': ['can0', 'can1', 'can2', 'can3'],
        }],
    )

    return LaunchDescription([
        can_arg,
        emergency_stop_node,
    ])

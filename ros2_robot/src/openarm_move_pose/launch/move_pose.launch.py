from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration("rate")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rate", default_value="5.0"),
            Node(
                package="openarm_move_pose",
                executable="move_pose_node",
                name="move_pose_node",
                output="screen",
            ),
            Node(
                package="openarm_move_pose",
                executable="ee_pose_publisher",
                name="ee_pose_publisher",
                output="screen",
                parameters=[{"rate": rate}],
            ),
        ]
    )

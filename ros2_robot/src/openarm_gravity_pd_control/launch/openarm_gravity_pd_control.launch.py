# Copyright 2025 OpenArm Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
Launch file for openarm_gravity_pd_control.

Steps performed at launch:
  1. Generate a bimanual URDF from openarm_description/urdf/robot/v10.urdf.xacro
  2. Write the URDF to a temporary file
  3. Start openarm_gravity_pd_node with the URDF path and control parameters
     (publishes /joint_states from CAN feedback by default)

Usage examples:
  # Default (can0=right, can1=left):
  ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py

  # Override CAN interfaces or gravity scale:
  ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py \\
      right_arm_can:=can0 left_arm_can:=can1 grav_scale:=0.90

  # Tune gripper open angle (negative = open direction, default -1.0472 rad ≈ -60 deg):
  ros2 launch openarm_gravity_pd_control openarm_gravity_pd_control.launch.py \\
      gripper_max_rad:=-1.0472
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ───────────────────────────────────────────────────
    right_can_arg = DeclareLaunchArgument(
        "right_arm_can",
        default_value="can0",
        description="SocketCAN interface name for the right arm",
    )
    left_can_arg = DeclareLaunchArgument(
        "left_arm_can",
        default_value="can1",
        description="SocketCAN interface name for the left arm",
    )
    grav_scale_arg = DeclareLaunchArgument(
        "grav_scale",
        default_value="0.95",
        description="Gravity torque scale factor (< 1.0 avoids upward drift)",
    )
    gripper_max_rad_arg = DeclareLaunchArgument(
        "gripper_max_rad",
        default_value="-1.0472",
        description=(
            "Open position of the gripper motor [rad] (negative = open direction). "
            "Matches openarm_hardware convention: closed=0.0, open=-1.0472 rad (~-60 deg). "
            "Normalized trigger input: 0.0=closed(0 rad), 1.0=open(gripper_max_rad)."
        ),
    )
    publish_joint_states_arg = DeclareLaunchArgument(
        "publish_joint_states",
        default_value="true",
        description="Publish CAN feedback as /joint_states",
    )
    joint_states_rate_arg = DeclareLaunchArgument(
        "joint_states_rate",
        default_value="100.0",
        description="Publishing rate for /joint_states [Hz]",
    )

    # ── Generate bimanual URDF from xacro ──────────────────────────────────
    description_share = get_package_share_directory("openarm_description")
    xacro_file = os.path.join(
        description_share, "urdf", "robot", "v10.urdf.xacro",
    )
    urdf_doc = xacro.process_file(xacro_file, mappings={"bimanual": "true"})
    urdf_content = urdf_doc.toxml()

    tmp = tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".urdf", prefix="openarm_bimanual_"
    )
    tmp.write(urdf_content)
    tmp.close()
    urdf_path = tmp.name

    # ── Joint limits yaml (single source of truth, shared with URDF/MoveIt) ─
    joint_limits_path = os.path.join(
        description_share, "config", "arm", "v10", "joint_limits.yaml",
    )

    # ── Config file ────────────────────────────────────────────────────────
    config_file = os.path.join(
        get_package_share_directory("openarm_gravity_pd_control"),
        "config", "control_params.yaml",
    )

    # ── Node ───────────────────────────────────────────────────────────────
    node = Node(
        package="openarm_gravity_pd_control",
        executable="openarm_gravity_pd_node",
        name="openarm_gravity_pd_node",
        output="screen",
        parameters=[
            config_file,
            {
                "urdf_path":       urdf_path,
                "joint_limits_path": joint_limits_path,
                "right_arm_can":   LaunchConfiguration("right_arm_can"),
                "left_arm_can":    LaunchConfiguration("left_arm_can"),
                "grav_scale":      LaunchConfiguration("grav_scale"),
                "gripper_max_rad": LaunchConfiguration("gripper_max_rad"),
                "publish_joint_states": LaunchConfiguration("publish_joint_states"),
                "joint_states_rate": LaunchConfiguration("joint_states_rate"),
            },
        ],
    )

    return LaunchDescription([
        right_can_arg,
        left_can_arg,
        grav_scale_arg,
        gripper_max_rad_arg,
        publish_joint_states_arg,
        joint_states_rate_arg,
        node,
    ])

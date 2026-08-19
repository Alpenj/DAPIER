# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_contract = PathJoinSubstitution(
        [FindPackageShare("dapier_so101_core"), "config", "so101_joint_contract.yaml"]
    )
    default_parameters = PathJoinSubstitution(
        [FindPackageShare("dapier_so101_teleop"), "config", "safe_teleop.yaml"]
    )

    joint_config_file = LaunchConfiguration("joint_config_file")
    parameter_file = LaunchConfiguration("parameter_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_config_file",
                default_value=default_contract,
                description="Versioned SO-101 joint order and safety limits",
            ),
            DeclareLaunchArgument(
                "parameter_file",
                default_value=default_parameters,
                description="Safe teleoperation ROS parameters",
            ),
            Node(
                package="dapier_so101_teleop",
                executable="safe_leader_follower",
                name="safe_leader_follower",
                output="screen",
                parameters=[parameter_file, {"joint_config_file": joint_config_file}],
            ),
        ]
    )

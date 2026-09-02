from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_parameters = str(
        Path(get_package_share_directory("dapier_safety_bridge"))
        / "config"
        / "safety_bridge.yaml"
    )
    parameter_file = LaunchConfiguration("parameter_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameter_file",
                default_value=default_parameters,
                description="Safety bridge parameter YAML",
            ),
            Node(
                package="dapier_safety_bridge",
                executable="dapier_safety_bridge_node",
                name="dapier_safety_bridge",
                output="screen",
                parameters=[parameter_file],
            ),
        ]
    )

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ros_arm'))
    robot_description = (share / 'urdf' / 'ros_arm.urdf').read_text()

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Arduino Uno serial device',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='ros_arm',
            executable='ros_arm_control',
            name='robot_arm_serial_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'center_angle': 90,
                'minimum_angle': 60,
                'maximum_angle': 120,
                'deadband_degrees': 2,
                'command_rate_hz': 15.0,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'ros_arm.rviz')],
            output='screen',
        ),
    ])

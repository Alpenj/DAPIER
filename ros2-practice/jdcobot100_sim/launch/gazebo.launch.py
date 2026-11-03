from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from jdcobot100_sim.gazebo_description import build_gazebo_description


def generate_launch_description():
    share = Path(get_package_share_directory('jdcobot100_sim'))
    gazebo_share = Path(get_package_share_directory('ros_gz_sim'))
    controller_config = share / 'config' / 'gazebo_controllers.yaml'
    robot_description = build_gazebo_description(
        share / 'urdf' / 'jdcobot100.urdf',
        controller_config,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        gazebo,
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen',
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'jdcobot100',
                '-allow_renaming', 'true',
            ],
            output='screen',
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'arm_position_controller',
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                ),
            ],
        ),
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='jdcobot100_sim',
                    executable='jdcobot100_auto_joint_publisher',
                    name='gazebo_auto_joint_publisher',
                    parameters=[{
                        'period_seconds': 1.0,
                        'amplitude_degrees': 10.0,
                        'publish_joint_states': False,
                        'publish_controller_commands': True,
                        'smooth_commands': True,
                    }],
                    output='screen',
                ),
            ],
        ),
    ])

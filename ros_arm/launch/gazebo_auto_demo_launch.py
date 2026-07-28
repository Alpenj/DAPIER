from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ros_arm'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    controller_config = share / 'config' / 'gazebo_controllers.yaml'
    robot_description = (
        share / 'urdf' / 'ros_arm_gazebo.urdf'
    ).read_text().replace('__CONTROLLER_CONFIG__', str(controller_config))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / 'launch' / 'gazebo.launch.py')),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ros_arm',
        ],
        output='screen',
    )

    load_controllers = TimerAction(
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
    )

    auto_test = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_arm',
                executable='ros_arm_auto_joint_publisher',
                name='gazebo_auto_joint_publisher',
                parameters=[{
                    'period_seconds': 1.0,
                    'amplitude_degrees': 10.0,
                    'publish_joint_states': False,
                    'publish_controller_commands': True,
                }],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        load_controllers,
        auto_test,
    ])

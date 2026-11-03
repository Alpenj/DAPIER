import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# NOTE (2026-08-10 ported to gz-sim / Harmonic):
# This ROS2 Jazzy install has no `gazebo_ros` / gzserver / gzclient at all
# -- Jazzy dropped Gazebo Classic support entirely -- so the original
# gzserver.launch.py / gzclient.launch.py / spawn_entity.py pipeline below
# was replaced with the gz-sim + ros_gz_sim + ros_gz_bridge equivalent
# (mirrors turtlebot3_gazebo's launch files, which are proven to work on
# this same machine).


def generate_launch_description():
    gazebo_pkg_name = 'ros_dd_gazebo'
    description_pkg_name = 'ros_dd_description'

    gazebo_pkg_share_dir = get_package_share_directory(gazebo_pkg_name)
    description_pkg_share_dir = get_package_share_directory(description_pkg_name)
    ros_gz_sim_share_dir = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ====================================================================
    # 1. URDF 파일 경로 정의 및 로봇 설명 매개변수 설정 (변경 없음)
    # ====================================================================
    urdf_path = os.path.join(description_pkg_share_dir, 'urdf', 'ros_dd.urdf')
    try:
        with open(urdf_path, 'r') as infp:
            robot_desc_data = infp.read()
    except EnvironmentError:
        print(f"ERROR: Cannot find URDF file at {urdf_path}")
        exit(1)
    robot_description = {'robot_description': robot_desc_data}

    # ====================================================================
    # 2. gz-sim이 model://hexa 를 찾을 수 있도록 리소스 경로 등록
    #    (GAZEBO_MODEL_PATH -> GZ_SIM_RESOURCE_PATH)
    # ====================================================================
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(gazebo_pkg_share_dir, 'models')
    )

    world_file = os.path.join(gazebo_pkg_share_dir, 'worlds', 'ros_dd.world')

    # ====================================================================
    # 3. 노드 실행 (gz-sim 서버/클라이언트, robot_state_publisher, spawner, bridge)
    # ====================================================================
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # URDF를 robot_description 토픽에서 읽어와 스폰 (원래 -topic robot_description 그대로 유지)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ros_dd_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.5',
            '-z', '0.2'
        ],
        output='screen'
    )

    bridge_params = os.path.join(gazebo_pkg_share_dir, 'params', 'ros_dd_bridge.yaml')
    start_gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    return LaunchDescription([
        set_env_vars_resources,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot_node,
        start_gazebo_bridge,
    ])

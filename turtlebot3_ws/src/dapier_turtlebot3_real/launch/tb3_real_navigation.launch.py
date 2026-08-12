"""Navigation2 launch for the physical Humble TurtleBot3 Burger from Jazzy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    real_share = get_package_share_directory("dapier_turtlebot3_real")
    tb3_share = get_package_share_directory("turtlebot3_navigation2")
    nav2_share = get_package_share_directory("nav2_bringup")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    real_params = RewrittenYaml(
        source_file=os.path.join(real_share, "param", "burger_real.yaml"),
        root_key="",
        param_rewrites={"yaml_filename": map_file},
        convert_types=True,
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "params_file": real_params,
            "use_sim_time": use_sim_time,
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(tb3_share, "rviz", "tb3_navigation2.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", description="Absolute path to a saved map YAML"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument("rviz", default_value="True"),
            nav2,
            rviz,
        ]
    )

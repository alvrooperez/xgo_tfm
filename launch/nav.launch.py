import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    repo_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    default_map = os.path.join(repo_dir, "maps", "room_map.yaml")
    default_params = os.path.join(repo_dir, "config", "nav2_params.yaml")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map yaml file to load"
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to the ROS2 parameters file to use for all launched nodes"
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "False",
            "autostart": "True",
            "use_composition": "True",
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        nav2_bringup
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('xgo_driver_cpp')
    default_params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path al archivo YAML de parametros'
    )

    driver_node = Node(
        package='xgo_driver_cpp',
        executable='xgo_driver_node',
        name='xgo_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        driver_node
    ])

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

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Puerto serie para la comunicacion con la MCU (ej. /dev/ttyAMA0 o /tmp/tty_robot)'
    )

    driver_node = Node(
        package='xgo_driver_cpp',
        executable='xgo_driver_node',
        name='xgo_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ]
    )

    return LaunchDescription([
        params_file_arg,
        serial_port_arg,
        driver_node
    ])

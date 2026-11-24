import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('xgo_behavior_tree')
    xml_path = os.path.join(pkg_share, 'config', 'bt_trees', 'rastreo_bt.xml')
    default_log_dir = os.path.join(tempfile.gettempdir(), 'xgo_bt_logs')

    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value=default_log_dir,
        description='Directory where BehaviorTree .btlog files are recorded'
    )

    # Configuración de los 3 robots de la brigada RASTREO
    robots = [
        {'name': 'robot1', 'groot_port': 1667},
        {'name': 'robot2', 'groot_port': 1668},
        {'name': 'robot3', 'groot_port': 1669},
    ]

    actions = [log_dir_arg]
    for r in robots:
        node = Node(
            package='xgo_behavior_tree',
            executable='bt_executor_node',
            name='bt_executor',
            namespace=r['name'],
            output='screen',
            parameters=[{
                'tree_xml_path': xml_path,
                'groot_port': r['groot_port'],
                'log_file': PathJoinSubstitution([
                    LaunchConfiguration('log_dir'),
                    f"ejecucion_{r['name']}.btlog"
                ])
            }]
        )
        actions.append(node)

    return LaunchDescription(actions)

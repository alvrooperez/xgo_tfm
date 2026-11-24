import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_xgo_sim = get_package_share_directory('xgo_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Paths
    world_path = os.path.join(pkg_xgo_sim, 'worlds', 'room.sdf')
    urdf_path = os.path.join(pkg_xgo_sim, 'urdf', 'xgo_mini2s.urdf.xacro')
    bridge_config_path = os.path.join(pkg_xgo_sim, 'config', 'bridge.yaml')
    nav2_params_path = os.path.join(pkg_xgo_sim, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(pkg_xgo_sim, 'maps', 'room_map.yaml')
    rviz_config_path = os.path.join(pkg_xgo_sim, 'rviz', 'nav2.rviz')

    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for navigation visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Robot Description
    robot_description_cmd = Command(['xacro ', urdf_path])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # Spawn Entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'xgo_mini2s',
            '-topic', 'robot_description',
            '-z', '0.05'
        ]
    )

    # ROS <-> Gazebo Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path
        }]
    )

    # Full Nav2 Bringup (Localization + Planner + Controller + Costmaps + BT Navigator)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params_path,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_composition': 'True'
        }.items()
    )

    # RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        map_arg,
        use_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        gz_bridge,
        nav2_bringup,
        rviz2_node
    ])

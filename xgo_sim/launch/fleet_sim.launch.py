import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_gz(context, *args, **kwargs):
    pkg_xgo_sim = get_package_share_directory('xgo_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_xgo_sim, 'worlds', 'room.sdf')

    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_args = f'-r -s {world_path}' if headless else f'-r {world_path}'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': gz_args}.items()
        )
    ]


def generate_launch_description():
    pkg_xgo_sim = get_package_share_directory('xgo_sim')

    urdf_path = os.path.join(pkg_xgo_sim, 'urdf', 'xgo_mini2s.urdf.xacro')
    bridge_config_path = os.path.join(pkg_xgo_sim, 'config', 'fleet_bridge.yaml')
    rviz_config_path = os.path.join(pkg_xgo_sim, 'rviz', 'sim.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless server mode (-s)'
    )

    launch_bt_arg = DeclareLaunchArgument(
        'launch_bt',
        default_value='true',
        description='Launch the 3 Behavior Tree executors for robot1, robot2, robot3'
    )

    simulate_heartbeat_arg = DeclareLaunchArgument(
        'simulate_heartbeat',
        default_value='true',
        description='Keep Wi-Fi heartbeat simulated internally in BT executors'
    )

    # Brigada RASTREO: 3 cuadrúpedos XGO Mini 2S en Gazebo Harmonic
    robots = [
        {
            'name': 'robot1',
            'x': '0.6',
            'y': '-0.2',
            'z': '0.05',
            'yaw': '0.0',
            'leg_rgba': '0.9 0.4 0.0 1.0',  # Naranja
        },
        {
            'name': 'robot2',
            'x': '0.0',
            'y': '0.0',
            'z': '0.05',
            'yaw': '0.0',
            'leg_rgba': '0.1 0.4 0.9 1.0',  # Azul
        },
        {
            'name': 'robot3',
            'x': '-0.6',
            'y': '-0.2',
            'z': '0.05',
            'yaw': '0.0',
            'leg_rgba': '0.1 0.8 0.3 1.0',  # Verde
        },
    ]

    actions = [
        use_rviz_arg,
        use_sim_time_arg,
        headless_arg,
        launch_bt_arg,
        simulate_heartbeat_arg,
        OpaqueFunction(function=launch_gz),
    ]

    for r in robots:
        robot_desc_cmd = Command([
            'xacro ', urdf_path,
            ' robot_ns:=', r['name'],
            ' leg_rgba:="', r['leg_rgba'], '"'
        ])
        robot_description = ParameterValue(robot_desc_cmd, value_type=str)

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=r['name'],
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': f"{r['name']}/",
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f"spawn_{r['name']}",
            output='screen',
            arguments=[
                '-name', r['name'],
                '-topic', f"/{r['name']}/robot_description",
                '-x', r['x'],
                '-y', r['y'],
                '-z', r['z'],
                '-Y', r['yaw']
            ]
        )

        actions.extend([rsp_node, spawn_node])

    # ROS <-> Gazebo Bridge para los 3 robots
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='fleet_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path
        }]
    )
    actions.append(gz_bridge)

    # Behavior Trees (si xgo_behavior_tree está instalado en el entorno)
    try:
        pkg_bt = get_package_share_directory('xgo_behavior_tree')
        bt_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bt, 'launch', 'fleet_behavior_trees.launch.py')
            ),
            launch_arguments={
                'simulate_heartbeat': LaunchConfiguration('simulate_heartbeat')
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_bt'))
        )
        actions.append(bt_launch)
    except PackageNotFoundError:
        pass

    # RViz2 opcional
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    actions.append(rviz2_node)

    return LaunchDescription(actions)

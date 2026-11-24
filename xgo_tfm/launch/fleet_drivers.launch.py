from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = [
        {'name': 'robot1', 'port': '/tmp/tty_robot1'},
        {'name': 'robot2', 'port': '/tmp/tty_robot2'},
        {'name': 'robot3', 'port': '/tmp/tty_robot3'},
    ]

    nodes = []
    for r in robots:
        node = Node(
            package='xgo_driver_cpp',
            executable='xgo_driver_node',
            name='xgo_driver_node',
            namespace=r['name'],
            output='screen',
            parameters=[{
                'serial_port': r['port'],
                'baudrate': 115200,
                'publish_rate_hz': 20.0,
                'cmd_vel_timeout_sec': 0.5
            }]
        )
        nodes.append(node)

    return LaunchDescription(nodes)

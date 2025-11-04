from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_driver',
            namespace='udp_bridge_node_cmd',
            executable='udp_bridge_node_exe',
            name='udp_bridge',
            # parameters=[LaunchConfiguration('src/transport_drivers/udp_driver/params/example_udp_params.yml')]
            parameters=[{
                "ip": '192.168.2.3',
                "port": 57410,
            }]
        ),

        Node(
            package='rr_udp_server',
            namespace='rr_udp_server_node',
            executable='rr_udp_server_node',
            name='rr_udp',
            # parameters=[LaunchConfiguration('src/transport_drivers/udp_driver/params/example_udp_params.yml')]
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            remappings=[('/udp_read', '/udp_bridge_node_cmd/udp_read')]
        )
    ])
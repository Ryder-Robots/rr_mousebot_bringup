from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='udp_driver',
            namespace='udp_bridge_node_cmd',
            executable='udp_bridge_node_exe',
            name='udp_bridge',
            parameters=[{
                "ip": '192.168.2.8',
                "port": 57410,
            }]
        ),

        LifecycleNode(
            package='rr_joystick',
            namespace='rr_sensors',
            output='screen',
            executable='rr_joystick_pub',
            name='joystick',
            arguments=['--ros-args', '--log-level', 'INFO'],
                parameters=[{'transport_plugin': 'rr_common_plugins::rr_udp_plugins::RrJoySubscriberUdpPlugin'}],
            remappings=[('/udp_read', '/udp_bridge_node_cmd/udp_read')]
        )

        # LifecycleNode(
        #     package='rr_state_mgm_srv',
        #     output='screen',
        #     namespace='state',
        #     executable='rr_state_mgm_srv_node',
        #     name="state_manager",
        #     arguments=['--ros-args', '--log-level', 'DEBUG']
        # )  
    ])

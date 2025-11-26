import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """launch composable nodes"""
    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='driver_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='udp_driver',
                    plugin='drivers::udp_driver::UdpReceiverNode',
                    parameters=[{
                        "ip": '192.168.2.8',
                        "port": 57410,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='udp_driver',
                    plugin='drivers::udp_driver::UdpSenderNode',
                    parameters=[{
                        "ip": '192.168.2.8',
                        "port": 57410,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='rr_joystick',
                    plugin='rrobot::rr_joystick::RRJoystickNode',
                    parameters=[{'transport_plugin': 'rr_common_plugins::rr_udp_plugins::RrJoySubscriberUdpPlugin'}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                )           
            ]
        )
    ])

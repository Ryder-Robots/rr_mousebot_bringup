# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """launch composable nodes"""
    return launch.LaunchDescription([

        # Low level communications, serial, and UDP, or bridges to flight controller.
        ComposableNodeContainer(
            name='driver_container',
            namespace='driver',
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
                    package='serial_driver',
                    plugin='drivers::serial_driver::SerialBridgeNode',
                    parameters=[{
                        "device_name": '/dev/ttyACM0',
                        "baud_rate": 115200,
                        "flow_control": "none",
                        "parity": "none",
                        "stop_bits": "1"
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),

        # Nodes in this container are one level away from raw hardware, but are still low level
        # they are expected to work extremely fast.
        ComposableNodeContainer(
            name='sensor_nodes',
            namespace='sensor',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ## Currently the joystick node doesn't want to behave itself when runnning as a component.
                ## I will need to investigate the reason why, but it seems fine if running as its own
                ## executable. (Mmmmm Gremlins)
                # ComposableNode(
                #     package='rr_joystick',
                #     plugin='rrobot::rr_joystick::RRJoystickNode',
                #     parameters=[{'transport_plugin': 'rr_common_plugins::rr_udp_plugins::RrJoySubscriberUdpPlugin'}, {'--log-level': 'DEBUG'}],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                # )
                ComposableNode(
                    package="rr_imu_action",
                    plugin="rr_imu_action::RrImuActionNode",
                    parameters=[{'transport_plugin': 'rr_common_plugins::rr_serial_plugins::ImuActionSerialPlugin'}],
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level': 'DEBUG'},
                    ],
                )
            ]
        ),

        # Buffer nodes. These nodes provide services and frames for long running services
        # such as Core Network (ML), path-planner etc
        ComposableNodeContainer(
            name='state_nodes',
            namespace='state',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='rr_state_mgm_srv',
                    plugin='rrobot::state_frame::RRStateJoyNode',
                    extra_arguments=[{'use_intra_process_comms': True}, {'--log-level': 'DEBUG'}],                   
                ),
            ]
        ),
    ])

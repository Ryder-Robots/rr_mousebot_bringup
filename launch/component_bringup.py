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

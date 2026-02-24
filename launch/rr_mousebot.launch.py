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
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """launch composable nodes"""

    # Include LDLidar launch
    # ldlidar_launch = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource([
    #         get_package_share_directory('ldlidar_node'),
    #         '/launch/ldlidar_bringup.launch.py'
    #     ]),
    #     launch_arguments={
    #         'node_name': 'ldlidar_node'
    #     }.items()
    # )

    return launch.LaunchDescription([
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{
        #         'autostart': True,
        #         'bond_timeout': 0.0,
        #         'node_names': [
        #             '/driver/serial_bridge_node',
        #             '/driver/udp_receiver_node',
        #             '/driver/udp_sender_node',
        #             '/sensor/rr_imu_action_node',
        #             'ldlidar_node',
        #         ]
        #     }],
        # ),

        # Include LDLidar bringup (lifecycle node)
        # ldlidar_launch,

        # Low level communications, serial, and UDP, or bridges to flight controller.
        ComposableNodeContainer(
            name='driver_container',
            namespace='driver',
            package='rclcpp_components',
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', 'info'],
            composable_node_descriptions=[

                ComposableNode(
                    package='rr_motor_controller',
                    plugin='rr_motor_controller::RrMotorController',
                    name='motor_left',
                    namespace='',
                    parameters=[{
                        'motor_pos': 0,
                        'ppr': 8,
                        'wheel_radius': 20,
                        'transport_plugin': 'rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin',
                        'pwm_pin': 18,
                        'dir_pin': 23,
                        'pwm_freq': 1000,
                        'encoder_pin': 9,
                        'encoder_timeout': 500000,
                    }],
                ),

                # ComposableNode(
                #     package='rr_motor_controller',
                #     plugin='rr_motor_controller::RrMotorControllerNode',
                #     name='motor_right',
                #     namespace='',
                #     parameters=[{
                #         'motor_pos': 1,
                #         'ppr': 8,
                #         'wheel_radius': 20,
                #         'transport_plugin': 'rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin',
                #         'pwm_pin': 19,
                #         'dir_pin': 24,
                #         'pwm_freq': 1000,
                #         'encoder_pin': 8,
                #         'encoder_timeout': 500000,
                #     }],
                # ),

                # Note that this is a lifecycle node within this version
                # ComposableNode(
                #     package='udp_driver',
                #     plugin='drivers::udp_driver::UdpReceiverNode',
                #     namespace='driver',
                #     name='udp_receiver_node',
                #     parameters=[{
                #         "ip": '0.0.0.0',
                #         "port": 57410,
                #     }],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                # ),

                # # Note that this is a lifecycle node within this version
                # ComposableNode(
                #     package='udp_driver',
                #     plugin='drivers::udp_driver::UdpSenderNode',
                #     namespace='driver',
                #     name='udp_sender_node',
                #     parameters=[{
                #         # transport_driver does not natively support
                #         # SO_BROADCAST option, so change IP Address as
                #         # appropriate
                #         "ip": '192.168.1.20',
                #         "port": 57410,
                #     }],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                # ),
            ],
        ),
    ])

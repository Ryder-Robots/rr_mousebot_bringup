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
from launch.actions import TimerAction

def generate_launch_description():
    """Launch composable nodes."""

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
        # Include LDLidar bringup (lifecycle node)
        # ldlidar_launch,

        # Low level communications, serial, and UDP, or bridges to flight controller.
        ComposableNodeContainer(
            name='driver_container',
            namespace='driver',
            package='rclcpp_components',
            executable='component_container_mt',
            arguments=['--ros-args', '--log-level', 'debug'],
            composable_node_descriptions=[

                ComposableNode(
                    package='rr_motor_controller',
                    plugin='rr_motor_controller::RrECU',
                    name='motor_controller',
                    namespace='driver',
                    parameters=[{
                        'motor_count': 2,
                        'ppr': 8,
                        'wheel_radius': 20,   # 20 mm radius = 40 mm diameter wheels
                        'wheel_base':   70,   # 70 mm between left and right wheel contact points
                        'pwm_freq': 1000,
                        # One pin per motor: index 0 = left, index 1 = right
                        #
                        # Note: for wiring, B represents H-Bridge B and A represents A on H-bridge
                        'encoder_pins': [17, 22],    # LEFT=17 (B), RIGHT=22 (A)
                        'pwm_pins':     [12, 13],    # LEFT=12(HW PWM0 B), RIGHT=13(HW PWM1 A)
                        'dir_pins':     [27, 23],    # LEFT=27 (B), RIGHT=23 (A)
                        'encoder_timeout': 500000,
                        # Raspberry Pi 4B GPIO plugin — see https://github.com/Ryder-Robots/rr_gpio_pi4b_pigpio_plugin
                        'transport_plugin': 'rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin',
                        # 6×6 row-major pose covariance [x, y, z, roll, pitch, yaw].
                        # Diagonal: x=0.01, y=0.01, z=1e6, roll=1e6, pitch=1e6, yaw=0.01
                        # z, roll, pitch set to 1e6 — not tracked on a flat floor.
                        'covariance': [
                            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
                            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
                            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
                            0.0,  0.0,  0.0,  0.0,  0.0,  0.01,
                        ],
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),

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
        TimerAction(
            period=3.0,
            actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': False,
                    'bond_timeout': 4.0,
                    'node_names': [
            #             'lidar_node',
                        '/driver/motor_controller',
            #             '/driver/serial_bridge_node',
            #             '/driver/udp_receiver_node',
            #             '/driver/udp_sender_node',
            #             '/sensor/rr_imu_action_node',
                    ]
                }],
            ),
            ]
        )
    ])

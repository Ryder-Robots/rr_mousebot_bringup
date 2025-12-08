#  Maze Solving Mouse Launch File

## Pre Install Steps

```bash
sudo apt update

# Install transport driver from the following documenation https://github.com/ros-drivers/transport_drivers

source install/setup.bash

# install interfaces
# https://github.com/Ryder-Robots/rr_interfaces

# install common base
# https://github.com/Ryder-Robots/rr_common_base

# install state manager service

sudo apt install ros-${ROS_DISTRO}-launch ros-${ROS_DISTRO}-launch-ros

```

## Launch Command

```bash
ros2 launch rr_mousebot_bringup rr_mousebot.launch.py
```

## Manually configuring nodes

```bash
 ros2 lifecycle get /serial_bridge_node
 # unconfigured [1]
 ros2 lifecycle set /serial_bridge_node configure
 # Transitioning successful
 ros2 lifecycle set /serial_bridge_node activate
 # Transitioning successful
```

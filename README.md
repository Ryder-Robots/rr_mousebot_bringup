# RR Mousebot Bringup

ROS 2 launch package for an autonomous maze-solving robot designed to comply with IEEE Micromouse competition specifications.

## Project Overview

This package provides the core system bringup for a micromouse-compliant robot. While the robot does not compete in official competitions, it is designed to meet the technical specifications established by IEEE Micromouse competition rules for educational and developmental purposes.

The micromouse competition, first introduced by IEEE Spectrum in 1977, challenges participants to design and build autonomous robots capable of navigating from a start position to the center of an unknown maze in the shortest time possible (Christiansen, 1977). This project implements a ROS 2-based system that adheres to these specifications.

### IEEE Micromouse Competition Specifications

The robot is designed to comply with standard IEEE Micromouse specifications:

**Maze Specifications:**
- 16 x 16 grid of unit squares (each 18cm x 18cm)
- Wall height: 5 cm
- Wall thickness: 1.2 cm (±5% tolerance)
- Passageway width: 16.8 cm
- Total maze size: approximately 8 feet (2.44m) square
- Wall colors: white sides with red tops
- Floor: non-gloss black painted wood
(Micromouse Online, n.d.; IEEE Region 2, 2018)

**Robot Design Requirements:**
- Maximum dimensions: 25cm x 25cm (no height restriction)
- Must be self-contained (no remote control or external assistance)
- Cannot use combustion-based energy sources
- Must be fully autonomous
- Superstructure may extend above maze walls
(Micromouse Online, n.d.)

**Competition Rules (for reference):**
- 10-minute time limit for maze exploration and solving
- No ROM changes or program downloads after maze is revealed
- Each exit from starting square begins a new run
- Must reach destination square (center) for time to be recorded
- No communication with robot during competition runs
(IEEE Spectrum, 1979)

## Architecture

The `rr_mousebot.launch.py` launch file initializes a multi-container ROS 2 system with composable nodes organized by function:

### Driver Container
Low-level communication interfaces for hardware control:
- **UDP Driver (Receiver/Sender)**: Network communication on 192.168.2.8:57410 for telemetry and debugging
- **Serial Bridge**: Communication with motor controllers and sensors via `/dev/ttyACM0` at 115200 baud (8N1 configuration)

### Sensor Container
Sensor processing nodes for maze perception and navigation (currently empty, awaiting sensor integration)

### State Container
State management and control logic:
- **State Joy Node**: Joystick/control state management for manual override and testing

All nodes are configured to use intra-process communication for reduced latency.

## Schematic Directory

The [schematic/](schematic/) directory contains FreeCAD design files (.FCStd) for all mechanical and electrical components of the micromouse robot. These 3D CAD models are used for prototyping, assembly planning, and dimensional verification to ensure compliance with IEEE Micromouse specifications.

Key component designs include:
- Robot chassis and body assemblies
- Motor and encoder mountings
- Wheel designs (drive and caster wheels)
- Electronics mounting (Raspberry Pi, Arduino Nano, power circuits)
- Battery pack housing
- Power management circuits (buck converters)

The `stl/` subdirectory will be populated with STL files in future versions for 3D printing of custom components.

For detailed information about individual components, see [schematic/README.md](schematic/README.md).

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

## References

Christiansen, D. (1977) 'The Amazing Micromouse Contest', *IEEE Spectrum*, 14(5). Available at: https://spectrum.ieee.org/the-amazing-micromouse-contest (Accessed: 13 December 2025).

IEEE Region 2 (2018) *Micromouse Competition Rules - IEEE R2 SAC 2018*. Available at: https://ewh.ieee.org/reg/2/sac-18/MicromouseRules.pdf (Accessed: 13 December 2025).

IEEE Spectrum (1979) 'The Amazing Micromouse Maze Contest', *IEEE Spectrum*, 16(6). Available at: https://spectrum.ieee.org/the-amazing-micromouse-contest (Accessed: 13 December 2025).

Micromouse Online (n.d.) *The Rules - Micromouse Book*. Available at: https://micromouseonline.com/micromouse-book/rules/ (Accessed: 13 December 2025).

Micromouse Online (n.d.) *Chassis Size - Micromouse Book*. Available at: https://micromouseonline.com/micromouse-book/the-chassis/chassis-size/ (Accessed: 13 December 2025).

## License

This project is licensed under the MIT License.

Copyright (c) 2025

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

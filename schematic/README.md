# Schematic Directory

This directory contains FreeCAD (.FCStd) design files for all mechanical and electrical components of the RR Mousebot. These 3D CAD models are used for design verification, assembly planning, and ensuring compliance with IEEE Micromouse competition specifications (maximum footprint: 25cm x 25cm).

## Directory Structure

```
schematic/
├── README.md           # This file
├── stl/                # STL files for 3D printing (to be populated in future versions)
└── *.FCStd             # FreeCAD design files
```

## Component Design Files

### Main Assembly Files

#### [mazebot.FCStd](mazebot.FCStd)
Complete robot assembly integrating all mechanical and electrical components. This is the master assembly file that shows how all individual components fit together to form the complete micromouse robot.

#### [body.FCStd](body.FCStd)
Main chassis/body design for the robot. This component houses all electronics, motors, sensors, and power systems while maintaining compliance with the 25cm x 25cm maximum footprint requirement.

### Drive System Components

#### [hexwheel.FCStd](hexwheel.FCStd)
Hexagonal wheel design for the main drive wheels. The hex pattern may provide improved traction and maze navigation characteristics compared to standard circular wheels.

#### [caster_wheel.FCStd](caster_wheel.FCStd)
Caster wheel assembly providing passive support and stability. Typically mounted at the front or rear of the robot to create a stable three or four-point contact with the maze floor.

#### [tt_motor.FCStd](tt_motor.FCStd)
TT (Toy Train) motor model used for robot propulsion. These compact DC gear motors are commonly used in small robotics applications due to their favorable size-to-power ratio.

#### [motor_encoder.FCStd](motor_encoder.FCStd)
Optical encoder assembly for motor shaft position and velocity feedback. Encoders are essential for accurate odometry and closed-loop motor control during maze navigation.

### Electronics Mounting

#### [Raspberry Pi 4 Model B.STEP.FCStd](Raspberry%20Pi%204%20Model%20B.STEP.FCStd)
Raspberry Pi 4 Model B mounting design. The Pi serves as the main compute platform running ROS 2 and high-level navigation algorithms.

#### [nano-breakout.FCStd](nano-breakout.FCStd)
Arduino Nano breakout board design for interfacing with sensors and motor controllers. The Nano handles real-time sensor reading and motor control, communicating with the Raspberry Pi via serial connection.

#### [pi-rough.FCStd](pi-rough.FCStd)
Rough/preliminary mounting design for Raspberry Pi. This may be an earlier iteration or alternative mounting configuration for the Pi board.

### Power System

#### [battery_pack.FCStd](battery_pack.FCStd)
Battery pack housing and mounting design. Proper battery placement is critical for maintaining the robot's center of gravity and ensuring stable maze navigation.

#### [buck_circuit.FCStd](buck_circuit.FCStd)
Buck converter circuit mounting for power regulation. Buck converters step down the battery voltage to appropriate levels for different components (e.g., 5V for Raspberry Pi, 3.3V for sensors, motor voltage regulation).

## Bill of Materials (BOM)

This section lists the commercial off-the-shelf (COTS) components required to build the RR Mousebot. Prices are in AUD and include GST unless otherwise noted.

### Computing and Control

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| Raspberry Pi 4 Model B 8GB | CE06974 | 1 | $145.00 | [Core Electronics](https://core-electronics.com.au/raspberry-pi-4-model-b-8gb.html) | Main compute platform for ROS 2 and navigation algorithms. 1.5GHz quad-core ARM Cortex-A72, Gigabit Ethernet, dual-band WiFi, Bluetooth 5.0 |
| Nano I/O Shield For Arduino Nano | DFR0012 | 1 | $12.95 | [Core Electronics](https://core-electronics.com.au/nano-i-o-shield-for-arduino-nano.html) | Expansion board with Gravity connectors for sensor/servo connections. Handles real-time motor control and sensor interfacing |

### Sensors

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| 360° Omni-directional Triangulation Lidar Dev Kit (8m range) | WS-24659 | 1 | $94.10 | [Core Electronics](https://core-electronics.com.au/360-omni-directional-triangulation-lidar-dev-kit-8m-range.html) | LD14P Lidar with 0.1-8m range, 1.5% accuracy, 6Hz scan rate. Includes USB-to-UART adapter. For maze wall detection and SLAM |
| Wheel Encoders for DFRobot 3PA and 4WD Rovers | SEN0038 | 2 | $10.15 | [Core Electronics](https://core-electronics.com.au/wheel-encoders-for-dfrobot-3pa-and-4wd-rovers.html) | 20 PPR optical encoders for odometry. Non-contact angular displacement sensing. Includes grating disks and mounting hardware |

### Mechanical Components

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| Supporting Swivel Caster Wheel - 1.3" Diameter | ADA2942 | 1 | $5.75 | [Core Electronics](https://core-electronics.com.au/supporting-swivel-caster-wheel-1-3-diameter.html) | 360° rotating support wheel. 32.4mm diameter, 42mm total height. Provides third-point stability for two-wheeled robot |

### Cables and Interconnects

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| USB OTG Cable - Female A to Micro A - 4" | CAB-11604 | 2 | $7.65 | [Core Electronics](https://core-electronics.com.au/usb-otg-cable-female-a-to-micro-a-4.html) | USB On-The-Go cable for connecting peripherals. 4" length |

### BOM Notes

- **Total Component Cost** (listed items only): ~$292 AUD (excludes motors, motor drivers, battery pack, structural materials, and 3D printed parts)
- **Additional Components Required** (not listed above):
  - TT gear motors (2x) for main drive wheels
  - Motor driver board (H-bridge for bidirectional control)
  - Battery pack (LiPo or Li-ion, voltage TBD based on motor specifications)
  - Buck converter modules for 5V (Raspberry Pi) and 3.3V (sensors) power rails
  - Arduino Nano or compatible microcontroller
  - Wiring, connectors, and miscellaneous hardware
  - 3D printed structural components (chassis, motor mounts, sensor brackets)

- **Quantity Discounts**: Available for most components when ordering 6+ or 10+ units
- **Availability**: Most items ship same business day; Lidar has lead time (ships Jan 6-9, 2025)
- **Supplier**: All components sourced from Core Electronics Australia for consistency

### Component Selection Rationale

- **Raspberry Pi 4 8GB**: Provides adequate computational power for ROS 2 nodes, SLAM algorithms, and path planning. 8GB RAM ensures headroom for development and debugging
- **Lidar Sensor**: 8m range exceeds maze dimensions (~2.5m), 1.5% accuracy suitable for 18cm grid navigation, UART interface compatible with both Raspberry Pi and Arduino
- **Wheel Encoders**: 20 PPR resolution provides ~1.8° angular resolution for odometry and closed-loop speed control
- **Caster Wheel**: Small form factor (32.4mm) minimizes footprint while providing stable support

## Design Workflow

1. **Individual Component Design**: Each mechanical and electrical component is designed in its own FreeCAD file with accurate dimensions and mounting features.

2. **Assembly Integration**: Components are imported and assembled in `mazebot.FCStd` to verify fit, clearances, and overall dimensions.

3. **Specification Verification**: The complete assembly is checked against IEEE Micromouse specifications:
   - Maximum footprint: 25cm x 25cm
   - Ground clearance compatible with 5cm maze walls
   - Center of gravity positioned for stable navigation

4. **STL Export** (future): Once designs are finalized, STL files will be exported to the `stl/` subdirectory for 3D printing.

## STL Subdirectory

The [stl/](stl/) subdirectory is reserved for STL (stereolithography) files that will be generated from the FreeCAD designs. These STL files will be:
- Optimized for 3D printing
- Properly oriented for minimal support material
- Scaled to actual dimensions
- Named to match their source FreeCAD files

**Status**: This directory will be populated in future versions as component designs are finalized and validated.

## Working with FreeCAD Files

To view or modify these designs:

1. Install FreeCAD (https://www.freecad.org/)
2. Open any `.FCStd` file in FreeCAD
3. Use the model tree to explore component features
4. Modify designs as needed for your specific hardware configuration

## Design Considerations

All component designs take into account:
- **IEEE Micromouse Compliance**: Maximum 25cm x 25cm footprint, no height restrictions
- **Maze Clearances**: 16.8cm passageway width, 5cm wall height
- **Component Integration**: Proper mounting points for electronics, sensors, and mechanical parts
- **Center of Gravity**: Low and centered for stable high-speed navigation
- **Serviceability**: Easy access to electronics and batteries for maintenance
- **Cable Routing**: Paths for power and data cables without interference

## File Naming Convention

- `*.FCStd`: FreeCAD native format files
- `*.STEP.FCStd`: FreeCAD files imported from STEP format (industry-standard CAD exchange format)
- `*.FCBak`: Automatic backup files created by FreeCAD (can be safely ignored)

## Version History

Component designs are version-controlled through Git. Refer to commit history for design evolution and changes. Major design iterations are documented in commit messages.

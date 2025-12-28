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

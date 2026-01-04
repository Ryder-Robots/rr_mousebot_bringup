# Schematic Directory

This directory contains FreeCAD (.FCStd) design files for all mechanical and electrical components of the RR Mousebot. These 3D CAD models are used for design verification, assembly planning, and ensuring compliance with IEEE Micromouse competition specifications (maximum footprint: 25cm x 25cm).

## Directory Structure

```
schematic/
├── README.md           # This file
├── stl/                # STL and 3MF files for 3D printing
│   ├── body.stl        # Main chassis STL file
│   ├── body.3mf        # Main chassis 3MF file (recommended)
│   ├── drive_shaft.stl # Drive shaft STL file
│   └── drive_shaft.3mf # Drive shaft 3MF file (recommended)
└── *.FCStd             # FreeCAD design files
```

## STL Files

The `stl/` subdirectory contains 3D printable files exported from the FreeCAD designs. Files are provided in both STL and 3MF formats:

- **3MF Format** (recommended): Preserves units, scale, and metadata. Use with Elegoo Slicer for best results.
- **STL Format**: Universal format compatible with all slicers.

**Available Components:**
- `body.stl` / `body.3mf`: Main robot chassis (40% infill recommended)
- `drive_shaft.stl` / `drive_shaft.3mf`: Motor drive shaft (100% infill required for strength)

For detailed printing instructions, slicer settings, and post-processing guidance, see [CONSTRUCTION.md](../CONSTRUCTION.md).

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
| Raspberry Pi 4 Model B 8GB | CE06974 | 1 | $145.00 | [Core Electronics](https://core-electronics.com.au/raspberry-pi-4-model-b-8gb.html) | Main compute platform for ROS 2 and navigation algorithms. 1.5GHz quad-core ARM Cortex-A72, Gigabit Ethernet, dual-band WiFi, Bluetooth 5.0. Typical current: ~3A at 5V |
| Arduino Nano 33 BLE Sense Rev2 | ABX00069 | 1 | $98.13 | [Core Electronics](https://core-electronics.com.au/arduino-nano-33-ble-sense-rev2.html) | nRF52840 microcontroller (64MHz, 1MB Flash, 256KB RAM). Includes 9-axis IMU, temp/humidity, pressure, proximity/gesture sensors, and digital mic. Bluetooth 5 via NINA B306. 3.3V I/O, ~50mA typical current. Handles real-time motor control and sensor interfacing |
| Nano I/O Shield For Arduino Nano | DFR0012 | 1 | $12.95 | [Core Electronics](https://core-electronics.com.au/nano-i-o-shield-for-arduino-nano.html) | Expansion board with Gravity connectors for sensor/servo connections. Compatible with Arduino Nano 33 BLE Sense Rev2 |

### Sensors

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| URM09 Ultrasonic Sensor (Gravity I²C) | SEN0304 | 3 | $22.65 | [Core Electronics](https://core-electronics.com.au/urm09-ultrasonic-sensor-gravity-i-c.html) | DFRobot I²C ultrasonic distance sensor. 2-500cm range (configurable), 1cm resolution, 1% accuracy, 50Hz max frequency. 3.3-5.5V supply, 20mA operating current. Built-in temperature compensation. For maze wall detection and obstacle avoidance. Connects to Nano I/O Shield I²C panel |
| Wheel Encoders for DFRobot 3PA and 4WD Rovers | SEN0038 | 2 | $10.15 | [Core Electronics](https://core-electronics.com.au/wheel-encoders-for-dfrobot-3pa-and-4wd-rovers.html) | 20 PPR optical encoders for odometry. Non-contact angular displacement sensing. Includes grating disks and mounting hardware |

### Motor Control

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| DC Gearbox Motor - TT Motor - 200RPM - 3 to 6VDC | ADA3777 | 2 | $7.20 | [Core Electronics](https://core-electronics.com.au/dc-gearbox-motor-tt-motor-200rpm-3-to-6vdc.html) | Adafruit TT motor with 1:48 gear ratio. 120-250 RPM, 150mA continuous current, 0.8kg.cm stall torque at 6V. 70x22x18mm, includes 200mm wire leads. For main drive wheels |
| Makerverse Motor Driver 2 Channel | CE08038 | 1 | $7.20 | [Core Electronics](https://core-electronics.com.au/makerverse-motor-driver-2-channel.html) | Dual H-bridge motor driver. 1.6A continuous (2A peak), 3-16V input, compatible with 3.3V/5V logic. Includes thermal protection, reverse-polarity protection, and 5V regulator. 35x30mm with M3 mounting holes |

### Power System

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| Samsung 18650 3500mAh 3.7V Rechargeable Battery | SB2632 | 2 | $33.50 | [Jaycar](https://www.jaycar.com.au/18650-samsung-3500mah-3-7v-18650-rechargeable-battery/p/SB2632) | Samsung INR18650-35E. 3500mAh capacity, 3.5A continuous/10A peak discharge. 7.4V nominal (2S configuration), 7000mAh total capacity provides ~51.8Wh energy storage |
| Dual 18650 Battery Holder | PH9207 | 1 | $6.95 | [Jaycar](https://www.jaycar.com.au/dual-18650-battery-holder/p/PH9207) | ABS plastic holder with 150mm leads. 75x39x20mm. Suitable for 2S (series) configuration providing 7.4V nominal output |
| DC-DC Adjustable Step-down Module 5A 75W | CE07271 | 1 | $2.55 | [Core Electronics](https://core-electronics.com.au/dc-dc-adjustable-step-down-module-5a-75w.html) | XL4015 buck converter. Input: 4-38V, Output: 1.25-36V adjustable, 5A max. 30-turn potentiometers for voltage/current adjustment. Provides 5V rail for Raspberry Pi and Lidar. Arduino Nano powered via USB from Raspberry Pi. 54x24x12mm with 3mm mounting holes |

### Mechanical Components

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| Wheel - 65mm (Rubber Tire, Pair) | ROB-13259 | 1 | $9.55 | [Core Electronics](https://core-electronics.com.au/wheel-65mm-rubber-tire-pair.html) | Sparkfun basic wheels with black rubber tires. 65mm diameter, sold as pair (2 wheels). Compatible with DAGU right angle gear motors. For main drive wheels on differential drive robot |
| Supporting Swivel Caster Wheel - 1.3" Diameter | ADA2942 | 1 | $5.75 | [Core Electronics](https://core-electronics.com.au/supporting-swivel-caster-wheel-1-3-diameter.html) | 360° rotating support wheel. 32.4mm diameter, 42mm total height. Provides third-point stability for robot |
| Brass Heat-Set Inserts for Plastic - M3 x 3mm - 50 pack | ADA4256 | 1 | $11.65 | [Core Electronics](https://core-electronics.com.au/brass-heat-set-inserts-for-plastic-m3-x-3mm-50-pack.html) | M3 threaded brass inserts for 3D printed parts. 4.5mm OD, 3mm length. Heat-press installation with soldering iron. Provides strong, reusable threaded mounting points in plastic chassis |
| Makerverse M3 Mounting Kit | CE08342 | 1 | $3.25 | [Core Electronics](https://core-electronics.com.au/makerverse-m3-mounting-kit.html) | Hardware kit with 4x M3x12mm standoffs and 8x 6mm screws. For mounting electronics to chassis (up to 3mm material thickness) |
| Ultra Mini Experimenters Board | HP9556 | 1 | $7.95 | [Jaycar](https://www.jaycar.com.au/ultra-mini-experimenters-board/p/HP9556) | Vero board 65x45mm with 640 holes (0.3mm, 2.54mm spacing). Supplied as pair (can be snapped apart). For power distribution and wiring connections between buck converters and system components |

### Cables and Interconnects

| Component | SKU | Quantity | Unit Price (inc GST) | Supplier | Notes |
|-----------|-----|----------|---------------------|----------|-------|
| USB OTG Cable - Female A to Micro A - 4" | CAB-11604 | 2 | $7.65 | [Core Electronics](https://core-electronics.com.au/usb-otg-cable-female-a-to-micro-a-4.html) | USB On-The-Go cable for connecting peripherals. 4" length |

### BOM Notes

- **Total Component Cost** (listed items only): ~$506 AUD (excludes battery charger and 3D printed parts)

- **Power System Summary**: 2S Li-ion configuration (2x 18650 cells in series)
  - Nominal voltage: 7.4V (fresh), ~5.5V (under load/partially discharged)
  - Total capacity: 7000mAh (51.8Wh)
  - Continuous discharge: 7A (3.5A per cell)
  - Peak discharge: 20A (10A per cell, 5 seconds)
  - **Buck Converter Configuration** (adjust using flathead screwdriver):
    - Single converter: 5.5V → 5.0V rail for Raspberry Pi only (3.0A continuous load)
    - Arduino Nano 33 BLE Sense Rev2: Powered via USB from Raspberry Pi (included in Pi's power budget)
  - Motor power: Battery voltage (7.4V nominal, 5.5V under load) direct from battery through motor driver

- **Battery Charging**:
  - **IMPORTANT**: Battery cells must be removed from the holder and charged using an external 18650 Li-ion battery charger
  - Compatible chargers available from Core Electronics and Jaycar Australia
  - Do NOT attempt to charge batteries while installed in the robot
  - Recommended: 2S (7.4V) Li-ion smart charger with balance charging capability

- **Total Current Load Analysis** (at 5.5V battery voltage under load):
  - **5V Rail** (via Buck Converter):
    - Raspberry Pi 4: 3.0A (typical under load, includes USB power to Arduino Nano)
    - Arduino Nano 33 BLE Sense Rev2: 0.05A (powered via USB from Raspberry Pi, included in Pi's budget)
    - URM09 Ultrasonic Sensors: 0.06A (3x 20mA via I²C from Nano I/O Shield)
    - Wheel Encoders: 0.04A (2x 20mA)
    - Motor Driver Control: 0.01A (minimal)
    - **Subtotal 5V Rail**: 3.08A (buck converter rated 5A, 62% utilization)
  - **Direct Battery (5.5V-7.4V)**:
    - TT Motors: 0.3A total (2x 150mA continuous)
    - **Subtotal Direct**: 0.3A
  - **System Total**: ~3.38A from battery (7A continuous capacity, 48% utilization)
  - **Runtime Estimate (Full System Active)**: 7000mAh / 3380mA ≈ 2.1 hours continuous operation

- **Operational Mode Battery Life Analysis**:
  - **Full System Runtime**:
    - Current draw: 3.38A (all systems active)
    - Runtime: 7000mAh / 3380mA ≈ **2.1 hours** (127 minutes) continuous operation
  - **Competition Scenario** (10-minute time limit):
    - Typical maze solving: 10-15 minutes @ 3.38A = 563-845mAh (8-12% battery)
    - **Result**: 8-12 full maze solving attempts per full battery charge
    - Three ultrasonic sensors provide comprehensive wall detection (front, left, right) with 2-500cm range and low combined power consumption (60mA total)

- **Motor System Summary**: 2x TT motors for differential drive
  - Operating voltage: 7.4V nominal (5.5V under load conditions)
  - Current draw: 150mA per motor (continuous), 300mA total
  - Speed: ~200 RPM at 6V (estimated ~180-200 RPM at 5.5V under load)
  - Motor driver provides 1.6A continuous capacity (>5x safety margin)

- **Assembly Notes**:
  - Buck converter requires tuning with flathead screwdriver to achieve precise 5.0V output
  - Use multimeter to verify output voltage before connecting to Raspberry Pi
  - Arduino Nano 33 BLE Sense Rev2 powered via USB connection from Raspberry Pi (no separate power rail needed)
  - Ultra Mini Experimenters Board (HP9556) provides power distribution bus for connecting buck converter output to system components
  - Brass heat-set inserts (ADA4256) provide threaded mounting points in 3D printed chassis for M3 screws

- **Additional Components Required** (not listed above):
  - 18650 Li-ion battery charger (2S/7.4V compatible) - available from Core Electronics or Jaycar
  - Wiring, connectors, and miscellaneous hardware
  - 3D printed structural components (chassis, motor mounts, sensor brackets)
  - M3 screws (various lengths) for use with heat-set inserts

- **Quantity Discounts**: Available for most components when ordering 6+ or 10+ units
- **Availability**: Most items ship same business day; Lidar has lead time (ships Jan 6-9, 2025)
- **Suppliers**: Components sourced from Core Electronics Australia and Jaycar Australia

### Component Selection Rationale

- **Raspberry Pi 4 8GB**: Provides adequate computational power for ROS 2 nodes, SLAM algorithms, and path planning. 8GB RAM ensures headroom for development and debugging. 3A typical current draw fits within 5A buck converter capacity
- **Arduino Nano 33 BLE Sense Rev2**: Powerful nRF52840 microcontroller (64MHz, 1MB Flash) handles real-time motor control and sensor interfacing. Integrated 9-axis IMU, environmental sensors, and Bluetooth 5 provide extensive sensing capabilities beyond basic motor control. Low power consumption (50mA typical) and 3.3V I/O compatible with modern sensors
- **Lidar Sensor**: 8m range exceeds maze dimensions (~2.5m), 1.5% accuracy suitable for 18cm grid navigation, UART interface compatible with both Raspberry Pi and Arduino
- **Wheel Encoders**: 20 PPR resolution provides ~1.8° angular resolution for odometry and closed-loop speed control
- **TT Motors**: Compact form factor (70x22x18mm) fits within micromouse footprint constraints. 1:48 gear ratio provides good balance between speed (~200 RPM) and torque (0.8kg.cm stall) for maze navigation. Low current draw (150mA continuous) ensures efficient battery usage. Pre-attached 200mm leads simplify wiring
- **Motor Driver**: 1.6A continuous current rating provides >5x safety margin over motor requirements (300mA total). Dual H-bridge enables independent bidirectional control for differential drive. Built-in 5V regulator, thermal protection, and compact form factor (35x30mm) with M3 mounting holes
- **Battery System**: Samsung 18650 cells chosen for high energy density (3500mAh), reliable performance, and proven track record. 2S configuration (7.4V nominal) provides appropriate voltage for motor driver input (3-16V range) and buck converter regulation. 51.8Wh total capacity supports ~1.9 hours continuous operation
- **Buck Converter**: Single XL4015 module provides reliable 5V regulation with 5A capacity exceeding system requirements (3.4A total load including Raspberry Pi, Lidar, encoders, and Arduino Nano via USB). Adjustable output voltage via 30-turn potentiometer enables precise 5.0V tuning with flathead screwdriver. Wide input range (4-38V) accommodates battery voltage variation during discharge. Arduino Nano powered via USB from Raspberry Pi eliminates need for separate 3.3V rail
- **Rubber Tire Wheels**: 65mm diameter wheels with rubber tires provide good traction for differential drive robot. Sold as pair (2 wheels), compatible with DAGU right angle gear motors and suitable for TT motor mounting. Compact diameter fits within micromouse footprint constraints while providing adequate ground clearance
- **Prototyping Board**: Ultra Mini Experimenters Board provides compact (65x45mm) power distribution platform for connecting buck converter outputs to system components. Vero-style board with 640 holes enables flexible wiring layout
- **Heat-Set Inserts**: Brass M3 inserts provide strong, reusable threaded mounting points in 3D printed chassis. Superior to directly threading into plastic, allowing repeated assembly/disassembly without thread degradation. 50-pack provides sufficient quantity for entire robot assembly
- **Caster Wheel**: Small form factor (32.4mm) provides additional support point for stability during omnidirectional maneuvers
- **Mounting Hardware**: M3 standoffs provide secure mounting for motor driver and other electronics while maintaining proper spacing from chassis

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

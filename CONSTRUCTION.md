# RR Mousebot Construction Guide

This guide provides detailed instructions for 3D printing and assembling the RR Mousebot chassis and mechanical components.

## STL Files

All 3D printable files are located in the `schematic/stl/` directory. Files are provided in both STL and 3MF formats:

**Available Files:**
- `body.stl` / `body.3mf`: Main robot chassis
- `drive_shaft.stl` / `drive_shaft.3mf`: Motor drive shaft

**Format Recommendations:**
- **3MF Format** (recommended): Use with Elegoo Slicer for preserved scale, units, and metadata
- **STL Format**: Universal compatibility with any slicer software

See [schematic/README.md](schematic/README.md) for complete list of available components.

---

## 3D Printing Configuration

### Equipment

- **Printer**: Elegoo Centauri Carbon (enclosed)
- **Filament Dryer**: Creality Filament Heater
- **Recommended Slicer**: Elegoo Slicer (or compatible)

### Material Selection

- **Prototyping**: PETG (for quick iteration and testing)
- **Final Build**: ABS (for superior strength and durability)

This guide focuses on PETG settings for prototyping. ABS settings will be added for final production builds.

---

## PETG Printing Configuration

### Filament Preparation

PETG is hygroscopic and must be thoroughly dried before printing to prevent moisture-related defects (bubbles, stringing, weak layer adhesion).

#### Creality Filament Heater Settings

**Pre-Loading Drying (New or Exposed Filament)**:
- Temperature: 65-70°C
- Duration: 4-6 hours
- Purpose: Remove absorbed moisture before loading into printer

**Quick Dry (Recently Exposed Filament)**:
- Temperature: 60°C
- Duration: 2-4 hours
- Purpose: Light moisture removal without risking filament softening
- Note: Avoid higher temperatures to prevent PETG deformation

**Storage Mode (Active Storage)**:
- Temperature: 50-55°C
- Duration: Indefinite (while filament is loaded)
- Purpose: Maintain dry state during extended print sessions

**Best Practice**: Pre-dry at 65°C for 4-6 hours, then maintain at 50-55°C during printing.

---

### Elegoo Centauri Carbon Printer Settings

Configure these settings in your slicer (Elegoo Slicer recommended) for optimal PETG prints on the Centauri Carbon.

#### Temperature Settings

| Parameter | Temperature | Notes |
|-----------|-------------|-------|
| Nozzle (Standard) | 230-250°C | Start at 240°C for initial calibration |
| Nozzle (First Layer) | 245°C | Higher temp improves bed adhesion |
| Heated Bed | 70-80°C | Preheat for 10 minutes before printing |
| Chamber (with Creality mod) | 40-50°C | Enclosed chamber maintains stable temperature |

**Temperature Tuning**:
- If stringing occurs: Reduce nozzle temp by 5°C increments
- If layer adhesion is weak: Increase nozzle temp by 5°C increments
- If warping occurs: Increase bed temp to 80°C

#### Print Speed Settings

| Parameter | Speed | Notes |
|-----------|-------|-------|
| First Layer | 20-30 mm/s | Slow for optimal bed adhesion |
| Perimeters | 30-40 mm/s | Balance quality and speed |
| Infill | 40-50 mm/s | Can be faster than perimeters |
| Travel Moves | 150-200 mm/s | Fast non-printing moves |
| Retraction Speed | 40 mm/s | Prevents stringing |

#### Cooling Settings

| Parameter | Setting | Notes |
|-----------|---------|-------|
| Part Cooling Fan | 10-30% | PETG requires minimal cooling |
| First Layer Fan | 0% | No cooling for first layer adhesion |
| Bridging Fan | 40-50% | Increase for overhangs/bridges |

**Cooling Notes**:
- PETG is sensitive to over-cooling which causes warping and poor layer adhesion
- Use minimum cooling for structural parts
- Increase cooling slightly for detailed features or overhangs

#### Retraction Settings

| Parameter | Setting | Notes |
|-----------|---------|-------|
| Retraction Distance | 0.5-1.0 mm | Start at 0.8mm for direct drive |
| Retraction Speed | 40 mm/s | Prevents nozzle clogging |
| Z-Hop on Retraction | 0.2 mm | Optional, reduces surface artifacts |

**Retraction Tuning**:
- Increase distance if stringing persists (up to 1.5mm max)
- Decrease if you observe clogging or under-extrusion after travels

#### Layer and Extrusion Settings

| Parameter | Setting | Notes |
|-----------|---------|-------|
| Layer Height | 0.2 mm | Standard quality (0.12-0.28mm range) |
| First Layer Height | 0.2 mm | Match standard layer height |
| Line Width | 0.4 mm | Match nozzle diameter |
| Infill Density | 20-40% | 20% for prototypes, 40% for structural parts |
| Infill Pattern | Gyroid or Grid | Gyroid for strength, Grid for speed |
| Wall Count | 3-4 | Minimum 3 for strength |
| Top/Bottom Layers | 4-5 | Ensures solid surfaces |

#### Bed Adhesion

| Parameter | Setting | Notes |
|-----------|---------|-------|
| Bed Surface | PEI Sheet or Glass with Glue Stick | Clean before each print |
| First Layer Flow | 105-110% | Slight over-extrusion for adhesion |
| Brim | Optional (5-10mm) | Use for small contact areas |
| Supports | Auto-generate | Enable for overhangs >50° |

**Bed Preparation**:
1. Clean PEI sheet with isopropyl alcohol before each print
2. For glass bed: Apply thin layer of glue stick or hairspray
3. Allow bed to preheat for 10 minutes to ensure even temperature distribution

---

### Elegoo Slicer Profile Template

Create a custom PETG profile in Elegoo Slicer with these settings:

```
Filament: PETG
Nozzle: 240°C (first layer 245°C)
Bed: 75°C
Chamber: 45°C
Speed: 40 mm/s (first layer 25 mm/s)
Cooling: 20% (first layer 0%)
Retraction: 0.8mm @ 40mm/s
Layer Height: 0.2mm
Walls: 4
Infill: 30% Gyroid
Supports: Auto (>50° threshold)
```

---

## Print Quality Troubleshooting

### Common PETG Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Stringing | Excessive moisture or temp too high | Dry filament at 65°C for 6 hours; reduce nozzle temp by 5°C |
| Warping | Insufficient bed temp or over-cooling | Increase bed temp to 80°C; reduce part cooling to 10% |
| Poor layer adhesion | Nozzle temp too low or wet filament | Increase nozzle temp to 245°C; dry filament thoroughly |
| Bubbling/Popping | Moisture in filament | Dry filament at 65-70°C for 6 hours minimum |
| Excessive stringing | Retraction settings incorrect | Increase retraction distance to 1.0mm; enable Z-hop 0.2mm |
| First layer not sticking | Bed not level or temp too low | Re-level bed; increase bed temp to 80°C; clean bed surface |

---

## Component-Specific Print Settings

### Chassis and Body Parts

**Files**: `body.stl` / `body.3mf`

**Priority**: Strength and durability

```
Infill: 40% Gyroid
Walls: 4
Top/Bottom Layers: 5
Layer Height: 0.2mm
Supports: Yes (auto-generated)
```

### Motor Mounts and Brackets

**Priority**: High strength, heat resistance

```
Infill: 50% Gyroid or Honeycomb
Walls: 5
Top/Bottom Layers: 6
Layer Height: 0.16mm (higher resolution)
Supports: Yes (manual placement recommended)
```

### Wheel Hubs and Drive Components

**Files**: `drive_shaft.stl` / `drive_shaft.3mf`

**Priority**: Maximum strength and precision

```
Infill: 100% Solid
Walls: 4
Top/Bottom Layers: 5
Layer Height: 0.12mm (fine detail)
Supports: Minimal (design for printability)
```

**Note**: Drive shafts must be printed at 100% infill to withstand motor torque and prevent mechanical failure during operation.

### Electronic Enclosures

**Priority**: Fit tolerance and assembly ease

```
Infill: 20% Grid (lightweight)
Walls: 3
Top/Bottom Layers: 4
Layer Height: 0.2mm
Supports: Minimal (use overhangs <45°)
```

---

## Heat-Set Insert Installation

After printing parts with heat-set insert holes:

1. **Preheat Soldering Iron**: 200-220°C for PETG
2. **Align Insert**: Position brass insert (M3 x 3mm) perpendicular to hole
3. **Apply Steady Pressure**: Press insert slowly and evenly until flush with surface
4. **Cool**: Allow part to cool for 2-3 minutes before removing iron
5. **Verify**: Test thread with M3 screw to ensure proper engagement

**Caution**: Do not exceed 250°C as PETG may deform around insert hole.

---

## Post-Processing

### Support Removal

- Remove support material carefully using flush cutters
- Sand contact points with 220-grit sandpaper for smooth finish
- Clean with compressed air to remove debris

### Surface Finishing (Optional)

- Light sanding with 400-grit sandpaper for smoother surfaces
- Acetone vapor smoothing NOT recommended for PETG (use for ABS only)
- **IMPORTANT**: Wear non-static gloves during sanding to prevent contaminants from adhering to components

### Component Washing

After support removal and sanding are complete, all printed components should be thoroughly cleaned:

1. **Wash Process**:
   - Use lukewarm water with a light detergent (dish soap works well)
   - Gently scrub all surfaces to remove dust, oils, and particulates from printing and post-processing
   - Pay special attention to areas where supports were removed
   - Rinse thoroughly with clean water to remove all detergent residue

2. **Drying**:
   - Allow components to air dry completely in a clean, dust-free environment
   - Can use compressed air to accelerate drying in hard-to-reach areas
   - Ensure components are **thoroughly dried** before proceeding to assembly
   - Typical drying time: 2-4 hours (longer for complex geometries with internal cavities)

3. **Safety Notes**:
   - Wear non-static gloves throughout the washing and drying process to prevent contamination
   - Do NOT use hot water as it may deform PETG parts
   - Avoid harsh chemicals or solvents that could weaken the plastic

### Assembly Preparation

- Test fit all components before final assembly
- Verify heat-set inserts are properly seated
- Clean all contact surfaces with isopropyl alcohol if additional cleaning is needed

---

## Electronic Assembly

### Nano I/O Shield (DFR0012) Configuration

The Nano I/O Shield provides convenient connectivity for the Arduino Nano 33 BLE Sense Rev2, but requires specific configuration to prevent damage and ensure proper operation.

#### **CRITICAL: Nano Switch Setting**

Set the shield's Nano switch to **V4** position for Arduino Nano 33 BLE Sense Rev2 compatibility.

- **Purpose**: Routes I²C signals (A4/SDA, A5/SCL) correctly to the Rev2's integrated I²C pins
- **Consequence of incorrect setting**: I²C communication failures with all connected sensors (LIDAR, encoders, etc.)

#### **CRITICAL: GND Jumper Cap Removal**

**⚠️ WARNING: FAILURE TO REMOVE THE JUMPER CAP CAN PERMANENTLY DAMAGE THE ARDUINO NANO 33 BLE SENSE REV2 ⚠️**

When using external 5V power to the shield (from buck converter) while the Arduino Nano is powered via USB from the Raspberry Pi, you **MUST** remove the GND jumper cap from the Servo power selection port.

**Jumper Cap Configuration**:
1. **Default Position**: The jumper cap is installed on the Servo power selection port connecting to USB (ships from factory in this position)
2. **Required Action**: **REMOVE** the jumper cap entirely or move it to a safe position away from the Servo power selection port
3. **Do NOT**: Leave the jumper cap connecting USB power to external servo power rail

**Why This Matters (Jumper Logic)**:
- The jumper cap in default position connects the Nano's USB 5V supply to the shield's servo/peripheral power rail
- When external 5V (from buck converter) is supplied to the shield's servo terminals, the jumper creates a parallel connection between two power sources
- This causes **voltage back-feeding** from the external regulator into the Arduino Nano's USB power circuit
- Back-feeding can exceed the USB power circuit's safe operating limits, resulting in permanent component damage
- **Removing the jumper isolates** the two power domains:
  - Arduino Nano powered via USB from Raspberry Pi (communication + microcontroller power)
  - Shield peripherals (servos, sensors) powered via external 5V from buck converter

**Power Configuration Summary**:
- **Arduino Nano 33 BLE Sense Rev2**: Powered via USB from Raspberry Pi (5V @ ~50mA)
- **Nano I/O Shield peripherals**: Powered via external 5V from buck converter connected to servo terminal blocks
- **Jumper Cap**: **REMOVED** to isolate the two power domains

### Power Distribution: Buck Converter to Nano I/O Shield

The buck converter provides 5V power to the Nano I/O Shield's servo/peripheral terminals for sensors and other connected devices.

**Wiring Instructions**:

| Buck Converter Terminal | Nano I/O Shield Terminal | Wire Gauge | Description |
|------------------------|--------------------------|------------|-------------|
| 5V Output (Positive) | Servo Terminal VCC/+ | 22-20 AWG | Regulated 5V power for peripherals |
| GND (Negative) | Servo Terminal GND/- | 22-20 AWG | Common ground return |

**Connection Notes**:
1. Use the servo terminal blocks on the Nano I/O Shield (typically labeled S1, S2, etc.)
2. Connect buck converter 5V output to the **VCC/+** pin of any servo terminal
3. Connect buck converter GND to the **GND/-** pin of the same servo terminal
4. All servo terminal power pins are internally connected, so connection to any terminal block is acceptable
5. Verify 5.0V output with multimeter before connecting to shield
6. **Ensure jumper cap is removed** as described above before applying external power

**Current Budget**:
- Total load on shield 5V rail: Sensors, encoders, and peripherals (see BOM for detailed current analysis)
- Buck converter capacity: 5A continuous (adequate margin for all shield loads)

### URM09 Ultrasonic Sensor Connection

The URM09 Ultrasonic Sensor connects to the Arduino Nano 33 BLE Sense Rev2 via the Nano I/O Shield's I²C panel.

**Connection Instructions**:

1. **Locate I²C Panel**: Find the I²C connector panel on the Nano I/O Shield (DFR0012)
2. **Connect Jumper Wires**: The URM09 includes jumper wires with the following color coding:
   - **Black**: GND (Ground)
   - **Red**: VCC (Power, 3.3-5.5V)
   - **Blue**: SCL (I²C Clock)
   - **Green**: SDA (I²C Data)

   Connect each wire to the corresponding pin on the Nano I/O Shield's I²C panel
3. **Power**: Sensor draws 20mA from the I²C connection (powered by shield's external 5V supply)
4. **I²C Address**: Default address is 0x11 (configurable via software if needed)

**Mounting Notes**:
- Mount sensor facing forward (same direction as Arduino Nano mini USB port)
- Ensure clear line of sight for ultrasonic waves (no obstructions within 2-500cm range)
- Sensor provides 1cm resolution with 1% accuracy for maze wall detection

For I²C communication protocol and Arduino firmware integration, see the [rr_ble33_mousebot repository](https://github.com/Ryder-Robots/rr_ble33_mousebot).

### D200 LIDAR Connection to Raspberry Pi

The 360° Omni-directional Triangulation LIDAR (D200 Developer Kit with LD14P) connects to the Raspberry Pi 4 Model B via UART for ROS 2 SLAM and mapping.

**Important Notes**:
- **DFR0566 GPIO Expansion HAT (Optional)**: The DFRobot GPIO Expansion HAT (DFR0566) provides convenient UART breakout terminals for connecting the D200 LIDAR. However, **the DFR0566 is optional** - the D200 can be connected directly to the Raspberry Pi GPIO pins. Both connection methods have been tested and verified to work. The DFR0566 simplifies wiring with screw terminals but direct GPIO connection is also fully supported.
- **⚠️ WIRING CAVEAT**: The cable supplied with the D200 LIDAR **does not use standard wire colors**. Always verify wire functions against the official documentation: [LD14P Development Manual (PDF)](https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf). Do not rely solely on wire colors - verify each wire's function before connecting.

#### Hardware Connection

**Connection Method 1: Using DFR0566 GPIO Expansion HAT (Recommended for Convenience)**

**D200 to Raspberry Pi 4B Wiring via DFR0566 UART Terminals**:

| D200 Wire Color | Function | DFR0566 Terminal | Raspberry Pi 4B GPIO | Physical Pin | Description |
|----------------|----------|------------------|---------------------|--------------|-------------|
| Red | RX (Receive) | UART T | GPIO14 TXD | Pin 8 | D200 receives data from Pi |
| Green | TX (Transmit) | UART R | GPIO15 RXD | Pin 10 | D200 transmits data to Pi |
| White | GND (Ground) | GND | Ground | Pin 6, 9, or 14 | Common ground |
| Black | VCC (Power) | 5V | 5V Power Rail | Pin 2 or 4 | 5V power supply |

**GPIO Pinout Reference**: [Raspberry Pi GPIO Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio)

**⚠️ CRITICAL**: Wire colors shown above are based on common D200 cable configurations. **Always verify wire functions using a multimeter or by consulting the [LD14P Development Manual](https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf)** before making connections. The cable supplied with your device may use different colors.

**Connection Explanation**:
- The D200's standard 4-wire cable aligns perfectly with Pi UART polarity through DFR0566's passthrough UART terminals
- **T terminal** (Transmit from Pi perspective) = Pi TX (GPIO14) → D200 RX (Red wire)
- **R terminal** (Receive from Pi perspective) = Pi RX (GPIO15) → D200 TX (Green wire)
- This configuration ensures proper bidirectional UART communication at 230400 baud

#### Physical Assembly

1. **Stack DFR0566 HAT on Raspberry Pi 4B**:
   - Align the 40-pin GPIO header
   - Press firmly to ensure full contact
   - Secure with standoffs if provided

2. **Connect D200 Cable to DFR0566 Terminals**:
   - Connect D200 **Red wire** (RX) to DFR0566 **UART T** terminal
   - Connect D200 **Green wire** (TX) to DFR0566 **UART R** terminal
   - Connect D200 **White wire** (GND) to DFR0566 **GND** terminal
   - Connect D200 **Black wire** (VCC) to DFR0566 **5V** terminal

3. **Verify Connections**:
   - Double-check wire colors match terminal assignments
   - Ensure wires are securely fastened in screw terminals
   - Confirm no loose connections that could cause intermittent operation

**Connection Method 2: Direct GPIO Connection (No DFR0566 Required)**

If you choose not to use the DFR0566 HAT, you can connect the D200 LIDAR directly to the Raspberry Pi 4B GPIO header. **This method has been tested and verified to work.**

**D200 to Raspberry Pi 4B Direct GPIO Wiring**:

| D200 Wire Color | Function | Raspberry Pi 4B GPIO | Physical Pin | Description |
|----------------|----------|---------------------|--------------|-------------|
| Red | RX (Receive) | GPIO14 TXD | Pin 8 | D200 receives data from Pi |
| Green | TX (Transmit) | GPIO15 RXD | Pin 10 | D200 transmits data to Pi |
| White | GND (Ground) | Ground | Pin 6, 9, or 14 | Common ground |
| Black | VCC (Power) | 5V Power Rail | Pin 2 or 4 | 5V power supply |

**GPIO Pinout Reference**: [Raspberry Pi GPIO Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio)

**⚠️ CRITICAL**: Wire colors shown above are based on common D200 cable configurations. **Always verify wire functions using a multimeter or by consulting the [LD14P Development Manual](https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf)** before making connections.

**Direct Connection Assembly**:

1. **Prepare GPIO Connections**:
   - Use female Dupont jumper wires to connect D200 cable to Raspberry Pi GPIO pins
   - Alternatively, solder wires directly to D200 cable and use female Dupont connectors for Pi GPIO

2. **Make Connections**:
   - Connect D200 **Red wire** (RX) to Pi **GPIO14 TXD** (Physical Pin 8)
   - Connect D200 **Green wire** (TX) to Pi **GPIO15 RXD** (Physical Pin 10)
   - Connect D200 **White wire** (GND) to Pi **Ground** (Physical Pin 6, 9, or 14)
   - Connect D200 **Black wire** (VCC) to Pi **5V** (Physical Pin 2 or 4)

3. **Verify Connections**:
   - Double-check wire colors match GPIO pin assignments
   - Ensure secure connections at GPIO header
   - Confirm no loose connections that could cause intermittent operation
   - **IMPORTANT**: Verify correct GPIO pins before applying power to prevent damage

#### Raspberry Pi UART Configuration

The Raspberry Pi's hardware UART must be enabled and configured for the D200 LIDAR.

**Enable UART via raspi-config**:

1. Open terminal on Raspberry Pi
2. Run configuration utility:
   ```bash
   sudo raspi-config
   ```
3. Navigate to: **Interface Options** → **Serial Port**
4. Configure as follows:
   - **"Would you like a login shell to be accessible over serial?"** → Select **NO**
     - This disables the console login shell on the serial port
   - **"Would you like the serial port hardware to be enabled?"** → Select **YES**
     - This enables the UART hardware interface
5. Exit raspi-config and reboot:
   ```bash
   sudo reboot
   ```

**Verification**:

After reboot, verify UART is enabled:
```bash
ls -l /dev/serial0
```

Expected output should show `/dev/serial0` linked to `/dev/ttyAMA0` (hardware UART).

**Test LIDAR Wiring and Communication**:

To verify the D200 LIDAR is properly connected and transmitting data:
```bash
stty -F /dev/ttyAMA0 230400 && cat /dev/ttyAMA0
```

**Expected Result**: You should see binary data streaming from the LIDAR. The output will appear as scrambled characters (this is normal - it's raw LIDAR scan data). Press `Ctrl+C` to stop.

**If no data appears**:
- Verify power connection (Black wire to 5V)
- Check ground connection (White wire to GND)
- Verify TX/RX connections are correct (Green to UART R, Red to UART T)
- Confirm LIDAR is spinning (motor should be audible/visible)
- Double-check wire functions against the [LD14P Development Manual](https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf)

#### UART Hardware Testing on Raspberry Pi 5 (Development Note)

**Note**: When testing the D200 LIDAR hardware on Raspberry Pi 5 for development purposes, UART configuration can be difficult to troubleshoot. While GPIO headers remain the same as other Raspberry Pi models (see [Raspberry Pi GPIO Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio)), it's recommended to verify UART functionality before connecting the LIDAR.

**UART Loopback Test Procedure**:

1. **Create Hardware Loopback**:
   - Place a jumper wire between **Pin 8 (GPIO14 TXD)** and **Pin 10 (GPIO15 RXD)**
   - This creates a loopback where transmitted data is immediately received back

2. **Install minicom** (if not already installed):
   ```bash
   sudo apt-get install minicom
   ```

3. **Launch minicom**:
   ```bash
   minicom -b 115200 -D /dev/ttyAMA0
   ```

4. **Disable Echo**:
   - Press `Ctrl+A` then `E` to toggle local echo OFF
   - This prevents double-character display (local echo + loopback echo)

5. **Test Serial Communication**:
   - Type any random characters on the keyboard
   - **Expected Result**: Characters should be echoed back in the terminal
   - If characters appear, UART hardware is working correctly
   - If no characters appear, UART may not be properly enabled or configured

6. **Exit minicom**:
   - Press `Ctrl+A` then `X` to exit
   - Remove the jumper wire before connecting the D200 LIDAR

**Troubleshooting**:
- If loopback test fails, verify UART is enabled in `raspi-config` (Interface Options → Serial Port)
- Check `/boot/firmware/config.txt` for `enable_uart=1`
- Ensure no other service is using `/dev/ttyAMA0` (check with `sudo lsof | grep ttyAMA0`)
- On Raspberry Pi 5, UART configuration may differ slightly from Pi 4 - consult official documentation if issues persist

#### GPIO Testing with WiringPi

WiringPi provides a convenient command-line tool for testing and verifying GPIO pin configurations on Raspberry Pi.

**WiringPi Repository**: https://github.com/WiringPi/WiringPi

**Installation**:

1. **Create temporary directory and download**:
   ```bash
   mkdir -p ${HOME}/tmp
   cd ${HOME}/tmp
   wget https://github.com/WiringPi/WiringPi/releases/download/3.16/wiringpi_3.16_arm64.deb
   chmod 755 ${HOME}/tmp
   ```

2. **Verify download integrity**:
   ```bash
   sha256sum ./wiringpi_3.16_arm64.deb
   ```

   **Expected hash**: `db494454383e72b9af5ca1617a77e571da1ceb4f50279f7efe45bb4c3c863ff6`

   ⚠️ **Do not proceed if the hash does not match** - re-download or check for updated release.

3. **Install the package**:
   ```bash
   sudo apt install ./wiringpi_3.16_arm64.deb
   ```

4. **Add user to GPIO group** (required for non-root GPIO access):
   ```bash
   sudo usermod -a -G gpio $USER
   ```
   Log out and log back in for group membership to take effect.

**Test GPIO Configuration**:

After installation, verify all GPIO pins are accessible and correctly configured:

```bash
gpio readall
```

This displays a table showing all GPIO pins, their modes (IN/OUT/ALT), and current values. Use this to:
- Verify UART pins (GPIO14/GPIO15) are in ALT mode for serial communication
- Confirm GPIO pins are available for encoder inputs
- Debug pin configuration issues before connecting hardware

#### ROS 2 Integration

**UART Device Path**: `/dev/serial0`
**Baud Rate**: 230400

The `ldlidar_ros2` lifecycle node will automatically detect and configure the D200 LIDAR on `/dev/serial0` at 230400 baud for ROS 2 `/scan` topic publishing.

**Node Configuration** (in launch file):
```python
'serial_port': '/dev/serial0',
'serial_baudrate': 230400,
'topic_name': 'scan',
'lidar_frame': 'base_laser',
'range_threshold': 0.005  # 5mm minimum range
```

**Setup Confirmation**:

1. Hardware: DFR0566 stacked on Pi 4B, D200 cable connected to UART/5V/GND terminals exactly as specified
2. Software: UART enabled via `raspi-config` (login shell disabled, hardware enabled)
3. ROS 2: `ldlidar_ros2` lifecycle node configured for `/dev/serial0` at 230400 baud
4. Result: Automatic `/scan` topic publishing for SLAM and navigation

For ROS 2 LIDAR driver configuration and lifecycle management, see the main launch file in this repository.

#### LD200 LIDAR ROS 2 Software Setup

The LD200 LIDAR (LD14P sensor) requires the `ldrobot-lidar-ros2` driver package for ROS 2 integration. This driver provides native ROS 2 Lifecycle node support with NAV2 lifecycle manager integration.

**Driver Repository**: https://github.com/Myzhar/ldrobot-lidar-ros2

**Installation Instructions**:

1. **Install dependencies**:
   ```bash
   sudo apt install libudev-dev
   ```

2. **Clone the ldrobot-lidar-ros2 repository** into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Myzhar/ldrobot-lidar-ros2.git
   ```

3. **Apply ROS 2 Kilted Patch** (REQUIRED for Kilted):

   The upstream driver does not yet support ROS 2 Kilted. Apply the patch from this repository:

   ```bash
   cd ~/ros2_ws/src/ldrobot-lidar-ros2
   patch -p1 < ../rr_mousebot_bringup/scripts/ldrobot-lidar-ros2-kilted.patch
   ```

   **Patch Contents** (for reference):
   - Adds `kilted` to the supported ROS 2 distributions list
   - Adds `-DFOUND_KILTED` preprocessor definition for Kilted builds
   - Sets default serial port to `/dev/ttyAMA0` (Raspberry Pi hardware UART)

   **Verify patch applied successfully**:
   ```bash
   git diff --stat
   ```

   Expected output shows modifications to `ldlidar_component/CMakeLists.txt`, `ldlidar_node/CMakeLists.txt`, and `ldlidar_node/params/ldlidar.yaml`.

4. **Install ROS dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. **Build the package**:
   ```bash
   colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --packages-select ldlidar ldlidar_component ldlidar_node
   ```

6. **Source the workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

**Device Configuration**:

The LD200 LIDAR connects via UART and appears as `/dev/ttyAMA0` on both Raspberry Pi 4B and Raspberry Pi 5 (hardware UART).

**Verify the device**:
```bash
ls -l /dev/ttyAMA0
```

**Test LIDAR Communication**:
```bash
stty -F /dev/ttyAMA0 230400 && cat /dev/ttyAMA0
```

Expected result: Binary data stream (appears as scrambled characters). Press `Ctrl+C` to stop.

**D200/LD14P Model Configuration**:

The D200 Developer Kit contains the LD14P sensor, which uses the same protocol as the LD19. Configure the driver to use `LDLiDAR_LD19` model.

The patch sets `/dev/ttyAMA0` as the default serial port. If you need to customize other parameters, edit `ldlidar_node/params/ldlidar.yaml`:

```yaml
/**:
  ros__parameters:
    general:
      debug_mode: false

    comm:
      serial_port: '/dev/ttyAMA0'  # Raspberry Pi hardware UART
      baudrate: 230400
      timeout_msec: 1000

    lidar:
      model: 'LDLiDAR_LD19'  # Use LD19 for D200/LD14P (same protocol)
      rot_verse: 'CCW'       # Counter-clockwise (ROS standard)
      units: 'M'             # Meters (ROS standard)
      frame_id: 'ldlidar_link'
      bins: 455              # Fixed scan size for SLAM Toolbox compatibility
      range_min: 0.03
      range_max: 15.0
```

**Launch the LIDAR Driver**:

The ldrobot-lidar-ros2 package is based on ROS 2 Lifecycle architecture. The node starts in `UNCONFIGURED` state.

**Option 1: Manual Lifecycle Control**

Start the node:
```bash
ros2 run ldlidar_node ldlidar_node
```

Configure the node (establishes connection):
```bash
ros2 lifecycle set /ldlidar_node configure
```

Activate the node (starts publishing scans):
```bash
ros2 lifecycle set /ldlidar_node activate
```

**Option 2: Launch with YAML Parameters**

```bash
ros2 launch ldlidar_node ldlidar.launch.py
```

This starts the node with parameters from `ldlidar.yaml` and a `robot_state_publisher` that provides the static TF transform [`ldlidar_base` → `ldlidar_link`].

**Option 3: Launch with NAV2 Lifecycle Manager (Recommended)**

```bash
ros2 launch ldlidar_node ldlidar_with_mgr.launch.py
```

This automatically handles lifecycle transitions using the NAV2 `lifecycle_manager`.

**Verify LIDAR Data**:

Check that the LIDAR is publishing scan data:
```bash
ros2 topic echo /ldlidar_node/scan
```

Expected output: `sensor_msgs/LaserScan` messages with range data.

**Lifecycle Node Integration** (for power management):

The `ldrobot-lidar-ros2` driver is a native ROS 2 Lifecycle node, ideal for the power-saving strategy described in the main README (LIDAR inactive during speed runs).

**Lifecycle Management**:

1. **Check node lifecycle state**:
   ```bash
   ros2 lifecycle get /ldlidar_node
   ```

2. **Configure the node** (establish connection):
   ```bash
   ros2 lifecycle set /ldlidar_node configure
   ```

3. **Activate the node** (start LIDAR scanning):
   ```bash
   ros2 lifecycle set /ldlidar_node activate
   ```

4. **Deactivate the node** (stop LIDAR scanning, save power):
   ```bash
   ros2 lifecycle set /ldlidar_node deactivate
   ```

5. **Cleanup the node**:
   ```bash
   ros2 lifecycle set /ldlidar_node cleanup
   ```

6. **Shutdown the node**:
   ```bash
   ros2 lifecycle set /ldlidar_node shutdown
   ```

**Power Management Strategy**:
- **Explore Mode**: Activate LIDAR for SLAM and mapping
- **Planning Phase**: Deactivate LIDAR to conserve power (~300mA savings)
- **Run Mode**: Keep LIDAR deactivated during speed runs

This lifecycle approach enables the 150-175 runs per battery charge as described in the README's "Lifecycle Management Approach" section.

**Integration with System Launch File**:

For integration into the main `rr_mousebot.launch.py` launch file, refer to the ldrobot-lidar-ros2 repository documentation and your existing lifecycle node patterns in the launch file.

**Troubleshooting**:

- **Permission denied on /dev/ttyAMA0**: Add your user to the `dialout` group:
  ```bash
  sudo usermod -aG dialout $USER
  ```
  Log out and log back in for changes to take effect.

- **Transitioning failed on configure**: Check node logs for connection errors. Verify UART is enabled and LIDAR is powered (motor should be spinning).

- **Invalid lidar.model error**: Ensure `model: 'LDLiDAR_LD19'` is set in `ldlidar.yaml`. The D200/LD14P uses the LD19 protocol. Valid models are: `LDLiDAR_LD06`, `LDLiDAR_LD19`, `LDLiDAR_STL27L`.

- **No data on /ldlidar_node/scan topic**: Ensure node is in `ACTIVE` state. Check `ros2 lifecycle get /ldlidar_node`.

- **Device not found**: Confirm `/dev/ttyAMA0` exists and UART hardware is enabled (see "Raspberry Pi UART Configuration" section above).

- **Build fails with "ROS2 kilted is not officially supported"**: Ensure the Kilted patch has been applied. Re-run the patch command from step 3.

### Motor and Encoder Wiring

The Micro DC Geared Motors with integrated encoders (FIT0450) connect to both the Makerverse Motor Driver 2 Channel (CE08038) for motor control and to the Arduino Nano 33 BLE Sense Rev2 via the Nano I/O Shield (DFR0012) for encoder signals.

**Motor Documentation**: [DFRobot Micro DC Motor with Encoder-SJ01 Wiki](https://wiki.dfrobot.com/Micro_DC_Motor_with_Encoder-SJ01_SKU__FIT0450)

**Arduino Pinout Reference**: [Arduino Nano 33 BLE Sense Rev2 Pinout](https://docs.arduino.cc/resources/pinouts/ABX00031-full-pinout.pdf)

**Connection Notes**:
- DIR pins control motor rotation direction (HIGH/LOW)
- PWM pins control motor speed (0-255 duty cycle)
- All control signals are 3.3V logic compatible with Arduino Nano 33 BLE Sense Rev2

#### Motor A (Left Motor) Complete Wiring

Each motor has a 6-pin connector with the following connections:

| Motor Pin | Pin Name | Connected Device | Device Pin | Description |
|-----------|----------|------------------|------------|-------------|
| 1 | Motor Power + | Motor Driver (MKRVRS) | A+ | Motor power supply +, 3-7.5V (Rated 6V) |
| 2 | Motor Power - | Motor Driver (MKRVRS) | A- | Motor power supply - |
| 3 | Encoder A Phase | Nano I/O Shield | D8 | Encoder A phase output (digital) |
| 4 | Encoder B Phase | Nano I/O Shield | D9 | Encoder B phase output (digital) |
| 5 | Encoder VCC | Nano I/O Shield | VCC (5V) | Encoder power supply (5V) |
| 6 | Encoder GND | Nano I/O Shield | GND | Encoder ground |

**Encoder Specifications**:
- 16 PPR (pulses per revolution) at motor shaft
- 1920 counts per revolution at output shaft (16 PPR × 120:1 gear ratio)
- Hall effect sensor type
- Operating voltage: 5V DC

#### Motor B (Right Motor) Complete Wiring

| Motor Pin | Pin Name | Connected Device | Device Pin | Description |
|-----------|----------|------------------|------------|-------------|
| 1 | Motor Power + | Motor Driver (MKRVRS) | B+ | Motor power supply +, 3-7.5V (Rated 6V) |
| 2 | Motor Power - | Motor Driver (MKRVRS) | B- | Motor power supply - |
| 3 | Encoder A Phase | Nano I/O Shield | D10 | Encoder A phase output (digital) |
| 4 | Encoder B Phase | Nano I/O Shield | D11 | Encoder B phase output (digital) |
| 5 | Encoder VCC | Nano I/O Shield | VCC (5V) | Encoder power supply (5V) |
| 6 | Encoder GND | Nano I/O Shield | GND | Encoder ground |

#### Motor Rotation Direction

**Motor A Clockwise Rotation**:
- Condition: Motor Power + (Pin 1) is positive via A+, Motor Power - (Pin 2) is negative via A-, DIR-A is HIGH
- Result: Motor shaft rotates clockwise when viewed from the shaft end
- Use case: Forward motion for left wheel

**Motor A Counter-Clockwise Rotation**:
- Condition: DIR-A is LOW (polarity reversal via H-bridge)
- Result: Motor shaft rotates counter-clockwise
- Use case: Reverse motion for left wheel

**Motor B Clockwise Rotation**:
- Condition: Motor Power + (Pin 1) is positive via B+, Motor Power - (Pin 2) is negative via B-, DIR-B is HIGH
- Result: Motor shaft rotates clockwise when viewed from the shaft end
- Use case: Forward motion for right wheel

**Motor B Counter-Clockwise Rotation**:
- Condition: DIR-B is LOW (polarity reversal via H-bridge)
- Result: Motor shaft rotates counter-clockwise
- Use case: Reverse motion for right wheel

#### Encoder Reading

**Quadrature Encoder Operation**:
- Each motor has two encoder output signals: A Phase and B Phase
- The phase relationship between A and B determines rotation direction
- Forward rotation: A leads B by 90° (A changes state before B)
- Reverse rotation: B leads A by 90° (B changes state before A)
- Each rising and falling edge can be counted for 4x resolution: 16 PPR × 4 = 64 counts/rev (motor shaft), 7680 counts/rev (output shaft)

**Encoder Pin Connections Summary**:
- Motor A: D8 (A Phase), D9 (B Phase)
- Motor B: D10 (A Phase), D11 (B Phase)
- Both encoders powered by 5V from Nano I/O Shield

#### Wiring Best Practices

1. **Color Coding Consistency**:
   - Use white wires for all DIR (direction) signals
   - Use yellow wires for all PWM (speed) signals
   - Motors include pre-soldered 6-pin connectors

2. **Wire Management**:
   - Secure wires with cable ties to prevent interference with moving parts
   - Keep motor power wires (pins 1-2) separated from encoder signal wires (pins 3-6) to minimize electrical noise
   - Route encoder signals away from motor driver PWM signals
   - Use shielded cable for encoder signals if noise issues occur

3. **Testing Sequence**:
   - Test encoder signals first before connecting motor power
   - Verify encoder counts increment/decrement correctly with manual rotation
   - Test each motor individually before full assembly
   - Verify rotation direction matches expected behavior
   - Confirm PWM speed control operates smoothly across 0-255 range
   - Verify encoder direction detection (A leads B vs B leads A)

4. **Safety**:
   - Double-check polarity before applying power
   - Ensure motor driver heat sink has adequate clearance for cooling
   - Monitor motor driver temperature during initial testing
   - Verify encoder VCC is connected to 5V (not 3.3V) for proper operation

For motor control and encoder reading firmware implementation, see the [rr_ble33_mousebot repository](https://github.com/Ryder-Robots/rr_ble33_mousebot).

---

## Material Storage

**PETG Storage Recommendations**:
- Store in airtight container with desiccant packets
- Ideal humidity: <15% RH
- If stored >1 week, re-dry at 60°C for 2-4 hours before use
- Keep away from direct sunlight and heat sources

---

## ABS Configuration (Future)

ABS settings for final production builds will be documented here once prototyping phase is complete. ABS requires:
- Higher temperatures (230-260°C nozzle, 100-110°C bed)
- Enclosed chamber (mandatory)
- Reduced cooling
- Acetone vapor smoothing capability

---

## Safety Notes

- **Ventilation**: Ensure adequate ventilation when printing PETG
- **Temperature**: Allow heated bed and nozzle to cool before handling
- **Fire Safety**: Never leave printer unattended during long prints
- **Soldering Iron**: Use proper stand and heat-resistant surface for heat-set insert installation

---

## Print Time Estimates

| Component | STL File | Estimated Print Time | Material Usage |
|-----------|----------|---------------------|----------------|
| Main Chassis | `body.stl` / `body.3mf` | 8-12 hours | 200-300g |
| Drive Shaft (pair) | `drive_shaft.stl` / `drive_shaft.3mf` | 1-2 hours | 20-30g |
| Motor Mounts (pair) | (Future release) | 2-4 hours | 40-60g |
| Electronic Enclosures | (Future release) | 3-6 hours | 80-120g |

**Total Estimated Material**: 350-530g PETG for complete robot

**Note**: Additional STL files for motor mounts, wheel hubs, and electronic enclosures will be added to `schematic/stl/` in future releases.

---

## References

- Elegoo Centauri Carbon Manual: [Manufacturer Documentation]
- Creality Filament Dryer Settings: [Product Specifications]
- PETG Material Properties: [Material Database]
- Elegoo Slicer Documentation: [Elegoo Support]

---

## Revision History

- 2025-12-29: Initial documentation with PETG prototyping settings
- Future: ABS production settings to be added

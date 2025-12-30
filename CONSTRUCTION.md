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

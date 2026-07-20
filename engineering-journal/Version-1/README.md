# Robot Version 1

| Category | Details |
|---|---|
| Version | Version 1 |
| Development Period | Add dates |
| Objective | Add the main objective of this version |
| Status | Draft / Tested / Archived / Current |
| Chassis | Add design summary |
| Main Controller | Raspberry Pi 5 |
| Secondary Controller | ESP32 |
| Drive System | Add motor, gearing, axle, and wheel information |
| Steering System | Add steering mechanism and servo information |
| Sensors | Add camera, LiDAR, IMU, encoders, and other sensors |
| Robot Dimensions | Add length × width × height |
| Robot Mass | Add measured mass |
| Main Challenges | Add the most important limitations |
| Outcome | Explain whether the version was kept, revised, or replaced |

---

## 1. Version Overview

Explain what this version was intended to achieve and what changed compared with the previous version.

## 2. Mechanical Design

### 2.1 Chassis

Describe:

- chassis shape and material,
- dimensions,
- mounting method,
- rigidity,
- center of mass,
- and accessibility for maintenance.

### 2.2 Drive System

Document:

- motor model and rated voltage,
- gearbox ratio,
- wheel diameter,
- expected and measured speed,
- torque reasoning,
- axle connection,
- and why this drive solution was selected.

### 2.3 Steering System

Document:

- steering geometry,
- steering actuator,
- maximum steering angle,
- turning radius,
- mechanical play,
- and alignment procedure.

### 2.4 Custom Parts

For each custom part, include:

- purpose,
- CAD screenshot,
- dimensions,
- material,
- manufacturing method,
- and final file location in `models/`.

## 3. Electrical and Power System

### 3.1 Power Architecture

Explain:

- battery voltage and capacity,
- motor power rail,
- Raspberry Pi and ESP32 power rail,
- regulators,
- fuses or protection,
- switches,
- grounding,
- and estimated peak current.

### 3.2 Wiring

Link to the wiring diagram in `schemes/electrical/`.

### 3.3 Problems and Risks

Examples:

- voltage drop,
- regulator overheating,
- electrical noise,
- loose connectors,
- shared-ground issues,
- and motor-current spikes.

## 4. Sensors and Placement

For every sensor, document:

| Sensor | Purpose | Position | Interface | Why This Position? |
|---|---|---|---|---|
| Camera |  |  | CSI / USB |  |
| LiDAR |  |  | UART / USB |  |
| IMU |  |  | I2C |  |
| Encoder |  |  | Digital |  |

Include field-of-view, blind spots, mounting height, calibration, and environmental limitations.

## 5. Software

### 5.1 Architecture

Describe the Raspberry Pi and ESP32 responsibilities.

### 5.2 Main States

Example states:

- waiting for start,
- initialization,
- direction detection,
- localization,
- lane following,
- obstacle handling,
- cornering,
- lap counting,
- parking search,
- parallel parking,
- safe stop.

### 5.3 Algorithms

Document the algorithms used for:

- lane or wall following,
- localization,
- obstacle detection,
- path planning,
- steering control,
- speed control,
- sensor fusion,
- and recovery from errors.

### 5.4 Code Location

Link to the relevant files in `src/`.

## 6. Testing

For every test, record:

| Test | Setup | Success Metric | Result | Decision |
|---|---|---|---|---|
| Straight-line test |  |  |  |  |
| Turning test |  |  |  |  |
| Sensor detection test |  |  |  |  |
| Full-lap test |  |  |  |  |

Link to detailed test files inside `tests/`.

## 7. Problems and Failure Modes

| Failure Mode | Observed Effect | Suspected Cause | Corrective Action | Status |
|---|---|---|---|---|
| Example |  |  |  | Open / Solved |

## 8. Decisions and Changes

Use evidence-based statements such as:

> We selected X instead of Y because test results showed...

## 9. Photos and Files

Place version-specific images in this version's `images/` folder.

Recommended files:

- `front.jpg`
- `top.jpg`
- `bottom.jpg`
- `wiring.jpg`
- `cad-overview.png`
- `test-setup.jpg`

## 10. Transition to the Next Version

Explain:

- what was retained,
- what was removed,
- what needed redesign,
- and the goals for the next version.

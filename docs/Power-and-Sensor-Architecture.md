# ⚡ Power & Sensor Architecture

> **Rubric criterion 2 — Power & Sensor Architecture.** This document covers the power system layout, the power budget, wiring, and every sensor: what it is, where it is placed, why, and how it is calibrated. The full wiring diagram lives in [`/schemes`](../schemes) and the exact pin map in [`other/pinout.md`](../other/pinout.md).

---

## 1. Design principle: three isolated sources, one star ground

The single most important power decision was to **split the vehicle onto three isolated power sources** and join them at a **single star ground**. Motors and servos create large, noisy current spikes; the Raspberry Pi and the I²C sensors are sensitive to voltage sag and ground bounce. Sharing one battery caused **brownouts and I²C errors** in early testing. Isolating the loads fixed it.

```
   ┌──────────────┐
   │ 9V pack #1   │──[switch]──► DRV8833 VM ──► DRIVE MOTOR
   └──────────────┘                              (noisy, high current)

   ┌──────────────┐
   │ 9V pack #2   │──► XL4015 buck (5.0V) ──► STEERING SERVO VCC
   └──────────────┘                              (spiky, inductive)

   ┌──────────────┐
   │ 2S 18650     │──► buck (5.1V) ──► USB-C ──► RASPBERRY PI 4
   │ Li-ion 7.4V  │                              (sensitive logic)
   └──────────────┘

   ┌──────────────┐
   │ RPLidar A1M8 │──── single USB cable (power + data) ──► Pi
   └──────────────┘

   ── ALL GROUNDS ──► ONE STAR POINT ──► single ref wire ──► Pi Pin 9
```

**Key rule:** the servo's return current is routed to *its own* converter ground, **never through the Pi**. This keeps servo current spikes out of the logic ground.

---

## 2. Power budget

| Rail | Source | Regulator | Feeds | Approx. draw |
|---|---|---|---|---|
| Motor VM | 9 V pack #1 | none (direct to DRV8833) | Drive motor | High, spiky (stall inrush) |
| Servo 5.0 V | 9 V pack #2 | XL4015 buck | Steering servo | Spiky under load |
| Pi 5.1 V | 2S 18650 (7.4 V) | buck → USB-C | Pi 4 + IMU + encoder | ~0.6–1.2 A continuous |
| LiDAR 5 V | Pi USB | (from Pi supply) | RPLidar A1M8 motor + sensor | ~0.3–0.5 A |
| 3.3 V logic | Pi onboard | Pi regulator | MPU6050, encoder | Low |

> ⚠️ **Why the Pi gets its own Li-ion pack:** the Pi + LiDAR is the largest *sustained* draw and the least tolerant of voltage sag. A dedicated 2S 18650 with a buck to 5.1 V keeps it above the brownout threshold even while the motor stalls.

---

## 3. Wiring / signal connections

Signal wiring from the Pi GPIO header (full table in [`other/pinout.md`](../other/pinout.md)):

| Pi pin | GPIO | Connects to |
|---|---|---|
| 1 | 3V3 | Encoder VCC, MPU6050 VCC |
| 3 | GPIO2 (SDA) | MPU6050 SDA |
| 5 | GPIO3 (SCL) | MPU6050 SCL |
| 9 | GND | **Star ground reference** |
| 11 | GPIO17 | Encoder A |
| 12 | GPIO18 | Servo signal (**hardware PWM** via pigpio) |
| 13 | GPIO27 | Encoder B |
| 16 | GPIO23 | DRV8833 IN1 |
| 18 | GPIO24 | DRV8833 IN2 |

Full Fritzing wiring diagram: [`/schemes`](../schemes).

---

## 4. Sensor suite

The vehicle localizes and navigates with three sensors. There is no GPS or external reference — everything is **onboard dead reckoning + LiDAR ranging**.

### 4.1 MPU6050 — IMU / gyro (heading)

| Property | Value |
|---|---|
| Interface | I²C (SDA GPIO2, SCL GPIO3) |
| Used axis | Z-axis gyro (yaw rate) |
| Placement | Top deck, near the vehicle's center of rotation |
| Role | Primary **heading** source — integrated to drive the steering PID |

**Placement reasoning:** mounting the IMU near the yaw axis minimizes translational acceleration bleeding into the gyro signal, giving a cleaner heading integral.

**Calibration & failure handling:**
- **Bias calibration at startup:** 300 samples of Z-gyro are averaged with the robot held still to compute `gyro_offset`, which is subtracted from every reading (`calibrate_gyro()`).
- **Thermal drift:** we measured ~1°/3 s drift traced to thermal bias. Mitigation: **re-calibrate at the start line after a 60 s warm-up**.
- **Deadzone:** small rates below `GYRO_DEADZONE` are zeroed so slow sensor noise doesn't accumulate into false heading, while real veers still register.

### 4.2 Rotary encoder — odometry (distance)

| Property | Value |
|---|---|
| Interface | Quadrature A/B (GPIO17, GPIO27) |
| Counts/rev | **205 (measured experimentally)** |
| Wheel diameter | 44 mm |
| Derived | `MM_PER_COUNT = π·44 / 205 ≈ 0.674 mm` |
| Direction | `ENCODER_DIRECTION = -1` |

**Calibration:** counts/rev were **measured**, not assumed, then **cross-checked** by pushing the robot exactly 500 mm along a tape measure and confirming the odometry read ~500 mm (`test_distance.py`).

### 4.3 RPLidar A1M8 — ranging (walls & corners)

| Property | Value |
|---|---|
| Interface | USB (`/dev/ttyUSB0` @ 115200), power + data on one cable |
| Placement | Highest point on the top deck (clear 360° sweep) |
| Mount orientation | **180° rotated** — compensated in software |
| Role | Wall-centering trim + front-wall corner trigger |

**Direction windows (robot frame, after the 180° remap):**

| Window | Angles | Used for |
|---|---|---|
| Front | 170–190° | Corner trigger (`front < 600 mm`) |
| Left | 80–100° | Centering + open-corridor detection |
| Right | 260–280° | Centering + open-corridor detection |
| Back | 350–360° / 0–10° | (available) |

**Calibration & failure handling:**
- Four-direction wall distances validated against a tape measure (`test_lidar_pi.py`).
- **Stale-stream bug fix:** a `stop()` + `reset()` is issued at startup after we discovered stale-descriptor errors on reconnect.
- Points with `quality == 0` or out of `MAX_DIST` (3000 mm) are discarded before use.

### 4.4 Raspberry Pi Camera — traffic-sign color (Obstacle Challenge)

| Property | Value |
|---|---|
| Part | 5 MP 175° fisheye CSI camera (OV5647, 1.7 mm lens) |
| Interface | CSI ribbon to the Pi |
| Placement | Front, facing forward, above the bumper |
| Role | Reads **red vs. green** traffic-sign pillars |
| Power | From the Pi (no separate rail) |

**How it's used:** LiDAR finds a pillar's *bearing*; the camera samples the HSV color at that bearing to decide the pass side. Red is matched with two hue ranges (hue wraps at 0/180°); a minimum pixel count rejects ambiguous frames. Details in [Software Architecture §6.3](./Software-Architecture.md#63-detecting-the-signs-sensor-fusion-lidar--camera). The Open Challenge does not use the camera.

---

## 5. Sensor fusion summary

```
 ENCODER ──(distance)──┐
                       ├──► DEAD-RECKONED POSE (x, y) ──► live map + odometry
 GYRO ─────(heading)───┘
                       └──► heading feeds STEERING PID (straights)
                                          + TURN profile (corners)

 LiDAR ──(front)──► corner trigger
       ──(left/right)──► wall-centering trim  +  turn-direction detection
```

Heading comes **only** from the gyro; distance comes **only** from the encoder; LiDAR never feeds the pose estimate directly — it corrects *where in the lane* the car sits and *when* to turn. This clean separation is what makes the failure modes easy to reason about (see [Systems Engineering](./Systems-Engineering.md)).

---

## 6. Reproducibility

- Wiring diagram: [`/schemes`](../schemes)
- Exact pinout + calibrated constants: [`other/pinout.md`](../other/pinout.md)
- Per-sensor bring-up scripts: [`src/tests/`](../src/tests)
- Component test log: [`other/test-procedure.md`](../other/test-procedure.md)
- Parts & sourcing: [`other/BOM.md`](../other/BOM.md)

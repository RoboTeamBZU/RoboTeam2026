# 🔧 Mechanical Design — Mobility & Chassis

> **Rubric criterion 1 — Mobility & Mechanical Design.** This document explains *what* the vehicle is built from, *why* each mechanical choice was made, and *how* those choices were validated through testing and iteration.

---

## 1. Overview

The vehicle is a **single-motor, front-steering (Ackermann-style) autonomous car** built on a rigid two-deck chassis. One brushed DC motor drives the rear axle; a servo steers the front wheels. This is the classic "car-like" layout the WRO Future Engineers rules assume, and it keeps the mechanics simple so engineering effort could go into sensing and control.

| Property | Value |
|---|---|
| Drive type | Rear-wheel drive, 1 × brushed DC motor |
| Steering | Front, servo-actuated Ackermann linkage |
| Wheel diameter | 44 mm |
| Track / wheelbase | See CAD in [`/models`](../models) |
| Turning basis | Gyro-closed-loop, ±60° servo range |
| Chassis | Two-deck (drivetrain below, electronics above) |
| Compute | Raspberry Pi 4 |

📷 *Add labeled photos to [`/v-photos`](../v-photos) (vehicle) and CAD renders to [`/models`](../models).*

---

## 2. Chassis structure

```
        ┌─────────────────────────────┐   ← Upper deck: Pi, IMU, LiDAR,
        │   [Pi]  [LiDAR]   [IMU]      │     power distribution
        │                             │
        └───────────┬─────────────────┘
                    │ standoffs (4×)
        ┌───────────┴─────────────────┐   ← Lower deck: motor, driver,
   ┌──┐ │ [servo]        [motor+enc]  │ ┌──┐  battery packs, servo
   │  │ │      Ackermann     ▐▐       │ │  │
   └──┘ └─────────────────────────────┘ └──┘
   front                                 rear
 (steered)                            (driven)
```

**Why two decks?** Separating the drivetrain (lower) from the sensing/compute stack (upper) does three things:

1. **Keeps the LiDAR plane clear.** The RPLidar A1M8 needs an unobstructed 360° sweep. Mounting it on the top deck above the tallest components guarantees a clean scan of the field walls.
2. **Shortens and isolates wiring.** High-current motor wiring stays on the lower deck; low-level signal wiring (I²C, encoder, servo PWM) stays up top, reducing noise coupling.
3. **Lowers the center of mass** by keeping the heavy battery packs low, which improves stability during hard cornering.

The two decks are joined by four metal standoffs, giving a torsionally rigid frame — important because chassis flex would change the LiDAR-to-wall geometry mid-run and corrupt the wall-centering trim.

---

## 3. Drivetrain — motor & speed/torque reasoning

A single **brushed DC gearmotor** drives the rear axle through the wheels (44 mm diameter). Drive power comes from a dedicated 9 V pack through a **DRV8833** H-bridge driver.

### Speed vs. torque trade-off

The WRO field is a small arena; the vehicle needs **enough top speed to be competitive but enough low-end torque to launch cleanly from a standstill** and to hold speed through gyro-corrected steering corrections. Our reasoning:

| Requirement | Consequence for gearing |
|---|---|
| Fast, clean laps | Favor higher RPM |
| Reliable start from rest | Need standstill torque |
| Predictable control loop | Speed must be repeatable, not twitchy |

We run the motor at **80 % duty on straights** (`SPEED_STRAIGHT = 0.8`) and **55 % in turns** (`SPEED_TURN = 0.55`) — slowing for corners keeps the gyro turn profile repeatable and prevents understeer.

### The standstill-torque problem (and fix)

Early testing (see [engineering journal](../engineering-journal)) showed the motor could **stall at low duty when starting from rest** — not enough torque to overcome static friction. We solved it two ways:

1. **Mechanically** — reduced drivetrain friction and confirmed free-spinning wheels.
2. **In software** — a **kick pulse**: the driver briefly commands full power (`KICK_SPEED = 1.0` for `KICK_TIME = 0.15 s`) whenever the motor starts from rest, then drops to the commanded duty. This is retained as a safety margin. See `motor()` in [`src/wro_open.py`](../src/wro_open.py).

---

## 4. Steering — Ackermann servo linkage

The front wheels are steered by a single **servo** driving an Ackermann-style linkage. Key parameters (from [`src/wro_open.py`](../src/wro_open.py)):

| Parameter | Value | Meaning |
|---|---|---|
| `SERVO_MAX` | 60° | Mechanical steering limit |
| `SERVO_DIRECTION` | +1 | Positive command = steer **left** |
| `TURN_FULL_LOCK` | 55° | Angle held during a 90° corner |
| `TURN_RELEASE_DEG` | 25° | Ease off lock this close to target heading |

**Why a servo (not differential/skid steering)?** A steered front axle matches the car-like geometry of the challenge, produces a **predictable turn radius**, and lets us decouple *steering* from *drive* — the drive motor holds speed while the servo alone corrects heading. This makes the control problem far simpler than balancing two drive motors.

### Servo jitter fix
The servo jittered on software PWM. We moved it to the Pi's **hardware PWM via `pigpio`** (daemon `pigpiod`, built from source), which eliminated the jitter and gave a rock-steady steering signal. Documented in [`other/test-procedure.md`](../other/test-procedure.md).

---

## 5. Sensor mounting (mechanical)

| Sensor | Mount location | Mechanical reasoning |
|---|---|---|
| **RPLidar A1M8** | Top deck, highest point | Clear 360° sweep of walls; mounted **180° rotated** — compensated in software (`LIDAR_MOUNT_OFFSET = 180`) rather than re-machining the mount |
| **MPU6050 (IMU)** | Top deck, near center of rotation | Placing it close to the vehicle's yaw axis minimizes translational acceleration coupling into the gyro reading |
| **Rotary encoder** | On the drive axle | Direct odometry from wheel rotation; 205 counts/rev measured experimentally |

The LiDAR's 180° rotation is a good example of **choosing the cheaper fix**: instead of building a new bracket, we remapped the angle windows in code (front `170–190°`, left `80–100°`, right `260–280°`).

---

## 6. Design iterations that changed the mechanics

| Version | Mechanical change | Why |
|---|---|---|
| V1 | Baseline single-motor + servo chassis | Establish a working car-like platform |
| V2 | Raised & isolated LiDAR to top deck | Earlier low mount caught part of its own chassis in the sweep |
| V3 | Added kick-pulse-friendly drivetrain, hardware-PWM servo mount | Fix standstill stall + steering jitter |
| Final | Rigid two-deck standoff frame, IMU relocated near yaw axis | Reduce flex + cleaner heading signal |

Full narrative with photos: [engineering journal](../engineering-journal).

---

## 7. Reproducibility

- **CAD / STL** files: [`/models`](../models)
- **Vehicle photos** (all six angles required by WRO): [`/v-photos`](../v-photos)
- **Calibrated constants** (wheel diameter, counts/rev, servo range): [`other/pinout.md`](../other/pinout.md)
- **Bill of materials**: [`other/BOM.md`](../other/BOM.md)

Anyone with the CAD, the BOM, and the calibrated constants above can reproduce this chassis.

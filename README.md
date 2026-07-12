Engineering materials
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2026.

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains schematic diagrams of the electromechanical components illustrating all the elements used in the vehicle and how they connect to each other
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` contains files for 3D-printed vehicle elements
* `other` contains supporting documentation: pinout reference, power architecture, and component test procedures

## Introduction

Our vehicle is a rear-wheel-drive Ackermann-steering robot built around a Raspberry Pi 4. Navigation for the Open Challenge is based on sensor fusion between three sensors, each with a single clear responsibility:

| Sensor | Role |
|---|---|
| MPU6050 gyroscope | Heading tracking and execution of 90° turns |
| RPLidar A1M8 | Wall distances: corridor centering and corner detection |
| N20 wheel encoder | Distance traveled (odometry for the live map) |

A Pi Camera is mounted for the Obstacle Challenge (color detection of traffic signs) and is not used in the Open Challenge.

### Software modules (`src/`)

**`wro_open.py`** — the Open Challenge program. It runs a three-state machine:

1. **STRAIGHT** — a PID controller (KP=0.9, KI=0.05, KD=0.15 with a low-pass filtered derivative) holds the heading to a "snapped" target (0°, 90°, 180°, 270°...). The LiDAR left/right wall distances produce a centering error that *trims* this target by at most ±6°, so the corridor centering can never overpower the gyro. The trim is smoothed over 4 scans with a 30 mm deadzone.
2. **TURNING** — when the front LiDAR distance drops below 600 mm for 3 consecutive scans, the target heading shifts by ±90° and a dedicated steering profile takes over: full 55° steering lock is held until the heading is within 25° of the target, then a steep linear ramp releases it. This produces sharp, repeatable corners. The PID integral is reset on exit to avoid wind-up being carried into the straight.
3. **FINISHED** — the run stops after 1080° of cumulative rotation (3 laps).

The driving direction is randomized by the competition rules, so it is **detected at the first corner**: when the front wall closes, whichever side reads open (>900 mm) determines clockwise vs counter-clockwise for the entire run.

Robustness measures: gyro bias is re-calibrated at the start line (immediately after pressing ENTER) because the MPU6050 bias drifts with temperature; a 1-second "blind window" after arming and a 0.4-second window after each turn prevent false corner triggers; the motor driver receives a brief full-power kick pulse whenever starting from standstill to overcome static friction.

**`robot_mapper.py`** — standalone odometry + mapping streamer. Fuses encoder distance with gyro heading into a dead-reckoned pose (x, y, θ), transforms every LiDAR point into world coordinates, and streams pose + points as JSON over UDP Wi-Fi at 10 Hz. `wro_open.py` contains the same streaming built in, so every competition-style run can be watched live.

**`map_viewer.py` / `map_viewer_3d.py` / `map_viewer_pro.py`** — PC-side visualizers (not run on the vehicle). The pro version renders a 2.5D scene: walls extruded to their real 100 mm height, a textured mat floor, the live scan as a separate real-time layer, the robot trail, and a telemetry HUD showing state, commanded vs measured speed, heading/compass, lap count, wall distances, run timer, distance traveled and packet rate. This was our main tuning instrument: steering oscillation, early/late corner triggers, and gyro drift are all directly visible on the map.

**`src/tests/`** — individual component validation scripts (servo, gyro, motor, encoder calibration, LiDAR wall distances, gyro-controlled turning). Each hardware module was brought up and verified in isolation before integration.

### Key hardware findings during development

* **Servo jitter** was eliminated by switching from software PWM to pigpio's DMA-based hardware-timed PWM (`PiGPIOFactory`). On Debian 13 the pigpio package is no longer in apt and was built from source.
* **Encoder calibration** was done experimentally: a measured 50 cm push yielded 205 counts per wheel revolution (0.674 mm per count with 44 mm wheels), rather than the datasheet-assumed 4200.
* **The LiDAR is mounted rotated 180°**, handled in software by remapping the direction windows (robot-front = LiDAR 170–190°).
* **Power is split across three isolated sources** with a single star ground: one 9 V pack for the drive motor (through a DRV8833), one 9 V pack through an XL4015 buck (5 V) for the steering servo, and a 2S Li-ion pack through a buck converter into the Pi's USB-C port (keeping the Pi's onboard input protection). The LiDAR is powered and read over a single USB connection. Routing servo return current away from the Pi's ground plane fixed a voltage-sag fault discovered during bring-up.

### Build / upload / run

1. Flash Raspberry Pi OS (Debian 13), enable SSH. Development is done from a PC over VS Code Remote-SSH.
2. Install dependencies on the Pi:
   ```
   pip install adafruit-circuitpython-mpu6050 rplidar-roboticia gpiozero --break-system-packages
   ```
   Build pigpio from source (github.com/joan2937/pigpio): `make && sudo make install`, then `sudo pigpiod` (added to root crontab `@reboot`).
3. Copy `src/` to the Pi (scp or git clone).
4. Optionally start `map_viewer_pro.py` on a PC on the same network (set `PC_IP` in the Pi-side scripts to the PC's address).
5. Place the vehicle at the start position, run `python wro_open.py`, wait for gyro calibration, press ENTER on the start signal.

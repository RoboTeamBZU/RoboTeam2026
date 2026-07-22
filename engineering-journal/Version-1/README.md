# Version 1 — Baseline Platform

**Goal:** get a car-like autonomous platform that drives straight and turns under gyro control.

## What we built
- Single brushed DC motor (rear drive) + steering servo (front, Ackermann).
- Raspberry Pi 4 as the controller; MPU6050 for heading; rotary encoder for distance.
- First version of the gyro-PID heading hold and a basic corner turn.

## What we tested
- Straight-line heading hold on the bench and on the floor.
- Manual 90° turns commanded from the gyro loop.
- First odometry read-outs from the encoder.

## What broke / what we learned
- **Wander:** with no LiDAR centering, the car drifted within the lane — heading hold alone isn't enough to stay centered.
- **Stall from rest:** the motor sometimes failed to launch at low duty (static friction > torque).
- **Encoder scale unknown:** counts/rev had to be *measured*, not assumed.

## Decisions carried into V2
- Add LiDAR for lane centering and corner detection.
- Measure `COUNTS_PER_REV` experimentally (→ 205) and cross-check against a tape measure.
- Investigate a launch fix for the stall.

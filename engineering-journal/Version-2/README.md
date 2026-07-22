# Version 2 — Clean Sensing & Power

**Goal:** get trustworthy sensor data and stop the Pi from resetting under load.

## What we changed
- **Raised the RPLidar to the top deck**, highest point, for an unobstructed 360° sweep.
- **Split the power** into three isolated sources with a single star ground (see [Power & Sensor Architecture](../../docs/Power-and-Sensor-Architecture.md)).
- Added LiDAR-based **lane centering** and **front-wall corner detection** to the state machine.

## What we tested
- Four-direction LiDAR wall distances vs. a tape measure (`test_lidar_pi.py`).
- Full loop of the track with centering trim active.
- Power stability while the motor stalled on purpose.

## What broke / what we learned
- **LiDAR caught its own chassis** in the sweep at the old low mount → moved it up.
- **Brownouts:** the shared battery sagged under motor current, causing Pi resets and I²C errors → isolating the Pi onto its own 2S Li-ion + buck fixed it.
- **LiDAR is mounted 180° rotated** → cheaper to remap angle windows in software than re-machine the bracket.
- **Stale-stream error** on LiDAR reconnect → added `stop()`+`reset()` at startup.

## Decisions carried into V3
- Fix the residual steering jitter (suspected software PWM).
- Add a proper launch fix for the standstill stall.

📷 Add to [`images/`](./images): LiDAR top-mount, power wiring/star ground, live map of a lap.

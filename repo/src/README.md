Control software
====

All code runs on the vehicle's Raspberry Pi 4 except the `map_viewer*` scripts, which run on a PC on the same Wi-Fi network and receive telemetry over UDP.

| File | Runs on | Purpose |
|---|---|---|
| `wro_open.py` | Pi | Open Challenge: state-machine navigation (gyro PID + LiDAR centering/corners) with built-in telemetry streaming |
| `robot_mapper.py` | Pi | Standalone odometry + LiDAR world-mapping streamer (mapping tests without driving logic) |
| `map_viewer.py` | PC | 2D top-down live map (matplotlib) |
| `map_viewer_3d.py` | PC | 2.5D live map (pyvista), walls extruded to real 100 mm height |
| `map_viewer_pro.py` | PC | 2.5D map + full telemetry HUD, chase camera, textured floor |
| `tests/` | Pi | Component bring-up and calibration scripts, one per hardware module |

Dependencies (Pi): `adafruit-circuitpython-mpu6050`, `rplidar-roboticia`, `gpiozero`, pigpio built from source with `pigpiod` running.
Dependencies (PC viewers): `numpy`, `matplotlib`, `pyvista`.

Calibrated constants used throughout: encoder 205 counts/rev (0.674 mm/count, 44 mm wheels); LiDAR direction windows remapped for a 180°-rotated mount; `GYRO_DIRECTION=-1`, `SERVO_DIRECTION=+1` (positive = left).

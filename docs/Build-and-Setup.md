# 🛠️ Build & Setup Guide

How to rebuild and run the vehicle from scratch. Pair this with the [BOM](../other/BOM.md), the [wiring diagram](../schemes), and the [pinout](../other/pinout.md).

---

## 1. Assemble the hardware

1. **Build the two-deck chassis** from the CAD/STL in [`/models`](../models); join the decks with 4 standoffs. See [Mechanical Design](./Mechanical-Design.md).
2. **Lower deck:** mount the drive motor + encoder, DRV8833, steering servo + Ackermann linkage, and the battery packs (keep them low).
3. **Upper deck:** mount the Raspberry Pi, MPU6050 (near the yaw axis), and the RPLidar at the highest point for a clear 360° sweep.
4. **Wire per** [`other/pinout.md`](../other/pinout.md) and [`/schemes`](../schemes). Critically: **three isolated power sources, one star ground**; route the servo return to its own converter ground, not the Pi.

---

## 2. Prepare the Raspberry Pi

```bash
# OS: Raspberry Pi OS (Debian). Enable I2C:
sudo raspi-config          # Interface Options -> I2C -> enable

# System deps
sudo apt update
sudo apt install -y python3-pip git

# pigpio built from source (needed for jitter-free hardware PWM)
# then run the daemon:
sudo pigpiod

# Python libraries
pip3 install adafruit-circuitpython-mpu6050 rplidar-roboticia gpiozero
```

Grant serial access to the LiDAR (`/dev/ttyUSB0`) if needed:
```bash
sudo usermod -a -G dialout $USER   # re-login after this
```

---

## 3. Prepare the PC (live map, optional)

The car drives fully autonomously without the PC. For live visualization:

```bash
pip install numpy matplotlib pyvista
```

Set the PC's IP in [`src/wro_open.py`](../src/wro_open.py) / [`src/robot_mapper.py`](../src/robot_mapper.py):
```python
PC_IP, PC_PORT = "192.168.1.42", 5005   # <-- your PC's LAN IP
```

---

## 4. Calibrate before every run

1. **Warm up** the electronics ~60 s (reduces gyro thermal drift).
2. Place the car at the **start line**, held still.
3. Launch the program — it **auto-calibrates the gyro** (300 samples) on startup.
4. Confirm the [component tests](../other/test-procedure.md) pass if anything was changed.

---

## 5. Run

**Open Challenge (autonomous, on the Pi):**
```bash
python3 src/wro_open.py
# press ENTER to arm; CTRL+C to stop
```

**Live map (optional, on the PC — start FIRST):**
```bash
python3 src/map_viewer_pro.py     # full telemetry HUD + 2.5D map
# then start wro_open.py or robot_mapper.py on the Pi
```

**Mapping test only (no driving logic):**
```bash
python3 src/robot_mapper.py       # push or drive; map builds either way
```

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Servo jitters | software PWM | ensure `pigpiod` is running; use GPIO18 hardware PWM |
| Pi reboots under load | shared/weak power | verify isolated Pi rail + star ground |
| Heading drifts | cold gyro | warm up + recalibrate at start line |
| LiDAR errors on start | stale stream | `stop()`+`reset()` already in code; re-seat USB |
| Car turns early/late | corner threshold | tune `TURN_TRIGGER_DIST` / `TURN_CONFIRM` |
| Car hugs one wall | centering sign | flip sign of `CENTER_GAIN` |

See the [Software Architecture](./Software-Architecture.md) doc for what every constant does.

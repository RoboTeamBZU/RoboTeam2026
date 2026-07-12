"""
robot_mapper.py — runs on the RASPBERRY PI.

Tracks the robot pose (x, y, heading) using encoder + gyro,
transforms LiDAR points into world coordinates, and streams
pose + points over UDP to the PC for live visualization.

Push the robot by hand or drive it — the map builds either way.

Companion file: map_viewer.py (runs on the PC).
"""

import adafruit_mpu6050
import board
import time
import threading
import math
import json
import socket

from gpiozero import RotaryEncoder
from rplidar import RPLidar

# =============================
# Network
# =============================

PC_IP   = "192.168.1.42"
PC_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# =============================
# Encoder (calibrated values)
# =============================

ENCODER_A         = 17
ENCODER_B         = 27
ENCODER_DIRECTION = -1
COUNTS_PER_REV    = 205
WHEEL_DIAMETER_MM = 44.0
MM_PER_COUNT      = (math.pi * WHEEL_DIAMETER_MM) / COUNTS_PER_REV

# =============================
# Gyro
# =============================

GYRO_DIRECTION = -1
GYRO_DEADZONE  = 0.5   # deg/s

# =============================
# LiDAR (robot frame: mounted 180 deg rotated)
# =============================

LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200
MAX_DIST       = 3000   # mm

# LiDAR 0 deg faces robot REAR. To convert a LiDAR angle to the
# robot frame (0 = robot front, CCW positive), we use:
#   robot_angle = 180 - lidar_angle   (then heading is added later)
LIDAR_MOUNT_OFFSET = 180.0

# =============================
# Hardware setup
# =============================

print("Initializing MPU6050...")
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

def calibrate_gyro(samples=300):
    print("Calibrating gyro — keep robot still...")
    total = 0.0
    for _ in range(samples):
        total += mpu.gyro[2]
        time.sleep(0.005)
    offset = total / samples
    print(f"Gyro offset: {offset:.6f} rad/s")
    return offset

gyro_offset = calibrate_gyro()

print("Initializing encoder...")
encoder = RotaryEncoder(ENCODER_A, ENCODER_B, max_steps=10000000)
encoder.steps = 0

print(f"Connecting to LiDAR on {LIDAR_PORT}...")
lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
lidar.stop()
lidar.reset()
time.sleep(0.5)

# =============================
# LiDAR background thread
# =============================

latest_scan = []
scan_lock   = threading.Lock()
running     = [True]

def scan_thread():
    for scan in lidar.iter_scans(max_buf_meas=500):
        if not running[0]:
            break
        with scan_lock:
            latest_scan.clear()
            latest_scan.extend(scan)

t = threading.Thread(target=scan_thread, daemon=True)
t.start()

# =============================
# Pose state
# =============================

x_mm      = 0.0
y_mm      = 0.0
heading   = 0.0     # degrees, 0 = initial facing, CCW positive
last_time = time.time()
last_counts = 0

def update_pose():
    """Dead reckoning: gyro for heading, encoder for distance."""
    global x_mm, y_mm, heading, last_time, last_counts

    now = time.time()
    dt  = now - last_time
    last_time = now

    # Heading from gyro
    gz = (mpu.gyro[2] - gyro_offset) * (180.0 / math.pi)
    if abs(gz) < GYRO_DEADZONE:
        gz = 0.0
    gz *= GYRO_DIRECTION
    heading += gz * dt

    # Distance from encoder since last frame
    counts       = encoder.steps * ENCODER_DIRECTION
    delta_counts = counts - last_counts
    last_counts  = counts
    delta_mm     = delta_counts * MM_PER_COUNT

    # Advance position along current heading
    h_rad = math.radians(heading)
    x_mm += delta_mm * math.cos(h_rad)
    y_mm += delta_mm * math.sin(h_rad)

def scan_to_world():
    """Transform current LiDAR scan into world-frame points."""
    with scan_lock:
        scan = list(latest_scan)

    points = []
    h = heading
    for quality, angle, distance in scan:
        if quality > 0 and 0 < distance <= MAX_DIST:
            # LiDAR angle -> robot frame -> world frame
            world_angle = math.radians(LIDAR_MOUNT_OFFSET - angle + h)
            px = x_mm + distance * math.cos(world_angle)
            py = y_mm + distance * math.sin(world_angle)
            # Round to save bandwidth
            points.append((round(px), round(py)))
    return points

# =============================
# Main loop
# =============================

print(f"""
========================================
ROBOT MAPPER — streaming to {PC_IP}:{PC_PORT}
========================================
Start map_viewer.py on the PC, then move
the robot (push or drive).
Press CTRL+C to stop.
========================================
""")

SEND_INTERVAL = 0.1   # seconds between packets (10 Hz)

try:
    while True:
        update_pose()
        points = scan_to_world()

        packet = {
            "x": round(x_mm),
            "y": round(y_mm),
            "h": round(heading, 1),
            "pts": points[:200],   # cap points per packet, plenty for walls
        }
        data = json.dumps(packet).encode()
        try:
            sock.sendto(data, (PC_IP, PC_PORT))
        except OSError:
            pass   # network hiccup — skip frame, keep tracking

        print(f"x:{x_mm:8.0f}mm  y:{y_mm:8.0f}mm  h:{heading:7.1f}deg  pts:{len(points):4d}", end="\r", flush=True)

        time.sleep(SEND_INTERVAL)

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    running[0] = False
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    encoder.close()
    print("Shutdown complete.")

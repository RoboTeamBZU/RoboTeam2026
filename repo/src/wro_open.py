"""
wro_open.py — WRO Open Challenge navigation. Runs on the Pi.

Architecture:
  GYRO    -> heading hold (PID) + executes 90 deg turns (full-lock profile)
  LIDAR   -> left/right centering trim + front-wall corner trigger
  ENCODER -> odometry for the live map
  UDP     -> streams pose/scan/state to map_viewer.py / map_viewer_3d.py

State machine:
  STRAIGHT -> gyro holds snapped target (0/90/180/270...), LiDAR trims it
  TURNING  -> full-lock steering until near target, then steep ramp
  FINISHED -> after 3 laps (1080 deg cumulative)

Turn direction DETECTED at first corner (WRO randomizes it).
Startup is ARMED after ENTER: state zeroed + 1 s corner-blind window.
"""

import adafruit_mpu6050
import board
import time
import threading
import math
import json
import socket

from gpiozero import AngularServo, PWMOutputDevice, RotaryEncoder
from gpiozero.pins.pigpio import PiGPIOFactory
from rplidar import RPLidar

# ============================================================
# CONFIG
# ============================================================

# ---- Network (live map) ----
PC_IP, PC_PORT = "192.168.1.42", 5005
SEND_EVERY_N_LOOPS = 3

# ---- Pins ----
SERVO_PIN = 18
MOTOR_IN1, MOTOR_IN2 = 23, 24
ENCODER_A, ENCODER_B = 17, 27

# ---- Motor ----
SPEED_STRAIGHT = 0.8
SPEED_TURN     = 0.55
KICK_SPEED     = 1.0
KICK_TIME      = 0.15

# ---- Servo ----
SERVO_MAX       = 60
SERVO_DIRECTION = 1        # positive = steer LEFT

# ---- Turn steering profile ----
TURN_FULL_LOCK   = 55      # servo angle held during the turn
TURN_RELEASE_DEG = 25      # ease off only when this close to target

# ---- Gyro ----
GYRO_DIRECTION = -1
GYRO_DEADZONE  = 0.05      # deg/s (low: don't eat slow veers)

# ---- Steering PID (heading hold on straights) ----
KP, KI, KD = 0.9, 0.05, 0.15
KD_FILTER  = 0.3
I_CLAMP    = 30

# ---- LiDAR ----
LIDAR_PORT, LIDAR_BAUD = "/dev/ttyUSB0", 115200
MAX_DIST = 3000
WIN_FRONT = [(170, 190)]     # robot frame; LiDAR mounted 180 deg rotated
WIN_LEFT  = [(80, 100)]
WIN_RIGHT = [(260, 280)]

# ---- Centering ----
CENTER_GAIN     = 0.010    # deg trim per mm of (left-right); flip sign if diverging
CENTER_TRIM_MAX = 6.0
CENTER_DEADZONE = 30       # mm
CENTER_SMOOTH_N = 4

# ---- Corner logic ----
TURN_TRIGGER_DIST = 600    # mm
TURN_CONFIRM      = 3      # consecutive scans
TURN_EXIT_TOL     = 4.0    # deg
OPEN_SIDE_MIN     = 900    # mm; side larger than this = open corridor
POST_TURN_BLIND   = 0.4    # s
START_BLIND       = 1.0    # s after ENTER with no corner triggers

# ---- Mission ----
LAP_LIMIT     = 1080.0
LAP_TOLERANCE = 15.0

# ---- Encoder (calibrated) ----
ENCODER_DIRECTION = -1
COUNTS_PER_REV    = 205
WHEEL_D_MM        = 44.0
MM_PER_COUNT      = math.pi * WHEEL_D_MM / COUNTS_PER_REV

# ============================================================
# HARDWARE SETUP
# ============================================================

print("pigpio servo factory...")
factory = PiGPIOFactory()

print("Servo...")
servo = AngularServo(SERVO_PIN, min_angle=-SERVO_MAX, max_angle=SERVO_MAX,
                     min_pulse_width=0.0005, max_pulse_width=0.0025,
                     pin_factory=factory)
servo.angle = 0

print("Motor...")
m1 = PWMOutputDevice(MOTOR_IN1, pin_factory=factory)
m2 = PWMOutputDevice(MOTOR_IN2, pin_factory=factory)

print("Encoder...")
encoder = RotaryEncoder(ENCODER_A, ENCODER_B, max_steps=10000000,
                        pin_factory=factory)
encoder.steps = 0

print("MPU6050...")
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

def calibrate_gyro(n=300):
    print("Calibrating gyro — keep robot STILL...")
    s = 0.0
    for _ in range(n):
        s += mpu.gyro[2]
        time.sleep(0.005)
    off = s / n
    print(f"  offset {off:.6f} rad/s")
    return off

gyro_offset = calibrate_gyro()

print("LiDAR...")
lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD)
lidar.stop(); lidar.reset(); time.sleep(0.5)

latest_scan, scan_lock, running = [], threading.Lock(), [True]
def scan_thread():
    for scan in lidar.iter_scans(max_buf_meas=500):
        if not running[0]:
            break
        with scan_lock:
            latest_scan.clear()
            latest_scan.extend(scan)
threading.Thread(target=scan_thread, daemon=True).start()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ============================================================
# HELPERS
# ============================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

_motor_moving = False
current_speed = 0.0        # what we COMMAND the motor right now (0..1)

def motor(speed):
    """Forward drive with a kick pulse when starting from rest."""
    global _motor_moving, current_speed
    current_speed = clamp(speed, 0.0, 1.0)
    if current_speed <= 0:
        m1.value = 0.0; m2.value = 0.0
        _motor_moving = False
        return
    if not _motor_moving:
        m1.value = KICK_SPEED; m2.value = 0.0
        time.sleep(KICK_TIME)
        _motor_moving = True
    m1.value = current_speed; m2.value = 0.0

def lidar_min(scan, windows):
    d = [dist for q, a, dist in scan
         if q > 0 and 0 < dist <= MAX_DIST
         and any(lo <= a <= hi for lo, hi in windows)]
    return min(d) if d else None

# ============================================================
# STATE
# ============================================================

heading     = 0.0
last_t      = time.time()
x_mm = y_mm = 0.0
last_counts = 0

target       = 0.0
snapped      = 0.0
turn_dir     = 0
state        = "STRAIGHT"
corner_count = 0
confirm      = 0
blind_until  = 0.0

integ = 0.0
last_err = 0.0
fderiv = 0.0
center_hist = []

def update_pose():
    global heading, last_t, x_mm, y_mm, last_counts
    now = time.time(); dt = now - last_t; last_t = now
    gz = (mpu.gyro[2] - gyro_offset) * (180.0 / math.pi)
    if abs(gz) < GYRO_DEADZONE: gz = 0.0
    heading += gz * GYRO_DIRECTION * dt

    c = encoder.steps * ENCODER_DIRECTION
    d_mm = (c - last_counts) * MM_PER_COUNT
    last_counts = c
    h = math.radians(heading)
    x_mm += d_mm * math.cos(h)
    y_mm += d_mm * math.sin(h)

def pid_steer():
    global integ, last_err, fderiv
    err = target - heading
    integ = clamp(integ + err, -I_CLAMP, I_CLAMP)
    raw_d = err - last_err
    fderiv = (1 - KD_FILTER) * raw_d + KD_FILTER * fderiv
    last_err = err
    out = (KP * err + KI * integ + KD * fderiv) * SERVO_DIRECTION
    return clamp(out, -SERVO_MAX, SERVO_MAX)

def turn_steer():
    """Full lock until TURN_RELEASE_DEG from target, then steep ramp."""
    err = target - heading
    if abs(err) > TURN_RELEASE_DEG:
        steer = TURN_FULL_LOCK * (1 if err > 0 else -1)
    else:
        steer = (TURN_FULL_LOCK / TURN_RELEASE_DEG) * err
    return clamp(steer * SERVO_DIRECTION, -SERVO_MAX, SERVO_MAX)

def smoothed_center_error(left, right):
    center_hist.append(left - right)
    if len(center_hist) > CENTER_SMOOTH_N:
        center_hist.pop(0)
    return sum(center_hist) / len(center_hist)

# ============================================================
# MAIN
# ============================================================

print("""
==================================================
WRO OPEN CHALLENGE
Direction auto-detected at first corner.
Live map streaming on. ENTER to start, CTRL+C stop.
==================================================""")
input()

# ---- ARM: clean state at the true start of the run ----
heading = 0.0
encoder.steps = 0
last_counts = 0
x_mm = y_mm = 0.0
snapped = target = 0.0
turn_dir = 0
corner_count = 0
confirm = 0
integ = last_err = fderiv = 0.0
center_hist.clear()
last_t = time.time()
blind_until = time.time() + START_BLIND

loop_n = 0
try:
    motor(SPEED_STRAIGHT)

    while True:
        loop_n += 1
        update_pose()

        if abs(heading) >= LAP_LIMIT - LAP_TOLERANCE:
            state = "FINISHED"
            motor(0); servo.angle = 0
            print(f"\n3 laps done. heading={heading:.1f}")
            break

        with scan_lock:
            scan = list(latest_scan)
        front = lidar_min(scan, WIN_FRONT)
        left  = lidar_min(scan, WIN_LEFT)
        right = lidar_min(scan, WIN_RIGHT)

        now = time.time()

        if state == "STRAIGHT":
            # ---- centering trim around snapped heading ----
            if left is not None and right is not None:
                err_mm = smoothed_center_error(left, right)
                if abs(err_mm) < CENTER_DEADZONE:
                    err_mm = 0.0
                trim = clamp(err_mm * CENTER_GAIN, -CENTER_TRIM_MAX, CENTER_TRIM_MAX)
                target = snapped + trim
            else:
                target = snapped

            # ---- corner detection ----
            if now >= blind_until and front is not None and front < TURN_TRIGGER_DIST:
                confirm += 1
            else:
                confirm = 0

            if confirm >= TURN_CONFIRM:
                if turn_dir == 0:
                    l_open = (left  or 0) > OPEN_SIDE_MIN
                    r_open = (right or 0) > OPEN_SIDE_MIN
                    if l_open and not r_open:
                        turn_dir = +1
                    elif r_open and not l_open:
                        turn_dir = -1
                    else:
                        turn_dir = +1 if (left or 0) >= (right or 0) else -1
                    print(f"\nDirection detected: {'LEFT/CCW' if turn_dir > 0 else 'RIGHT/CW'}")

                corner_count += 1
                snapped += 90.0 * turn_dir
                target   = snapped
                state    = "TURNING"
                confirm  = 0
                motor(SPEED_TURN)
                print(f"\nCorner {corner_count}: turning to {snapped:.0f}")

            servo.angle = pid_steer()

        elif state == "TURNING":
            if abs(target - heading) <= TURN_EXIT_TOL:
                state = "STRAIGHT"
                blind_until = time.time() + POST_TURN_BLIND
                center_hist.clear()
                integ = 0.0                 # no wind-up carried into the straight
                motor(SPEED_STRAIGHT)
                servo.angle = pid_steer()
            else:
                servo.angle = turn_steer()

        # ---- stream to map viewer ----
        if loop_n % SEND_EVERY_N_LOOPS == 0:
            pts = []
            h = heading
            for q, a, dist in scan[:400]:
                if q > 0 and 0 < dist <= MAX_DIST:
                    wa = math.radians(180.0 - a + h)
                    pts.append((round(x_mm + dist * math.cos(wa)),
                                round(y_mm + dist * math.sin(wa))))
            pkt = {"x": round(x_mm), "y": round(y_mm), "h": round(heading, 1),
                   "pts": pts, "state": state, "spd": current_speed,
                   "f": front, "l": left, "r": right,
                   "laps": round(abs(heading) / 360.0, 2)}
            try:
                sock.sendto(json.dumps(pkt).encode(), (PC_IP, PC_PORT))
            except OSError:
                pass

        print(f"{state:9s} spd:{current_speed:4.2f} lap:{abs(heading)/360:4.2f} "
              f"h:{heading:7.1f} tgt:{target:7.1f} "
              f"F:{front or -1:5.0f} L:{left or -1:5.0f} R:{right or -1:5.0f}   ",
              end="\r", flush=True)

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nManual stop.")

finally:
    running[0] = False
    motor(0)
    servo.angle = 0
    time.sleep(0.3)
    try:
        lidar.stop(); lidar.stop_motor(); lidar.disconnect()
    except Exception:
        pass
    servo.close(); m1.close(); m2.close(); encoder.close()
    print("Shutdown complete.")

import adafruit_mpu6050
import board
import time
from gpiozero import AngularServo, PWMOutputDevice

# =============================
# Config
# =============================

SERVO_PIN = 18
MOTOR_IN1 = 23
MOTOR_IN2 = 24

MOTOR_SPEED     = 0.5    # speed while turning
SERVO_MAX_LEFT  = -60
SERVO_MAX_RIGHT =  60
SERVO_DIRECTION = 1      # positive servo value = turns LEFT (your convention)

GYRO_DIRECTION  = -1     # from your previous working code
GYRO_DEADZONE   = 0.5    # deg/s

# Turn control
TURN_TOLERANCE  = 3.0    # degrees — how close to target counts as "done"
TURN_STEER      = 40     # servo angle used while turning (not full lock, smoother)

# Simple P controller for the approach — slows steering as target nears
TURN_KP         = 1.2

# =============================
# MPU6050 Setup
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
    print(f"Offset = {offset:.6f} rad/s")
    return offset

gyro_offset = calibrate_gyro()

# =============================
# Hardware Setup
# =============================

print("Initializing servo and motor...")
servo = AngularServo(
    SERVO_PIN,
    min_angle=SERVO_MAX_LEFT,
    max_angle=SERVO_MAX_RIGHT,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)
servo.angle = 0

motor_in1 = PWMOutputDevice(MOTOR_IN1)
motor_in2 = PWMOutputDevice(MOTOR_IN2)

def motor_forward(speed):
    motor_in1.value = max(0.0, min(1.0, speed))
    motor_in2.value = 0.0

def motor_stop():
    motor_in1.value = 0.0
    motor_in2.value = 0.0

# =============================
# Heading tracking
# =============================

heading   = 0.0
last_time = time.time()

def update_heading():
    global heading, last_time
    now = time.time()
    dt  = now - last_time
    last_time = now

    gz = (mpu.gyro[2] - gyro_offset) * (180.0 / 3.14159)  # deg/s
    if abs(gz) < GYRO_DEADZONE:
        gz = 0.0
    gz *= GYRO_DIRECTION

    heading += gz * dt
    return heading

# =============================
# Turn function
# =============================

def turn_to(target_degrees):
    """
    Turn the robot until heading reaches target_degrees.
    Positive target = turn in positive heading direction.
    Steering eases off as the target approaches (P control),
    which prevents overshoot from full-lock turning.
    """
    global heading

    print(f"\nTurning to {target_degrees:+.1f}°  (current: {heading:+.1f}°)")
    motor_forward(MOTOR_SPEED)

    while True:
        heading = update_heading()
        error   = target_degrees - heading

        if abs(error) <= TURN_TOLERANCE:
            break

        # P control on steering angle, capped at TURN_STEER
        steer = TURN_KP * error
        steer = max(-TURN_STEER, min(TURN_STEER, steer))
        steer *= SERVO_DIRECTION

        servo.angle = steer

        print(f"  heading: {heading:+7.2f}°   error: {error:+7.2f}°   servo: {steer:+6.1f}°", end="\r")
        time.sleep(0.01)

    motor_stop()
    servo.angle = 0
    print(f"\nDone. Final heading: {heading:+.2f}°  (target was {target_degrees:+.1f}°)")

# =============================
# Main
# =============================

print("""
========================================
TURN TEST
========================================
Enter a target heading in degrees:
  +90  = turn left 90°  (if headings increase left)
  -90  = turn right 90°
  0    = return to start heading
Empty input or CTRL+C to quit.

Note: targets are ABSOLUTE headings,
not relative turns. After turning to 90,
entering 90 again does nothing.
========================================
""")

try:
    while True:
        raw = input("Target heading (deg): ").strip()
        if raw == "":
            break
        try:
            target = float(raw)
        except ValueError:
            print("Not a number, try again.")
            continue

        turn_to(target)

except KeyboardInterrupt:
    print("\nInterrupted.")

finally:
    motor_stop()
    servo.angle = 0
    time.sleep(0.3)
    servo.close()
    motor_in1.close()
    motor_in2.close()
    print("Shutdown complete.")

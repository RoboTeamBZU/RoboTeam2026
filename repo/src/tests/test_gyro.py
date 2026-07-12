import adafruit_mpu6050
import board
import time

# =============================
# Gyro Config
# =============================

# Deadzone — ignore gyro readings below this (deg/s)
# Eliminates drift when robot is stationary
GYRO_DEADZONE = 0.5

# Direction — flip to -1 if rotation direction is reversed
GYRO_DIRECTION = 1

# Calibration samples
CALIBRATION_SAMPLES = 300

# =============================
# MPU6050 Setup
# =============================

print("Initializing MPU6050...")
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)

# =============================
# Calibration
# =============================

def calibrate_gyro(samples=CALIBRATION_SAMPLES):
    """
    Read gyro Z axis many times while stationary to find the
    average drift offset. This offset is subtracted from every
    future reading so stationary = 0 deg/s.
    """
    print(f"Calibrating gyro — keep robot still ({samples} samples)...")
    total = 0.0
    for _ in range(samples):
        total += mpu.gyro[2]   # Z axis = yaw (rotation around vertical axis)
        time.sleep(0.005)
    offset = total / samples
    print(f"Gyro Z offset = {offset:.6f} rad/s")
    return offset

gyro_offset = calibrate_gyro()

# =============================
# Runtime State
# =============================

heading          = 0.0   # degrees, cumulative
last_time        = time.time()

# =============================
# Heading Update
# =============================

def update_heading():
    """
    Integrate gyro Z axis to get cumulative heading in degrees.

    MPU6050 via adafruit returns rad/s — convert to deg/s.
    Multiply by dt to get degrees turned this frame.
    Accumulate into heading.

    Positive heading = clockwise rotation (adjust GYRO_DIRECTION if reversed)
    """
    global heading, last_time

    now = time.time()
    dt  = now - last_time
    last_time = now

    gyro_z_rads = mpu.gyro[2] - gyro_offset          # rad/s, offset removed
    gyro_z_degs = gyro_z_rads * (180.0 / 3.14159)    # convert to deg/s

    # Apply deadzone — ignore tiny drift when stationary
    if abs(gyro_z_degs) < GYRO_DEADZONE:
        gyro_z_degs = 0.0

    gyro_z_degs *= GYRO_DIRECTION

    heading += gyro_z_degs * dt

    return heading

# =============================
# Lap Counting
# =============================

def get_laps(heading):
    """
    Each lap = 360 degrees of cumulative rotation.
    Works for both clockwise and counterclockwise circuits.
    """
    return abs(heading) / 360.0

# =============================
# Test Loop
# =============================

print("\nReading heading — rotate the robot and watch the value change.")
print("Press CTRL+C to stop.\n")
print(f"{'Heading':>10} {'Laps':>8} {'Gyro Z (deg/s)':>18}")
print("-" * 40)

try:
    while True:
        heading = update_heading()
        laps    = get_laps(heading)

        gyro_z_raw  = mpu.gyro[2] - gyro_offset
        gyro_z_degs = gyro_z_raw * (180.0 / 3.14159)

        print(f"{heading:10.2f}°  {laps:8.3f}  {gyro_z_degs:18.3f}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print(f"\nFinal heading: {heading:.2f}°")
    print(f"Total laps:    {get_laps(heading):.3f}")
    print("Done.")

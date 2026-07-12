from gpiozero import RotaryEncoder
import time

# =============================
# Encoder Config
# =============================

ENCODER_A = 17
ENCODER_B = 27

ENCODER_DIRECTION = -1        # your encoder counts negative when moving forward
COUNTS_PER_REV    = 2000      # measured: one wheel revolution = 2000 counts
WHEEL_DIAMETER_MM = 44.0

MM_PER_COUNT = (3.14159265 * WHEEL_DIAMETER_MM) / COUNTS_PER_REV   # = 0.0691 mm

# =============================
# Setup
# =============================

print("Initializing encoder...")
encoder = RotaryEncoder(ENCODER_A, ENCODER_B, max_steps=10000000)
encoder.steps = 0

print(f"""
========================================
DISTANCE TEST
========================================
COUNTS_PER_REV: {COUNTS_PER_REV}
MM_PER_COUNT:   {MM_PER_COUNT:.4f} mm
Wheel diameter: {WHEEL_DIAMETER_MM} mm

Push the robot in a straight line.
Verify against a tape measure:
  50 cm push should read ~500 mm

Press CTRL+C to finish and show total.
========================================
""")

try:
    while True:
        counts      = encoder.steps * ENCODER_DIRECTION
        distance_mm = counts * MM_PER_COUNT

        print(f"Counts: {counts:8d}   Distance: {distance_mm:8.1f} mm  ({distance_mm/10:7.1f} cm)   ", end="\r", flush=True)
        time.sleep(0.05)

except KeyboardInterrupt:
    counts      = encoder.steps * ENCODER_DIRECTION
    distance_mm = counts * MM_PER_COUNT
    print(f"\n\nFinal distance: {distance_mm:.1f} mm  ({distance_mm/10:.1f} cm)")
    print(f"Final counts:   {counts}")
    print("If a 50 cm push reads ~250 mm, halve COUNTS_PER_REV to 1000.")
    print("If it reads ~1000 mm, double COUNTS_PER_REV to 4000.")

finally:
    encoder.close()

from gpiozero import RotaryEncoder
from gpiozero import PWMOutputDevice
import time
import threading

# =============================
# Encoder Config
# =============================

ENCODER_A = 17
ENCODER_B = 27

# Assumed values — update once motor specs are confirmed
PPR            = 7       # pulses per revolution of motor shaft
GEAR_RATIO     = 150     # gear ratio
WHEEL_DIAMETER = 44.0    # mm

COUNTS_PER_REV = PPR * GEAR_RATIO * 4   # x4 for quadrature = 4200
MM_PER_COUNT   = (3.14159 * WHEEL_DIAMETER) / COUNTS_PER_REV

# =============================
# Motor Config (to drive encoder)
# =============================

MOTOR_IN1 = 23
MOTOR_IN2 = 24
MOTOR_SPEED = 0.4

# =============================
# Setup
# =============================

print("Initializing encoder...")
encoder = RotaryEncoder(ENCODER_A, ENCODER_B, max_steps=1000000)

print("Initializing motor...")
motor_in1 = PWMOutputDevice(MOTOR_IN1)
motor_in2 = PWMOutputDevice(MOTOR_IN2)

def motor_forward(speed):
    motor_in1.value = speed
    motor_in2.value = 0.0

def motor_stop():
    motor_in1.value = 0.0
    motor_in2.value = 0.0

# =============================
# Test 1 — Raw pulse check
# =============================

print(f"""
========================================
ENCODER TEST
========================================
Assumed specs:
  PPR:            {PPR}
  Gear ratio:     {GEAR_RATIO}
  Counts per rev: {COUNTS_PER_REV}
  MM per count:   {MM_PER_COUNT:.4f} mm
  Wheel diameter: {WHEEL_DIAMETER} mm
========================================

TEST 1: Raw pulse check
Motor will run forward for 3 seconds.
Watch if counts change — if they stay
at 0 the encoder wiring is wrong.

Press ENTER to start...
""")
input()

encoder.steps = 0
print("Running motor forward for 3 seconds...")
motor_forward(MOTOR_SPEED)

for i in range(30):
    time.sleep(0.1)
    print(f"  Counts: {encoder.steps:8d}  Distance: {encoder.steps * MM_PER_COUNT:8.1f} mm", end="\r")

motor_stop()
print(f"\nFinal count after 3 seconds: {encoder.steps}")

if encoder.steps == 0:
    print("WARNING: No counts detected — check encoder wiring:")
    print("  VCC → Pi Pin 1 (3.3V)")
    print("  GND → Pi Pin 9")
    print("  A   → Pi Pin 11 (GPIO17)")
    print("  B   → Pi Pin 13 (GPIO27)")
else:
    print("Encoder is working!")

# =============================
# Test 2 — Direction check
# =============================

print("""
TEST 2: Direction check
Motor will run forward — counts should INCREASE.
Then backward — counts should DECREASE.
Press ENTER to continue...
""")
input()

encoder.steps = 0

print("Forward 2 seconds...")
motor_forward(MOTOR_SPEED)
time.sleep(2)
motor_stop()
forward_counts = encoder.steps
print(f"  Counts after forward: {forward_counts}")

time.sleep(0.5)

print("Backward 2 seconds...")
motor_in1.value = 0.0
motor_in2.value = MOTOR_SPEED
time.sleep(2)
motor_stop()
print(f"  Counts after backward: {encoder.steps}")

if forward_counts > 0:
    print("Direction: CORRECT — forward = positive counts")
elif forward_counts < 0:
    print("Direction: REVERSED — swap encoder A and B wires, or swap ENCODER_A and ENCODER_B pins in code")
else:
    print("Direction: NO COUNTS — encoder not working")

# =============================
# Test 3 — Live distance tracking
# =============================

print("""
TEST 3: Live distance tracking
Robot will run and show real-time distance.
Press CTRL+C to stop.
""")
input()

encoder.steps = 0
motor_forward(MOTOR_SPEED)

print(f"{'Counts':>10} {'Distance (mm)':>15} {'Distance (cm)':>15}")
print("-" * 45)

try:
    while True:
        steps    = encoder.steps
        distance = steps * MM_PER_COUNT
        print(f"{steps:10d} {distance:15.1f} {distance/10:15.1f}", end="\r")
        time.sleep(0.05)

except KeyboardInterrupt:
    motor_stop()
    total_distance = abs(encoder.steps) * MM_PER_COUNT
    print(f"\n\nTotal distance traveled: {total_distance:.1f} mm  ({total_distance/10:.1f} cm)")
    print("Done.")

finally:
    motor_stop()
    motor_in1.close()
    motor_in2.close()
    encoder.close()


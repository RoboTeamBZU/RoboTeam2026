from gpiozero import PWMOutputDevice
import time

# DRV8833 Channel A
MOTOR_IN1 = 23
MOTOR_IN2 = 24

print("Initializing DRV8833 motor driver...")
motor_in1 = PWMOutputDevice(MOTOR_IN1)
motor_in2 = PWMOutputDevice(MOTOR_IN2)

def motor_forward(speed):
    motor_in1.value = speed
    motor_in2.value = 0.0

def motor_backward(speed):
    motor_in1.value = 0.0
    motor_in2.value = speed

def motor_stop():
    motor_in1.value = 0.0
    motor_in2.value = 0.0

try:
    print("Forward 50% speed for 2 seconds...")
    motor_forward(0.5)
    time.sleep(2)

    print("Stopping for 1 second...")
    motor_stop()
    time.sleep(1)

    print("Backward 50% speed for 2 seconds...")
    motor_backward(0.5)
    time.sleep(2)

    print("Stopping...")
    motor_stop()

    print("Ramping up forward...")
    for speed in range(0, 11):
        motor_forward(speed / 10)
        print(f"  Speed: {speed * 10}%")
        time.sleep(0.3)

    print("Ramping down...")
    for speed in range(10, -1, -1):
        motor_forward(speed / 10)
        print(f"  Speed: {speed * 10}%")
        time.sleep(0.3)

    motor_stop()
    print("Motor test complete.")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    motor_stop()
    motor_in1.close()
    motor_in2.close()


Full pinout & power reference
====

## Power
```
9V pack 1 --switch--> DRV8833 VM   (motor)
9V pack 2 --> XL4015 (5.0V) --> Servo VCC
2S 18650 (7.4V) --> buck (5.1V) --> USB-C --> Pi
LiDAR: single USB cable to Pi (power + data, /dev/ttyUSB0 @ 115200)
All GND --> star point --> one wire --> Pi pin 9
```

## GPIO
```
Pin 1  3.3V    Encoder VCC + MPU6050 VCC
Pin 3  GPIO2   MPU6050 SDA
Pin 5  GPIO3   MPU6050 SCL
Pin 9  GND     star ground reference
Pin 11 GPIO17  Encoder A
Pin 12 GPIO18  Servo signal
Pin 13 GPIO27  Encoder B
Pin 16 GPIO23  DRV8833 IN1
Pin 18 GPIO24  DRV8833 IN2
```

## Calibrated constants
```
Encoder: 205 counts/rev, MM_PER_COUNT = 0.674 (44 mm wheel), ENCODER_DIRECTION = -1
Gyro:    GYRO_DIRECTION = -1, deadzone 0.15 deg/s, calibrate at start line after warm-up
Servo:   SERVO_DIRECTION = +1 (positive = left), range ±60°
LiDAR:   mounted 180° rotated. Robot frame windows:
         front 170-190°, left 80-100°, right 260-280°, back 350-360°+0-10°
```

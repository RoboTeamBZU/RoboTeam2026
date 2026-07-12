Electromechanical diagrams
====

Wiring diagram of the vehicle (Fritzing). Summary of the architecture:

**Power (three isolated sources, one star ground):**
- 9 V pack #1 → switch → DRV8833 VM (drive motor only)
- 9 V pack #2 → XL4015 buck (5.0 V) → steering servo VCC
- 2S Li-ion (7.4 V) → buck converter (5.1 V) → Raspberry Pi USB-C
- RPLidar A1M8: powered and read over a single USB connection to the Pi
- All grounds joined at one star point; the Pi connects to it with a single reference wire (Pin 9). Servo return current is routed to its own converter's ground, never through the Pi.

**Signal connections (Pi GPIO):**

| Pi pin | GPIO | Connects to |
|---|---|---|
| 1 | 3V3 | Encoder VCC, MPU6050 VCC |
| 3 | GPIO2 (SDA) | MPU6050 SDA |
| 5 | GPIO3 (SCL) | MPU6050 SCL |
| 9 | GND | Star ground (reference) |
| 11 | GPIO17 | Encoder A |
| 12 | GPIO18 | Servo signal (hardware PWM via pigpio) |
| 13 | GPIO27 | Encoder B |
| 16 | GPIO23 | DRV8833 IN1 |
| 18 | GPIO24 | DRV8833 IN2 |

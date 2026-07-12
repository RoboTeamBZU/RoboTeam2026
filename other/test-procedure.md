Component test procedure
====

Each module was validated in isolation before integration (scripts in `src/tests/`):

1. **Servo** — sweep test; jitter eliminated with pigpio hardware PWM (built from source on Debian 13).
2. **Gyro** (`test_gyro.py`) — heading integration; verified 90°/360° physical rotations; ~1°/3s drift traced to thermal bias → re-calibrate at start line after 60 s warm-up.
3. **Motor** (`test_motor.py`) — direction + ramp test; standstill torque solved mechanically; software kick-pulse retained as safety.
4. **Encoder** (`test_encoder.py`, `test_distance.py`) — counts/rev measured experimentally (205); verified with a 50 cm tape-measure push reading 500 mm.
5. **LiDAR** (`test_lidar_pi.py`) — four-direction wall distances; startup `stop()+reset()` added after discovering stale-stream descriptor errors; direction windows remapped for the 180° mount.
6. **Turning** (`test_turn.py`) — gyro-closed-loop turns to absolute headings with P-controlled steering release.
7. **Mapping** — pushed-lap test: walls form closed straight lines and the trail returns to origin, validating encoder+gyro dead reckoning before autonomous runs.

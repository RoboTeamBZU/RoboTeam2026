# Version 3 — Steady Control

**Goal:** eliminate the two reliability killers — steering jitter and launch stall.

## What we changed
- **Moved the servo to hardware PWM** via `pigpio` (`pigpiod` daemon, built from source) on GPIO18 → jitter gone.
- **Added a kick pulse** to the motor: full power for 0.15 s on any start-from-rest, then drop to commanded duty (`motor()` in `wro_open.py`).
- Tuned the **turn profile**: full lock (55°) until 25° from target, then a linear ramp to a clean exit within 4°.

## What we tested
- Servo sweep test — confirmed steady signal, no twitch (`test_motor.py`, `test_turn.py`).
- Repeated launches from a dead stop — reliable every time.
- Closed-loop 90° turns to absolute headings (`test_turn.py`).

## What broke / what we learned
- Steering jitter was **software PWM timing**, not the servo or linkage.
- The kick pulse is kept as a **permanent safety margin** even after reducing drivetrain friction.
- Slowing to 55% in turns removed understeer and made the radius repeatable.

## Decisions carried into Final
- Address heading drift bleeding into straights.
- Stop PID integral wind-up from corners.
- Smooth the centering trim against noisy single scans.

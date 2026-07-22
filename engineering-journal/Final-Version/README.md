# Final Version — Repeatability

**Goal:** turn a car that *can* do a lap into one that does **3 consistent laps, run after run**.

## What we changed
- **Start-line gyro recalibration** after a 60 s warm-up → cancels thermal drift at the moment it matters.
- **Reset the PID integral on turn exit** → no corner wind-up carried into the next straight.
- **4-scan smoothing** on the centering error + 30 mm deadzone → the wheel stops reacting to single noisy scans.
- **Blind windows**: 1 s after arming (no false start-line corner) and 0.4 s after each turn (no double-trigger).
- Rigid two-deck standoff frame; IMU relocated near the yaw axis for a cleaner heading signal.

## What we tested
- Full **3-lap autonomous runs** (1080° cumulative), direction auto-detected at corner 1.
- **Pushed-lap map validation:** walls close into straight lines and the trail returns to origin — dead-reckoning drift within tolerance before every autonomous session.
- Repeatability across multiple back-to-back runs.

## Result
- Consistent 3-lap completion with the car staying centered and taking corners cleanly.
- Fully autonomous **without** the PC — telemetry/mapping is non-critical, so a venue Wi-Fi issue can't cause a DNF.

## Obstacle Challenge — a second, deliberative brain
For the Obstacle Challenge we took a different approach from the reactive Open-Challenge state machine: [`grid_goto.py`](../../src/grid_goto.py) builds a **live 50 mm occupancy grid** and runs **A\* pathfinding** to a goal cell, re-planning ~2×/second as the map fills in.
- **Traffic signs as geometry:** each red/green pillar becomes a one-sided **"virtual wall"**, so the planner has exactly one legal way past — no fragile if/else steering.
- **Sensor fusion:** LiDAR finds each pillar's bearing; the **Pi Camera** reads its color (HSV, red hue-wrap handled) with 3-hit confirmation before it affects the plan.
- **Recovery:** reverse-and-replan on dead-ends/stalls, goal-nudging out of wall zones, and a start-cell escape bubble.
- This is also what drives the **parallel-parking** approach — the bay is just a goal cell.

## Known limits / next steps
- Gyro-only heading still drifts slowly over long runs → next: light LiDAR/SLAM correction to bound it.
- Fold the two configs (open vs. obstacle calibration constants) into one shared config file.

📷 Add to [`images/`](./images): final vehicle (all angles), 3-lap map, start-line calibration.

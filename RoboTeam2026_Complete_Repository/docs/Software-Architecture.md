# Software Architecture and Obstacle Strategy

## System Responsibilities

### Raspberry Pi

- computer vision,
- mapping and localization,
- planning,
- high-level state machine,
- logging,
- and communication with the ESP32.

### ESP32

- motor control,
- steering servo control,
- encoder reading,
- low-level timing,
- emergency stop,
- and feedback to the Raspberry Pi.

## State Machine

Add the final state-machine diagram in `schemes/software/`.

Suggested states:

1. `WAIT_FOR_START`
2. `INITIALIZE`
3. `DETECT_DIRECTION`
4. `LOCALIZE`
5. `FOLLOW_TRACK`
6. `HANDLE_RED_PILLAR`
7. `HANDLE_GREEN_PILLAR`
8. `CORNERING`
9. `COUNT_LAP`
10. `SEARCH_FOR_PARKING`
11. `PARALLEL_PARK`
12. `STOP`
13. `RECOVERY`

## Localization and Mapping

Explain how the robot determines:

- its starting section,
- driving direction,
- current section,
- position within the lane,
- obstacle position,
- and parking-lot position.

## Obstacle Strategy

Explain how a red or green pillar affects the desired path.

## Path Planning

Document whether the robot uses geometric waypoints, occupancy-grid planning, A*, or another method.

## Control

Document steering and speed controllers, including tuning methods and limits.

## Edge Cases

Include:

- random starting zones,
- partial obstacle detection,
- blocked view,
- wrong initial pose,
- sensor disagreement,
- corner entry at high speed,
- and recovery before crossing an obstacle incorrectly.

## Validation Metrics

Examples:

- full-lap success rate,
- pillar-side accuracy,
- localization error,
- average lap time,
- maximum steering error,
- and parking success rate.

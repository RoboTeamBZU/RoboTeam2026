# Power and Sensor Architecture

## Power Budget

| Component | Voltage | Typical Current | Peak Current | Power Source |
|---|---:|---:|---:|---|
| Raspberry Pi 5 | 5 V |  |  |  |
| ESP32 | 3.3/5 V |  |  |  |
| Drive motor |  |  |  |  |
| Steering servo |  |  |  |  |
| LiDAR |  |  |  |  |
| Camera |  |  |  |  |
| IMU |  |  |  |  |

Add the total typical and peak current and verify that the battery, wiring, connectors, and regulators can safely supply it.

## Power Distribution

Explain each power rail and where regulation occurs.

## Sensor Selection

For every sensor, compare at least one alternative.

| Requirement | Selected Sensor | Alternative | Reason for Selection |
|---|---|---|---|
| Wall or obstacle distance |  |  |  |
| Visual detection |  |  |  |
| Orientation |  |  |  |
| Wheel motion |  |  |  |

## Sensor Placement

Explain placement using robot geometry and field geometry.

## Calibration

Document:

- camera calibration,
- color thresholds,
- LiDAR alignment,
- IMU bias correction,
- encoder scale,
- and repeatability checks.

## Failure Points and Mitigation

Examples include glare, shadows, vibration, electrical noise, blind spots, wire disconnection, and regulator failure.

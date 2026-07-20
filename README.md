# RoboTeamBZU — WRO Future Engineers 2026

This repository contains the engineering materials for RoboTeamBZU's self-driving vehicle developed for the WRO Future Engineers 2026 competition.

> **Project status:** Active development  
> Mechanical, electrical, and software designs are still being tested and improved.

## Repository Contents

| Folder | Description |
|---|---|
| `engineering-journal/` | Development history, design versions, decisions, tests, failures, and improvements |
| `docs/` | Final technical documentation and engineering journal PDF |
| `models/` | Fusion 360, STEP, and STL files for custom parts |
| `schemes/` | Mechanical, electrical, wiring, block, and software diagrams |
| `src/` | Raspberry Pi and ESP32 source code |
| `tests/` | Test procedures, measured results, tuning, and failure analysis |
| `t-photos/` | Team photos |
| `v-photos/` | Vehicle photos from all required sides |
| `video/` | Links to autonomous driving demonstration videos |
| `other/` | Bill of materials, specifications, licenses, and supporting files |

## Robot Overview

| Item | Current Selection |
|---|---|
| Main controller | Raspberry Pi 5 |
| Support controller | ESP32 |
| Camera | To be confirmed |
| LiDAR | To be confirmed |
| IMU | To be confirmed |
| Drive motor | To be confirmed |
| Steering actuator | To be confirmed |
| Battery system | To be confirmed |
| Robot dimensions | To be measured |
| Robot mass | Approximately 700 g, subject to change |

## Competition Tasks

The robot is being developed for:

1. **Open Challenge** — complete three autonomous laps while navigating a randomly configured track.
2. **Obstacle Challenge** — complete three autonomous laps, pass red and green traffic signs from the required side, and perform parallel parking.

## Engineering Goals

- Stable autonomous navigation from randomized starting positions
- Reliable lane and wall perception
- Correct red and green traffic-sign obedience
- Accurate localization and obstacle mapping
- Reproducible mechanical, electrical, and software design
- Evidence-based engineering decisions through testing and iteration

## Current Development Workflow

1. Design or modify a subsystem.
2. Record the change in the relevant engineering-journal version.
3. Test the change using a repeatable procedure.
4. Record measured results and observed failure modes.
5. Keep, revise, or reject the change based on evidence.
6. Commit the change with a clear Git message.

## Quick Links

- [Engineering Journal](engineering-journal/README.md)
- [Mechanical Design](docs/Mechanical-Design.md)
- [Power and Sensor Architecture](docs/Power-and-Sensor-Architecture.md)
- [Software Architecture](docs/Software-Architecture.md)
- [Testing Workflow](tests/README.md)
- [Build and Setup Guide](docs/Build-and-Setup.md)
- [Bill of Materials](other/BOM.md)
- [Video Links](video/video.md)

## Team

Add the names and responsibilities of the 2–3 student team members here.

| Team Member | Role |
|---|---|
| Member 1 | Mechanical design and manufacturing |
| Member 2 | Electronics and embedded systems |
| Member 3 | Computer vision and autonomous navigation |

## License and Attribution

All third-party libraries, hardware documentation, and reused open-source resources must be listed in [`other/licenses.md`](other/licenses.md).

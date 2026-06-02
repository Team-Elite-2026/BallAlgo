# BallAlgo — Project Context

## Robot Platform

- **Competition:** RoboCup Junior Soccer Open League
- **Form factor:** 18 cm diameter circle; ball intake at the front
- **Coordinate system:** 0° = front (forward), 90° = right — use this convention for all headings, angles, and motion math

## Architecture: Dual-MCU Stack

This repo is one side of a dual-MCU robot. Two codebases divide hardware ownership — don't duplicate responsibilities across them.

| Side | Board | Path |
|------|-------|------|
| **BallAlgo** (this repo) | Raspberry Pi CM5 | `BallAlgo/` |
| **Offense2026** | Teensy 4.1 | `Offense2026/` |

The Pi sends perception results to the Teensy over UART (`/dev/serial0`). Do not add motor or low-level sensor control to the Pi side unless intentionally restructuring the architecture.

## Hardware

| Component | Owner | Notes |
|-----------|-------|-------|
| 4 drive motors | Teensy | Placed at 35° from the sides (holonomic layout) |
| Compass | Teensy | Heading reference |
| Line sensors | Teensy | Field line detection |
| Light gate | Teensy | Intake ball detection |
| Camera | Pi | Ball / field vision |
| LD19 LiDAR | Pi | Obstacle / ranging |

## BallAlgo (this repo)

- **Role:** Vision (camera), LD19 LiDAR, higher-level perception and processing
- **Production entry:** C++ binary `ballalgo` — build via `CMakeLists.txt`, sources in `src/`, config in `src/config.hpp`
- **Motion docs:** `MOTION.md`
- **LiDAR docs:** `LD19.md` — read before touching lidar code
- **Bench tools (Python, not competition):** `tools/` — `hsv_picker.py`, `lidar_visual.py`, `py_lidar_bench/`
- **UART config:** baud rate and port in `src/config.hpp`

## Offense2026 (Teensy)

- **Role:** Real-time motor control, line sensors, compass, switches, movement, defense/orbit
- **Entry:** `Offense2026/src/main.cpp`
- **Build system:** PlatformIO, `env:teensy41`

## Division of Responsibility

| Subsystem | Pi (BallAlgo) | Teensy (Offense2026) |
|-----------|:---:|:---:|
| Camera / ball tracking | Yes | — |
| LD19 LiDAR | Yes | — |
| Drive, orbit, defense, line follow | — | Yes |
| Motors & low-level sensors | — | Yes |

## Where to Start

- **LiDAR / camera / serial protocol:** `src/` and `LD19.md`
- **Motion, PID, hardware drivers:** `Offense2026/src/`
- **UART message format changes:** update both sides together — Pi and Teensy must stay in sync
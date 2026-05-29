# BallAlgo C++ (`ballalgo`)

Production entry point for Raspberry Pi CM5. Replaces `deployment/` Python.

## Build (on Pi)

```bash
sudo apt install build-essential cmake libopencv-dev libeigen3-dev \
  libgpiod-dev libcamera-dev pkg-config

cd BallAlgo
cmake -B build
cmake --build build -j
./build/ballalgo
```

## Runtime

- Camera: native `libcamera`
- UART: `/dev/serial0` @ 2M — ASCII perception + binary `ActionChunk` (magic `0xCEFAEDFE`)
- LiDAR: `/dev/ttyAMA4` @ 230400, GPIO12 PWM held LOW
- Config: [`src/config.hpp`](src/config.hpp), thresholds: [`thresholds.json`](thresholds.json)

## Architecture

| Module | Role |
|--------|------|
| `vision/SectorTracker` | HSV ball sectors + goals |
| `lidar/` | LD19 reader + wall localizer |
| `estimation/` | Pose + ball 4-state Kalman |
| `motion/AStar3D` | 3D grid planner (x, y, heading) |
| `motion/HermiteSpline` | Path smoothing (Eigen) |
| `motion/VelocityProfile` | Seven-phase jerk-limited S-curve along path |
| `motion/Protocol` | Teensy-compatible chunks + clock pong |

## Bench tools

See [`tools/README.md`](tools/README.md).

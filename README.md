# BallAlgo — Pi Quick Reference

Commands for building and running BallAlgo on the Raspberry Pi CM5. Assumes the repo is already on the Pi.

All commands below are run from the **BallAlgo repo root** unless noted.

```bash
cd ~/BallAlgo   # adjust path if your clone lives elsewhere
```

---

## First-time setup (dependencies)

Run once on a fresh Pi OS image:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  libeigen3-dev libopencv-dev \
  libgpiod-dev libcamera-dev
```

Optional — Foxglove live-debug sidecar:

```bash
pip install -r foxglove_sim/requirements.txt
```

---

## Build

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

Binary output: `build/ballalgo`

Rebuild after pulling new code (no re-configure needed unless `CMakeLists.txt` changed):

```bash
git pull
cmake --build build -j$(nproc)
```

Optional CMake flags:

```bash
# Disable LiDAR support
cmake -S . -B build -DBALLALGO_ENABLE_LIDAR=OFF
cmake --build build -j$(nproc)
```

Optional system install:

```bash
sudo cmake --install build
```

---

## Run

`thresholds.json` must be in the working directory (it ships in the repo root).

```bash
./build/ballalgo
```

UART to the Teensy is on `/dev/serial0` (configured in `src/config.hpp`).

---

## Foxglove live debug (optional)

**Terminal 1 — sidecar** (WebSocket server + optional MCAP recording):

```bash
python3 foxglove_sim/sidecar.py
```

**Terminal 2 — runtime:**

```bash
./build/ballalgo
```

On your laptop, open the Foxglove desktop app and connect to:

```text
ws://<pi-ip>:8765
```

Import a layout from `foxglove_sim/layouts/ballalgo_overview_app_layout.json`.

Config: `foxglove_sim/foxglove.conf` (port, stream toggles, MCAP path).

## Offline planner case (real C++ planner, fake inputs)

This is a standalone planner harness for Foxglove. It is **not** the production
camera/LiDAR/serial pipeline.

It lets you provide:

- robot start pose
- ball start position
- ball field velocity
- chosen goal

and then reuses the real `MotionPlanner::debugPlan(...)` path generation stack to
publish the resulting path/trajectory into the normal Foxglove planner panels.

Build target:

```bash
cmake -S . -B build
cmake --build build --target ballalgo_planner_case -j$(nproc)
```

Run with Foxglove:

**Terminal 1 — sidecar**

```bash
python3 foxglove_sim/sidecar.py
```

**Terminal 2 — planner case**

```bash
./build/ballalgo_planner_case \
  --robot-pose-centered-cm 0 0 0 \
  --ball-position-centered-cm 20 30 \
  --ball-velocity-field-mps 0.2 -0.1 \
  --goal yellow
```

Useful flags:

- `--robot-pose x_mm y_mm heading_deg`
- `--robot-pose-centered-cm x_cm y_cm heading_deg`
- `--ball-position x_mm y_mm`
- `--ball-position-centered-cm x_cm y_cm`
- `--robot-velocity-field-mps vx vy`
- `--ball-velocity-field-mps vx vy`
- `--goal blue|yellow`
- `--goal-point x_mm y_mm`
- `--goal-point-centered-cm x_cm y_cm`
- `--publish-hz hz`
- `--duration-s seconds`

Coordinate conventions:

- field `mm`: bottom-left origin, matching the runtime/localizer
- centered `cm`: field center at `(0, 0)`, convenient for manual cases
- velocity inputs: field-frame `m/s`

---

## Useful paths

| Item | Location |
|------|----------|
| Production config | `src/config.hpp` |
| Vision thresholds | `thresholds.json` |
| Motion pipeline docs | `docs/pipeline_context.md` |
| LiDAR docs | `docs/LD19.md` |
| Foxglove docs | `foxglove_sim/README.md` |

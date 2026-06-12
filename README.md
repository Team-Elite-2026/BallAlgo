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

Import a layout from `foxglove_sim/layouts/trajectory_debug_layout.json` for
trajectory replay/debug work, or use `foxglove_sim/layouts/ballalgo_overview_app_layout.json`
for the broader runtime view.

Config: `foxglove_sim/foxglove.conf` (port, stream toggles, MCAP path).

Trajectory replay cases and generators live under `tests/trajectory_cases/`
and `tools/trajectory_debug/`.

For commanded-goal preview work, you can also launch the local editor:

```bash
python3 tools/trajectory_debug/case_editor_server.py --build-dir build
```

and open `http://127.0.0.1:8765` in a browser on the Pi or over SSH port
forwarding.

---

## Useful paths

| Item | Location |
|------|----------|
| Production config | `src/config.hpp` |
| Vision thresholds | `thresholds.json` |
| Motion pipeline docs | `docs/pipeline_context.md` |
| LiDAR docs | `docs/LD19.md` |
| Foxglove docs | `foxglove_sim/README.md` |

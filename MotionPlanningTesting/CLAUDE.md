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

---

## MotionPlanningTesting Sub-project

This directory is a **standalone C++ testbed** for the ball-acquisition motion planner. It is not part of the production `ballalgo` binary — it exists to develop and visualize the planning pipeline before integrating into the main codebase.

### Build

```bash
cmake -S . -B build
cmake --build build -j4
# Binary: build/bin/motion_planning
```

Requires Eigen3 (`apt install libeigen3-dev`).

### Run Modes

```bash
./build/bin/motion_planning           # EKF sensor-fusion demo (human-readable table)
./build/bin/motion_planning --csv     # EKF CSV output for plot_ekf.py
./build/bin/motion_planning --astar   # A* + spline terminal test (4 scenarios)
./build/bin/motion_planning --json    # JSON export → consumed by visualize_planner.py

MPLBACKEND=Agg python3 visualize_planner.py   # render planner_visualization.png
```

---

### Source Files

| File | Role |
|------|------|
| `src/main.cpp` | Entry point. Four modes: EKF demo, CSV, --astar terminal test, --json export |
| `src/astar.hpp/cpp` | A* planner + heading schedule |
| `src/hermite_spline.hpp/cpp` | Cubic Hermite spline over A* output |
| `src/robot_ekf.hpp/cpp` | Extended Kalman Filter for robot position (mouse + LiDAR fusion) |
| `src/ball_kalman.hpp/cpp` | Kalman filter for ball position (camera measurements) |
| `src/kalman.hpp/cpp` | Generic Kalman filter base (used by BallKalman) |
| `visualize_planner.py` | Python visualizer — calls `--json`, draws with matplotlib |

---

### Planning Pipeline

```
EKF (robot x,y)  ──┐
BallKalman (ball x,y) ──┤
                        ▼
                  AStar::plan(robot_x, robot_y, robot_heading, ball_x, ball_y)
                        │
                        ├─ Computes approach point (10 cm behind ball, aligned with goal)
                        ├─ Runs 8-directional A* on 37×49 grid (5 cm/cell)
                        ├─ Ball is circular obstacle (radius = BALL_CLEAR_CM = 8 cm)
                        └─ computeHeadings(): smooth arc-length heading schedule
                             robot_heading → approachHeading (strike angle)
                        │
                        ▼
                  HermiteSpline::build(astar_result)
                        │
                        ├─ Tangent direction = heading schedule
                        ├─ Tangent magnitude = avg adjacent chord lengths (C1 continuity)
                        └─ SplinePoint::heading = robot ORIENTATION (not direction of travel)
                        │
                        ▼
                  Output: SplinePoint[] {x, y, heading} at ~1 cm resolution
                  → Send (x, y) as position target to Teensy
                  → Send heading as orientation command to Teensy
```

---

### Key Constants — Where to Change Them

All algorithm constants live in **`src/astar.hpp`**. Change them once; everything else picks them up automatically (the JSON export reads them at runtime, Python reads the JSON).

```cpp
// src/astar.hpp
CELL_CM       = 5.0    // grid resolution (cm). Smaller = finer path, slower search.
BALL_CLEAR_CM = 8.0    // obstacle radius around ball (cm).
                       // Must be < approach_cm or the target will be inside the obstacle.
                       // Current 8 cm → blocks 3×3 grid cells around ball.

// AStar::plan() defaults (can be overridden per call)
goal_x/y      = 0, 120   // attack goal centre (cm)
approach_cm   = 10.0     // distance from ball centre to stand. Must be > BALL_CLEAR_CM.
```

**Test scenario positions** live in **two places** in `src/main.cpp` that must be kept in sync:
- `runJsonExport()` (line ~160) — used by `visualize_planner.py`
- `runAStarTest()` (line ~290) — used by `--astar` terminal output

Each scenario row: `{ title, rx, ry, rh, bx, by, gx, gy, approach_cm }`
- `rx/ry` robot position (cm), `rh` heading (°)
- `bx/by` ball position (cm)
- `gx/gy` goal centre (cm, usually 0, 120)
- `approach_cm` stand-behind distance

Field bounds: x ∈ [-90, +90] cm, y ∈ [-120, +120] cm.

---

### Coordinate System (critical — applies everywhere)

```
+y = forward (toward attack goal)    heading 0°  = facing +y
+x = right                           heading 90° = facing +x
origin = field centre                clockwise positive
```

**Grid mapping:**
- `world_x = (col - 18) × 5 cm`   — col 18 = x=0 (centre)
- `world_y = (row - 24) × 5 cm`   — row 24 = y=0 (centre)

Attack goal is at y = +120 cm (top), defence goal at y = -120 cm (bottom).

**Heading vector from angle:**
```
dx = sin(heading_rad)   // world +x component
dy = cos(heading_rad)   // world +y component
```

---

### Heading Schedule Design

The robot is holonomic — facing angle and direction of travel are independent. The heading schedule controls **orientation** (where the robot faces), not movement direction.

- **Start:** robot's current IMU heading
- **End:** `approachHeading` = atan2(ball - approach_point) — the exact angle that aligns the robot with ball and goal
- **Interpolation:** linear by cumulative arc length, shortest angular path (handles 350° → 10° as +20°, not -340°)
- `SplinePoint::heading` = orientation schedule value, NOT the spline tangent direction

If `evaluate()` returns a heading that seems wrong, it's the **direction of travel** (velocity of the curve). Use the heading from `build()` output instead.

---

### Common Debugging Traps

**A* returns no path (`found = false`)**
- Ball is cornered against field boundary — the approach point may be outside the field
- `approach_cm` ≤ `BALL_CLEAR_CM` — approach point lands inside the obstacle zone
- Ball very close to field edge — the circular obstacle overlaps the boundary

**Approach point is inside the obstacle circle**
- Always ensure `approach_cm > BALL_CLEAR_CM`. Default is 10 > 8. ✓
- Verify with: `approachPoint()` distance from ball should equal `approach_cm` exactly

**Heading arrows in visualization point the wrong direction**
- Check that `heading_vec(h)` uses `(sin(h), cos(h))` — NOT `(cos(h), sin(h))`
- Convention is 0°=+y so dy component uses cos, dx uses sin

**Visualizer fails to find binary**
- Python looks for `build/bin/motion_planning` relative to `visualize_planner.py`
- Run `cmake --build build` first
- The user modified the binary path to `bin/motion_planning` (no `build/` prefix) — check `get_planner_data()` in `visualize_planner.py` if it breaks

**Spline looks kinked at a node**
- C1 continuity requires the same tangent scale on both sides of an interior node
- `build()` uses average of adjacent chord lengths — don't change to per-segment chord without also ensuring matching scales at shared nodes

**JSON output has scientific notation**
- `std::fixed << std::setprecision(4)` must be set before any float output in `runJsonExport()`
- Integer fields (cols, rows, node_count) are unaffected by `std::fixed` ✓

---

### Visualizer Architecture

```
visualize_planner.py
  └─ get_planner_data()
       └─ subprocess: ./build/bin/motion_planning --json → stdout JSON
            ├─ top-level constants (field size, cell size, ball_clear_cm, ...)
            └─ scenarios[]: robot, ball, goal, approach, astar{path_cm, ...}, spline[{x,y,heading}]
  └─ draw_scenario(ax, data, constants)
       ├─ draw_field()  — uses constants from JSON (grid size, goal positions)
       ├─ A* path plotted as blue dashed line with dots
       ├─ Hermite spline plotted as solid red curve
       ├─ Heading arrows every HDG_EVERY=6 spline points (robot orientation)
       ├─ Approach target: green circle + arrow pointing toward ball
       └─ Stats overlay: node count, path length, rotation, strike angle
```

**Python has no algorithm code.** All computation is in C++. If you change `BALL_CLEAR_CM` in `astar.hpp`, rebuild, and re-run the visualizer — the obstacle circle radius updates automatically from the JSON.

---

### Integration Path to Production

When integrating into `ballalgo`:
1. Copy `src/astar.hpp/cpp` and `src/hermite_spline.hpp/cpp` into the main `src/`
2. Call `AStar::plan(ekf.getX(), ekf.getY(), compass_deg, ball_kf.getX(), ball_kf.getY())`
3. Walk the returned `spline[]` — send `(x, y)` as position target and `heading` as orientation to Teensy over UART
4. Re-plan every camera frame or whenever ball position changes significantly
5. `approach_cm` and `BALL_CLEAR_CM` may need tuning based on physical robot behaviour



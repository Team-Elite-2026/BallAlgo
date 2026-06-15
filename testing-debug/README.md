# Trajectory Replay Testing Debug

This folder is the single place to start when you want to test the Pi `BallAlgo`
trajectory replay path against the `Offense2026` Teensy code.

## What We Added

There are two main parts:

1. Pi-side replay generation and Foxglove publishing in `BallAlgo`
2. Teensy-side trajectory execution and replay debug prints in `Offense2026`

## Where Everything Lives

### Pi-side replay code

- `src/main.cpp`
- `src/motion/TrajectoryReplay.hpp`
- `src/motion/TrajectoryReplay.cpp`
- `src/motion/TrajectoryReplayRunner.hpp`
- `src/motion/TrajectoryReplayRunner.cpp`
- `src/motion/ActionChunkPublisher.cpp`
- `src/FoxGloveSim/FoxgloveTelemetryPublisher.cpp`
- `foxglove_sim/sidecar.py`
- `foxglove_sim/layouts/trajectory_debug_layout.json`

### Pi-side helper scripts

- `tools/trajectory_debug/generate_cases.py`
- `tools/trajectory_debug/run_replay.py`
- `tools/trajectory_debug/verify_cases.py`
- `tools/trajectory_debug/plot_case.py`
- `tools/trajectory_debug/case_editor_server.py`
- `tools/trajectory_debug/case_editor.html`
- `uart_test_pi.py`

### Replay case specs

These are the editable source specs:

- `tests/trajectory_cases/specs/forward_only.json`
- `tests/trajectory_cases/specs/strafe_only.json`
- `tests/trajectory_cases/specs/rotate_only.json`
- `tests/trajectory_cases/specs/diagonal.json`
- `tests/trajectory_cases/specs/translate_while_rotating.json`
- `tests/trajectory_cases/specs/accelerate_then_stop.json`
- `tests/trajectory_cases/specs/chunk_boundary_handoff.json`
- `tests/trajectory_cases/specs/num_actions_0_stop_chunk.json`
- `tests/trajectory_cases/specs/planner_to_pose_demo.json`

### Generated replay artifacts

These are the actual `.traj` files used by the replay runner:

- `tests/trajectory_cases/generated/forward_only.traj`
- `tests/trajectory_cases/generated/strafe_only.traj`
- `tests/trajectory_cases/generated/rotate_only.traj`
- `tests/trajectory_cases/generated/diagonal.traj`
- `tests/trajectory_cases/generated/translate_while_rotating.traj`
- `tests/trajectory_cases/generated/accelerate_then_stop.traj`
- `tests/trajectory_cases/generated/chunk_boundary_handoff.traj`
- `tests/trajectory_cases/generated/num_actions_0_stop_chunk.traj`
- `tests/trajectory_cases/generated/planner_to_pose_demo.traj`

SVG previews for each case are in the same `generated/` folder.

### Teensy-side execution code

- `../Offense2026/src/main.cpp`
- `../Offense2026/src/TrajectoryExecutor.h`
- `../Offense2026/src/TrajectoryExecutor.cpp`
- `../Offense2026/src/Cam.cpp`

## Current Temporary Test Wiring Assumption

For the current trajectory test setup, the Teensy code was temporarily changed so:

- Pi <-> Teensy trajectory comms use `Serial2`
- `LinePCBComm` is effectively disabled from the active loop
- mouse velocity is forced to `0,0`
- line avoidance is effectively disabled
- `Cam.cpp` was moved off `Serial2` so it does not steal Pi bytes

This is temporary test-mode behavior for debugging the replay path.

## What Each Test Case Is For

- `forward_only`: short straight forward motion
- `strafe_only`: pure lateral motion
- `rotate_only`: pure spin
- `diagonal`: combined x/y translation
- `translate_while_rotating`: translation and spin together
- `accelerate_then_stop`: ramp motion, then stop chunk
- `chunk_boundary_handoff`: two chunks back-to-back
- `num_actions_0_stop_chunk`: explicit zero-action stop chunk
- `planner_to_pose_demo`: real C++ planner-generated path to a pose

## Pi Quick Start

Run all Pi commands from the `BallAlgo` repo root.

```bash
cd ~/BallAlgo
```

### 1. Rebuild Pi runtime

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

### 2. Regenerate replay artifacts

```bash
python3 tools/trajectory_debug/generate_cases.py
```

Generate only one case:

```bash
python3 tools/trajectory_debug/generate_cases.py --spec tests/trajectory_cases/specs/forward_only.json
```

### 3. Verify artifacts

```bash
python3 tools/trajectory_debug/verify_cases.py
```

### 4. Start Foxglove sidecar

```bash
python3 foxglove_sim/sidecar.py
```

### 5. Run a replay case

In a second terminal:

```bash
python3 tools/trajectory_debug/run_replay.py forward_only
```

Or start sidecar automatically:

```bash
python3 tools/trajectory_debug/run_replay.py forward_only --with-sidecar
```

You can also run the runtime directly:

```bash
./build/ballalgo --trajectory-replay-artifact tests/trajectory_cases/generated/forward_only.traj
```

## Foxglove

Connect Foxglove to:

```text
ws://<pi-ip>:8765
```

Import this layout:

```text
foxglove_sim/layouts/trajectory_debug_layout.json
```

### Topics to watch

- `/traj/target`
- `/traj/teensy_raw`
- `/traj/error`
- `/planner/profile`
- `/planner/scene/path`
- `/robot/pose`
- `/robot/twist`

## Teensy Quick Start

Flash the current `Offense2026` branch to the Teensy.

Open the USB serial monitor and look for replay debug prints like:

```text
[TEENSY REPLAY] pong clock_offset_us=...
[TEENSY REPLAY] queued chunk traj=...
[TEENSY REPLAY] activated chunk traj=...
```

### How to interpret them

- No `queued chunk`:
  Teensy is not receiving/parsing the Pi chunk
- `queued chunk` but no `activated chunk`:
  timing/scheduling issue
- `activated chunk` but no wheel motion:
  execution started, so look at motor output or remaining gating

## Fast Communication Tests

### Pi-side UART sanity check

This just checks whether bytes are coming back on the expected Pi UART:

```bash
python3 uart_test_pi.py --read-only --port /dev/serial0 --baud 2000000
```

If the Teensy is alive on the right link, you should see `RX:` output, though it
may look garbled because the protocol is binary.

## Visual Planner Editor

For commanded-goal path previews:

```bash
python3 tools/trajectory_debug/case_editor_server.py --build-dir build
```

Then open:

```text
http://127.0.0.1:8765
```

## Useful One-Case Commands

### Forward only

```bash
python3 tools/trajectory_debug/generate_cases.py --spec tests/trajectory_cases/specs/forward_only.json
python3 tools/trajectory_debug/run_replay.py forward_only --with-sidecar
```

### Chunk handoff

```bash
python3 tools/trajectory_debug/generate_cases.py --spec tests/trajectory_cases/specs/chunk_boundary_handoff.json
python3 tools/trajectory_debug/run_replay.py chunk_boundary_handoff --with-sidecar
```

### Stop chunk

```bash
python3 tools/trajectory_debug/generate_cases.py --spec tests/trajectory_cases/specs/num_actions_0_stop_chunk.json
python3 tools/trajectory_debug/run_replay.py num_actions_0_stop_chunk --with-sidecar
```

## Troubleshooting Checklist

### If `/traj/*` topics show no data

- rebuild `BallAlgo`
- restart sidecar
- reconnect Foxglove
- confirm replay mode prints:

```text
trajectory replay artifact: ...
```

### If replay mode starts but nothing is sent

Check Pi replay debug lines:

```text
[REPLAY] serial_open=yes/no
[REPLAY] clock_sync_ping_seen=yes/no
[REPLAY] armed ...
[REPLAY] sent chunk_index=...
```

### If Pi says chunk was sent but robot still does nothing

Check Teensy USB serial for:

```text
[TEENSY REPLAY] queued chunk ...
[TEENSY REPLAY] activated chunk ...
```

If Pi says sent, but Teensy never says queued, that is still a UART/wiring/port issue.

## Recommended Test Order

1. `forward_only`
2. `strafe_only`
3. `rotate_only`
4. `diagonal`
5. `translate_while_rotating`
6. `accelerate_then_stop`
7. `chunk_boundary_handoff`
8. `num_actions_0_stop_chunk`


# Trajectory Replay Cases

This folder holds reusable trajectory replay specs and generated artifacts for
Stage 1 trajectory-executor debugging.

## Structure

- `specs/*.json`: editable case definitions
- `generated/*.traj`: replay artifacts produced from those specs

## Case Kinds

### `planner_goal`

Uses the real C++ `MotionPlanner::debugPlanToPose()` path/chunk generation via
the `ballalgo_planner_case` binary.

Required fields:

```json
{
  "case_name": "planner_to_pose_demo",
  "kind": "planner_goal",
  "start_pose": { "x_mm": 910.0, "y_mm": 420.0, "heading_deg": 0.0 },
  "goal_pose": { "x_mm": 1320.0, "y_mm": 1480.0, "heading_deg": 90.0 }
}
```

### `manual_sequence`

Builds one or more `ActionChunk`-style sequences without calling the planner.
Useful for protocol/executor tests like pure translation, pure rotation, chunk
handoff, or `num_actions = 0` stop chunks.

Chunk segments can be constant:

```json
{
  "duration_ms": 200,
  "vx": 0.0,
  "vy": 0.45,
  "omega": 0.0
}
```

or linearly ramped:

```json
{
  "duration_ms": 80,
  "vx_start": 0.0,
  "vx_end": 0.0,
  "vy_start": 0.0,
  "vy_end": 0.70,
  "omega_start": 0.0,
  "omega_end": 0.0
}
```

Use `"stop_chunk": true` to emit a `num_actions = 0` chunk.

## Generate Artifacts

From the `BallAlgo` repo root:

```bash
cmake -S . -B build -DBALLALGO_BUILD_APP=OFF -DBALLALGO_BUILD_TESTS=OFF
cmake --build build --target ballalgo_planner_case -j4
python3 tools/trajectory_debug/generate_cases.py
```

Generate one case:

```bash
python3 tools/trajectory_debug/generate_cases.py --spec tests/trajectory_cases/specs/forward_only.json
```

## Inspect and Plot

Inspect an artifact with the C++ tool:

```bash
./build/ballalgo_planner_case --inspect-artifact tests/trajectory_cases/generated/forward_only.traj
```

Generate an SVG preview:

```bash
python3 tools/trajectory_debug/plot_case.py tests/trajectory_cases/generated/planner_to_pose_demo.traj
```

Verify the generated examples:

```bash
python3 tools/trajectory_debug/verify_cases.py
```

## Visual Case Editor

For quick commanded-goal path previews, launch the local editor:

```bash
python3 tools/trajectory_debug/case_editor_server.py --build-dir build
```

Then open:

```text
http://127.0.0.1:8765
```

The editor lets you click a start pose and commanded goal on a field grid,
adjust headings and starting velocity, and preview the real C++
`MotionPlanner::debugPlanToPose()` output before saving a spec or replaying a
case.

## Replay on the Real Runtime

Once the full `ballalgo` runtime is built on the Pi, run:

```bash
python3 tools/trajectory_debug/run_replay.py tests/trajectory_cases/generated/forward_only.traj --with-sidecar
```

That launches the runtime in `--trajectory-replay-artifact` mode so the Pi can
still read Teensy telemetry, LiDAR, and publish Foxglove topics while the chunk
sequence is being replayed.

## LiDAR Pose-Track Test (drive a rectangle, record + plot the LiDAR pose)

This validates the real C++ `LidarLocalizer` (with deskewing) by driving the robot
in a closed rectangle while recording the estimated pose, then plotting the track
with time as a color gradient.

The `rectangle_loop` case drives a 600 mm × 600 mm CCW square at 0.30 m/s with a
fixed heading (pure translation, `omega = 0`), starting at field `(600, 900)` mm.

1. Generate the artifact:

   ```bash
   python3 tools/trajectory_debug/generate_cases.py \
       --spec tests/trajectory_cases/specs/rectangle_loop.json
   ```

2. On the Pi, replay it **with the sidecar** so `/robot/pose` is recorded to MCAP
   (`foxglove_sim/recordings/<label>.mcap`). The pose comes from the same
   `MotionPipeline::updateLidar` path the robot normally uses, so deskewing applies:

   ```bash
   python3 tools/trajectory_debug/run_replay.py rectangle_loop --with-sidecar
   ```

3. Plot the recorded track (newest recording is picked automatically). Overlay the
   commanded rectangle for comparison:

   ```bash
   python3 tools/trajectory_debug/plot_pose_track.py \
       --artifact tests/trajectory_cases/generated/rectangle_loop.traj
   ```

   This writes two SVGs next to the recording: `…​.pose_track.svg` (position track
   colored by elapsed time — dark = start, yellow = end, circle = start, square =
   end, with a time colorbar and the dashed commanded square overlaid) and
   `…​.pose_track.velocity.svg` (field-frame `vx(t)` and `vy(t)` LiDAR velocity).

For the full step-by-step writeup (build flags, deps, Foxglove notes), see
[docs/lidar_pose_track_test.md](../../docs/lidar_pose_track_test.md).

> Note: the LiDAR localizer uses `kLidarYawOffsetDeg = 180` in `src/config.hpp`
> (matching `tools/py_lidar_bench/config.py`). Changing it requires rebuilding the
> `ballalgo` runtime.

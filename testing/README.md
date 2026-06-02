# BallAlgo Testing Harness

Dedicated offline testing tools for the motion planner and action-chunk stack.

This folder is for algorithm validation, not the competition runtime.

## What This Tests

- `motion/AStar3D`
- `motion/HermiteSpline`
- `motion/VelocityProfile`
- `motion/MotionPlanner::planToPose(...)`
- Perfect-state execution of the generated action chunk with no noise and no actuator lag
- Production-style rolling replanning, where the planner reissues a fresh chunk at a fixed control rate

The initial simulation mode assumes:

- perfect starting pose
- perfect heading input
- no wheel slip
- no localization noise
- ideal chunk execution

That lets us answer the first question cleanly:
"If the robot knows exactly where it is, does the planner itself generate a sensible trajectory to the target pose?"

By default, the bench runs in `rolling_replan` mode, which is the production-faithful behavior:

1. plan a chunk from the current pose
2. execute only the short horizon that would actually run before the next publish
3. replan from the updated pose
4. repeat until the goal is reached or a limit is hit

Use `--mode single_chunk` only when you explicitly want to inspect one isolated plan/chunk.

## Build

You can build just the testing tool if you do not want the camera runtime:

```bash
cd BallAlgo
cmake -S . -B build-test -DBALLALGO_BUILD_APP=OFF -DBALLALGO_BUILD_TESTING=ON
cmake --build build-test -j
```

This produces:

- `build-test/ballalgo_planner_bench`

## Single Perfect-State Case

Run one case from a known start pose to a target pose.

Coordinates are the centered-cm field coordinates used by the debug stream.

```bash
cd BallAlgo
./build-test/ballalgo_planner_bench \
  --start-x-cm 40 \
  --start-y-cm -60 \
  --start-heading-deg 0 \
  --mode rolling_replan \
  --goal-x-cm 0 \
  --goal-y-cm 0 \
  --goal-heading-deg 0 \
  --output testing/artifacts/case_center.json \
  --label center_test
```

The JSON artifact contains:

- one or more replans, each with:
  - waypoints from A*
  - dense spline path samples
  - velocity-profile samples
  - the exact planned action chunk
- the flattened command stream actually executed by the perfect-state simulation
- a full perfect-state executed pose trace

If you want the older single-plan view:

```bash
./build-test/ballalgo_planner_bench \
  --start-x-cm 40 \
  --start-y-cm -60 \
  --start-heading-deg 0 \
  --mode single_chunk \
  --goal-x-cm 0 \
  --goal-y-cm 0 \
  --goal-heading-deg 0 \
  --output testing/artifacts/case_center_single_chunk.json \
  --label center_test_single_chunk
```

## Visualize One Case

Requires `matplotlib`.

```bash
cd BallAlgo/testing
python3 visualize_plan.py artifacts/case_center.json
python3 visualize_chunks.py artifacts/case_center.json
```

What to look at:

- field plot: start, goal, all replanned paths, replan start points, perfect-state trace
- body velocity commands: `vx`, `vy`, resultant speed
- body acceleration commands: `ax`, `ay`, resultant acceleration
- heading trace and goal heading
- position error vs time

## Run A Sweep

This runs many perfect-state cases over a start-pose grid against one fixed goal.

```bash
cd BallAlgo/testing
python3 run_pose_sweep.py \
  --bench ../build-test/ballalgo_planner_bench \
  --output-dir artifacts/sweep_center \
  --mode rolling_replan \
  --goal-x-cm 0 \
  --goal-y-cm 0 \
  --goal-heading-deg 0 \
  --start-x-min-cm -60 \
  --start-x-max-cm 60 \
  --start-y-min-cm -100 \
  --start-y-max-cm 100 \
  --step-cm 20 \
  --start-heading-deg 0
```

This writes:

- one JSON artifact per case
- `summary.json` with final position errors and action counts

## Recommended Investigation Order

1. Perfect state, no noise, heading `0 deg`
2. Perfect state, no noise, heading `90 deg`
3. Perfect state, no noise, heading `-90 deg`
4. Same sweep with different start headings

If the perfect-state simulation fails, the bug is in planner math or frame transforms.
If the perfect-state simulation succeeds but the robot fails, the bug is likely in:

- heading convention mismatch
- localization
- action execution / drivetrain mapping
- runtime frame transforms between Pi and Teensy

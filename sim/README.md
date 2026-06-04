# BallAlgo Simulation

Dedicated offline simulation tools for the Pi motion stack.

This package is the production-faithful simulation home for `BallAlgo`. Its default mode routes through the real ball-aware planner entrypoint on the Pi:

1. build robot pose state
2. convert field-frame ball state into the body-frame `BallState` production expects
3. derive body-frame goal angle from a fixed field goal target
4. call `MotionPlanner::plan(...)`
5. execute only the publish-period slice of the returned chunk
6. replan from the updated pose

The legacy pose-target harness still exists as a low-level mode for isolating direct pose-planning math.

## Build

```bash
cd BallAlgo
cmake -S . -B build-test -DBALLALGO_BUILD_APP=OFF -DBALLALGO_BUILD_TESTING=ON
cmake --build build-test -j
```

This produces:

- `build-test/ballalgo_sim`
- `build-test/ballalgo_planner_bench`

`ballalgo_planner_bench` is now a compatibility wrapper that defaults to pose-target mode for older workflows.

## Production Route Simulation

Run the production-faithful ball planner simulation:

```bash
cd BallAlgo
./build-test/ballalgo_sim \
  --mode production_ball_plan \
  --start-x-cm 40 \
  --start-y-cm -60 \
  --start-heading-deg 0 \
  --ball-x-cm 0 \
  --ball-y-cm 0 \
  --ball-vx-cm-s 0 \
  --ball-vy-cm-s 0 \
  --goal yellow \
  --output sim/artifacts/production_case.json \
  --label production_case
```

Important notes:

- ball inputs are field-frame centered-cm and centered-cm/s
- `--goal blue|yellow` resolves to a hardcoded goal centered on the field midline
- the simulator uses a 158 cm white-line field width and a 7.4 cm goal depth for this goal placement
- replanning cadence defaults to `config::kChunkPublishHz`
- only the chunk slice that would execute before the next publish is applied

## Pose Target Compatibility Mode

Run the low-level direct pose planner:

```bash
./build-test/ballalgo_sim \
  --mode pose_target \
  --start-x-cm 40 \
  --start-y-cm -60 \
  --start-heading-deg 0 \
  --goal-x-cm 0 \
  --goal-y-cm 0 \
  --goal-heading-deg 0 \
  --output sim/artifacts/pose_target_case.json \
  --label pose_target_case
```

You can still use the old executable name:

```bash
./build-test/ballalgo_planner_bench \
  --start-x-cm 40 \
  --start-y-cm -60 \
  --start-heading-deg 0 \
  --goal-x-cm 0 \
  --goal-y-cm 0 \
  --goal-heading-deg 0 \
  --output testing/artifacts/pose_target_case.json \
  --label pose_target_case
```

That wrapper automatically defaults to `--mode pose_target`.

## Visualization

Requires `matplotlib`.

```bash
cd BallAlgo/sim
python3 visualize_plan.py artifacts/production_case.json
python3 visualize_chunks.py artifacts/production_case.json
```

The plan plot shows:

- replanned A* waypoints and spline paths
- executed trace
- replan start points
- per-replan planned target points
- ball position and velocity in production mode
- goal target point

The chunk plot shows:

- body-frame velocity commands
- body-frame acceleration commands
- angular command channels

## Animation

Render a saveable animation:

```bash
cd BallAlgo/sim
python3 render_animation.py artifacts/production_case.json --output artifacts/production_case.gif
```

Supported outputs:

- `.gif` via Pillow
- `.mp4` via `ffmpeg`

The animation includes:

- a top-down field view
- robot trail and current heading
- current field-velocity and acceleration arrows
- the moving ball
- synchronized robot velocity and acceleration graphs

## One-Shot Workflow

Run the simulator and automatically save the standard plots and animation:

```bash
cd BallAlgo/sim
python3 run_simulation.py -- \
  --mode production_ball_plan \
  --start-x-cm 40 \
  --start-y-cm -60 \
  --start-heading-deg 0 \
  --ball-x-cm 0 \
  --ball-y-cm 0 \
  --ball-vx-cm-s 0 \
  --ball-vy-cm-s 0 \
  --goal yellow \
  --output artifacts/production_case.json \
  --label production_case
```

This writes:

- the JSON artifact
- a plan overview PNG
- a chunk-signal PNG
- an animation GIF by default

## Sweeps

Production ball planner sweep:

```bash
python3 run_pose_sweep.py \
  --bench ../build-test/ballalgo_sim \
  --output-dir artifacts/production_sweep \
  --mode production_ball_plan \
  --ball-x-cm 0 \
  --ball-y-cm 0 \
  --goal yellow \
  --start-x-min-cm -60 \
  --start-x-max-cm 60 \
  --start-y-min-cm -100 \
  --start-y-max-cm 100 \
  --step-cm 20 \
  --start-heading-deg 0
```

Pose-target sweep:

```bash
python3 run_pose_sweep.py \
  --bench ../build-test/ballalgo_planner_bench \
  --output-dir artifacts/pose_target_sweep \
  --mode pose_target \
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

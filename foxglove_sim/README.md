# BallAlgo Foxglove Live Debug

The sidecar streams the current legacy-control pipeline:

- camera frame and ball overlay
- robot pose and ball estimate
- robot/body velocity and acceleration estimates
- raw Teensy telemetry
- Pi control intent sent to the Teensy (`/control/intent`)

Important limitation: this branch does not currently publish a true Teensy
"desired heading" / orbit-output signal back to the Pi. The live scene shows the
robot heading and the ball angle relative to the robot, but not a separate
verified desired-heading vector.

The old planner path, trajectory target, replay, and tracking-error topics were
removed from this branch.

## Run

Terminal 1:

```bash
python3 foxglove_sim/sidecar.py
```

Terminal 2:

```bash
./build/ballalgo
```

Connect Foxglove Studio to:

```text
ws://<pi-ip>:8765
```

Configuration lives in `foxglove_sim/foxglove.conf`.

## Shared Layout Sync

Foxglove's REST API can create and update shared organization layouts. It does
not expose personal layouts, so API sync targets the team-shared version of the
layout.

Build the checked-in app layout JSON:

```bash
python3 foxglove_sim/layouts/build_app_layout.py
```

Then sync it to Foxglove:

```bash
python3 foxglove_sim/layouts/sync_layout.py --name "BallAlgo Live Debug"
```

Required environment:

- `FOXGLOVE_API_KEY`

Optional environment / flags:

- `FOXGLOVE_LAYOUT_ID` to pin updates to one exact shared layout
- `FOXGLOVE_LAYOUT_FOLDER` to scope or create the layout under a specific folder
- `FOXGLOVE_LAYOUT_PERMISSION` (`ORG_WRITE`, `ORG_READ`, or `CREATOR_WRITE`)

If multiple shared layouts have the same name, the sync script exits instead of
guessing which one to overwrite.

## Main Topics

- `/field/scene/static`
- `/field/scene/live`
- `/robot/pose`
- `/ball/pose`
- `/robot/state`
- `/ball/state`
- `/robot/twist`
- `/robot/accel`
- `/robot/angular`
- `/ball/twist`
- `/ball/range`
- `/control/intent`
- `/teensy/raw`
- `/lidar/scan`
- `/camera/front/image`
- `/camera/front/annotations`
- `/debug/log`

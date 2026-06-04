# BallAlgo Foxglove Sim

This folder is the live-debugging replacement for the custom visualization role
of `sim/`.

The v1 goal is:

1. stream live Pi telemetry into Foxglove over LAN with low latency
2. record the same topics to MCAP for replay
3. keep BallAlgo's control loop isolated from Foxglove I/O

## What Foxglove Should Replace

Instead of maintaining separate custom plotting and animation scripts, Foxglove
should become the primary viewer for:

- generated splines and planned paths on the field
- LiDAR visualization
- velocity, acceleration, and angular telemetry plots
- recorded replay/debug sessions
- camera streams with detections and annotations

The current `sim/` folder still remains the source of truth for offline
planner-only testing. This folder is for the live/debugging observability
pipeline.

## Recommended Architecture

### Local network, real-time debugging

For normal lab debugging, do **not** send live data to the Foxglove cloud API.

Run a Foxglove WebSocket server directly on the Pi and connect to it from the
Foxglove desktop app on your laptop over the local network:

- Pi runs a Foxglove SDK server
- laptop opens `Foxglove WebSocket`
- Foxglove subscribes to topics in real time

Preferred connection shape:

```text
BallAlgo runtime on Pi
  -> Foxglove SDK WebSocket server on Pi
  -> Foxglove desktop app on laptop over LAN
```

Use a connection string like:

```text
ws://<pi-ip>:8765
```

### Remote debugging over the internet

Only if the Pi is not directly reachable should you use Foxglove remote access.

That path routes through the Foxglove platform, uses a device/gateway identity,
and is designed for NAT/firewall traversal and unreliable links. That is the
case where traffic effectively goes "through their system."

## Will This Make Debugging Slower?

Not necessarily, if we structure it correctly.

### What stays fast

- planner outputs
- pose / ball / target telemetry
- transforms
- path and spline overlays
- velocity / acceleration / angular plots

These are light compared with video.

### What can slow things down

- encoding high-rate camera frames
- shipping dense LiDAR at full raw rate
- writing MCAP synchronously on the main control loop
- emitting very large scene updates every frame

### Performance rules

To keep BallAlgo responsive:

- never block the planner/control loop on Foxglove I/O
- publish telemetry from a queue or sidecar thread/process
- send static field geometry once per session, not every frame
- publish incremental path/scene updates instead of redrawing everything
- prefer compressed image/video topics for camera streaming
- downsample LiDAR for visualization if the full raw stream is unnecessary
- write MCAP asynchronously

## What To Publish

The topic catalog is defined in [topic_catalog.py](./topic_catalog.py).

### 1. Field + spline/path drawing

Use the 3D panel in 2D mode.

Publish:

- `/field/scene/static`
- `/planner/scene/path`
- `/planner/scene/waypoints`
- `/planner/scene/target`
- `/robot/pose`

Recommended schema:

- `foxglove.SceneUpdate`

This lets us draw:

- static field outline
- goal markers
- A* waypoints
- spline path
- executed trace
- target/strike point

### 2. LiDAR visualization

Two good options:

- `foxglove.PointCloud` for direct scan visualization
- `foxglove.SceneUpdate` if we want a 2D field overlay or simplified landmarks

Recommended split:

- `/lidar/points` for the scan itself
- `/lidar/localization/scene` for extracted walls, landmarks, or estimated pose

### 3. Velocity / acceleration / angular graphs

Publish numeric time series for:

- robot linear velocity
- robot linear acceleration
- robot angular velocity
- robot angular acceleration
- ball velocity

These go into Plot panels.

Recommended topics:

- `/robot/twist`
- `/robot/accel`
- `/robot/angular`
- `/ball/twist`

### 4. Recorded playback

Always record the same live topics to MCAP at the same time they are streamed.

This gives us:

- live debugging through WebSocket
- replayable sessions through MCAP
- one consistent topic model for both

Recommended session outputs:

- one MCAP per run/test
- metadata with label, start time, git SHA if available, config profile, and note

### 5. Camera stream with detections

Use:

- image/video topic for the frame
- annotation topic for overlays

Recommended topics:

- `/camera/front/image`
- `/camera/front/annotations`

Recommended schema:

- `foxglove.CompressedImage` or `foxglove.RawImage`
- `foxglove.ImageAnnotations`

This is the cleanest way to stream frames while keeping bounding boxes,
centroids, labels, or confidence overlays aligned to the image panel.

## Suggested Initial Scope

### Phase 1

Implement first:

- robot pose
- ball pose
- target/strike pose
- spline/path scene
- velocity and acceleration plots
- MCAP recording

This alone gives strong planner debugging value with low overhead.

### Phase 2

Add:

- LiDAR point cloud / 2D scene overlay
- camera image + annotations

These are the higher-bandwidth parts and should come after the core telemetry
stream is stable.

## What Is Implemented In v1

### Runtime side

- `BallAlgo` emits structured debug snapshots over a **local Unix socket**
- no Foxglove SDK dependency is added to the C++ runtime
- the runtime can drop visualization snapshots instead of blocking the control
  loop

### Sidecar side

- [sidecar.py](./sidecar.py) starts a **Foxglove WebSocket server** on the Pi
- the same sidecar records to **MCAP** when enabled in config
- topics are declared in [topic_catalog.py](./topic_catalog.py)
- custom JSON schemas live in [schema_catalog.py](./schema_catalog.py)
- shared config lives in [foxglove.conf](./foxglove.conf)

## Current v1 Topics

Implemented now:

- `/session/info`
- `/field/scene/static`
- `/field/scene/live`
- `/planner/scene/path`
- `/robot/pose`
- `/ball/pose`
- `/robot/twist`
- `/robot/accel`
- `/robot/angular`
- `/ball/twist`
- `/debug/log`

Deferred placeholders remain in config/catalog for:

- LiDAR point cloud / localization overlays
- camera stream
- image annotations

## Recommended BallAlgo Foxglove Layout

### Field panel

Use one `3D` panel in top-down `2D` mode and drive it only from the scene topics:

- `/field/scene/static`
- `/field/scene/live`
- `/planner/scene/path`

Do not use `/robot/pose` or `/ball/pose` as visible layers in the field panel.
Keep those raw topics for debugging, replay inspection, or message browsing.

The intended field panel view is:

- green field surface
- white field border, center line, center circle, and penalty boxes
- blue left goal and yellow right goal outlines
- robot body marker
- robot heading arrow
- robot velocity arrow
- ball marker
- active spline / trajectory
- target pose marker + target heading arrow

### Plot panels

Recommended plot bindings:

#### Robot linear velocity

- `/robot/twist.vx_body_m_s`
- `/robot/twist.vy_body_m_s`
- `/robot/twist.speed_body_m_s`

Optional field-frame overlays:

- `/robot/twist.vx_field_m_s`
- `/robot/twist.vy_field_m_s`
- `/robot/twist.speed_field_m_s`

#### Robot linear acceleration

- `/robot/accel.ax_body_m_s2`
- `/robot/accel.ay_body_m_s2`
- `/robot/accel.magnitude_body_m_s2`

Optional field-frame overlays:

- `/robot/accel.ax_field_m_s2`
- `/robot/accel.ay_field_m_s2`
- `/robot/accel.magnitude_field_m_s2`

#### Robot angular kinematics

- `/robot/angular.heading_deg`
- `/robot/angular.omega_deg_s`
- `/robot/angular.alpha_deg_s2`

#### Ball velocity

- `/ball/twist.vx_body_m_s`
- `/ball/twist.vy_body_m_s`
- `/ball/twist.speed_body_m_s`

Optional field-frame overlays:

- `/ball/twist.vx_field_m_s`
- `/ball/twist.vy_field_m_s`
- `/ball/twist.speed_field_m_s`

## Config

Edit [foxglove.conf](./foxglove.conf) to turn streams on/off and change rates.

Important knobs:

- `enabled`
- `record_mcap`
- `stream_paths`
- `stream_pose`
- `stream_ball`
- `stream_velocity`
- `stream_logs`
- `stream_lidar`
- `stream_camera`
- `stream_annotations`
- `socket_path`
- `websocket_host`
- `websocket_port`

## Running v1

Install the SDK in your BallAlgo Python environment:

```bash
pip install -r foxglove_sim/requirements.txt
```

Start the sidecar on the Pi from the `BallAlgo` root:

```bash
python3 foxglove_sim/sidecar.py
```

Then run the normal `ballalgo` runtime.

On your laptop, open Foxglove and connect to:

```text
ws://<pi-ip>:8765
```

## MCAP Replay

When `record_mcap = true`, recordings are written under the configured
`record_dir` with a timestamp-based session label.

Open those `.mcap` files directly in Foxglove for replay.

## Testing Without The Pi

You can exercise the full Foxglove sidecar locally with deterministic fake data
that uses the **same Unix socket payload format and framing** as the real `BallAlgo`
runtime.

### Fake runtime only

If the sidecar is already running, publish fake telemetry into it:

```bash
python3 foxglove_sim/fake_runtime.py --scenario replan_demo
```

Useful scenarios:

- `replan_demo`: moving robot, moving ball, curved planner path
- `orbit_ball`: robot circles the ball with a strike target
- `straight_line`: simpler path + motion for panel setup/debugging
- `idle`: static robot/ball snapshot

### One-command local demo

This launches the sidecar and the fake runtime together:

```bash
python3 foxglove_sim/run_fake_foxglove_demo.py --scenario replan_demo
```

If your default `python3` does not have `foxglove-sdk` installed, point the
launcher at a different interpreter for the sidecar:

```bash
python3 foxglove_sim/run_fake_foxglove_demo.py \
  --scenario replan_demo \
  --sidecar-python /path/to/python-with-foxglove-sdk
```

Then connect Foxglove to:

```text
ws://127.0.0.1:8765
```

## Recommended 2D Field View

For the field panel, use a **3D** panel switched into **2D mode**, and add only:

- `/field/scene/static`
- `/field/scene/live`
- `/planner/scene/path`

Do **not** rely on `/robot/pose` or `/ball/pose` for the main field panel if you
want a clean soccer-style top-down view. Those raw pose topics are still useful
for Raw Messages and debugging, but Foxglove may render them with a generic 3D
pose marker that is not what we want for soccer.

## Folder Contents

- [foxglove.conf](./foxglove.conf): shared runtime + sidecar config
- [config.py](./config.py): sidecar config parser
- [schema_catalog.py](./schema_catalog.py): JSON schema catalog for custom topics
- [sidecar.py](./sidecar.py): Foxglove WebSocket + MCAP sidecar
- [topic_catalog.py](./topic_catalog.py): canonical topic/schema map
- [requirements.txt](./requirements.txt): Python SDK dependency list

## Practical Recommendation

For your use case, the right default is:

1. real-time Foxglove WebSocket server on the Pi over LAN
2. simultaneous MCAP recording on the Pi
3. Foxglove desktop app for live and replay debugging

That avoids depending on cloud/API ingest during normal development and keeps
latency much lower than routing everything through remote access.

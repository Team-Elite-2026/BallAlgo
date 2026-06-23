# LiDAR Pose-Track Test (drive a square, record + plot LiDAR position & velocity)

This test drives the robot in a closed square via a pre-generated trajectory while
the **real C++ `LidarLocalizer`** (with deskewing) estimates the pose every loop.
The pose stream is recorded to an MCAP file by the Foxglove sidecar, then plotted as:

- a **time-gradient position track** on the field plane, and
- **LiDAR velocity vs time** graphs for the field-frame `vx` and `vy`.

It also exercises the temporary `kDeskewVelocityFromLidar` mode, where the deskewer's
motion estimate comes from the LiDAR-driven pose Kalman filter instead of the mouse
sensor.

---

## What you need

- The robot on the field (LiDAR sees the walls; the Teensy executes the chunks).
- These config values in `src/config.hpp` (rebuild after changing either):
  - `kLidarYawOffsetDeg = 180.f`
  - `kDeskewVelocityFromLidar = true`  *(temporary; set back to `false` to restore mouse-based deskew)*
- Python deps for the plotter (one-time): `pip install --break-system-packages mcap mcap-protobuf-support protobuf`

> **Does Foxglove Studio need to be open/connected?** No. The sidecar writes the
> MCAP file on its own as soon as it starts. You only need the **sidecar process**
> running (the `--with-sidecar` flag below starts it); a connected Studio GUI is
> optional. No CSV is involved — all data is read back from the MCAP.

---

## Step 1 — Build

```bash
cmake --build build --target ballalgo -j4
```

(If the build dir doesn't exist yet:
`cmake -S . -B build && cmake --build build --target ballalgo -j4`)

## Step 2 — Generate the square trajectory

```bash
python3 tools/trajectory_debug/generate_cases.py \
    --spec tests/trajectory_cases/specs/rectangle_loop.json
```

This writes `tests/trajectory_cases/generated/rectangle_loop.traj` — a 600 mm × 600 mm
CCW square at 0.30 m/s with fixed heading (pure translation), starting at field
`(600, 900)` mm. Total drive time ≈ 8 s.

## Step 3 — Drive it on the robot, recording to MCAP

```bash
python3 tools/trajectory_debug/run_replay.py rectangle_loop --with-sidecar
```

- `--with-sidecar` launches `foxglove_sim/sidecar.py`, which records `/robot/pose`
  and `/robot/twist` (among others) into `foxglove_sim/recordings/<label>.mcap`.
- The runtime runs in `--trajectory-replay-artifact` mode: the Teensy executes the
  square open-loop while the Pi keeps estimating pose from LiDAR.
- Let it run the full ~8 s, then stop with `Ctrl+C`.

## Step 4 — Plot position track + velocity graphs

```bash
python3 tools/trajectory_debug/plot_pose_track.py \
    --artifact tests/trajectory_cases/generated/rectangle_loop.traj
```

With no MCAP path it auto-picks the newest recording. Outputs two SVGs next to the
recording:

- `…​.pose_track.svg` — position track, colored by elapsed time (viridis: dark = start,
  yellow = end), with a circle = start, square = end, time colorbar, and the dashed
  commanded square overlaid.
- `…​.pose_track.velocity.svg` — two stacked panels: field-frame `vx(t)` and `vy(t)`
  in m/s. For the square you should see the clean per-side signature
  (`+vx → +vy → −vx → −vy`).

To target a specific recording or rename the output:

```bash
python3 tools/trajectory_debug/plot_pose_track.py \
    foxglove_sim/recordings/<label>.mcap \
    --artifact tests/trajectory_cases/generated/rectangle_loop.traj \
    --output /tmp/square_run.svg
```

Useful flags: `--no-velocity` (skip the velocity graph), `--topic` / `--velocity-topic`
(override channel names, default `/robot/pose` and `/robot/twist`).

---

## Notes

- **Why the velocity is "LiDAR velocity":** with `kDeskewVelocityFromLidar = true`,
  the pose Kalman filter is driven purely by LiDAR (no mouse fusion), so its velocity
  estimate — published as `vx_field_m_s` / `vy_field_m_s` on `/robot/twist` — reflects
  LiDAR only.
- **Body vs field:** the square is heading-fixed at 0°, so field `vx`/`vy` equal the
  robot's right/forward velocities for this test.
- **Deskew only acts while moving.** At rest, velocity ≈ 0 and deskew is a no-op, so
  any at-rest jitter is handled by the pose smoothing filter, not the deskewer.
- **Revert when done:** set `kDeskewVelocityFromLidar = false` in `src/config.hpp` and
  rebuild to restore normal mouse-based dead-reckoning + deskew.
```

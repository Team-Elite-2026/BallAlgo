# BallAlgo + Offense2026 - Current Code Review Findings

**Date:** 2026-06-16
**Active branches reviewed:** `BallAlgo` = `Trajectory-Planning-Testing`, `Offense2026` = `FinalDraft1`
**Scope:** active Pi runtime, active Teensy runtime, Pi<->Teensy protocol, team link, build/test health

## Method

- Read `BallAlgo/.cursor/rules/Project-Context.mdc` and the active runtime sources in `BallAlgo/src/` and `Offense2026/src/`
- Ran a tests-only BallAlgo CMake build with `-DBALLALGO_BUILD_APP=OFF`
- Ran `ctest` against the current BallAlgo unit tests
- Built small local probes against the current `ballalgo_motion_core` library to verify offense/defense geometry behavior
- Did not run a full Offense2026 firmware build because `pio/platformio` is not installed in this review environment
- Did not treat the local `libcamera` package absence as a source bug; it only blocked the full BallAlgo app configure on this machine

## Executive Summary

Several previously documented integration issues are now genuinely fixed: the legacy ASCII perception path is gone, the Pi/Teensy binary telemetry contract matches, the TeamLink idle-poll teardown bug is fixed, the Pi/Teensy feedforward constants now agree, and the recent motion fixes closed the checked-in Teensy compile error plus the offense/defense heading and geometry regressions.

The biggest current blockers are:

1. Pose validity still depends on LiDAR; the bearing-only camera update cannot initialize or keep the pose alive
2. The Teensy kicker deassert path is still broken even though the Pi now transmits kick/dribbler bytes correctly
3. Ball velocity semantics are still inconsistent across filtering, team fusion, offense, and defense
4. Line avoidance is still invisible to the Pi planner
5. Blocking UART/RFCOMM writes can still freeze the single-threaded Pi loop
6. The Teensy still sends raw IMU heading to the Pi without applying calibration zero
7. LD19 intake can still fall behind and age the localization scan stream
8. Restarting the Pi can still lock out future chunks until the Teensy is reset

The BallAlgo motion/team tests currently pass, and there is now direct coverage for offense/defense heading and shallow-angle defense geometry. There are still no tests covering chunk timing, line-avoidance integration, or Pi<->Teensy protocol edge cases. Most of the high-severity issues below still sit in untested paths.

## Resolved Since The Previous Findings Pass

- `BallAlgo/src/planner/PlannerCaseMain.cpp` is back; the old BallAlgo source-level CMake break is gone
- The Pi->Teensy link is now a single binary framed protocol; the legacy ASCII perception path is removed
- `TeensyTelemetryPayload` matches on both sides and carries heading, mouse velocity, omega, start/goal state, mode override, and latency
- `TeamLink::readPeer()` no longer drops the Bluetooth link just because a poll tick had no complete frame
- Pi and Teensy now share the same feedforward constants: `kMotorKs = 0.119`, `kMotorKv = 0.139`, `kMotorKa = 0.00074`
- `Offense2026/src/TrajectoryExecutor.h` now compiles; the missing semicolon is fixed
- Offense goal-line handling now follows the field Y axis and selects the correct near-goal state
- Offense and defense terminal heading helpers now use the project convention `0 deg = +y/front`, `90 deg = +x/right`
- Defense shallow-angle geometry now reaches the 55 cm side segment instead of collapsing inward to 40 cm
- Pi action chunks now serialize real `kick` and `dribblerPower` bytes instead of always zero-filling them

## Critical

- No confirmed critical source-level blockers remain in the reviewed trees after the current fixes.

## High

### H1. Ball velocity semantics are still inconsistent across filtering, team fusion, offense, and defense

- **Subsystem:** BallAlgo estimation and strategy handoff
- **Files:** `BallAlgo/src/estimation/BallKalman.cpp`, `BallAlgo/src/main.cpp`, `BallAlgo/src/motion/DefensePose.cpp`
- **Problem:**
  - The ball KF still models motion in the robot body frame, even though that frame is rotating and translating with the robot
  - `makeFusedBodyBall()` rotates fused field velocity into body axes without adding robot velocity
  - `fieldBallFromBody()` converts back to field velocity by adding robot velocity
- **Robot behavior:** defense can add the robot's own velocity back into an already field-relative ball estimate, so moving-robot intercept and pacing decisions are computed from the wrong ball motion.

### H2. Pose validity still depends on LiDAR; camera goal bearings cannot initialize or sustain localization

- **Subsystem:** BallAlgo pose estimation
- **Files:** `BallAlgo/src/estimation/PoseKalman.cpp`
- **Problem:** only `update()` from LiDAR sets `init_` and refreshes `ageSinceMeasurementS_`. `updateGoalBearing()` requires `init_` and never refreshes staleness on its own.
- **Robot behavior:**
  - with LiDAR disabled or disconnected, pose never becomes valid
  - during a LiDAR dropout, pose can go stale even while both goals are visible
  - defense planning falls back to stop chunks because `debugPlanDefense()` exits early on invalid pose

### H3. The Teensy kicker deassert path can still latch the solenoid high

- **Subsystem:** Pi->Teensy actuation contract
- **Files:** `Offense2026/src/Movement.cpp`
- **Problem:**
  - `Movement::kickBackground()` only deasserts the kicker if `active > 2`, but `kick()` sets `active = 0`, so a kick can leave the solenoid driven longer than intended
- **Robot behavior:** the Pi can now request kick/dribbler actions, but the Teensy-side shutoff path is still unsafe and can hold the kicker on longer than intended.

### H4. Line avoidance is still invisible to the Pi planner

- **Subsystem:** Pi/Teensy safety arbitration
- **Files:** `BallAlgo/src/motion/Protocol.hpp`, `BallAlgo/src/main.cpp`, `Offense2026/src/main.cpp`
- **Problem:** line detection remains a Teensy-only override. The Pi keeps planning toward the same out-of-bounds target because there is still no line-state field in telemetry and no boundary model in the planner.
- **Robot behavior:** at the field edge the robot can get stuck in a lunge-brake-lunge cycle: the Teensy stops it on the line, the line clears briefly, then the next Pi chunk sends it back into the same edge.

### H5. Blocking UART/RFCOMM writes can still freeze the single-threaded Pi loop

- **Subsystem:** BallAlgo IO
- **Files:** `BallAlgo/src/io/RobotSerial.cpp`, `BallAlgo/src/team/TeamLink.cpp`
- **Problem:** `RobotSerial::writeAll()` spins until the UART drains, and `TeamLink::sendFrame()` spins on `EAGAIN/EWOULDBLOCK` with no timeout. Both run in the same single-threaded process that owns camera capture, estimation, planning, and clock sync.
- **Robot behavior:** a stalled UART or Bluetooth socket can freeze the whole robot process mid-match.

### H6. The Teensy still sends raw IMU heading to the Pi; calibration zero is not applied on the planning/execution path

- **Subsystem:** heading pipeline
- **Files:** `Offense2026/src/TrajectoryExecutor.cpp`, `Offense2026/src/CompassSensor.cpp`, `Offense2026/src/Calibration.cpp`
- **Problem:** the calibration flow stores `zeroedAngle`, but telemetry and chunk execution still use `imu.getOrientation()` directly.
- **Robot behavior:** if the team expects the calibration action to define field zero, the Pi never sees that adjusted heading. Localization, goal constants, and chunk rotation can all be offset by a constant heading bias.

### H7. LD19 intake still reads only one 512-byte chunk per camera loop

- **Subsystem:** BallAlgo LiDAR IO
- **Files:** `BallAlgo/src/lidar/Ld19Reader.cpp`
- **Problem:** `pollPoints()` performs only one nonblocking `read()` of up to 512 bytes per camera loop. The sensor stream is fast enough that any slow vision loop can fall behind permanently.
- **Robot behavior:** when the loop rate drops, LiDAR data ages in the kernel buffer, scans mix revolutions, and deskew/localization timing drifts away from reality.

### H8. Restarting the Pi can still lock out all future chunks until the Teensy is reset

- **Subsystem:** chunk protocol
- **Files:** `BallAlgo/src/motion/MotionPlanner.cpp`, `Offense2026/src/TrajectoryExecutor.cpp`
- **Problem:** the Pi restarts trajectory IDs from 1 on process start, while the Teensy rejects any chunk whose `trajectory_id <= active_chunk.trajectory_id`.
- **Robot behavior:** if the Pi process restarts mid-session, the robot can ignore all subsequent chunks until the Teensy is power-cycled or otherwise reset.

## Medium

### M1. Clock sync still depends on 32-bit `micros()` and treats exact-zero offset as "unlocked"

- **Subsystem:** Teensy chunk timing
- **Files:** `Offense2026/src/TrajectoryExecutor.cpp`
- **Problem:** `clock_offset_us == 0` is still used as a "not locked" sentinel, and all timing math is still based on 32-bit `micros()` values widened to 64-bit after the fact.
- **Robot behavior:** a legitimate zero offset is misclassified, and long uptimes can still break queued-chunk start timing and action indexing after `micros()` wraps.

### M2. A camera grab failure still starves serial, LiDAR, team link, and chunk publishing for that loop

- **Subsystem:** BallAlgo main loop sequencing
- **Files:** `BallAlgo/src/main.cpp`
- **Problem:** on `camera.grab()` failure, the loop immediately `continue`s before serial polling, predict/update, LiDAR intake, team link, clock sync, or chunk publishing.
- **Robot behavior:** a camera hiccup can momentarily silence the rest of the robot stack and let the Teensy fall into its grace-brake path.

### M3. The Pi frame parser still trusts arbitrary payload lengths

- **Subsystem:** BallAlgo protocol parsing
- **Files:** `BallAlgo/src/motion/Protocol.cpp`
- **Problem:** `unpackFrames()` still accepts any 16-bit payload length. A corrupted `plen` can force the parser to wait for tens of kilobytes before CRC resync.
- **Robot behavior:** one corrupted length field can stall telemetry and ping handling long enough to disturb control timing.

### M4. Hot-path serial logging is still heavy enough to distort control timing

- **Subsystem:** Teensy loop timing
- **Files:** `Offense2026/src/Switches.cpp`, `Offense2026/src/Movement.cpp`
- **Problem:**
  - `Switch::lightgate()` prints every time telemetry is sent, which is 100 Hz in `TrajectoryExecutor::processSerial()`
  - `Movement::movement()` and the correction helpers still print multiple `String`-built messages during line avoidance
- **Robot behavior:** USB serial logging still adds avoidable jitter exactly in safety-critical paths.

### M5. `Movement` deadband logic is still broken in the active line-avoidance path

- **Subsystem:** Teensy line-avoidance fallback motion
- **Files:** `Offense2026/src/Movement.cpp`
- **Problem:** both `findCorrectionRelZero()` and `findCorrectionRelOffset()` set `correction = 0` inside a deadband and then immediately continue into the next `if` chain because the `else` is missing.
- **Robot behavior:** the deadband never actually holds, so line-avoidance heading correction jitters instead of settling cleanly.

### M6. Planner timing still mixes incompatible speed ceilings

- **Subsystem:** BallAlgo planning math
- **Files:** `BallAlgo/src/motion/AStar3D.cpp`, `BallAlgo/src/motion/MotionLimits.hpp`, `BallAlgo/src/motion/MotionPlanner.cpp`, `BallAlgo/src/config.hpp`
- **Problem:** Pi and Teensy feedforward constants now match, but the planner still mixes:
  - `wheelProjVMax()` from motor no-load RPM for A* cost/heuristics
  - `kVMaxX/kVMaxY = 0.8/0.6 m/s` for offense intercept and defense terminal-velocity logic
- **Robot behavior:** different parts of the planner still estimate path time for different robots.

### M7. Ball distance units are still ambiguous across the Pi/Teensy boundary

- **Subsystem:** vision distance calibration
- **Files:** `BallAlgo/src/vision/VisionMath.cpp`, `BallAlgo/src/config.hpp`
- **Problem:** `calibrateBallDist()` feeds Pi code that treats the value as millimeters via `kBallDistToM = 0.001`, while earlier Teensy-side logic and thresholds were historically centimeter-based.
- **Robot behavior:** if the team assumes the wrong unit, the Ball KF and any distance thresholds can be off by 10x.

### M8. `kCameraFps = 120` is still not enforced in camera configuration

- **Subsystem:** BallAlgo camera timing assumptions
- **Files:** `BallAlgo/src/config.hpp`, `BallAlgo/src/camera/CameraCapture.cpp`
- **Problem:** the motion/ball prediction code assumes 120 FPS, but `CameraCapture` still does not explicitly force a 120 FPS sensor mode.
- **Robot behavior:** prediction horizons and any px/frame heuristics drift if the negotiated camera mode is not actually 120 FPS.

### M9. `AStar3D` still reports success with a straight path through the obstacle when the goal is unreachable

- **Subsystem:** BallAlgo path planning fallback
- **Files:** `BallAlgo/src/motion/AStar3D.cpp`
- **Problem:** when the goal node is unreachable, `plan()` still returns `true` and emits a two-point straight path from start to goal.
- **Robot behavior:** callers cannot distinguish failure from success, so the spline/profiler can still drive straight through the inflated ball obstacle.

### M10. When action-chunk publishing is disabled, the Pi still stops draining pending protocol frames

- **Subsystem:** BallAlgo chunk publisher / clock sync
- **Files:** `BallAlgo/src/motion/ActionChunkPublisher.cpp`
- **Problem:** the early return before `takePendingFrames()` / `clock_.processFrames()` is still present when `kEnableActionChunks` is false.
- **Robot behavior:** "telemetry only" mode can still accumulate protocol frames and silently fail to maintain clock sync.

## Low / Cleanup

- `Defense defense;` in `Offense2026/src/main.cpp` is constructed but never used
- `BallAlgo/src/io/RobotSerial.cpp::pollHeading()` is dead code
- `config::kEnablePlannerCompare` is dead code
- `ProfileSample.phi` is still read by `profileDurationS()` but never written
- `Offense2026/src/Switches.cpp` still uses `pinMode(INPUT)` plus `digitalWrite(HIGH)` instead of `INPUT_PULLUP`
- `Offense2026/src/Bluetooth.cpp` and `.h` are still commented-out dead code

## Verification Notes

- BallAlgo tests-only build: passed
- BallAlgo `ctest`: passed (`ballalgo_team_tests`, `ballalgo_motion_tests`, `ballalgo_protocol_tests`)
- Full BallAlgo app configure: blocked locally by missing `libcamera` package on this machine, not by a source-level BallAlgo compile failure
- Direct motion tests now confirm:
  - a straight-ahead offense strike requests `heading=0`
  - an enemy goal-mouth ball enters the near-enemy-goal-line offense state
  - a straight-ahead defense block requests `heading=0`
  - shallow-angle defense geometry returns the `55 cm` side-line target
- Direct protocol tests now confirm:
  - Pi-packed action chunks carry non-zero `kick` and `dribblerPower` bytes on the wire

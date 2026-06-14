# BallAlgo + Offense2026 — Deep Adversarial Code Review

**Date:** 2026-06-11
**Scope:** `BallAlgo/` (Pi, full read of src/tests/build), `Offense2026/` (Teensy, full read of two branches), Pi↔Teensy integration, build/config/docs.
**Method:** full static read of every non-vendored source file, cross-referencing both sides of every protocol, an actual CMake configure + unit-test run, and a compiled numerical probe harness run against the real BallAlgo motion-core sources to verify suspected math bugs empirically.

> **Important caveat on branches:** the `Offense2026` working tree **changed branches mid-review** (git reflog: `lineControl` → `NewPathAlgoTest`). Findings are labeled:
> - **[LC]** = `lineControl` branch (commit `7bc4526`): the WorldModel / OffensePlanner / DefensePlanner / MotionProtocol / MotionExecutor / DriveArbiter stack.
> - **[NPA]** = `NewPathAlgoTest` branch (commit `06ac381`, currently checked out): the TrajectoryExecutor / LinePCBComm / LcdController stack.
>
> These two firmwares have **materially different integration contracts with BallAlgo**, and each is broken in different ways. Deciding which branch is canonical is itself a top-priority action item.

> **Team-provided hardware evidence honored:** ball angle / goal angle calculations and the LiDAR pose are reported as validated on hardware. Findings touching those paths are framed as *internal convention inconsistencies whose risk falls on the other consumers of the same data*, not as claims that the validated outputs are wrong.

---

## Executive Summary

**Overall health: the individual subsystems are ambitiously engineered (EKF + LiDAR Hough localizer + 3D A* + S-curve profiler + action-chunk streaming), but the system as a whole has never been provably wired together.** The single biggest theme is that the Pi and the Teensy disagree about almost every contract between them: which serial port, which message types, which frames/conventions, which motor model, which field dimensions, and who handles kick/dribble. A secondary theme is that the new motion pipeline's math has several verifiable internal bugs (A* heading bins, defense goal-line geometry, ball-filter frame semantics) that unit tests would have caught — and there are no unit tests for any of the math.

**Verified empirically during this review:**
- The default BallAlgo build **fails at `cmake` configure** (missing `src/planner/PlannerCaseMain.cpp`).
- The A* planner assigns heading bins with the wrong angle convention (probe output shows a straight +x path forced to heading 0°=+y with spurious turn-in-place moves at both ends).
- The defense goal-line solver puts the blocking pose at the wrong point for shallow ball angles (probe: ball level with goal → target (40, 0) cm, inside the keep-out arc, instead of (55, 0)).
- The offense state machine classifies a ball at the attack-goal mouth as "ball at boundary" because its goal-line logic uses the X axis while the configured goals sit on the Y axis.
- The Pi's motor model believes the robot can do 3.6–5.2 m/s; the Teensy's models (both branches) max out far lower; planned trajectories are physically unexecutable as timed.
- The 11 team-coordination unit tests compile clean and pass — they are the only tests in either repo.

**Biggest risk areas, in order:**
1. **Pi↔Teensy serial contract** — on [NPA] the camera parser listens on the wrong UART entirely; on [LC] the Teensy never sends the binary telemetry the Pi requires. In neither configuration does the full documented pipeline (telemetry → EKF → chunks → executor) close the loop.
2. **Robot-to-robot Bluetooth** — a connection-teardown bug makes the link flap on every idle poll; team play is effectively dead code in production.
3. **Motion math** — A* heading bins, frame-rotation handedness between Pi and Teensy, ball-velocity frame double-counting, defense geometry.
4. **Safety gaps** — [LC] motion ISR drives motors regardless of the start switch and races the main loop; kicker pin can latch HIGH [NPA]; NaN propagation from line PCA to motor PWM [LC].

**Confidence:** High on everything labeled "verified" (build, probe outputs, tests). High on protocol mismatches (byte-level cross-reading of both sides). Medium on convention/handedness findings that interact with physical mounting (flagged for a single decisive cross-repo test rather than asserted as broken). FoxGlove sim internals were deliberately deprioritized per instruction.

---

## CRITICAL

> **Canonical branch update (2026-06-14):** Offense2026 now lives on the merged
> **`FinalDraft1`** branch (the [LC] `MotionExecutor`/`MotionProtocol`/`DriveArbiter`
> stack no longer exists; the [NPA] `TrajectoryExecutor` stack is the one true
> motion path). All findings below were re-verified against `FinalDraft1`. Resolved
> findings **C1, C2, C5, C6, C9, C10** were removed.

### C3. Cam ASCII perception parser — RESOLVED (perception path removed; Pi plans both roles)
- **Subsystem:** Pi→Teensy perception protocol
- **Status:** **RESOLVED on FinalDraft1 via architecture change.** The team chose "Pi plans everything": the Pi runs the camera/vision and plans **both** offense and defense, streaming ready-to-execute binary action chunks; the Teensy is role-agnostic and only executes chunks. Consequences applied:
  - Teensy-side `Cam.cpp`/`Cam.h` (the ASCII serial *parser* — not the camera or any vision math, which live on the Pi) **deleted**. `Orbit`/dampen **deleted**. `runOffense`/`runDefense` collapsed into one `runRobot()` that executes chunks + line safety.
  - Only `TrajectoryExecutor::processSerial()` reads `Serial3` now — single owner, so the former two-reader bus race is gone by construction.
  - Pi ASCII path removed: `formatPerception`, the send block, and `kEnableLegacyAsciiPerception` deleted. The Pi→Teensy link is now purely the binary framed protocol (chunk `0x03`, telemetry `0x04`, ping/pong).
- **Net:** former C2 (bus contention) and C3 (ASCII parser bugs) no longer exist — the offending code is gone. Role switching is now a pure Pi-side decision (LCD selection → telemetry `modeOverride` → `RoleArbiter` → offense/defense chunks).
- **Follow-up (minor):** the now-dead Teensy `Defense` class is still constructed in `main.cpp` but never used — safe to delete in a later cleanup.

### C4. Teensy→Pi telemetry contract — RESOLVED
- **Subsystem:** Pi↔Teensy telemetry
- **Status:** **RESOLVED on FinalDraft1.** `TrajectoryExecutor::sendTelemetry()` now sends the full 24-byte `TeensyTelemetryPayload` (heading, mouse vx/vy, omega, hasBall, startEnabled, goalIsBlue, modeOverride, serialLatencyUs) as a framed `0x04` packet on the Pi link (`Serial3`). The Pi's `TeensyTelemetryPayload` ([Protocol.hpp:24-38](BallAlgo/src/motion/Protocol.hpp#L24-L38)) is byte-identical (`static_assert(sizeof == 24)`), `parseTeensyTelemetry` takes the full-parse branch, and `RobotSerial::handleFrame` ([RobotSerial.cpp:104-129](BallAlgo/src/io/RobotSerial.cpp#L104-L129)) populates `goalIsBlue`/`startEnabled`/`modeOverride` from it. `setMatchState()` is fed the live LCD switch state each loop. The 20-byte legacy path still exists only as a backward-compat fallback in `parseTeensyTelemetry`. The [LC] half of this finding is moot (that firmware no longer exists).

### C7. [LC] Motion executor ISR races/no rotation/no start gate — MOOT
- **Subsystem:** Teensy motion execution
- **Status:** **MOOT on FinalDraft1.** The entire [LC] motion stack (`MotionExecutor.cpp`, `MotionProtocol.cpp`, `DriveArbiter.cpp`, `MotionConfig.h`) does not exist on this branch. The canonical executor is `TrajectoryExecutor`, which rotates global→body in `execute()` (now with the correct transpose — see former C10), applies a 20 ms grace hold, and is gated behind `lcdController.isStartEnabled()` in `runOffense()` before `execute()` runs. No ISR/main-loop dual-writer exists. Keep this entry only as a reminder not to resurrect the [LC] stack.

### C8. Pi and Teensy disagree on the motor model — OPEN (needs bench characterization)
- **Subsystem:** motion planning ↔ execution
- **Status:** **STILL OPEN — cannot be fixed in code without hardware measurement.** Updated facts on FinalDraft1:
  - Pi: 12 V bus, kS 0.5, kV 0.08, kA 0.02 ([config.hpp:87-89](BallAlgo/src/config.hpp#L87-L89)); `kVMaxX/kVMaxY = 0.8/0.6 m/s` ([config.hpp:151-152](BallAlgo/src/config.hpp#L151-L152)) coexist with a `wheelProjVMax` motor model that implies ~3.6–5.2 m/s — these two speed ceilings are used by different code paths and disagree by ~6×.
  - Teensy: kS 0.119, **kV 5.35 V·s/rad**, kA 0.17 ([TrajectoryExecutor.cpp:24-26](Offense2026/src/TrajectoryExecutor.cpp#L24-L26)). `kV 5.35` is ~67× the Pi's `kV 0.08`. (Note: `readBatteryVoltage()` now reads the real ADC — the hardcoded-12V sub-issue is fixed.)
- **Why not auto-fixed:** the correct constants are physical properties of the drivetrain (kS/kV/kA, true vMax). Picking numbers without a bench characterization would just replace one wrong set with another. **Action required:** bench-characterize kS/kV/kA and vMax once, then assert the *same* values on both sides (ideally a single shared generated header), and make the Pi's planner ceiling (`wheelProjVMax`) and `kVMaxX/Y` consistent with each other.

---

## HIGH

### H1. Defense goal-line solver wrong for shallow ball angles (verified)
- **Subsystem:** BallAlgo defense math
- **Files:** [BallAlgo/src/motion/DefensePose.cpp:45-67](BallAlgo/src/motion/DefensePose.cpp#L45-L67); spec: [BallAlgo/docs/pipeline_context.md](BallAlgo/docs/pipeline_context.md) (Defense Step 1.2)
- **Problem:** the spec's side-line branch is `m <= 5/11`; the code uses `m <= -5/11`. The boundary geometry (top line y=40, side x=55, corner arc center (40,25) r=15 — from C=2000) tangency points are at m=1 and m=+5/11. For `m ∈ (−5/11, +5/11)` the code falls into the arc quadratic whose discriminant is negative there; it clamps `disc` to 0 and emits x=40·…, y=m·x — a point **inside** the arc, not on the boundary.
- **Evidence (probe):** ball at goal + (800, 0) mm → target (40.0, 0.0) cm rel. goal. Correct side-line answer: (55, 0).
- **Robot behavior:** exactly when the ball swings wide and level with the goal — a prime scoring approach — the keeper stations ~15 cm too central/deep (possibly inside the penalty area) and the intercept-velocity override geometry (`goalLineTangentUnitFromTarget`) is computed for the wrong segment.
- **Verify:** probe case; also sweep m from −1 to 1 and plot targets — discontinuity at ±5/11.

### H2. Offense goal-line states use the X axis; the configured goals are on the Y axis (verified)
- **Subsystem:** BallAlgo offense pose
- **Files:** [BallAlgo/src/motion/OffensePose.cpp:91-105,126-195](BallAlgo/src/motion/OffensePose.cpp#L91-L105), [BallAlgo/src/config.hpp:75-78,122-126](BallAlgo/src/config.hpp#L75-L78)
- **Problem:** `kBlueGoalXMm = W/2, kBlueGoalYMm = 0` / yellow at `y = H` (goals on the short walls, consistent with the 1820×2430 field) — but `classifyOffensePoseState`, `CollectBallNearEnemyGoalLine`, `CollectBallBehindRobotNearOurGoalLine`, and `setForwardSafeStrikeTarget` all treat **X** as the attack axis (`kOffenseEnemyGoalLineXMm = 1680` on an axis only 1820 long where the goal sits at x=910). The config comment even says "Assumes yellow-goal attack runs in the +X field direction", contradicting the goal constants 50 lines above.
- **Evidence (probe):** ball placed at the yellow-goal mouth (910, 2330) → state `collect_ball_at_boundary`, target (910, 2240), heading 90° — sideways "boundary retrieval" behavior at the moment it should be a normal strike; the real "near enemy goal line" recovery (back out, face the ball) can never trigger where it's needed.
- **Robot behavior:** wrong special-case behavior near both goals; "forward safe" targets offset along the wrong axis push the strike pose sideways.
- **Verify:** probe case; sweep ball y from 100→2300 at x=910 and log `offensePoseStateName`.

### H3. Ball velocity frame semantics are inconsistent across producer/consumers (double-counted robot velocity)
- **Subsystem:** BallAlgo estimation ↔ strategy
- **Files:** [BallAlgo/src/estimation/BallKalman.cpp](BallAlgo/src/estimation/BallKalman.cpp) (filters in **body** frame), [BallAlgo/src/main.cpp:76-96](BallAlgo/src/main.cpp#L76-L96) (`makeFusedBodyBall`: body vel = pure rotation of field vel), [BallAlgo/src/motion/DefensePose.cpp:34-43](BallAlgo/src/motion/DefensePose.cpp#L34-L43) (`fieldBallFromBody`: field vel = rotation **+ robot velocity**)
- **Problem:**
  1. The camera ball KF runs a constant-velocity model in the **rotating body frame** (spec says filter in absolute field coordinates). While the robot translates/rotates, a stationary ball acquires phantom body-frame velocity and the CV prediction is wrong in that frame (no ω×r term anywhere).
  2. The two conversion helpers disagree about what `BallState.vx` means: `makeFusedBodyBall` outputs field-relative velocity merely rotated into body axes; `fieldBallFromBody` then converts back by **adding** robot velocity. Round-tripping the fused ball through defense adds the robot's own velocity to the ball once. Offense intercept (`predictBallBody`) treats the same field as body-apparent velocity. At most one of these three interpretations can be right.
- **Robot behavior:** moving-robot intercept and defense pacing (`ω_ray`, `V_scalar`, `ω_target`) computed from corrupted ball velocity; worst exactly during fast play.
- **Verify:** sim a stationary ball + robot translating at 0.5 m/s; log `fusedFieldBall.vMmS` and `DefensePoseResult.futureBallV*` — both should be ≈0.

### H4. Pi restart permanently locks out new trajectories (both branches)
- **Subsystem:** chunk protocol
- **Files:** [LC] [Offense2026/src/MotionProtocol.cpp:174-181](Offense2026/src/MotionProtocol.cpp#L174-L181); [NPA] [Offense2026/src/TrajectoryExecutor.cpp:160-164](Offense2026/src/TrajectoryExecutor.cpp#L160-L164); Pi id source: [BallAlgo/src/motion/MotionPlanner.cpp:178](BallAlgo/src/motion/MotionPlanner.cpp#L178)
- **Problem:** the Teensy drops any chunk with `trajectory_id <= active_id`. The Pi's id counter starts at 1 on process start. Restart the Pi mid-session (crash, ssh kill, watchdog) and every new chunk is "stale" until it exceeds the old session's counter (could be thousands). The docs' `is_first_chunk_of_match` reset exists only as a boot-time flag, not a session-reset mechanism, and there is no timeout-based acceptance.
- **Robot behavior:** after a Pi restart the robot ignores all motion commands until Teensy power-cycle — in a match this is a dead robot that *looks* connected (pings/pongs still flow).
- **Verify:** bench: stream chunks, restart the Pi process, observe executor stay idle.

### H5. Clock sync precision/wrap problems
- **Subsystem:** chunk timing
- **Files:** [LC] [Offense2026/src/MotionProtocol.cpp:108-122,219](Offense2026/src/MotionProtocol.cpp#L108-L122) (offset EMA in **float**); [NPA] [Offense2026/src/TrajectoryExecutor.cpp:168-175,240](Offense2026/src/TrajectoryExecutor.cpp#L168-L175) (`clock_offset_us == 0` used as "not locked" sentinel; `micros()` is 32-bit)
- **Problem:** [LC] stores the Pi↔Teensy offset (≈ Pi `steady_clock` µs since boot — can be 10¹⁰–10¹¹) in a `float` EMA: 24-bit mantissa → 1–8 ms quantization at those magnitudes → action indices off by ±1–2 slots and start-time jitter. [NPA] fixed precision (int64) but (a) a legitimate offset of exactly 0 is misread as "unlocked, execute immediately", and (b) all execution math is anchored on 32-bit `micros()` cast to 64-bit — after 71.6 min of Teensy uptime the wrap breaks `t_now_us >= t_start_queued_us` and `elapsed_us` (practice sessions exceed this routinely).
- **Verify:** [LC] print `(int64)clockOffsetEma_` error vs an int64 recompute after the Pi has been up >1 day; [NPA] soak test past 72 minutes.

### H6. Kick/dribble are structurally impossible under Pi control; kicker pin can latch HIGH
- **Subsystem:** actuation / Pi strategy gap
- **Files:** [BallAlgo/src/motion/Protocol.cpp:77-79](BallAlgo/src/motion/Protocol.cpp#L77-L79) (`pl[28]=poseValid; off += 3` — bytes 29/30 left **zero**), [Offense2026/src/TrajectoryExecutor.h:29-41](Offense2026/src/TrajectoryExecutor.h#L29-L41) (claims Pi packs `kick`/`dribblerPower` there), [Offense2026/src/TrajectoryExecutor.cpp:249-251](Offense2026/src/TrajectoryExecutor.cpp#L249-L251) (applies them on every swap), [Offense2026/src/Movement.cpp:152-177](Offense2026/src/Movement.cpp#L152-L177)
- **Problem:** grep of all of BallAlgo: **no kick or dribbler logic exists on the Pi** — the wire bytes are zero-filled padding. [NPA] dutifully executes them: every chunk swap calls `setDribbler(0)` (dribbler permanently off, even if some other code turned it on) and `kick` never fires. Separately, `Movement::kick()` holds `kickerPin` HIGH while `timer <= kickHold` but the only turn-off path, `kickBackground()`, is gated on `active > 2` which is never set (`kick()` sets `active = 0`) — a single `kick()` call near a timer boundary can leave the solenoid energized indefinitely (coil burnout / battery drain).
- **Robot behavior:** under the new pipeline this robot cannot shoot or dribble — i.e., cannot score except by pushing. The [LC] branch keeps kick/dribble in the Teensy planner instead, which is a coherent design — one more reason the two branches' contracts must be reconciled deliberately.

### H7. Line override vs. action chunks: the Pi never learns about lines; the executor re-arms instantly
- **Subsystem:** safety arbitration
- **Files:** [LC] [Offense2026/src/main.cpp:202-230](Offense2026/src/main.cpp#L202-L230); [NPA] [Offense2026/src/main.cpp:150-170](Offense2026/src/main.cpp#L150-L170); Pi protocol: no line/boundary message type exists ([BallAlgo/src/motion/Protocol.hpp:12-15](BallAlgo/src/motion/Protocol.hpp#L12-L15))
- **Problem:** when the line sensors fire, the Teensy suppresses/brakes chunk execution for that iteration — but the Pi keeps publishing fresh chunks at 60 Hz toward the same out-of-bounds target (its A* has **no field-boundary or line model at all**; it happily plans to x=0). The instant the line clears the dead-band, the next chunk drives back into it: a lunge-brake limit cycle at the boundary. The telemetry payload has no field to report line state, so the Pi cannot re-plan away. ([NPA]'s ordering is better — line check before `execute()` — but the cycle is the same.)
- **Verify:** sim or bench at a wall with a goal target beyond it.

### H8. Pose validity requires LiDAR; goal-bearing fusion can neither initialize nor sustain the EKF
- **Subsystem:** BallAlgo estimation
- **Files:** [BallAlgo/src/estimation/PoseKalman.cpp:86-117,155-165](BallAlgo/src/estimation/PoseKalman.cpp#L86-L117)
- **Problem:** only `update()` (LiDAR) sets `init_` and resets `ageSinceMeasurementS_`. `updateGoalBearing()` requires `init_` and never refreshes the staleness clock; `predictMouse` only ages it. Consequences: (a) with LiDAR disabled/unplugged (`BALLALGO_ENABLE_LIDAR=OFF` is an advertised build option!) the pose is *never* valid → planner falls back to body-chase forever, defense/role logic dead; (b) during a LiDAR dropout >0.18 s the pose invalidates even while two goals are visible — the documented Step-1b "camera angles correct lateral drift" provides zero availability benefit.
- **Robot behavior:** a LiDAR cable/dirt failure silently downgrades the robot to camera-chase with no localization, including for defense (which then emits stop chunks — see `debugPlanDefense` early-out).
- **Verify:** run with lidar unplugged; `poseValid` never becomes 1 despite both goals visible.

### H9. Blocking serial/Bluetooth writes inside the single control loop
- **Subsystem:** BallAlgo io
- **Files:** [BallAlgo/src/io/RobotSerial.cpp:57-73](BallAlgo/src/io/RobotSerial.cpp#L57-L73) (`writeAll` spins with `usleep(100)` until the UART drains), [BallAlgo/src/team/TeamLink.cpp:310-329](BallAlgo/src/team/TeamLink.cpp#L310-L329) (`sendFrame` spins with **no** sleep on EAGAIN)
- **Problem:** the vision/EKF/planner loop is single-threaded. A wedged UART (cable, Teensy reboot with flow-control weirdness) or a stalled RFCOMM socket converts a write into an unbounded busy-wait — the robot freezes mid-match (camera grab also blocks forever, M16). There is no watchdog anywhere in the process.
- **Verify:** disconnect the Teensy RX line under load; the loop rate collapses.

### H10. [NPA] Telemetry heading is the raw BNO orientation; the zeroed/field calibration is never applied on the Pi path
- **Subsystem:** heading pipeline
- **Files:** [Offense2026/src/TrajectoryExecutor.cpp:185](Offense2026/src/TrajectoryExecutor.cpp#L185) (`p.headingDeg = imu.getOrientation()`), [Offense2026/src/CompassSensor.cpp:34-56](Offense2026/src/CompassSensor.cpp#L34-L56), [Offense2026/src/Calibration.cpp:19-23](Offense2026/src/Calibration.cpp#L19-L23)
- **Problem:** `getOrientation()` is the absolute BNO heading (also an **int** — 1° quantization). `zeroedAngle` (captured at calibration) is applied in `currentOffset()` for local control but **not** in the telemetry sent to the Pi, and `execute()` likewise rotates chunks by the raw orientation. The Pi's field frame (LiDAR walls, goal positions) assumes heading 0 = field +y. Unless the BNO happens to read 0 when facing the far goal, every Pi field computation and the chunk rotation carry a constant unknown rotation — and they must carry the *same* one to cancel, which nothing guarantees (Pi uses telemetry heading for planning; Teensy re-reads the IMU at execution: those agree, but the LiDAR-derived x/y and goal constants do not rotate with them).
- **Verify:** calibrate facing a corner, then command a +y chunk; measure actual travel vs wall axes.

### H11. LD19 intake is capped at 512 bytes per camera-loop iteration
- **Subsystem:** BallAlgo lidar io
- **Files:** [BallAlgo/src/lidar/Ld19Reader.cpp:147-171](BallAlgo/src/lidar/Ld19Reader.cpp#L147-L171)
- **Problem:** one `read()` of ≤512 bytes per `pollPoints()` call, called once per camera frame. LD19 streams ~21.6 kB/s (230400 baud). If the vision loop drops below ~45 Hz (heavy morphology + 10× dilate + planner + JPEG encode for Foxglove make this plausible), intake falls behind *permanently*: kernel buffer overflows, scans become a mix of old and new revolutions, and the deskewer's timestamps drift from reality. Should loop `read()` until EAGAIN.
- **Verify:** add a counter of bytes-read vs expected; artificially slow the loop to 30 Hz.

### H12. [LC] Line PCA can emit NaN move angles straight into motor PWM
- **Subsystem:** line detection / motor safety
- **Files:** [Offense2026/src/LineDetection.cpp:211-246](Offense2026/src/LineDetection.cpp#L211-L246), [Offense2026/src/Motor.cpp:19-35](Offense2026/src/Motor.cpp#L19-L35)
- **Problem:** for a perfectly horizontal detected line (principal direction y≈0 → `slope = 0`), `m_perp = -1.0/slope = -inf`, `y_intersect = m_perp * x_intersect = -inf·0 = NaN`, `angle = atan2(x, NaN) = NaN`. `angle != -5` → `lineDetected = true`, NaN avoidance angle flows through `DriveArbiter` → `Trig::Sin(NaN)` → `Motor::setSpeed(NaN)` → `analogWrite(255*NaN)` (Motor has no clamp and no NaN guard; the commented-out `constrain` was removed).
- **Robot behavior:** rare but catastrophic: undefined PWM on all four wheels exactly while standing on a line.
- **Verify:** unit-feed two sensor activations symmetric about the x-axis into `Calculate()`.

---

## MEDIUM

### M1. Ball distance unit ambiguity across the boundary (flag for one-time verification)
- `calibrateBallDist` ([VisionMath.cpp:16-19](BallAlgo/src/vision/VisionMath.cpp#L16-L19)) yields ~40 at 100 px; the Pi multiplies by `kBallDistToM = 0.001` (treats as **mm**) for the ball KF, while the Teensy treats the same number as **cm** (`kBallCloseDistCm = 28`, lightgate forces `ballDist = 12`). If the curve is cm-calibrated (consistent with Teensy thresholds), the Pi's ball KF positions are 10× too small; if mm, every Teensy distance threshold is wrong. Team reports angle/distance validated — so document the unit and fix `kBallDistToM` or the Teensy constants to match; they currently cannot both be right. Also `estimateBallSigmaMm` ([main.cpp:54-56](BallAlgo/src/main.cpp#L54-L56)) assumes cm input from a meters-stored state — consistent only under one of the two readings.

### M2. Friction gamma constants disagree: KF uses 0.95, prediction/defense use 0.96
- [config.hpp:106 vs 113](BallAlgo/src/config.hpp#L106-L113). The spec says γ=0.95. Strike intercept and defense impact-time use `kBallPredictionDamping=0.96` while the filter that *produces* the velocities decays with 0.95 — systematic over-prediction of ball travel, growing with horizon.

### M3. Look-ahead pipelining is partially implemented and unmeasured
- [ActionChunkPublisher.cpp:114-143](BallAlgo/src/motion/ActionChunkPublisher.cpp#L114-L143): `kPipelineLatencyUs = 8000` is a guess (docs say "constant we have to find by timing"); the blend covers position/velocity but the documented acceleration blend (`a_start = α·a_chunk`) is absent (start accel always 0 into the profiler); `predictChunkState` integrates chunk velocity without heading/rotation. If real plan+serialize time exceeds 8 ms (A* + spline + profile at 60 Hz on a CM5 with vision in the same thread — plausible), every chunk starts in the past and the Teensy skips its head segment.

### M4. Velocity profiler deviations from the documented S-curve
- [VelocityProfile.cpp:135-179](BallAlgo/src/motion/VelocityProfile.cpp#L135-L179): no jerk-limited approach into the ceiling (`min()` clamp creates acceleration steps — exactly what the 7-phase profile exists to avoid); `sDotStart` is clamped to the ceiling but not validated against the backward pass (over-speed entries produce infeasible first actions); `profileDurationS` fallback reads `ProfileSample.phi` which **no code ever sets** (always 0 → wrong directional aMax) — [MotionPlanner.cpp:28-43,154-160](BallAlgo/src/motion/MotionPlanner.cpp#L28-L43).

### M5. A* fallback and obstacle model weaknesses
- [AStar3D.cpp:173-178](BallAlgo/src/motion/AStar3D.cpp#L173-L178): unreachable goal → returns a straight start→goal 2-point "path" **through the obstacle** with `true` status; callers can't distinguish it from success, so the spline drives through the ball it was told to avoid. `blocked()` checks only the ball circle: no wall inflation (plans flush to x=0), no enemy robots (docs list this as optional, fine — but walls aren't optional). Start-cell quantization (±35 mm) plus obstacle radius 80 mm vs strike offset 120 mm leaves only ~5 mm margin before the goal cell itself is blocked.

### M6. LiDAR pose is smoothed twice before the EKF
- [LidarLocalizer.cpp:627-658](BallAlgo/src/lidar/LidarLocalizer.cpp#L627-L658) applies hold/EMA(α)/step-clamp, then [PoseKalman::update](BallAlgo/src/estimation/PoseKalman.cpp#L86-L106) treats the result as an independent measurement with fixed `R`. Correlated, pre-filtered measurements violate the KF assumptions and add lag to the "snap" the docs call for. Works (per team validation) but will under-correct during fast motion; consider feeding `rawXMm/rawYMm` with quality-scaled R instead.

### M7. Deskewer time-base details
- [Ld19Reader.cpp:100-122](BallAlgo/src/lidar/Ld19Reader.cpp#L100-L122): LD19 hardware timestamps are aligned to the Pi clock **once** (first frame) — sensor-vs-Pi crystal drift accumulates for the whole session with no re-anchoring. [LidarDeskewer::deskew](BallAlgo/src/lidar/LidarDeskewer.cpp#L194-L218) applies `v(t_i)·(t_i−t_ref)` (constant-velocity over the whole span, sampled at the point's own time) rather than integrating the motion history. `serialLatencyUs` from telemetry is never used to shift motion-sample timestamps. Each is small; together they bound deskew accuracy.

### M8. Main-loop sequencing: one camera failure starves every other subsystem
- [BallAlgo/src/main.cpp:244-255](BallAlgo/src/main.cpp#L244-L255): on `grab()` failure the loop `continue`s — skipping serial polling, EKF predict, LiDAR intake, team link poll/publish, chunk publishing, and clock-sync pong service. A camera hiccup therefore also silences team comms and lets the Teensy hit its grace brake. Camera-open failure at startup exits the process entirely (no LiDAR-only degraded mode). Also `grab()` waits on a condition variable with **no timeout** (M16 overlap) and the 4-buffer completed queue means perception can run consistently ~3 frames (~25 ms) behind at 120 fps.

### M9. Pi accumulates protocol frames and never answers pings when chunk publishing is gated off
- [ActionChunkPublisher::publish:160-165](BallAlgo/src/motion/ActionChunkPublisher.cpp#L160-L165) early-returns (when `kEnableActionChunks=false` — a supported config — or no role/goal) **before** `takePendingFrames`/`ClockSync::processFrames`. `RobotSerial::pendingFrames_` then grows without bound (~10 pings/s [NPA]) and clock sync never locks. Slow leak + silent sync failure in a "telemetry-only" configuration.

### M10. Corrupt length field stalls the Pi frame parser for up to 64 kB
- [Protocol.cpp:125-155](BallAlgo/src/motion/Protocol.cpp#L125-L155): a flipped bit in `plen` (up to 65535) makes `unpackFrames` wait for `7+plen+4` bytes before the CRC check can fail and resync — ~0.3 s of telemetry/ping loss at 2 Mbaud per corruption. The Teensy side caps `MAX_PAYLOAD_LEN = 1232` [NPA] / discards >2048 [LC]; the Pi should cap similarly (max legit payload is 24).

### M11. Role arbitration edge cases
- [RoleArbiter.cpp:94-98](BallAlgo/src/team/RoleArbiter.cpp#L94-L98): same-claim conflict resets *both* robots to id-default roles regardless of who actually sees the ball → a defense-id robot with the ball can be yanked back to defense for ≥3 confirm samples; under asymmetric packet delay the pair can oscillate claim→default→switch. `testSplitBrainRecovery` covers one direction only. Also `shouldSwitchRoles` compares `peer.state.ballDistanceCm` (sender-side stale by up to 300 ms) against live local distance with a fixed 15 cm margin — at ball speeds ≥1 m/s the staleness dwarfs the margin.

### M12. [LC]/[NPA] heading-deadband logic bugs
- [NPA] [Movement.cpp:30-40,55-65](Offense2026/src/Movement.cpp#L30-L40): `if (abs(diff) < 5) correction = 0; } if (diff > 90) …` — missing `else`: the deadband assignment is immediately overwritten by the `diff > 0` branch; deadband is dead code (the [LC] HeadingController got this right with an early return). Both branches run the PID on `abs(error)` so the derivative term has the wrong sign half the time (Kd is tiny; Ki=0 saves them from worse).

### M13. [NPA] Defense mode is unfinished and inconsistent with offense gating
- [Offense2026/src/main.cpp:173-216](Offense2026/src/main.cpp#L173-L216): `runDefense()` ends after the goal-angle checks without ever commanding the block behavior (`Defense defense` object is never called; the legacy `defenseCalc` is dead on this branch). Defense robots stop or blindly chase. Also `runDefense` gates on `lcdController.isStartEnabled()` (honors LCD override) while `runOffense` gates on raw `switches.start()` — the LCD start button cannot stop an offense robot.

### M14. Lightgate pull-up idiom is AVR-era; duplicate Switch instances
- [Switches.cpp:4-13,47-57](Offense2026/src/Switches.cpp#L4-L13): `pinMode(41, INPUT); digitalWrite(41, HIGH);` — on Teensy 4.x the supported way is `INPUT_PULLUP`; relying on the AVR idiom risks a floating input, and a floating lightgate randomly forces `ballDist = 12` ([Cam.cpp:34-35](Offense2026/src/Cam.cpp#L34-L35)) and `hasPossession` [LC]. `Cam` also constructs its **own** `Switch` instance (global ctor re-runs pinModes; two objects answer "lightgate?" independently).

### M15. Camera FPS is configured nowhere
- `kCameraFps = 120` ([config.hpp:23](BallAlgo/src/config.hpp#L23)) is used for KF friction normalization and intercept horizons, but `CameraCapture` never sets `FrameDurationLimits` — the sensor runs at whatever the Viewfinder mode negotiates. If actual fps ≠ 120, `kBallKfCamDtS`-anchored γ normalization is fine (dt-based) but `predictBallBody`'s `samples = t/dtCam` and the [LC] velocity px/frame units shift.

### M16. [LC] Defense legacy `hardStop` never decays
- [Defense.cpp:96-116](Offense2026/src/Defense.cpp#L96-L116): `hardStop` is only ever set to 200 or 0, but the blocked branch tests `hardStop <= 100` — as written, once `angleDiff > 170` with a valid previous angle, the code flips `defenseAngle` by 180° **every loop iteration** (oscillation) instead of once per stint; the intended decay counter was never implemented.

### M17. [LC] Field-dimension mismatch Pi vs Teensy
- Pi: 1820×2430 mm (half-length 121.5 cm); Teensy: `kFieldHalfLengthCm = 146.5` ([StrategyConfig.h:58-59](Offense2026/src/StrategyConfig.h#L58-L59)). Goal field points at y=±146.5 are 25 cm beyond anything the Pi's `ly` (centered-cm, ±121.5) can report, so goal-fallback aim (`fieldTargetToRobotAngle`) and search targets are biased toward the walls. The two repos describe different physical fields; the RCJ interior is 182×243 cm — the Pi matches, the Teensy's 293 cm length does not.

### M18. [NPA] Hot-loop Serial spam throttles the control loop
- `runOffense()` prints ~8 `String`-concatenated lines per iteration unconditionally ([main.cpp:111-136](Offense2026/src/main.cpp#L111-L136)), plus per-byte prints inside `Cam`. USB serial is fast but `String` heap churn + printing every loop measurably caps loop rate — which directly quantizes trajectory action indexing (`elapsed/dt_ms`) and line-avoidance latency. The [LC] `DebugReporter` (rate-limited to 200 ms) did this right.

### M19. BNO055 robustness: bring-up can hang the robot forever
- [CompassSensor.cpp:9-31](Offense2026/src/CompassSensor.cpp#L9-L31): `begin()` does `while(1);` if the IMU is missing; `callibrate()` loops until mag==3 with no timeout (robot must be waved by hand; a bad mag environment = robot never starts). `Wire.begin()` is called but the sensor is constructed on `Wire2` (the Adafruit driver begins its own bus, so this line is just dead/misleading). Heading is `int` degrees → 1° quantization for telemetry, executor rotation, and PID input.

### M20. Pose-adapter freshness vs Pi sentinel encoding
- [PoseAdapter.cpp:14-27](Offense2026/src/PoseAdapter.cpp#L14-L27) treats exactly `-5` as invalid, but the Pi sends `lround((pos − half)·0.1)` ints — a legitimate pose at centered-cm = −5 (i.e., 50 mm left of center) is indistinguishable from the lost sentinel and is dropped. Same sentinel-collision exists for ball angle −5 ≈ 355° (the Pi clamps found angles to [0,360), so only the pose case is real).

### M21. Threshold-name/channel-order coupling is a maintenance trap
- [CameraCapture.cpp:334-344](BallAlgo/src/camera/CameraCapture.cpp#L334-L344) swaps channels for **both** BGR888 and RGB888 (the two `cvtColor` calls are the same permutation), and `thresholds.json` hue ranges are visibly swapped relative to their names (`ball` 103–123 = blue-ish hue band; `blueGoal` 6–24 = orange band). The pipeline is self-consistent (calibrated end-to-end, per team validation), but anyone who "fixes" the cvtColor or recalibrates with a standard tool will silently invert ball/goal detection. At minimum document it next to both files. `kIgnoreHsvValue=true` also widens V to 0–255 for *goals* as well as ball — gray walls/floor inside goal hue bands pass on saturation alone.

### M22. Sector tracker details
- [SectorTracker.cpp:92-101](BallAlgo/src/vision/SectorTracker.cpp#L92-L101): `lookahead` can only be 1 or 2, so `kLookaheadMax=3` is unreachable (dead config); ring radius 6 visits the same opposite sector twice (wasted full-mask `bitwise_and` per frame); `firstFrame_` is named inverted (false = first frame). Velocity EMA is px/frame with no dt normalization — at variable fps the `kLookaheadSpeedThresh` changes meaning.

### M23. Team-link publish gating quirks
- [TeamLink.cpp:58-85](BallAlgo/src/team/TeamLink.cpp#L58-L85): `localSeq_` increments on skipped sends (gaps in seq are normal — fine, but makes seq-based loss diagnostics useless); `isMajorStateChange` bypasses the rate limit but a flapping `ballVisible` at the stale boundary can burst-send at loop rate; `unpackTeamStateFrames` clears the whole buffer on one bad frame, discarding any good queued frames behind it (acceptable, but worth knowing).

---

## LOW

1. **`Movement::movement()` [NPA] always re-normalizes to full magnitude** ([Movement.cpp:105-111](Offense2026/src/Movement.cpp#L105-L111)): after heading correction it divides by max power unconditionally (the [LC] version only clamps when >1) — every command saturates one wheel at `speedfactor`, so heading correction trades directly against translation in a way that differs between branches.
2. **`Motor::setSpeed` has no input clamp** ([Motor.cpp:19-35](Offense2026/src/Motor.cpp#L19-L35)); `analogWrite(pwmPin, 255*abs(speed))` with speed >1 relies on core clamping; the constructor silently swaps `in1`/`in2` (intentional direction flip, but undocumented).
3. **`Trig::wrapAngle` only wraps once** ([trig.cpp:32-36](Offense2026/src/trig.cpp#L32-L36)) — inputs beyond ±540° return out-of-range (current call sites are bounded).
4. **`Ld19Reader` discards a whole 47-byte window on CRC failure** ([Ld19Reader.cpp:163-169](BallAlgo/src/lidar/Ld19Reader.cpp#L163-L169)) instead of resyncing at the next 0x54 inside it — drops one valid frame per corruption (self-heals).
5. **`headingBetweenPointsDeg` uses `atan2(dy,dx)`** ([OffensePose.cpp:57-59](BallAlgo/src/motion/OffensePose.cpp#L57-L59)) while sibling code uses `atan2(dx,dy)` — the target *heading* convention differs between offense special-cases and the strike fallback (`headingDeg + goalDeg` at [OffensePose.cpp:287](BallAlgo/src/motion/OffensePose.cpp#L287)); harmless only if those headings are never executed precisely (today the A* bug C9 masks it).
6. **FoxgloveConfig path is CWD-relative** (`foxglove_sim/foxglove.conf`) — running `ballalgo` from anywhere but the repo root silently loses the config (defaults applied, no warning).
7. **`fillStopChunkIfEmpty` emits 50 zero-velocity actions** — on [NPA] a zero-action chunk means "stop", but these are *full* chunks of zeros with normal ids; harmless, just wasteful bandwidth (1.2 kB per stop at 60 Hz ≈ 75 kB/s of zeros).
8. **`uart_test_pi.py`** is fine; `tools/` scripts not deeply reviewed (out of scope).
9. **Switch reads have no debounce** ([Switches.cpp](Offense2026/src/Switches.cpp)) — goal-side/start glitches propagate instantly into behavior (and into telemetry → Pi role logic on a fixed branch).
10. **`using namespace std`, `#include <iostream>` in Teensy code** ([NPA] Cam.cpp) — heap-heavy `std::string` in an embedded parse loop.

---

## Dead / Unnecessary / Stale Code

| Item | Location | Notes |
|---|---|---|
| `strikePoseBody()` | [BallAlgo/src/motion/StrikePose.cpp:11-17](BallAlgo/src/motion/StrikePose.cpp#L11-L17) | Never called; OffensePose re-implements behind-ball targets. Uses the contested `polarToBodyXY` convention. |
| `RobotSerial::pollHeading()` | [RobotSerial.cpp:135-139](BallAlgo/src/io/RobotSerial.cpp#L135-L139) | Never called. |
| `kEnablePlannerCompare` | [config.hpp:179](BallAlgo/src/config.hpp#L179) | No references anywhere. |
| `ProfileSample.phi` | motion headers + [MotionPlanner.cpp:154-160](BallAlgo/src/motion/MotionPlanner.cpp#L154-L160) | Never written; `profileDurationS` reads it (M4). |
| `kLookaheadMax` | [config.hpp:49](BallAlgo/src/config.hpp#L49) | Unreachable (M22). |
| `team_state.proto` | [BallAlgo/src/team/team_state.proto](BallAlgo/src/team/team_state.proto) | Documentation-only (hand-rolled encoder is used); field numbers currently match — keep them in sync or generate. |
| `Bluetooth.cpp/.h` | [Offense2026/src/Bluetooth.cpp](Offense2026/src/Bluetooth.cpp) | 100% commented out. Teensy-side team comms abandoned (moved to Pi). Delete. |
| `lidar_reader.cpp/.h`, `LidarLocalizer.cpp/.h` [NPA] | Offense2026/src | Unreferenced Teensy-side LD19 experiment. **Landmine:** `lidar_reader` defaults to `Serial3` — the Pi link — if ever revived. |
| `Defense` object [NPA] | [main.cpp:38](Offense2026/src/main.cpp#L38) | Constructed, never used (M13). |
| `MotionConfig.h ALPHA_WHEEL` [LC] | [MotionConfig.h:26](Offense2026/src/MotionConfig.h#L26) | Unused; `Movement.cpp` defines its own `kWheelAlpha` with a *different element order* — a trap for anyone "unifying" them. |
| `kickoffEnabled` | snapshot fields both branches | Read from the switch, never used by any planner. |
| `Orbit::GetToPosition` | both branches | No callers. |
| `thresholds.json` extra keys (`crop`, `sharpness`, …) | repo root | Not parsed by `Thresholds.cpp` (regex reads ball/goals/offsets/mask1 only) — stale calibration-GUI fields. |
| Committed `CMakeFiles/`, root `Makefile`, `cmake_install.cmake` | BallAlgo root, **tracked in git** | In-source build artifacts incl. compiled `a.out` binaries and dirs for two deleted targets (`ballalgo 2.dir`, `ballalgo_planner_bench.dir`). `.gitignore` lists them but they were committed first. `git rm -r --cached` them. |
| `sim/ballalgo_sim 2` (empty dir), `__pycache__`, `.DS_Store` | both repos | Junk. |
| `Adafruit_VL53L0X` lib dep [LC] | [platformio.ini:18](Offense2026/platformio.ini#L18) | No source references it. |
| Python visualizers in `src/` | Offense2026/src/*.py | Tooling living inside the firmware source tree (PlatformIO will try to glob `src/` — they're not C++, harmless, but misplaced; `main_cam.py` is untracked stray). |

---

## Missing Tests (ranked by what would have caught real bugs found above)

1. **Protocol round-trip Pi-encoder ↔ Teensy-parser** (would catch C3, C4, H6, the legacy-telemetry semantics, and any future schema drift): encode with `packActionChunk`/`formatPerception`/telemetry structs, decode with the *actual* firmware parser code compiled host-side.
2. **A* convention/heading tests** (C9): straight-line plans in ±x/±y with fixed start/goal headings; assert no parasitic heading changes and correct bin decode.
3. **Defense goal-line geometry sweep** (H1): m ∈ [−2, 2], assert targets lie on the union of top-line/side-line/arc.
4. **Offense state classification map** (H2): grid of ball positions → expected states.
5. **Frame-convention invariants** (C10/H8): `bodyToFieldOffsetMm(fieldToBodyOffsetMm(v)) == v`; camera-angle → body → bearing → predicted-bearing closure for a synthetic goal.
6. **Ball KF frame test** (H3): stationary ball + moving/rotating robot ⇒ fused field velocity ≈ 0.
7. **VelocityProfile**: vStart > braking-feasible, zero-length paths, ceiling continuity; assert monotone time, bounded accel.
8. **TeamLink framing over a lossy pipe** (C6 would have been instantly visible in a socketpair test).
9. **Ld19 frame parser**: CRC fuzz, wrap of hw timestamp, partial reads.
10. **LineDetection PCA** (H12): symmetric activations, single sensor, horizontal/vertical lines — assert no NaN.
11. **Offense2026 has zero tests of any kind**; even host-compilable pure-logic units (orbit, Defense, GeometryUtils, parsers) are untested.

---

## Build / Configuration Problems

1. **Configure fails** (C1) — missing `src/planner/PlannerCaseMain.cpp`.
2. **Committed build artifacts** shadow the broken config: a stale root `Makefile` + `CMakeFiles/` make `make` *appear* to work on machines that have the old cache, hiding C1 from whoever committed it.
3. `BALLALGO_ENABLE_LIDAR=OFF` produces a binary whose pose can never validate (H8) — the advertised option yields a silently crippled robot.
4. `pkg_check_modules(LIBGPiod libgpiod)` — case-typo'd variable (`LIBGPiod_FOUND`) works but will bite anyone renaming; bluez detection gates team comms silently (no startup log when BT is compiled out).
5. Per-robot config (`kRobotId`, `kPeerBtAddress`, `kRobotMode`, presets) is all `constexpr`/compile-time across both repos — two robots require two divergent builds with no runtime check that they differ (C6 trap).
6. [LC] `platformio.ini` carries an unused sensor lib (VL53L0X); no `lib_deps` pinning beyond semver carets — firmware builds are not reproducible.
7. `install(FILES thresholds.json DESTINATION share/ballalgo OPTIONAL)` but the binary loads `thresholds.json` from CWD only — the installed copy is never found by the installed binary.

---

## Docs / Spec Drift (`docs/pipeline_context.md` vs implementation)

| Doc claim | Reality |
|---|---|
| Field "182 cm × 132 cm" | Config is 1820×2430 mm (and Teensy [LC] says 293 cm length — three values in play). |
| EKF state `[x y θ v ω]`, IMU-authoritative θ | Implemented as `[x y vx vy]` with external heading — *documented deviation, reasonable* — but the doc's rotation matrix `[c s; −s c]` is the transpose of the code's (C10). |
| Ball KF "absolute ball coordinates" | Implemented in body frame (H3). |
| "NN outputs true real-world distance d_nn" | Implemented as a power-law pixel calibration (`0.00255·px^2.096`) — fine, but the doc's "visibility certainty from the vision network" became blob-area ratio ([SectorTracker.cpp:159-172](BallAlgo/src/vision/SectorTracker.cpp#L159-L172)). |
| Step 2 blends position, velocity **and acceleration** | Acceleration blend not implemented (M3). |
| Step 3 cost: heading change Δθ free of travel direction | Heading forced to travel direction; bin convention bug (C9). |
| Step 4 7-phase jerk-limited S-curve | Phase 3 (jerk-limited approach to ceiling) replaced by a clamp (M4). |
| Step 7 dual-buffer + hot swap + 20 ms grace | [NPA]: implemented ✔. [LC]: single buffer, no rotation, no grace (C7). |
| Step 8 proportional voltage safeguard, `read_battery_rail_voltage()` | [NPA]: scaling ✔ but battery hardcoded 12 V (TODO); [LC]: per-wheel clamp, battery hardcoded 7.4 V. |
| Defense Step 1.2 `m <= 5/11` | Code uses −5/11 (H1). |
| "compass data must follow (0=North, 90=East)" | Raw un-zeroed BNO; zeroing applied only to local control (H10). |
| README: layout `ballalgo_overview_app_layout.json` | File doesn't exist; only `ballalgo_standard_layout.json` is in the repo (foxglove README references both). |
| README: "Binary output: build/ballalgo" | Build fails (C1). |
| Teensy [NPA] header: "Pi packs kick/dribblerPower" | Pi sends zero padding (H6). |

---

## Integration / Protocol Risk Matrix

| Path | [LC] status | [NPA] status |
|---|---|---|
| Pi→Teensy action chunks (0x03) | Parsed ✔ but executed without frame rotation, ISR races loop, no start-switch gate (C7) | Parsed ✔, rotated ✔, gated ✔; kick/dribbler bytes misinterpreted as commands (H6); restart lockout (H4) |
| Teensy→Pi telemetry (0x04) | **Never sent** — Pi blind (C4) | Sent (legacy 20 B) — goalIsBlue/start/mode forced to defaults (C4) |
| Teensy→Pi heading ASCII `…h` | Sent, **never parsed** (C4) | Not sent (good — superseded) |
| Pi→Teensy ASCII perception | Parser OK & guarded, but Pi default-disables it (C3 context) | Parser incompatible (`e` exponent bug) **and** wrong UART (C2, C3) |
| Clock sync ping/pong | Works; float-precision offset (H5) | Works; int64 ✔; 0-sentinel + micros wrap (H5); pongs unanswered if Pi chunk-gating off (M9) |
| Robot↔robot Bluetooth | n/a (Teensy side deleted) | n/a — Pi↔Pi RFCOMM flaps on idle (C6) |
| Line override ↔ chunks | brake/re-arm limit cycle; Pi unaware of lines (H7) | same cycle, better ordering (H7) |
| Mouse odometry | never reaches Pi (C4) | LinePCB → Teensy → Pi, units asserted m/s→mm/s; LinePCB firmware lives outside this repo — **unverifiable here** |
| Units across boundary | ball dist mm-vs-cm (M1); px/frame velocities by contract | same; field length 243 vs 293 cm (M17) |

---

## "Planned in docs but not actually implemented"

- **Kick/dribbler command path from the Pi** — wire fields exist, Pi never populates them, no Pi-side shooting strategy at all (H6).
- **Acceleration term of the Step-2 look-ahead blend** (M3) and a *measured* `pipeline_latency_us`.
- **Enemy-robot obstacle inflation** (docs Step 3, marked optional — absent, as is *wall* inflation, which isn't optional).
- **Phase-3 jerk-limited ceiling approach** of the S-curve (M4).
- **Battery-voltage reading** on the Teensy (`readBatteryVoltage` TODO returns 12.0) and **bench-characterized motor constants** (`TODO: replace with bench-characterised values` — kV=5.35 placeholder, C8).
- **Strike pose "desired target velocities"** — offense terminal velocity is always a zero-hold (`setTerminalVelocityHold`); the moving-strike velocity from the docs is only realized for defense intercepts.
- **Defense behavior on [NPA]** — `runDefense` is a stub (M13).
- **Vision-only (goal-bearing) localization keeping pose alive without LiDAR** — bearing updates exist but can neither initialize nor sustain validity (H8).
- **`is_first_chunk_of_match` reset semantics** from the docs (per-match, not per-boot) — both branches implement per-boot only (H4).
- **Motor wiring verification** — `TODO: verify index→motor assignment against physical wiring` sits on the exact line that maps voltages to motors ([TrajectoryExecutor.cpp:348-353](Offense2026/src/TrajectoryExecutor.cpp#L348-L353)).

---

## Suggested order of attack (highest leverage first)

1. Decide the canonical Offense2026 branch; delete or archive the other. Every integration fix depends on this.
2. Fix the build (C1) and de-commit the build artifacts; add a CI step that runs `cmake + ctest` from clean.
3. Wire the serial contract end-to-end on the chosen branch: correct UART (C2), telemetry schema incl. goalIsBlue/start (C4), attack-goal selection (C5), restart-safe trajectory ids (H4).
4. Write the cross-repo protocol round-trip test and the frame-convention test (C10) — they pin down everything else.
5. Fix TeamLink idle-teardown (C6) before any two-robot work.
6. Fix A* heading bins (C9), defense geometry (H1), offense axes (H2), ball-velocity frames (H3); add the unit tests listed above so they stay fixed.
7. Reconcile the motor model with one bench characterization shared by both repos (C8).

---

*Generated by an automated deep review session: every C/C++ source in both repos was read in full (Offense2026 on two branches); BallAlgo team tests were compiled and passed (11/11); math findings marked "verified" were reproduced with a compiled probe against the unmodified BallAlgo motion-core sources. No repository files were modified.*

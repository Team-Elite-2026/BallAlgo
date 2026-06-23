#include "motion/MotionPlanner.hpp"

#include "config.hpp"
#include "params.hpp"
#include "motion/MotionLimits.hpp"
#include "motion/OffensePose.hpp"
#include "vision/VisionMath.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ballalgo {

namespace {

int headingToBin(float angleDeg, int bins) {
  const float wrapped = std::fmod(std::fmod(angleDeg, 360.f) + 360.f, 360.f);
  const float stepDeg = 360.f / static_cast<float>(bins);
  return static_cast<int>(wrapped / stepDeg) % bins;
}

float wrapAngleDeg(float angleDeg) {
  float wrapped = std::fmod(angleDeg + 180.f, 360.f);
  if (wrapped < 0) wrapped += 360.f;
  return wrapped - 180.f;
}

// Max speed (m/s) the robot can sustain pointing along `headingDeg`, given the
// anisotropic per-axis caps (kVMaxX/kVMaxY). Heading is clockwise-positive with
// 0 deg = field +y = robot forward, so the forward unit vector is (sin, cos) and
// this is the radius of the velocity ellipse along that direction.
float maxForwardSpeedMps(float headingDeg) {
  const float r = headingDeg * static_cast<float>(M_PI / 180.0);
  const float sx = std::sin(r) / params::get().vMaxX;
  const float cy = std::cos(r) / params::get().vMaxY;
  const float denom = std::sqrt(sx * sx + cy * cy);
  return denom > 1e-6f ? 1.f / denom : 0.f;
}

float profileDurationS(const std::vector<ProfileSample>& profile) {
  if (profile.size() < 2) return 0.f;
  float durationS = 0.f;
  for (size_t i = 0; i + 1 < profile.size(); ++i) {
    const float dsM = std::max(0.f, profile[i + 1].sMm - profile[i].sMm) / 1000.f;
    if (dsM <= 1e-6f) continue;
    const float avgV = 0.5f * (profile[i].v + profile[i + 1].v);
    if (avgV > 0.02f) {
      durationS += dsM / avgV;
      continue;
    }
    const float aMax = motion::aMaxDir(profile[i].phi, params::get().aMaxX, params::get().aMaxY);
    durationS += std::sqrt(2.f * dsM / std::max(0.1f, aMax));
  }
  return durationS;
}

float initialOffenseInterceptTimeS(const BallState& ball) {
  const float maxPlanSpeedMps = std::max(params::get().vMaxX, params::get().vMaxY);
  const float travelTimeS =
      std::hypot(ball.xM, ball.yM) / std::max(0.05f, maxPlanSpeedMps);
  return std::clamp(travelTimeS, 0.f, params::get().strikeInterceptMaxTimeS);
}

void fillStopChunkIfEmpty(PlannedChunk& chunk) {
  if (!chunk.actions.empty()) return;
  MotionAction stop{};
  chunk.actions.assign(config::kChunkMaxActions, stop);
}

void fillBallDebugFromOffensePose(BallPlanDebug& debug, const OffensePoseResult& offensePose) {
  debug.offensePose = offensePose;
  debug.usedStrikePosePlan = offensePose.state == OffensePoseState::NormalStrike;
  debug.strikeTargetBodyXM = offensePose.strikeTargetBodyXM;
  debug.strikeTargetBodyYM = offensePose.strikeTargetBodyYM;
  debug.predictedBallBodyXM = offensePose.predictedBallBodyXM;
  debug.predictedBallBodyYM = offensePose.predictedBallBodyYM;
  debug.predictedBallBodyVXMps = offensePose.predictedBallBodyVXMps;
  debug.predictedBallBodyVYMps = offensePose.predictedBallBodyVYMps;
  debug.ballFieldXMm = offensePose.predictedBallFieldXMm;
  debug.ballFieldYMm = offensePose.predictedBallFieldYMm;
  debug.targetXMm = offensePose.targetXMm;
  debug.targetYMm = offensePose.targetYMm;
  debug.targetHeadingDeg = offensePose.targetHeadingDeg;
  debug.targetVxFieldMps = offensePose.targetVxFieldMps;
  debug.targetVyFieldMps = offensePose.targetVyFieldMps;
  debug.targetOmegaRadS = offensePose.targetOmegaRadS;
}

// Overwrite the chunk's tail (last ~10 actions) with a single held terminal
// velocity. Chunks carry GLOBAL-frame targets (the Teensy rotates into the body
// frame), so the field-frame velocity is written directly. No-op when inactive.
void applyTerminalVelocity(PlannedChunk& chunk, bool active, float vxField, float vyField,
                           float omega) {
  if (!active || chunk.actions.empty()) return;
  const int terminalCount = std::min<int>(10, static_cast<int>(chunk.actions.size()));
  for (int i = static_cast<int>(chunk.actions.size()) - terminalCount;
       i < static_cast<int>(chunk.actions.size()); ++i) {
    MotionAction& a = chunk.actions[static_cast<size_t>(i)];
    a.vx = vxField;
    a.vy = vyField;
    a.omega = omega;
    a.ax = 0.f;
    a.ay = 0.f;
    a.alpha = 0.f;
  }
}

void buildChunkToTarget(const PoseState& pose, float headingDeg, float goalXMm, float goalYMm,
                        float goalHeadingDeg, AStar3D& astar, HermiteSpline& spline,
                        VelocityProfile& profiler, PlannedChunk& chunk,
                        std::vector<Waypoint3>* waypoints, std::vector<PathSample>* path,
                        std::vector<ProfileSample>* profile,
                        std::vector<TrajectorySpeedSample>* trajectorySpeedProfile,
                        float* astarCostS = nullptr, float endSpeedMps = 0.f) {
  int st = headingToBin(headingDeg, config::kAstarHeadingBins);
  int gt = headingToBin(goalHeadingDeg, config::kAstarHeadingBins);

  std::vector<Waypoint3> localWaypoints;
  std::vector<PathSample> localPath;
  std::vector<ProfileSample> localProfile;
  std::vector<TrajectorySpeedSample> localTrajectorySpeedProfile;

  std::vector<Waypoint3>& waypointsRef = waypoints != nullptr ? *waypoints : localWaypoints;
  std::vector<PathSample>& pathRef = path != nullptr ? *path : localPath;
  std::vector<ProfileSample>& profileRef = profile != nullptr ? *profile : localProfile;
  std::vector<TrajectorySpeedSample>& trajectorySpeedProfileRef =
      trajectorySpeedProfile != nullptr ? *trajectorySpeedProfile : localTrajectorySpeedProfile;

  astar.plan(pose.xMm, pose.yMm, st, goalXMm, goalYMm, gt, waypointsRef, astarCostS);
  if (waypointsRef.size() == 1) {
    Waypoint3 start{};
    start.xMm = pose.xMm;
    start.yMm = pose.yMm;
    start.thetaDeg = headingDeg;
    Waypoint3 goal{};
    goal.xMm = goalXMm;
    goal.yMm = goalYMm;
    goal.thetaDeg = goalHeadingDeg;
    waypointsRef.front() = start;
    waypointsRef.push_back(goal);
  } else if (!waypointsRef.empty()) {
    waypointsRef.front().xMm = pose.xMm;
    waypointsRef.front().yMm = pose.yMm;
    waypointsRef.front().thetaDeg = headingDeg;
    waypointsRef.back().xMm = goalXMm;
    waypointsRef.back().yMm = goalYMm;
    waypointsRef.back().thetaDeg = goalHeadingDeg;
  }

  // Step 4.1: analytic Hermite spline. Start tangent = current velocity (field
  // mm/s) so S'(0) matches the robot's real direction of travel. End tangent is
  // forward-relative to the goal pose at endSpeedMps (0 = stop at goal), so the
  // robot arrives moving straight along goalHeadingDeg.
  const float goalRad = goalHeadingDeg * static_cast<float>(M_PI / 180.0);
  const float vEndMmS = endSpeedMps * 1000.f;
  const float vxEndMmS = vEndMmS * std::sin(goalRad);
  const float vyEndMmS = vEndMmS * std::cos(goalRad);
  HermiteSplineData data = spline.buildData(waypointsRef, goalHeadingDeg, pose.vxMmS, pose.vyMmS,
                                            vxEndMmS, vyEndMmS, 10);
  pathRef = data.samples;

  // Step 4.3: motor-model 3-pass S-curve profile (global-frame commands).
  const float vStartMps = std::hypot(pose.vxMmS, pose.vyMmS) / 1000.f;
  const int numSteps = std::max(50, static_cast<int>(pathRef.size()));
  std::vector<ProfilePointS> motorProfile =
      profiler.computeMotorModel(data, vStartMps, endSpeedMps, numSteps);

  // Step 4.4: slice into fixed-dt GLOBAL-frame action chunk.
  chunk.actions = profiler.discretizeGlobal(motorProfile, chunk.dtMs, config::kChunkMaxActions);

  // Keep a lightweight ProfileSample track for duration fallback / telemetry.
  profileRef.clear();
  profileRef.reserve(motorProfile.size());
  for (const auto& p : motorProfile) {
    ProfileSample ps{};
    ps.v = std::hypot(p.vx, p.vy);
    profileRef.push_back(ps);
  }

  trajectorySpeedProfileRef.clear();
  trajectorySpeedProfileRef.reserve(motorProfile.size());
  for (const auto& p : motorProfile) {
    TrajectorySpeedSample sample{};
    sample.progress01 = p.s;
    sample.speedMps = p.sDot;
    trajectorySpeedProfileRef.push_back(sample);
  }
}

}  // namespace

MotionPlanner::MotionPlanner()
    : astar_(config::kFieldWidthMm, config::kFieldHeightMm, config::kAstarCellMm,
             config::kAstarHeadingBins) {}

static uint64_t nowPiUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

uint64_t MotionPlanner::nextTrajId() { return nowPiUs(); }

PlannedChunk MotionPlanner::plan(const PoseState& pose, const BallState& ball, float goalDeg,
                                 float headingDeg, bool fullPlanner) {
  return debugPlan(pose, ball, goalDeg, headingDeg, fullPlanner, nullptr).chunk;
}

BallPlanDebug MotionPlanner::debugPlan(const PoseState& pose, const BallState& ball, float goalDeg,
                                       float headingDeg, bool fullPlanner) {
  return debugPlan(pose, ball, goalDeg, headingDeg, fullPlanner, nullptr);
}

PlannedChunk MotionPlanner::plan(const PoseState& pose, const BallState& ball, float goalDeg,
                                 float headingDeg, bool fullPlanner,
                                 const FieldTarget* goalFieldTarget) {
  return debugPlan(pose, ball, goalDeg, headingDeg, fullPlanner, goalFieldTarget).chunk;
}

BallPlanDebug MotionPlanner::debugPlan(const PoseState& pose, const BallState& ball, float goalDeg,
                                       float headingDeg, bool fullPlanner,
                                       const FieldTarget* goalFieldTarget) {
  BallPlanDebug debug;
  debug.startPose = pose;
  debug.ball = ball;
  debug.goalDeg = goalDeg;
  debug.usedGoalFieldTarget = goalFieldTarget != nullptr;
  if (goalFieldTarget != nullptr) {
    debug.goalFieldXMm = goalFieldTarget->xMm;
    debug.goalFieldYMm = goalFieldTarget->yMm;
  }
  debug.startHeadingDeg = headingDeg;
  debug.fullPlanner = fullPlanner;
  debug.chunk.trajectoryId = nextTrajId();
  debug.chunk.dtMs = config::kChunkDtMs;
  debug.chunk.startTimePi = nowPiUs() + config::kSerialLatencyMarginUs;

  if (!ball.visible) {
    if (fullPlanner && pose.valid) {
      debug.usedCenterFallback = true;
      debug.targetXMm = config::kFieldWidthMm * 0.5f;
      debug.targetYMm = config::kFieldHeightMm * 0.5f;
      debug.targetHeadingDeg = headingDeg;
      debug.targetErrMm = std::hypot(debug.targetXMm - pose.xMm, debug.targetYMm - pose.yMm);
      debug.withinTargetTolerance =
          debug.targetErrMm <= params::get().commandGoalPositionToleranceMm;
      buildChunkToTarget(pose, headingDeg, debug.targetXMm, debug.targetYMm, debug.targetHeadingDeg,
                         astar_, spline_, profiler_, debug.chunk, &debug.waypoints, &debug.path,
                         &debug.profile, &debug.trajectorySpeedProfile);
    }
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  if (!fullPlanner || !pose.valid) {
    debug.usedBodyChaseFallback = true;
    debug.targetErrMm = std::hypot(ball.xM * 1000.f, ball.yM * 1000.f);
    debug.withinTargetTolerance =
        debug.targetErrMm <= params::get().commandGoalPositionToleranceMm;
    float dist = std::hypot(ball.xM, ball.yM);
    float sp = std::min(0.3f, dist * 0.5f);
    MotionAction action{};
    if (dist > 1e-3f) {
      // Body-frame chase direction toward the ball, rotated into the GLOBAL
      // frame (chunks are global; the Teensy rotates back to body using its
      // heading). Body->world per production convention.
      const float bvx = sp * ball.xM / dist;
      const float bvy = sp * ball.yM / dist;
      const float h = headingDeg * static_cast<float>(M_PI / 180.0);
      const float c = std::cos(h);
      const float s = std::sin(h);
      action.vx = c * bvx - s * bvy;
      action.vy = s * bvx + c * bvy;
    }
    debug.chunk.actions.assign(config::kChunkMaxActions, action);
    return debug;
  }

  const bool hasGoalFieldTarget = goalFieldTarget != nullptr;
  const float goalFieldXMm = hasGoalFieldTarget ? goalFieldTarget->xMm : 0.f;
  const float goalFieldYMm = hasGoalFieldTarget ? goalFieldTarget->yMm : 0.f;
  float interceptTimeS = initialOffenseInterceptTimeS(ball);

  for (int iteration = 0; iteration < params::get().strikeInterceptMaxIterations; ++iteration) {
    const OffensePoseResult offensePose =
        computeOffensePose(pose, ball, goalDeg, headingDeg, hasGoalFieldTarget, goalFieldXMm,
                           goalFieldYMm, interceptTimeS);
    debug.interceptTimeS = interceptTimeS;
    debug.interceptIterations = iteration + 1;
    fillBallDebugFromOffensePose(debug, offensePose);
    debug.targetErrMm = std::hypot(debug.targetXMm - pose.xMm, debug.targetYMm - pose.yMm);
    debug.withinTargetTolerance =
        debug.targetErrMm <= params::get().commandGoalPositionToleranceMm;

    // Step 3: inflate the (predicted) ball as a circular obstacle so the path
    // curves around it to the behind-ball strike pose.
    astar_.setObstacle(debug.ballFieldXMm, debug.ballFieldYMm, params::get().astarBallClearMm);
    float astarCostS = 0.f;
    // Strike approaches arrive at max forward speed (forward-relative to the goal
    // pose) to carry momentum into the kick; collect poses still stop at v=0.
    const float strikeEndSpeedMps =
        debug.usedStrikePosePlan ? maxForwardSpeedMps(debug.targetHeadingDeg) : 0.f;
    buildChunkToTarget(pose, headingDeg, debug.targetXMm, debug.targetYMm, debug.targetHeadingDeg,
                       astar_, spline_, profiler_, debug.chunk, &debug.waypoints, &debug.path,
                       &debug.profile, &debug.trajectorySpeedProfile, &astarCostS,
                       strikeEndSpeedMps);
    astar_.clearObstacle();
    const float pathTimeS = astarCostS > 0.f ? astarCostS : profileDurationS(debug.profile);
    const float nextInterceptTimeS =
        std::clamp(pathTimeS, 0.f, params::get().strikeInterceptMaxTimeS);
    if (std::fabs(nextInterceptTimeS - interceptTimeS) <= params::get().strikeInterceptConvergeS ||
        pathTimeS >= params::get().strikeInterceptMaxTimeS ||
        iteration + 1 == params::get().strikeInterceptMaxIterations) {
      break;
    }
    interceptTimeS = nextInterceptTimeS;
  }
  if (debug.usedStrikePosePlan) {
    // The strike profile now arrives at max forward speed; match the terminal
    // tail-stamp to that forward-relative velocity so the held command is
    // continuous with the planned arrival (the stamp would otherwise zero it).
    const float vEnd = maxForwardSpeedMps(debug.targetHeadingDeg);
    const float r = debug.targetHeadingDeg * static_cast<float>(M_PI / 180.0);
    debug.offensePose.targetVxFieldMps = vEnd * std::sin(r);
    debug.offensePose.targetVyFieldMps = vEnd * std::cos(r);
    debug.offensePose.targetOmegaRadS = 0.f;
  }
  applyTerminalVelocity(debug.chunk, debug.offensePose.usesTerminalVelocity,
                        debug.offensePose.targetVxFieldMps, debug.offensePose.targetVyFieldMps,
                        debug.offensePose.targetOmegaRadS);
  fillStopChunkIfEmpty(debug.chunk);
  return debug;
}

PlannedChunk MotionPlanner::planToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                                       float headingDeg) {
  return debugPlanToPose(pose, goal, headingDeg).chunk;
}

PlannedChunk MotionPlanner::planDefense(const PoseState& pose, const BallState& ball,
                                        float headingDeg,
                                        const DefenseFieldTarget& defendedGoal) {
  return debugPlanDefense(pose, ball, headingDeg, defendedGoal).chunk;
}

DefensePlanDebug MotionPlanner::debugPlanDefense(const PoseState& pose, const BallState& ball,
                                                 float headingDeg,
                                                 const DefenseFieldTarget& defendedGoal) {
  DefensePlanDebug debug;
  debug.startPose = pose;
  debug.ball = ball;
  debug.defendedGoal = defendedGoal;
  debug.startHeadingDeg = headingDeg;
  debug.chunk.trajectoryId = nextTrajId();
  debug.chunk.dtMs = config::kChunkDtMs;
  debug.chunk.startTimePi = nowPiUs() + config::kSerialLatencyMarginUs;

  if (!pose.valid || !ball.visible) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  DefensePoseResult initialDefensePose =
      computeDefensePose(pose, ball, headingDeg, defendedGoal, {}, 0.f);
  if (!initialDefensePose.valid) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  PlannedChunk initialChunk = debug.chunk;
  std::vector<Waypoint3> initialWaypoints;
  std::vector<PathSample> initialPath;
  std::vector<ProfileSample> initialProfile;
  float astarCostS = 0.f;
  buildChunkToTarget(pose, headingDeg, initialDefensePose.targetXMm,
                     initialDefensePose.targetYMm, initialDefensePose.targetHeadingDeg, astar_,
                     spline_, profiler_, initialChunk, &initialWaypoints, &initialPath,
                     &initialProfile, nullptr, &astarCostS);
  const float pathTimeS = astarCostS > 0.f ? astarCostS : profileDurationS(initialProfile);

  debug.defensePose =
      computeDefensePose(pose, ball, headingDeg, defendedGoal, initialWaypoints, pathTimeS);
  if (!debug.defensePose.valid) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  debug.targetErrMm =
      std::hypot(debug.defensePose.targetXMm - pose.xMm, debug.defensePose.targetYMm - pose.yMm);
  debug.withinTargetTolerance = debug.targetErrMm <= params::get().commandGoalPositionToleranceMm;

  buildChunkToTarget(pose, headingDeg, debug.defensePose.targetXMm, debug.defensePose.targetYMm,
                     debug.defensePose.targetHeadingDeg, astar_, spline_, profiler_, debug.chunk,
                     &debug.waypoints, &debug.path, &debug.profile,
                     &debug.trajectorySpeedProfile);
  applyTerminalVelocity(debug.chunk, debug.defensePose.usesInterceptVelocity,
                        debug.defensePose.targetVxFieldMps, debug.defensePose.targetVyFieldMps,
                        debug.defensePose.targetOmegaRadS);
  fillStopChunkIfEmpty(debug.chunk);
  return debug;
}

CommandedPosePlanDebug MotionPlanner::debugPlanToPose(const PoseState& pose,
                                                      const CommandedPoseGoal& goal,
                                                      float headingDeg) {
  CommandedPosePlanDebug debug;
  debug.startPose = pose;
  debug.goal = goal;
  debug.startHeadingDeg = headingDeg;
  debug.chunk.trajectoryId = nextTrajId();
  debug.chunk.dtMs = config::kChunkDtMs;
  debug.chunk.startTimePi = nowPiUs() + config::kSerialLatencyMarginUs;

  if (!pose.valid) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  debug.posErrMm = std::hypot(goal.xMm - pose.xMm, goal.yMm - pose.yMm);
  debug.headingErrDeg = std::fabs(wrapAngleDeg(goal.headingDeg - headingDeg));
  debug.withinTolerance =
      debug.posErrMm <= params::get().commandGoalPositionToleranceMm &&
      debug.headingErrDeg <= params::get().commandGoalHeadingToleranceDeg;
  if (debug.withinTolerance) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  buildChunkToTarget(pose, headingDeg, goal.xMm, goal.yMm, goal.headingDeg, astar_, spline_,
                     profiler_, debug.chunk, &debug.waypoints, &debug.path, &debug.profile,
                     &debug.trajectorySpeedProfile);
  fillStopChunkIfEmpty(debug.chunk);
  return debug;
}

}  // namespace ballalgo

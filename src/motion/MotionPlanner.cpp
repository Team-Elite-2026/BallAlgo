#include "motion/MotionPlanner.hpp"

#include "config.hpp"
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
    const float aMax = motion::aMaxDir(profile[i].phi, config::kAMaxX, config::kAMaxY);
    durationS += std::sqrt(2.f * dsM / std::max(0.1f, aMax));
  }
  return durationS;
}

float initialOffenseInterceptTimeS(const BallState& ball) {
  const float maxPlanSpeedMps = std::max(config::kVMaxX, config::kVMaxY);
  const float travelTimeS =
      std::hypot(ball.xM, ball.yM) / std::max(0.05f, maxPlanSpeedMps);
  return std::clamp(travelTimeS, 0.f, config::kStrikeInterceptMaxTimeS);
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

void applyTerminalVelocity(PlannedChunk& chunk, const DefensePoseResult& defensePose,
                           float headingDeg) {
  if (!defensePose.usesInterceptVelocity || chunk.actions.empty()) return;
  float vxBody = 0.f;
  float vyBody = 0.f;
  fieldVelToBody(defensePose.targetVxFieldMps * 1000.f, defensePose.targetVyFieldMps * 1000.f,
                 headingDeg, vxBody, vyBody);
  const int terminalCount = std::min<int>(10, static_cast<int>(chunk.actions.size()));
  for (int i = static_cast<int>(chunk.actions.size()) - terminalCount;
       i < static_cast<int>(chunk.actions.size()); ++i) {
    chunk.actions[static_cast<size_t>(i)].vx = vxBody;
    chunk.actions[static_cast<size_t>(i)].vy = vyBody;
    chunk.actions[static_cast<size_t>(i)].omega = defensePose.targetOmegaRadS;
    chunk.actions[static_cast<size_t>(i)].ax = 0.f;
    chunk.actions[static_cast<size_t>(i)].ay = 0.f;
    chunk.actions[static_cast<size_t>(i)].alpha = 0.f;
  }
}

void applyTerminalVelocity(PlannedChunk& chunk, const OffensePoseResult& offensePose,
                           float headingDeg) {
  if (!offensePose.usesTerminalVelocity || chunk.actions.empty()) return;
  float vxBody = 0.f;
  float vyBody = 0.f;
  fieldVelToBody(offensePose.targetVxFieldMps * 1000.f, offensePose.targetVyFieldMps * 1000.f,
                 headingDeg, vxBody, vyBody);
  const int terminalCount = std::min<int>(10, static_cast<int>(chunk.actions.size()));
  for (int i = static_cast<int>(chunk.actions.size()) - terminalCount;
       i < static_cast<int>(chunk.actions.size()); ++i) {
    chunk.actions[static_cast<size_t>(i)].vx = vxBody;
    chunk.actions[static_cast<size_t>(i)].vy = vyBody;
    chunk.actions[static_cast<size_t>(i)].omega = offensePose.targetOmegaRadS;
    chunk.actions[static_cast<size_t>(i)].ax = 0.f;
    chunk.actions[static_cast<size_t>(i)].ay = 0.f;
    chunk.actions[static_cast<size_t>(i)].alpha = 0.f;
  }
}

void buildChunkToTarget(const PoseState& pose, float headingDeg, float goalXMm, float goalYMm,
                        float goalHeadingDeg, AStar3D& astar, HermiteSpline& spline,
                        VelocityProfile& profiler, PlannedChunk& chunk,
                        std::vector<Waypoint3>* waypoints, std::vector<PathSample>* path,
                        std::vector<ProfileSample>* profile, float* astarCostS = nullptr) {
  int st = headingToBin(headingDeg, config::kAstarHeadingBins);
  int gt = headingToBin(goalHeadingDeg, config::kAstarHeadingBins);

  std::vector<Waypoint3> localWaypoints;
  std::vector<PathSample> localPath;
  std::vector<ProfileSample> localProfile;

  std::vector<Waypoint3>& waypointsRef = waypoints != nullptr ? *waypoints : localWaypoints;
  std::vector<PathSample>& pathRef = path != nullptr ? *path : localPath;
  std::vector<ProfileSample>& profileRef = profile != nullptr ? *profile : localProfile;

  astar.plan(pose.xMm, pose.yMm, st, goalXMm, goalYMm, gt, waypointsRef, astarCostS);
  pathRef = spline.fit(waypointsRef, goalHeadingDeg);

  float vStart = 0;
  if (pathRef.size() >= 2) {
    float phi0 = std::atan2(pathRef[1].yMm - pathRef[0].yMm, pathRef[1].xMm - pathRef[0].xMm);
    const float vxField = pose.vxMmS / 1000.f;
    const float vyField = pose.vyMmS / 1000.f;
    vStart = vxField * std::cos(phi0) + vyField * std::sin(phi0);
  }

  profileRef = profiler.build(pathRef, std::max(0.f, vStart));
  chunk.actions =
      profiler.discretize(profileRef, pathRef, headingDeg, chunk.dtMs, config::kChunkMaxActions);
}

}  // namespace

MotionPlanner::MotionPlanner()
    : astar_(config::kFieldWidthMm, config::kFieldHeightMm, config::kAstarCellMm,
             config::kAstarHeadingBins) {}

uint64_t MotionPlanner::nextTrajId() { return ++trajId_; }

static uint64_t nowPiUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

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
          debug.targetErrMm <= config::kCommandGoalPositionToleranceMm;
      buildChunkToTarget(pose, headingDeg, debug.targetXMm, debug.targetYMm, debug.targetHeadingDeg,
                         astar_, spline_, profiler_, debug.chunk, &debug.waypoints, &debug.path,
                         &debug.profile);
    }
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  if (!fullPlanner || !pose.valid) {
    debug.usedBodyChaseFallback = true;
    debug.targetErrMm = std::hypot(ball.xM * 1000.f, ball.yM * 1000.f);
    debug.withinTargetTolerance =
        debug.targetErrMm <= config::kCommandGoalPositionToleranceMm;
    float dist = std::hypot(ball.xM, ball.yM);
    float sp = std::min(0.3f, dist * 0.5f);
    MotionAction action{};
    if (dist > 1e-3f) {
      action.vx = sp * ball.xM / dist;
      action.vy = sp * ball.yM / dist;
    }
    debug.chunk.actions.assign(config::kChunkMaxActions, action);
    return debug;
  }

  const bool hasGoalFieldTarget = goalFieldTarget != nullptr;
  const float goalFieldXMm = hasGoalFieldTarget ? goalFieldTarget->xMm : 0.f;
  const float goalFieldYMm = hasGoalFieldTarget ? goalFieldTarget->yMm : 0.f;
  float interceptTimeS = initialOffenseInterceptTimeS(ball);

  for (int iteration = 0; iteration < config::kStrikeInterceptMaxIterations; ++iteration) {
    const OffensePoseResult offensePose =
        computeOffensePose(pose, ball, goalDeg, headingDeg, hasGoalFieldTarget, goalFieldXMm,
                           goalFieldYMm, interceptTimeS);
    debug.interceptTimeS = interceptTimeS;
    debug.interceptIterations = iteration + 1;
    fillBallDebugFromOffensePose(debug, offensePose);
    debug.targetErrMm = std::hypot(debug.targetXMm - pose.xMm, debug.targetYMm - pose.yMm);
    debug.withinTargetTolerance =
        debug.targetErrMm <= config::kCommandGoalPositionToleranceMm;

    float astarCostS = 0.f;
    buildChunkToTarget(pose, headingDeg, debug.targetXMm, debug.targetYMm, debug.targetHeadingDeg,
                       astar_, spline_, profiler_, debug.chunk, &debug.waypoints, &debug.path,
                       &debug.profile, &astarCostS);
    const float pathTimeS = astarCostS > 0.f ? astarCostS : profileDurationS(debug.profile);
    const float nextInterceptTimeS =
        std::clamp(pathTimeS, 0.f, config::kStrikeInterceptMaxTimeS);
    if (std::fabs(nextInterceptTimeS - interceptTimeS) <= config::kStrikeInterceptConvergeS ||
        pathTimeS >= config::kStrikeInterceptMaxTimeS ||
        iteration + 1 == config::kStrikeInterceptMaxIterations) {
      break;
    }
    interceptTimeS = nextInterceptTimeS;
  }
  applyTerminalVelocity(debug.chunk, debug.offensePose, headingDeg);
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
                     &initialProfile, &astarCostS);
  const float pathTimeS = astarCostS > 0.f ? astarCostS : profileDurationS(initialProfile);

  debug.defensePose =
      computeDefensePose(pose, ball, headingDeg, defendedGoal, initialWaypoints, pathTimeS);
  if (!debug.defensePose.valid) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  debug.targetErrMm =
      std::hypot(debug.defensePose.targetXMm - pose.xMm, debug.defensePose.targetYMm - pose.yMm);
  debug.withinTargetTolerance = debug.targetErrMm <= config::kCommandGoalPositionToleranceMm;

  buildChunkToTarget(pose, headingDeg, debug.defensePose.targetXMm, debug.defensePose.targetYMm,
                     debug.defensePose.targetHeadingDeg, astar_, spline_, profiler_, debug.chunk,
                     &debug.waypoints, &debug.path, &debug.profile);
  applyTerminalVelocity(debug.chunk, debug.defensePose, headingDeg);
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
      debug.posErrMm <= config::kCommandGoalPositionToleranceMm &&
      debug.headingErrDeg <= config::kCommandGoalHeadingToleranceDeg;
  if (debug.withinTolerance) {
    fillStopChunkIfEmpty(debug.chunk);
    return debug;
  }

  buildChunkToTarget(pose, headingDeg, goal.xMm, goal.yMm, goal.headingDeg, astar_, spline_,
                     profiler_, debug.chunk, &debug.waypoints, &debug.path, &debug.profile);
  fillStopChunkIfEmpty(debug.chunk);
  return debug;
}

}  // namespace ballalgo

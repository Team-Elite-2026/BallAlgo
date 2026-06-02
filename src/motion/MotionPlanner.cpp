#include "motion/MotionPlanner.hpp"

#include "config.hpp"
#include "motion/StrikePose.hpp"

#include <chrono>
#include <cmath>

namespace ballalgo {

namespace {

int headingToBin(float angleDeg, int bins) {
  const float wrapped = std::fmod(std::fmod(angleDeg, 360.f) + 360.f, 360.f);
  const float stepDeg = 360.f / static_cast<float>(bins);
  return static_cast<int>(wrapped / stepDeg) % bins;
}

float headingBetweenPointsDeg(float fromX, float fromY, float toX, float toY) {
  return std::atan2(toY - fromY, toX - fromX) * 180.f / static_cast<float>(M_PI);
}

float wrapAngleDeg(float angleDeg) {
  float wrapped = std::fmod(angleDeg + 180.f, 360.f);
  if (wrapped < 0) wrapped += 360.f;
  return wrapped - 180.f;
}

void fillStopChunkIfEmpty(PlannedChunk& chunk) {
  if (!chunk.actions.empty()) return;
  MotionAction stop{};
  chunk.actions.assign(config::kChunkMaxActions, stop);
}

void buildChunkToTarget(const PoseState& pose, float headingDeg, float goalXMm, float goalYMm,
                        float goalHeadingDeg, AStar3D& astar, HermiteSpline& spline,
                        VelocityProfile& profiler, PlannedChunk& chunk,
                        std::vector<Waypoint3>* waypoints, std::vector<PathSample>* path,
                        std::vector<ProfileSample>* profile) {
  int st = headingToBin(headingDeg, config::kAstarHeadingBins);
  int gt = headingToBin(goalHeadingDeg, config::kAstarHeadingBins);

  std::vector<Waypoint3> localWaypoints;
  std::vector<PathSample> localPath;
  std::vector<ProfileSample> localProfile;

  std::vector<Waypoint3>& waypointsRef = waypoints != nullptr ? *waypoints : localWaypoints;
  std::vector<PathSample>& pathRef = path != nullptr ? *path : localPath;
  std::vector<ProfileSample>& profileRef = profile != nullptr ? *profile : localProfile;

  astar.plan(pose.xMm, pose.yMm, st, goalXMm, goalYMm, gt, waypointsRef);
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
  return debugPlan(pose, ball, goalDeg, headingDeg, fullPlanner).chunk;
}

BallPlanDebug MotionPlanner::debugPlan(const PoseState& pose, const BallState& ball, float goalDeg,
                                       float headingDeg, bool fullPlanner) {
  BallPlanDebug debug;
  debug.startPose = pose;
  debug.ball = ball;
  debug.goalDeg = goalDeg;
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

  debug.usedStrikePosePlan = true;
  strikePoseBody(ball.xM, ball.yM, goalDeg, debug.strikeTargetBodyXM, debug.strikeTargetBodyYM);
  ballFieldMm(pose.xMm, pose.yMm, debug.strikeTargetBodyXM, debug.strikeTargetBodyYM, headingDeg,
              debug.targetXMm, debug.targetYMm);
  ballFieldMm(pose.xMm, pose.yMm, ball.xM, ball.yM, headingDeg, debug.ballFieldXMm,
              debug.ballFieldYMm);
  debug.targetHeadingDeg =
      headingBetweenPointsDeg(debug.targetXMm, debug.targetYMm, debug.ballFieldXMm, debug.ballFieldYMm);
  debug.targetErrMm = std::hypot(debug.targetXMm - pose.xMm, debug.targetYMm - pose.yMm);
  debug.withinTargetTolerance = debug.targetErrMm <= config::kCommandGoalPositionToleranceMm;
  buildChunkToTarget(pose, headingDeg, debug.targetXMm, debug.targetYMm, debug.targetHeadingDeg,
                     astar_, spline_, profiler_, debug.chunk, &debug.waypoints, &debug.path,
                     &debug.profile);
  fillStopChunkIfEmpty(debug.chunk);
  return debug;
}

PlannedChunk MotionPlanner::planToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                                       float headingDeg) {
  return debugPlanToPose(pose, goal, headingDeg).chunk;
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

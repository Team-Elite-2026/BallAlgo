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
  PlannedChunk chunk;
  chunk.trajectoryId = nextTrajId();
  chunk.dtMs = config::kChunkDtMs;
  chunk.startTimePi = nowPiUs() + config::kSerialLatencyMarginUs;

  auto buildChunkToTarget = [&](float gx, float gy, float goalHeadingDeg) {
    int st = headingToBin(headingDeg, config::kAstarHeadingBins);
    int gt = headingToBin(goalHeadingDeg, config::kAstarHeadingBins);
    std::vector<Waypoint3> wps;
    astar_.plan(pose.xMm, pose.yMm, st, gx, gy, gt, wps);
    auto path = spline_.fit(wps, goalHeadingDeg);
    float vStart = 0;
    if (path.size() >= 2) {
      float phi0 = std::atan2(path[1].yMm - path[0].yMm, path[1].xMm - path[0].xMm);
      const float vxField = pose.vxMmS / 1000.f;
      const float vyField = pose.vyMmS / 1000.f;
      vStart = vxField * std::cos(phi0) + vyField * std::sin(phi0);
    }
    auto prof = profiler_.build(path, std::max(0.f, vStart));
    chunk.actions =
        profiler_.discretize(prof, path, headingDeg, chunk.dtMs, config::kChunkMaxActions);
  };

  if (!ball.visible) {
    if (fullPlanner && pose.valid) {
      buildChunkToTarget(config::kFieldWidthMm * 0.5f, config::kFieldHeightMm * 0.5f, headingDeg);
    }
    fillStopChunkIfEmpty(chunk);
    return chunk;
  }

  if (!fullPlanner || !pose.valid) {
    float dist = std::hypot(ball.xM, ball.yM);
    float sp = std::min(0.3f, dist * 0.5f);
    MotionAction a{};
    if (dist > 1e-3f) {
      a.vx = sp * ball.xM / dist;
      a.vy = sp * ball.yM / dist;
    }
    chunk.actions.assign(config::kChunkMaxActions, a);
    return chunk;
  }

  float tx, ty;
  strikePoseBody(ball.xM, ball.yM, goalDeg, tx, ty);
  float goalXMm = 0;
  float goalYMm = 0;
  float ballXMm = 0;
  float ballYMm = 0;
  ballFieldMm(pose.xMm, pose.yMm, tx, ty, headingDeg, goalXMm, goalYMm);
  ballFieldMm(pose.xMm, pose.yMm, ball.xM, ball.yM, headingDeg, ballXMm, ballYMm);
  buildChunkToTarget(goalXMm, goalYMm,
                     headingBetweenPointsDeg(goalXMm, goalYMm, ballXMm, ballYMm));
  fillStopChunkIfEmpty(chunk);
  return chunk;
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

  int st = headingToBin(headingDeg, config::kAstarHeadingBins);
  int gt = headingToBin(goal.headingDeg, config::kAstarHeadingBins);
  astar_.plan(pose.xMm, pose.yMm, st, goal.xMm, goal.yMm, gt, debug.waypoints);
  debug.path = spline_.fit(debug.waypoints, goal.headingDeg);
  float vStart = 0;
  if (debug.path.size() >= 2) {
    float phi0 = std::atan2(debug.path[1].yMm - debug.path[0].yMm,
                            debug.path[1].xMm - debug.path[0].xMm);
    const float vxField = pose.vxMmS / 1000.f;
    const float vyField = pose.vyMmS / 1000.f;
    vStart = vxField * std::cos(phi0) + vyField * std::sin(phi0);
  }
  debug.profile = profiler_.build(debug.path, std::max(0.f, vStart));
  debug.chunk.actions = profiler_.discretize(debug.profile, debug.path, headingDeg,
                                             debug.chunk.dtMs, config::kChunkMaxActions);
  fillStopChunkIfEmpty(debug.chunk);
  return debug;
}

}  // namespace ballalgo

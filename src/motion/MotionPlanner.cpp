#include "motion/MotionPlanner.hpp"

#include "config.hpp"
#include "motion/StrikePose.hpp"

#include <chrono>

namespace ballalgo {

namespace {

int headingToBin(float angleDeg, int bins) {
  const float wrapped = std::fmod(std::fmod(angleDeg, 360.f) + 360.f, 360.f);
  const float stepDeg = 360.f / static_cast<float>(bins);
  return static_cast<int>(wrapped / stepDeg) % bins;
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
                               float headingDeg, int latencyUs, bool fullPlanner) {
  PlannedChunk chunk;
  chunk.trajectoryId = nextTrajId();
  chunk.dtMs = config::kChunkDtMs;
  chunk.startTimePi = nowPiUs() + latencyUs + config::kSerialLatencyMarginUs;

  if (!ball.visible) {
    MotionAction z;
    chunk.actions.assign(config::kChunkMaxActions, z);
    return chunk;
  }

  if (!fullPlanner || !pose.valid) {
    float dist = std::hypot(ball.xM, ball.yM);
    float sp = std::min(0.3f, dist * 0.5f);
    MotionAction a;
    if (dist > 1e-3f) {
      a.vx = sp * ball.xM / dist;
      a.vy = sp * ball.yM / dist;
    }
    chunk.actions.assign(config::kChunkMaxActions, a);
    return chunk;
  }

  float tx, ty;
  strikePoseBody(ball.xM, ball.yM, goalDeg, tx, ty);
  float gx, gy;
  ballFieldMm(pose.xMm, pose.yMm, tx, ty, headingDeg, gx, gy);
  int st = headingToBin(headingDeg, config::kAstarHeadingBins);
  int gt = headingToBin(goalDeg, config::kAstarHeadingBins);
  std::vector<Waypoint3> wps;
  astar_.plan(pose.xMm, pose.yMm, st, gx, gy, gt, wps);
  auto path = spline_.fit(wps, goalDeg);
  float vStart = 0;
  if (path.size() >= 2) {
    float phi0 = std::atan2(path[1].yMm - path[0].yMm, path[1].xMm - path[0].xMm);
    vStart = pose.vxBody * std::cos(phi0) + pose.vyBody * std::sin(phi0);
  }
  auto prof = profiler_.build(path, std::max(0.f, vStart));
  chunk.actions = profiler_.discretize(prof, path, headingDeg, chunk.dtMs, config::kChunkMaxActions);
  return chunk;
}

}  // namespace ballalgo

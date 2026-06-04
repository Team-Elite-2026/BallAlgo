#include "motion/ActionChunkPublisher.hpp"

#include "config.hpp"
#include "motion/Protocol.hpp"

#include <chrono>

namespace ballalgo {

namespace {

PlannerDebugSnapshot makeSnapshot(const CommandedPosePlanDebug& debug) {
  PlannerDebugSnapshot snapshot;
  snapshot.valid = true;
  snapshot.commandedGoalMode = true;
  snapshot.withinTolerance = debug.withinTolerance;
  snapshot.trajectoryId = debug.chunk.trajectoryId;
  snapshot.startTimePi = debug.chunk.startTimePi;
  snapshot.dtMs = debug.chunk.dtMs;
  snapshot.targetXMm = debug.goal.xMm;
  snapshot.targetYMm = debug.goal.yMm;
  snapshot.targetHeadingDeg = debug.goal.headingDeg;
  snapshot.path = debug.path;
  return snapshot;
}

PlannerDebugSnapshot makeSnapshot(const BallPlanDebug& debug) {
  PlannerDebugSnapshot snapshot;
  snapshot.valid = true;
  snapshot.commandedGoalMode = false;
  snapshot.withinTolerance = debug.withinTargetTolerance;
  snapshot.usedCenterFallback = debug.usedCenterFallback;
  snapshot.usedBodyChaseFallback = debug.usedBodyChaseFallback;
  snapshot.usedStrikePosePlan = debug.usedStrikePosePlan;
  snapshot.trajectoryId = debug.chunk.trajectoryId;
  snapshot.startTimePi = debug.chunk.startTimePi;
  snapshot.dtMs = debug.chunk.dtMs;
  snapshot.targetXMm = debug.targetXMm;
  snapshot.targetYMm = debug.targetYMm;
  snapshot.targetHeadingDeg = debug.targetHeadingDeg;
  snapshot.path = debug.path;
  return snapshot;
}

}  // namespace

bool ActionChunkPublisher::publish(RobotSerial& serial, std::vector<uint8_t>& rx,
                                   const PoseState& pose, const BallState& ball, float goalDeg,
                                   float headingDeg, bool offenseActive,
                                   const CommandedPoseGoal* commandedGoal) {
  if (!config::kEnableActionChunks || (!offenseActive && commandedGoal == nullptr)) {
    latestDebug_ = {};
    return true;
  }
  serial.readSome(rx);
  clock_.processBuffer(serial, rx);
  auto now = std::chrono::steady_clock::now();
  double t = std::chrono::duration<double>(now.time_since_epoch()).count();
  if (t - lastPublish_ < 1.0 / config::kChunkPublishHz) return true;
  lastPublish_ = t;
  PlannedChunk chunk;
  if (commandedGoal != nullptr) {
    auto debug = planner_.debugPlanToPose(pose, *commandedGoal, headingDeg);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  } else {
    auto debug = planner_.debugPlan(pose, ball, goalDeg, headingDeg, pose.valid);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  }
  auto pkt = packActionChunk(chunk.trajectoryId, chunk.startTimePi, chunk.dtMs, chunk.actions,
                             static_cast<int>(chunk.actions.size()), pose.vxBody, pose.vyBody,
                             pose.valid);
  return serial.write(pkt);
}

}  // namespace ballalgo

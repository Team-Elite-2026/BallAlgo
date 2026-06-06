#include "motion/ActionChunkPublisher.hpp"

#include "config.hpp"
#include "motion/Protocol.hpp"
#include "vision/VisionMath.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ballalgo {

namespace {

uint64_t nowPiUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

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

ActionChunkPublisher::PredictedState ActionChunkPublisher::predictChunkState(
    uint64_t queryPi) const {
  PredictedState st;
  st.xMm = lastChunk_.startXMm;
  st.yMm = lastChunk_.startYMm;
  if (!lastChunk_.valid || lastChunk_.actions.empty() || lastChunk_.dtMs == 0) return st;

  const double dtS = lastChunk_.dtMs / 1000.0;
  // Integrate global velocities from the chunk start up to the query time.
  double tElapsed =
      (queryPi > lastChunk_.startTimePi)
          ? static_cast<double>(queryPi - lastChunk_.startTimePi) / 1e6
          : 0.0;
  double x = lastChunk_.startXMm;
  double y = lastChunk_.startYMm;
  const int n = static_cast<int>(lastChunk_.actions.size());
  double tAcc = 0.0;
  int idx = 0;
  for (; idx < n && tAcc + dtS <= tElapsed; ++idx) {
    x += lastChunk_.actions[idx].vx * 1000.0 * dtS;  // m/s -> mm/s
    y += lastChunk_.actions[idx].vy * 1000.0 * dtS;
    tAcc += dtS;
  }
  // Partial last step.
  if (idx < n && tElapsed > tAcc) {
    const double partial = tElapsed - tAcc;
    x += lastChunk_.actions[idx].vx * 1000.0 * partial;
    y += lastChunk_.actions[idx].vy * 1000.0 * partial;
  }
  const int curIdx = std::min(idx, n - 1);
  st.xMm = static_cast<float>(x);
  st.yMm = static_cast<float>(y);
  st.vxMmS = lastChunk_.actions[curIdx].vx * 1000.0f;
  st.vyMmS = lastChunk_.actions[curIdx].vy * 1000.0f;
  return st;
}

PoseState ActionChunkPublisher::computeStartState(const PoseState& ekf, float headingDeg,
                                                  uint64_t nowPi,
                                                  uint64_t& startTimePiOut) const {
  const uint64_t tLookup = nowPi + config::kPipelineLatencyUs;
  startTimePiOut = tLookup;
  const float latS = static_cast<float>(config::kPipelineLatencyUs) / 1e6f;

  PoseState start = ekf;
  if (!lastChunk_.valid || !ekf.valid) {
    // No executing trajectory: dead-reckon the EKF state forward by the latency.
    start.xMm = ekf.xMm + ekf.vxMmS * latS;
    start.yMm = ekf.yMm + ekf.vyMmS * latS;
    return start;
  }

  const PredictedState predCurr = predictChunkState(nowPi);
  const PredictedState predLook = predictChunkState(tLookup);

  // Tracking error between the live EKF and where the chunk thinks we are now.
  const float e = std::hypot(ekf.xMm - predCurr.xMm, ekf.yMm - predCurr.yMm);
  const float ratio = e / config::kTrackingSigmaMm;
  const float alpha = std::exp(-ratio * ratio);

  start.xMm = alpha * predLook.xMm + (1.f - alpha) * (ekf.xMm + ekf.vxMmS * latS);
  start.yMm = alpha * predLook.yMm + (1.f - alpha) * (ekf.yMm + ekf.vyMmS * latS);
  start.vxMmS = alpha * predLook.vxMmS + (1.f - alpha) * ekf.vxMmS;
  start.vyMmS = alpha * predLook.vyMmS + (1.f - alpha) * ekf.vyMmS;
  fieldVelToBody(start.vxMmS, start.vyMmS, headingDeg, start.vxBody, start.vyBody);
  return start;
}

void ActionChunkPublisher::recordChunk(const PlannedChunk& chunk, const PoseState& startPose,
                                       float headingDeg) {
  lastChunk_.valid = true;
  lastChunk_.actions = chunk.actions;
  lastChunk_.dtMs = chunk.dtMs;
  lastChunk_.startTimePi = chunk.startTimePi;
  lastChunk_.startXMm = startPose.xMm;
  lastChunk_.startYMm = startPose.yMm;
  lastChunk_.startHeadingDeg = headingDeg;
}

bool ActionChunkPublisher::publish(RobotSerial& serial, std::vector<uint8_t>& rx,
                                   const PoseState& pose, const BallState& ball, float goalDeg,
                                   float headingDeg, bool offenseActive,
                                   const CommandedPoseGoal* commandedGoal) {
  if (!config::kEnableActionChunks || (!offenseActive && commandedGoal == nullptr)) {
    latestDebug_ = {};
    lastChunk_.valid = false;
    return true;
  }
  serial.readSome(rx);
  clock_.processBuffer(serial, rx);
  auto now = std::chrono::steady_clock::now();
  double t = std::chrono::duration<double>(now.time_since_epoch()).count();
  if (t - lastPublish_ < 1.0 / config::kChunkPublishHz) return true;
  lastPublish_ = t;

  // Step 2: roll the executing trajectory forward to the look-ahead horizon and
  // blend with the live EKF state to obtain the planner start node.
  uint64_t startTimePi = 0;
  const PoseState startPose = computeStartState(pose, headingDeg, nowPiUs(), startTimePi);

  PlannedChunk chunk;
  if (commandedGoal != nullptr) {
    auto debug = planner_.debugPlanToPose(startPose, *commandedGoal, headingDeg);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  } else {
    auto debug = planner_.debugPlan(startPose, ball, goalDeg, headingDeg, startPose.valid);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  }
  // Stamp execution start at the look-ahead horizon (plan begins from the
  // predicted future state, not "now").
  chunk.startTimePi = startTimePi;
  latestDebug_.startTimePi = startTimePi;

  recordChunk(chunk, startPose, headingDeg);

  auto pkt = packActionChunk(chunk.trajectoryId, chunk.startTimePi, chunk.dtMs, chunk.actions,
                             static_cast<int>(chunk.actions.size()), pose.vxBody, pose.vyBody,
                             pose.valid);
  return serial.write(pkt);
}

}  // namespace ballalgo

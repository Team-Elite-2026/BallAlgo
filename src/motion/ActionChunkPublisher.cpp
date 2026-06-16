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
  snapshot.defenseMode = false;
  snapshot.withinTolerance = debug.withinTolerance;
  snapshot.trajectoryId = debug.chunk.trajectoryId;
  snapshot.startTimePi = debug.chunk.startTimePi;
  snapshot.dtMs = debug.chunk.dtMs;
  snapshot.targetXMm = debug.goal.xMm;
  snapshot.targetYMm = debug.goal.yMm;
  snapshot.targetHeadingDeg = debug.goal.headingDeg;
  snapshot.path = debug.path;
  snapshot.trajectorySpeedProfile = debug.trajectorySpeedProfile;
  return snapshot;
}

PlannerDebugSnapshot makeSnapshot(const BallPlanDebug& debug) {
  PlannerDebugSnapshot snapshot;
  snapshot.valid = true;
  snapshot.commandedGoalMode = false;
  snapshot.defenseMode = false;
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
  snapshot.trajectorySpeedProfile = debug.trajectorySpeedProfile;
  return snapshot;
}

PlannerDebugSnapshot makeSnapshot(const DefensePlanDebug& debug) {
  PlannerDebugSnapshot snapshot;
  snapshot.valid = true;
  snapshot.commandedGoalMode = false;
  snapshot.defenseMode = true;
  snapshot.withinTolerance = debug.withinTargetTolerance;
  snapshot.trajectoryId = debug.chunk.trajectoryId;
  snapshot.startTimePi = debug.chunk.startTimePi;
  snapshot.dtMs = debug.chunk.dtMs;
  snapshot.targetXMm = debug.defensePose.targetXMm;
  snapshot.targetYMm = debug.defensePose.targetYMm;
  snapshot.targetHeadingDeg = debug.defensePose.targetHeadingDeg;
  snapshot.path = debug.path;
  snapshot.trajectorySpeedProfile = debug.trajectorySpeedProfile;
  return snapshot;
}

struct ActuatorCommand {
  uint8_t kick = 0;
  uint8_t dribblerPower = 0;
};

ActuatorCommand chooseActuatorCommand(const PoseState& pose, const BallState& ball,
                                      float headingDeg, bool offenseActive, bool hasBall,
                                      const FieldTarget* offenseGoalFieldTarget,
                                      uint64_t nowPiUs, uint64_t& lastKickPiUs) {
  ActuatorCommand command;
  if (!offenseActive) return command;

  const float ballDistanceMm = std::hypot(ball.xM, ball.yM) * 1000.f;
  if (hasBall || (ball.visible && ballDistanceMm <= config::kDribblerCaptureDistMm)) {
    command.dribblerPower = config::kDribblerActivePower;
  }

  if (!hasBall || !pose.valid || offenseGoalFieldTarget == nullptr) return command;
  if (lastKickPiUs != 0 && nowPiUs - lastKickPiUs < config::kKickCooldownUs) return command;

  float goalBodyXMm = 0.f;
  float goalBodyYMm = 0.f;
  fieldToBodyOffsetMm(offenseGoalFieldTarget->xMm - pose.xMm,
                      offenseGoalFieldTarget->yMm - pose.yMm, headingDeg, goalBodyXMm,
                      goalBodyYMm);
  if (goalBodyYMm < config::kKickMinGoalForwardMm) return command;

  const float goalAngleDeg =
      std::atan2(goalBodyXMm, goalBodyYMm) * 180.f / static_cast<float>(M_PI);
  if (std::fabs(goalAngleDeg) > config::kKickAimToleranceDeg) return command;

  command.kick = 1u;
  lastKickPiUs = nowPiUs;
  return command;
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
  lastChunk_.kick = chunk.kick;
  lastChunk_.dribblerPower = chunk.dribblerPower;
}

TrajectoryTargetSample ActionChunkPublisher::currentTargetSample(uint64_t queryPiUs,
                                                                 float headingDeg) const {
  if (!lastChunk_.valid) return {};
  return sampleTrajectoryTarget(lastChunk_.actions, latestDebug_.trajectoryId,
                                lastChunk_.startTimePi, lastChunk_.dtMs, headingDeg, queryPiUs,
                                lastChunk_.kick, lastChunk_.dribblerPower);
}

bool ActionChunkPublisher::publish(RobotSerial& serial, const PoseState& pose, const BallState& ball,
                                   float goalDeg, float headingDeg, bool offenseActive,
                                   bool hasBall, const FieldTarget* offenseGoalFieldTarget,
                                   const DefenseFieldTarget* defendedGoal,
                                   const CommandedPoseGoal* commandedGoal) {
  if (!config::kEnableActionChunks ||
      (!offenseActive && defendedGoal == nullptr && commandedGoal == nullptr)) {
    latestDebug_ = {};
    lastChunk_.valid = false;
    return true;
  }
  std::vector<ProtocolFrame> frames;
  serial.takePendingFrames(frames);
  clock_.processFrames(serial, frames);
  auto now = std::chrono::steady_clock::now();
  double t = std::chrono::duration<double>(now.time_since_epoch()).count();
  if (t - lastPublish_ < 1.0 / config::kChunkPublishHz) return true;
  lastPublish_ = t;

  // Step 2: roll the executing trajectory forward to the look-ahead horizon and
  // blend with the live EKF state to obtain the planner start node.
  const uint64_t nowPi = nowPiUs();
  uint64_t startTimePi = 0;
  const PoseState startPose = computeStartState(pose, headingDeg, nowPi, startTimePi);

  PlannedChunk chunk;
  if (commandedGoal != nullptr) {
    auto debug = planner_.debugPlanToPose(startPose, *commandedGoal, headingDeg);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  } else if (!offenseActive && defendedGoal != nullptr) {
    auto debug = planner_.debugPlanDefense(startPose, ball, headingDeg, *defendedGoal);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  } else {
    auto debug =
        planner_.debugPlan(startPose, ball, goalDeg, headingDeg, startPose.valid,
                           offenseGoalFieldTarget);
    latestDebug_ = makeSnapshot(debug);
    chunk = std::move(debug.chunk);
  }
  // Stamp execution start at the look-ahead horizon (plan begins from the
  // predicted future state, not "now").
  chunk.startTimePi = startTimePi;
  latestDebug_.startTimePi = startTimePi;
  const ActuatorCommand actuatorCommand =
      chooseActuatorCommand(startPose, ball, headingDeg, offenseActive, hasBall,
                            offenseGoalFieldTarget, nowPi, lastKickPiUs_);
  chunk.kick = actuatorCommand.kick;
  chunk.dribblerPower = actuatorCommand.dribblerPower;

  recordChunk(chunk, startPose, headingDeg);

  auto pkt = packActionChunk(chunk.trajectoryId, chunk.startTimePi, chunk.dtMs, chunk.actions,
                             static_cast<int>(chunk.actions.size()), pose.vxBody, pose.vyBody,
                             pose.valid, chunk.kick, chunk.dribblerPower);
  return serial.write(pkt);
}

}  // namespace ballalgo

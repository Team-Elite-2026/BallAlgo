#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/RobotSerial.hpp"
#include "motion/ClockSync.hpp"
#include "motion/MotionPlanner.hpp"

#include <vector>

namespace ballalgo {

struct PlannerDebugSnapshot {
  bool valid = false;
  bool commandedGoalMode = false;
  bool defenseMode = false;
  bool withinTolerance = false;
  bool usedCenterFallback = false;
  bool usedBodyChaseFallback = false;
  bool usedStrikePosePlan = false;
  uint64_t trajectoryId = 0;
  uint64_t startTimePi = 0;
  uint16_t dtMs = 0;
  float targetXMm = 0;
  float targetYMm = 0;
  float targetHeadingDeg = 0;
  std::vector<PathSample> path;
  std::vector<TrajectorySpeedSample> trajectorySpeedProfile;
};

class ActionChunkPublisher {
 public:
  bool publish(RobotSerial& serial, const PoseState& pose, const BallState& ball, float goalDeg,
               float headingDeg, bool offenseActive,
               const DefenseFieldTarget* defendedGoal = nullptr,
               const CommandedPoseGoal* commandedGoal = nullptr);
  const PlannerDebugSnapshot& latestDebugSnapshot() const { return latestDebug_; }

 private:
  // Step 2: snapshot of the last chunk we sent, used to roll the executing
  // trajectory forward to the look-ahead horizon when (re)planning.
  struct LastChunk {
    bool valid = false;
    std::vector<MotionAction> actions;  // GLOBAL-frame targets
    uint16_t dtMs = 0;
    uint64_t startTimePi = 0;
    float startXMm = 0;
    float startYMm = 0;
    float startHeadingDeg = 0;
  };

  // Integrated global state predicted from the executing chunk at a query time.
  struct PredictedState {
    float xMm = 0;
    float yMm = 0;
    float vxMmS = 0;
    float vyMmS = 0;
  };

  PredictedState predictChunkState(uint64_t queryPi) const;

  // Returns the look-ahead-blended start pose to feed the planner and the
  // absolute Pi time at which the new chunk should begin executing.
  PoseState computeStartState(const PoseState& ekf, float headingDeg, uint64_t nowPi,
                              uint64_t& startTimePiOut) const;

  void recordChunk(const PlannedChunk& chunk, const PoseState& startPose, float headingDeg);

  ClockSync clock_;
  MotionPlanner planner_;
  double lastPublish_ = 0;
  PlannerDebugSnapshot latestDebug_;
  LastChunk lastChunk_;
};

}  // namespace ballalgo

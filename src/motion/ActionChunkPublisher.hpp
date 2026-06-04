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
};

class ActionChunkPublisher {
 public:
  bool publish(RobotSerial& serial, std::vector<uint8_t>& rx, const PoseState& pose,
               const BallState& ball, float goalDeg, float headingDeg, bool offenseActive,
               const CommandedPoseGoal* commandedGoal = nullptr);
  const PlannerDebugSnapshot& latestDebugSnapshot() const { return latestDebug_; }

 private:
  ClockSync clock_;
  MotionPlanner planner_;
  double lastPublish_ = 0;
  PlannerDebugSnapshot latestDebug_;
};

}  // namespace ballalgo

#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/RobotSerial.hpp"
#include "motion/ClockSync.hpp"
#include "motion/MotionPlanner.hpp"

#include <vector>

namespace ballalgo {

class ActionChunkPublisher {
 public:
  bool publish(RobotSerial& serial, std::vector<uint8_t>& rx, const PoseState& pose,
               const BallState& ball, float goalDeg, float headingDeg, bool offenseActive);

 private:
  ClockSync clock_;
  MotionPlanner planner_;
  double lastPublish_ = 0;
};

}  // namespace ballalgo

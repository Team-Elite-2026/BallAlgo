#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/RobotSerial.hpp"
#include "lidar/LidarLocalizer.hpp"
#include "motion/ClockSync.hpp"
#include "motion/MotionPlanner.hpp"

#include <deque>
#include <vector>

namespace ballalgo {

class MotionPipeline {
 public:
  MotionPipeline();
  PoseState updateLidar(const std::vector<LidarPoint>& pts, float headingDeg, double dtS);
  BallState updateBall(double angleDeg, double distCal, bool found, double dtS);
  bool tickPublish(RobotSerial& serial, std::vector<uint8_t>& rx, const PoseState& pose,
                   const BallState& ball, float goalDeg, float headingDeg, bool offenseActive);

 private:
  LidarLocalizer localizer_;
  PoseKalman poseKf_;
  BallKalman ballKf_;
  ClockSync clock_;
  MotionPlanner planner_;
  double lastReplan_ = 0;
  double lastPublish_ = 0;
};

}  // namespace ballalgo

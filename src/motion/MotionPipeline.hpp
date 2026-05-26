#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "lidar/LidarLocalizer.hpp"

#include <vector>

namespace ballalgo {

class MotionPipeline {
 public:
  MotionPipeline();
  PoseState updateLidar(const std::vector<LidarPoint>& pts, float headingDeg, double dtS);
  BallState updateBall(double angleDeg, double distCal, bool found, double dtS);

 private:
  LidarLocalizer localizer_;
  PoseKalman poseKf_;
  BallKalman ballKf_;
};

}  // namespace ballalgo

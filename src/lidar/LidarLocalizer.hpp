#pragma once

#include "lidar/Ld19Reader.hpp"

#include <vector>

namespace ballalgo {

struct PoseEstimate {
  float xMm = 0;
  float yMm = 0;
  bool valid = false;
};

class LidarLocalizer {
 public:
  LidarLocalizer(float fieldW, float fieldH, float yawOffsetDeg);
  PoseEstimate update(const std::vector<LidarPoint>& points, float headingDeg);

 private:
  static float median(std::vector<float>& v);
  static float robustAxis(std::vector<float>& estimates);
  float fieldW_, fieldH_, yawOffset_;
  float minDist_ = 80, maxDist_ = 6000;
  int minIntensity_ = 20;
};

}  // namespace ballalgo

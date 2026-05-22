#pragma once

#include "motion/AStar3D.hpp"

#include <vector>

namespace ballalgo {

struct PathSample {
  float xMm, yMm, thetaDeg, sMm;
};

class HermiteSpline {
 public:
  std::vector<PathSample> fit(const std::vector<Waypoint3>& waypoints, float goalThetaDeg);
};

}  // namespace ballalgo

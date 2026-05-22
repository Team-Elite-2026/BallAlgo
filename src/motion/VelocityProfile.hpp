#pragma once

#include "motion/HermiteSpline.hpp"

#include <vector>

namespace ballalgo {

struct ProfileSample {
  float sMm;
  float vCap;
  float v;
  float a;
  float phi;
  float kappa;
};

struct MotionAction {
  float vx, vy, omega, ax, ay, alpha;
};

class VelocityProfile {
 public:
  std::vector<ProfileSample> build(const std::vector<PathSample>& path, float vStart);
  std::vector<MotionAction> discretize(const std::vector<ProfileSample>& prof,
                                     const std::vector<PathSample>& path, float headingDeg,
                                     int dtMs, int maxActions);
};

}  // namespace ballalgo

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

// One s-domain point of the motor-model profile. Velocities/accelerations are
// in the GLOBAL (field) frame, SI units (Step 4.3/4.4 of pipeline_context.md).
struct ProfilePointS {
  float s = 0;
  float sDot = 0;
  float sDdot = 0;
  float vx = 0, vy = 0, omega = 0;
  float ax = 0, ay = 0, alpha = 0;
};

class VelocityProfile {
 public:
  // Spec Step 4.3: time-optimal 3-pass S-curve over the analytic spline using
  // the full motor back-EMF + traction model. vStart/vEnd are path speeds (m/s).
  std::vector<ProfilePointS> computeMotorModel(const HermiteSplineData& spline, float vStartMps,
                                               float vEndMps, int numSteps);

  // Spec Step 4.4: slice an s-domain profile into fixed dt time steps, emitting
  // GLOBAL-frame GlobalAction targets (vx,vy,omega,ax,ay,alpha).
  std::vector<MotionAction> discretizeGlobal(const std::vector<ProfilePointS>& prof, int dtMs,
                                             int maxActions);
};

}  // namespace ballalgo

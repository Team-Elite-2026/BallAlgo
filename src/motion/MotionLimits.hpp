#pragma once

#include "config.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ballalgo::motion {

inline float vMaxDir(float phi, float vmaxX, float vmaxY) {
  float c = std::cos(phi), s = std::sin(phi);
  float d = (c / vmaxX) * (c / vmaxX) + (s / vmaxY) * (s / vmaxY);
  if (d < 1e-9f) return std::max(vmaxX, vmaxY);
  return 1.f / std::sqrt(d);
}

// Spec Step 3: asymmetric directional velocity ceiling from the 4-wheel motor
// model. phiLocal is the travel direction in the robot body frame measured the
// same way the wheels are (0 = +y / forward). The most-loaded wheel limits the
// achievable chassis speed:  v <= max_wheel_vel / |cos(alpha_i - phiLocal)|.
// Returns m/s.
inline float wheelProjVMax(float phiLocal) {
  const float maxWheelOmegaNoLoad =
      (config::kMotorNoLoadRpm * 2.0f * static_cast<float>(M_PI)) / 60.0f;
  const float maxWheelOmega = maxWheelOmegaNoLoad - config::kMotorLoadCompOmega;
  const float maxWheelVel = maxWheelOmega * config::kWheelRadiusM;
  const std::array<float, 4> alpha = {config::kWheelAngle0, config::kWheelAngle1,
                                      config::kWheelAngle2, config::kWheelAngle3};
  float vMax = 9999.0f;
  for (int i = 0; i < 4; ++i) {
    const float proj = std::sin(alpha[i]) * std::sin(phiLocal) +
                       std::cos(alpha[i]) * std::cos(phiLocal);
    if (std::fabs(proj) > 0.001f) {
      vMax = std::min(vMax, maxWheelVel / std::fabs(proj));
    }
  }
  return vMax;
}

// Highest straight-line speed independent of drive angle (used as the
// admissible A* heuristic denominator).
inline float wheelProjVMaxStraight() {
  return std::max(wheelProjVMax(0.0f), wheelProjVMax(static_cast<float>(M_PI) * 0.5f));
}

inline float aMaxDir(float phi, float amaxX, float amaxY) {
  float c = std::cos(phi), s = std::sin(phi);
  float d = (c / amaxX) * (c / amaxX) + (s / amaxY) * (s / amaxY);
  if (d < 1e-9f) return std::max(amaxX, amaxY);
  return 1.f / std::sqrt(d);
}

}  // namespace ballalgo::motion

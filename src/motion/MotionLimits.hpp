#pragma once

#include <cmath>

namespace ballalgo::motion {

inline float vMaxDir(float phi, float vmaxX, float vmaxY) {
  float c = std::cos(phi), s = std::sin(phi);
  float d = (c / vmaxX) * (c / vmaxX) + (s / vmaxY) * (s / vmaxY);
  if (d < 1e-9f) return std::max(vmaxX, vmaxY);
  return 1.f / std::sqrt(d);
}

inline float aMaxDir(float phi, float amaxX, float amaxY) {
  float c = std::cos(phi), s = std::sin(phi);
  float d = (c / amaxX) * (c / amaxX) + (s / amaxY) * (s / amaxY);
  if (d < 1e-9f) return std::max(amaxX, amaxY);
  return 1.f / std::sqrt(d);
}

}  // namespace ballalgo::motion

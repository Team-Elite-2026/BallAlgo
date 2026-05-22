#include "motion/StrikePose.hpp"

#include "config.hpp"
#include "vision/VisionMath.hpp"

#include <cmath>

namespace ballalgo {

void strikePoseBody(float bx, float by, float goalDeg, float& tx, float& ty) {
  float ux, uy;
  polarToBodyXY(goalDeg, 1.0, ux, uy);
  tx = bx - config::kStrikeOffsetM * ux;
  ty = by - config::kStrikeOffsetM * uy;
}

void ballFieldMm(float rx, float ry, float bx, float by, float headingDeg, float& fx, float& fy) {
  const double h = headingDeg * M_PI / 180.0;
  const double c = std::cos(h), s = std::sin(h);
  fx = rx + static_cast<float>((c * bx * 1000 - s * by * 1000));
  fy = ry + static_cast<float>((s * bx * 1000 + c * by * 1000));
}

}  // namespace ballalgo

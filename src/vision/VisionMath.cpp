#include "vision/VisionMath.hpp"

#include <cmath>

namespace ballalgo {

AngleDist angleAndDistance(int cx, int cy, int xoff, int yoff) {
  const double midx = cx - xoff;
  const double midy = cy - yoff;
  AngleDist r;
  r.angleDeg = std::atan2(midx, midy) * 180.0 / M_PI + 180.0;
  r.distPx = std::hypot(midx, midy);
  return r;
}

double calibrateBallDist(double distPx) {
  if (distPx < 0) return -5;
  return 0.00255386 * std::pow(distPx, 2.09612);
}

void polarToBodyXY(double angleDeg, double distM, double& xM, double& yM) {
  const double rad = angleDeg * M_PI / 180.0;
  xM = distM * std::cos(rad);
  yM = distM * std::sin(rad);
}

void fieldVelToBody(float vxMmS, float vyMmS, float headingDeg, float& vxB, float& vyB) {
  const double h = headingDeg * M_PI / 180.0;
  const double c = std::cos(h), s = std::sin(h);
  vxB = static_cast<float>((c * vxMmS + s * vyMmS) / 1000.0);
  vyB = static_cast<float>((-s * vxMmS + c * vyMmS) / 1000.0);
}

}  // namespace ballalgo

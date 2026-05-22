#include "motion/HermiteSpline.hpp"

#include <Eigen/Dense>

#include <cmath>

namespace ballalgo {

std::vector<PathSample> HermiteSpline::fit(const std::vector<Waypoint3>& waypoints,
                                           float goalThetaDeg) {
  std::vector<PathSample> dense;
  if (waypoints.size() < 2) return dense;
  std::vector<float> t(waypoints.size(), 0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    float dx = waypoints[i].xMm - waypoints[i - 1].xMm;
    float dy = waypoints[i].yMm - waypoints[i - 1].yMm;
    t[i] = t[i - 1] + std::hypot(dx, dy);
  }
  if (t.back() < 1e-3f) t.back() = 1.f;
  const int samples = std::max(50, static_cast<int>(waypoints.size()) * 10);
  float sAcc = 0;
  float prevX = waypoints[0].xMm, prevY = waypoints[0].yMm;
  for (int i = 0; i < samples; ++i) {
    float u = static_cast<float>(i) / (samples - 1);
    float tu = u * t.back();
    size_t seg = 1;
    while (seg < t.size() && t[seg] < tu) ++seg;
    if (seg >= waypoints.size()) seg = waypoints.size() - 1;
    size_t i0 = seg - 1;
    float t0 = t[i0], t1 = t[seg];
    float alpha = (t1 > t0) ? (tu - t0) / (t1 - t0) : 0.f;
    const auto& p0 = waypoints[i0];
    const auto& p1 = waypoints[seg];
    float m0x = 0, m0y = 0, m1x = 0, m1y = 0;
    if (i0 > 0) {
      m0x = (p1.xMm - waypoints[i0 - 1].xMm) * 0.5f;
      m0y = (p1.yMm - waypoints[i0 - 1].yMm) * 0.5f;
    } else {
      m0x = p1.xMm - p0.xMm;
      m0y = p1.yMm - p0.yMm;
    }
    m1x = p1.xMm - p0.xMm;
    m1y = p1.yMm - p0.yMm;
    float a2 = 2 * alpha * alpha * alpha - 3 * alpha * alpha + 1;
    float a3 = alpha * alpha * alpha - 2 * alpha * alpha + alpha;
    float b2 = -2 * alpha * alpha * alpha + 3 * alpha * alpha;
    float b3 = alpha * alpha * alpha - alpha * alpha;
    float x = a2 * p0.xMm + a3 * m0x + b2 * p1.xMm + b3 * m1x;
    float y = a2 * p0.yMm + a3 * m0y + b2 * p1.yMm + b3 * m1y;
    if (i > 0) sAcc += std::hypot(x - prevX, y - prevY);
    prevX = x;
    prevY = y;
    float th = (i == samples - 1) ? goalThetaDeg
                                  : std::atan2(y - p0.yMm, x - p0.xMm) * 180.f / static_cast<float>(M_PI);
    dense.push_back({x, y, th, sAcc});
  }
  return dense;
}

}  // namespace ballalgo

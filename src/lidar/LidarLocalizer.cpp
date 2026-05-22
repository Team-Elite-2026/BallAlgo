#include "lidar/LidarLocalizer.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

LidarLocalizer::LidarLocalizer(float fieldW, float fieldH, float yawOffsetDeg)
    : fieldW_(fieldW), fieldH_(fieldH), yawOffset_(yawOffsetDeg) {}

float LidarLocalizer::median(std::vector<float>& v) {
  if (v.empty()) return 0;
  size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  if (v.size() % 2 == 0) {
    float a = v[mid];
    std::nth_element(v.begin(), v.begin() + mid - 1, v.end());
    return 0.5f * (a + v[mid - 1]);
  }
  return v[mid];
}

float LidarLocalizer::robustAxis(std::vector<float>& estimates) {
  float med = median(estimates);
  std::vector<float> filtered;
  for (float e : estimates)
    if (std::fabs(e - med) < 180.f) filtered.push_back(e);
  if (filtered.size() >= 3) return median(filtered);
  return med;
}

PoseEstimate LidarLocalizer::update(const std::vector<LidarPoint>& points, float headingDeg) {
  std::vector<float> xs, ys;
  for (const auto& p : points) {
    float d = static_cast<float>(p.distanceMm);
    if (d < minDist_ || d > maxDist_ || p.intensity < minIntensity_) continue;
    float angDeg = headingDeg + yawOffset_ + p.angleCd * 0.01f;
    float rad = angDeg * static_cast<float>(M_PI / 180.0);
    float dx = std::cos(rad), dy = std::sin(rad);
    if (std::fabs(dx) >= std::fabs(dy)) {
      if (std::fabs(dx) < 1e-4f) continue;
      float x = (dx > 0) ? (fieldW_ - d * dx) : (-d * dx);
      if (x > -200 && x < fieldW_ + 200) xs.push_back(x);
    } else {
      if (std::fabs(dy) < 1e-4f) continue;
      float y = (dy > 0) ? (fieldH_ - d * dy) : (-d * dy);
      if (y > -200 && y < fieldH_ + 200) ys.push_back(y);
    }
  }
  PoseEstimate pe;
  pe.valid = xs.size() >= 4 && ys.size() >= 4;
  if (pe.valid) {
    pe.xMm = std::clamp(robustAxis(xs), 0.f, fieldW_);
    pe.yMm = std::clamp(robustAxis(ys), 0.f, fieldH_);
  }
  return pe;
}

}  // namespace ballalgo

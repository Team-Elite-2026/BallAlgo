#include "motion/StrikePose.hpp"

#include "config.hpp"
#include "params.hpp"
#include "vision/VisionMath.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

void strikePoseBody(float bx, float by, float goalDeg, float& tx, float& ty) {
  double ux = 0.0;
  double uy = 0.0;
  polarToBodyXY(goalDeg, 1.0, ux, uy);
  tx = bx - static_cast<float>(params::get().strikeOffsetM * ux);
  ty = by - static_cast<float>(params::get().strikeOffsetM * uy);
}

void ballFieldMm(float rx, float ry, float bx, float by, float headingDeg, float& fx, float& fy) {
  const double h = headingDeg * M_PI / 180.0;
  const double c = std::cos(h), s = std::sin(h);
  fx = rx + static_cast<float>((c * bx * 1000 - s * by * 1000));
  fy = ry + static_cast<float>((s * bx * 1000 + c * by * 1000));
}

PredictedBallPose predictBallBody(float bx, float by, float vx, float vy, float timeS) {
  PredictedBallPose predicted;
  predicted.vx = vx;
  predicted.vy = vy;

  const float dtCam = 1.f / static_cast<float>(config::kCameraFps);
  const float samples = std::max(0.f, timeS / dtCam);
  const float gamma = params::get().ballPredictionDamping;
  float dampedTime = timeS;
  if (std::fabs(1.f - gamma) > 1e-5f) {
    dampedTime = dtCam * (1.f - std::pow(gamma, samples)) / (1.f - gamma);
    predicted.vx = vx * std::pow(gamma, samples);
    predicted.vy = vy * std::pow(gamma, samples);
  }

  predicted.xM = bx + vx * dampedTime;
  predicted.yM = by + vy * dampedTime;
  return predicted;
}

}  // namespace ballalgo

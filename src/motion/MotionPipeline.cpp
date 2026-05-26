#include "motion/MotionPipeline.hpp"

#include "config.hpp"
#include "vision/VisionMath.hpp"

namespace ballalgo {

MotionPipeline::MotionPipeline()
    : localizer_(config::kFieldWidthMm, config::kFieldHeightMm, config::kLidarYawOffsetDeg) {}

PoseState MotionPipeline::updateLidar(const std::vector<LidarPoint>& pts, float headingDeg,
                                      double dtS) {
  poseKf_.predict(dtS);
  auto est = localizer_.update(pts, headingDeg);
  poseKf_.update(est, headingDeg);
  return poseKf_.state(headingDeg);
}

BallState MotionPipeline::updateBall(double angleDeg, double distCal, bool found, double dtS) {
  ballKf_.predict(dtS);
  if (found && distCal >= 0) {
    double x = 0;
    double y = 0;
    polarToBodyXY(angleDeg, distCal * config::kBallDistToM, x, y);
    ballKf_.update(x, y, true);
  } else {
    ballKf_.update(0, 0, false);
  }
  return ballKf_.state();
}

}  // namespace ballalgo

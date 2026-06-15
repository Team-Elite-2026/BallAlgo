#include "motion/MotionPipeline.hpp"

#include "config.hpp"
#include "vision/VisionMath.hpp"

namespace ballalgo {

MotionPipeline::MotionPipeline()
    : localizer_(config::kFieldWidthMm, config::kFieldHeightMm, config::kLidarYawOffsetDeg) {}

void MotionPipeline::predictStep(const TeensyOdometry& odo, double dtS, uint64_t timestampUs) {
  if (config::kEnableMouseFusion && odo.mouseFresh) {
    poseKf_.predictMouse(odo.mouseVxBodyMmS, odo.mouseVyBodyMmS, odo.headingDeg, dtS);
  } else {
    poseKf_.predict(dtS);
  }
  // Only ingest motion samples when telemetry is fresh. Stale odometry
  // carries the wrong timestamp attribution; the deskewer's own extrapolation
  // and kMaxMotionDataAgeS guard handle brief dropouts correctly.
  if (odo.mouseFresh) {
    deskewer_.addMotionSample(timestampUs,
                              odo.mouseVxBodyMmS,
                              odo.mouseVyBodyMmS,
                              odo.omegaRadS);
  }
}

PoseState MotionPipeline::updateLidar(const std::vector<LidarPoint>& pts, float headingDeg) {
  std::vector<LidarPoint> corrected = pts;
  deskewer_.deskew(corrected);
  lastDeskewedScan_ = corrected;
  auto est = localizer_.update(corrected, headingDeg);
  poseKf_.update(est, headingDeg);
  return poseKf_.state(headingDeg);
}

void MotionPipeline::updateGoalBearings(const GoalBearingObs* obs, int count, float headingDeg) {
  if (!config::kEnableGoalBearingFusion || obs == nullptr) return;
  for (int i = 0; i < count; ++i) {
    const GoalBearingObs& o = obs[i];
    if (!o.valid) continue;
    // Aside (spec): skip very low certainty observations entirely.
    if (o.certainty < config::kGoalCertaintyThreshold) continue;
    poseKf_.updateGoalBearing(o.bearingRad, o.goalXMm, o.goalYMm, headingDeg, o.certainty);
  }
}

PoseState MotionPipeline::poseState(float headingDeg) const { return poseKf_.state(headingDeg); }

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

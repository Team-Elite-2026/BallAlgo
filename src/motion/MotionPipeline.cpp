#include "motion/MotionPipeline.hpp"

#include "config.hpp"
#include "params.hpp"
#include "vision/VisionMath.hpp"

#include <cmath>

namespace ballalgo {

namespace {

// Smallest signed difference a-b, wrapped to [-180, 180] degrees.
float headingDeltaDeg(float a, float b) {
  float d = std::fmod(a - b + 180.f, 360.f);
  if (d < 0.f) d += 360.f;
  return d - 180.f;
}

}  // namespace

MotionPipeline::MotionPipeline()
    : localizer_(config::kFieldWidthMm, config::kFieldHeightMm, config::kLidarYawOffsetDeg) {}

void MotionPipeline::predictStep(const TeensyOdometry& odo, double dtS, uint64_t timestampUs) {
  // When deriving the deskew motion from LiDAR, keep the pose KF free of mouse
  // influence so its velocity estimate reflects LiDAR only.
  if (config::kDeskewVelocityFromLidar) {
    poseKf_.predict(dtS);
    return;
  }

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

PoseState MotionPipeline::updateLidar(const std::vector<LidarPoint>& pts, float headingDeg,
                                      uint64_t timestampUs) {
  std::vector<LidarPoint> corrected = pts;
  // Deskew uses the motion history populated on prior frames (one-frame delay),
  // so this corrects the current scan with the previous LiDAR-derived velocity.
  deskewer_.deskew(corrected);
  lastDeskewedScan_ = corrected;
  auto est = localizer_.update(corrected, headingDeg);
  poseKf_.update(est, headingDeg);
  const PoseState pose = poseKf_.state(headingDeg);

  // Feed the KF-smoothed body velocity back into the deskewer for the next scan.
  if (config::kDeskewVelocityFromLidar && pose.valid && timestampUs != 0) {
    float omegaRadS = 0.f;
    if (haveLastLidarUpdate_ && timestampUs > lastLidarUpdateUs_) {
      const float dtS = static_cast<float>(timestampUs - lastLidarUpdateUs_) * 1e-6f;
      if (dtS > 1e-4f) {
        // Heading is clockwise-positive; math yaw rate (CCW+) is the negated rate.
        omegaRadS = -headingDeltaDeg(headingDeg, lastHeadingDeg_) *
                    static_cast<float>(M_PI / 180.0) / dtS;
      }
    }
    deskewer_.addMotionSample(timestampUs, pose.vxBody, pose.vyBody, omegaRadS);
    lastHeadingDeg_ = headingDeg;
    lastLidarUpdateUs_ = timestampUs;
    haveLastLidarUpdate_ = true;
  }

  return pose;
}

void MotionPipeline::updateGoalBearings(const GoalBearingObs* obs, int count, float headingDeg) {
  if (!config::kEnableGoalBearingFusion || obs == nullptr) return;
  for (int i = 0; i < count; ++i) {
    const GoalBearingObs& o = obs[i];
    if (!o.valid) continue;
    // Aside (spec): skip very low certainty observations entirely.
    if (o.certainty < params::get().goalCertaintyThreshold) continue;
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

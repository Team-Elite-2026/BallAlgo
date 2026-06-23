#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/RobotSerial.hpp"
#include "lidar/LidarDeskewer.hpp"
#include "lidar/LidarLocalizer.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

// One fixed goal post observation for the Step 1b bearing-only EKF update.
struct GoalBearingObs {
  bool valid = false;
  double bearingRad = 0;  // camera bearing relative to robot heading
  double goalXMm = 0;
  double goalYMm = 0;
  double certainty = 0;  // c in [0,1]
};

class MotionPipeline {
 public:
  MotionPipeline();

  // Step 1a high-frequency predict. Uses body-frame mouse velocities rotated by
  // the IMU heading when fresh odometry is available; otherwise a plain
  // constant-velocity propagation.
  void predictStep(const TeensyOdometry& odo, double dtS, uint64_t timestampUs);

  // Step 1a map update: LiDAR absolute position snap. Returns the fused pose.
  // timestampUs (steady_clock us) is used when kDeskewVelocityFromLidar feeds the
  // KF-estimated body velocity back into the deskewer for the next scan.
  PoseState updateLidar(const std::vector<LidarPoint>& pts, float headingDeg,
                        uint64_t timestampUs = 0);

  // Step 1b: apply any valid goal bearing observations (occlusion-weighted).
  void updateGoalBearings(const GoalBearingObs* obs, int count, float headingDeg);

  PoseState poseState(float headingDeg) const;

  BallState updateBall(double angleDeg, double distCal, bool found, double dtS);

  const std::vector<LidarPoint>& lastDeskewedScan() const { return lastDeskewedScan_; }

 private:
  LidarDeskewer deskewer_;
  LidarLocalizer localizer_;
  PoseKalman poseKf_;
  BallKalman ballKf_;
  std::vector<LidarPoint> lastDeskewedScan_;

  // For kDeskewVelocityFromLidar: derive yaw rate from the heading delta between
  // consecutive LiDAR updates.
  float lastHeadingDeg_ = 0;
  uint64_t lastLidarUpdateUs_ = 0;
  bool haveLastLidarUpdate_ = false;
};

}  // namespace ballalgo

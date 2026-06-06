#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/RobotSerial.hpp"
#include "lidar/LidarLocalizer.hpp"

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
  void predictStep(const TeensyOdometry& odo, double dtS);

  // Step 1a map update: LiDAR absolute position snap. Returns the fused pose.
  PoseState updateLidar(const std::vector<LidarPoint>& pts, float headingDeg);

  // Step 1b: apply any valid goal bearing observations (occlusion-weighted).
  void updateGoalBearings(const GoalBearingObs* obs, int count, float headingDeg);

  PoseState poseState(float headingDeg) const;

  BallState updateBall(double angleDeg, double distCal, bool found, double dtS);

 private:
  LidarLocalizer localizer_;
  PoseKalman poseKf_;
  BallKalman ballKf_;
};

}  // namespace ballalgo

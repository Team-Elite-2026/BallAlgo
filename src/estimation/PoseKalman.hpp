#pragma once

#include "lidar/LidarLocalizer.hpp"

#include <Eigen/Dense>

namespace ballalgo {

struct PoseState {
  bool valid = false;
  float xMm = 0, yMm = 0;
  float vxMmS = 0, vyMmS = 0;
  float vxBody = 0, vyBody = 0;
};

// Robot global EKF (Step 1a/1b of docs/pipeline_context.md).
//
// State vector x = [x, y, vx, vy] in field frame (mm, mm/s). The Teensy IMU
// heading is treated as the absolute external authority (theta_r) and so is not
// an EKF state; it parameterises the body->world rotation instead. Likewise
// omega is supplied externally. This matches the spec's intent of "tightly
// coupling high-frequency odometry with low-frequency global updates".
//
// Sensor cycle:
//   predictMouse() : ~1 kHz dead-reckoning from body-frame mouse velocities,
//                    rotated to world via the IMU heading (DWNA process model).
//   updateLidar()  : ~15 Hz absolute position snap that erases dead-reckoning
//                    drift.
//   updateGoalBearing() : camera bearing-only correction of (x, y) lateral
//                    drift with occlusion-scaled measurement variance.
class PoseKalman {
 public:
  // Constant-velocity time propagation with DWNA acceleration process noise.
  void predict(double dtS);

  // Step 1a high-frequency predict: integrate body-frame mouse velocities
  // (mm/s) rotated into the world frame by the IMU heading, then correct the
  // velocity states toward that measurement.
  void predictMouse(double vxBodyMmS, double vyBodyMmS, float headingDeg, double dtS);

  // Step 1a map update: hard LiDAR position snap.
  void update(const PoseEstimate& meas, float headingDeg);

  // Step 1b: bearing-only correction toward a fixed goal post. measBearingRad is
  // the camera bearing of the goal relative to the robot heading (the same
  // quantity h(x) predicts). certainty c in [0,1] scales R; updates below the
  // configured threshold are ignored by the caller.
  void updateGoalBearing(double measBearingRad, double goalXMm, double goalYMm,
                         float headingDeg, double certainty);

  PoseState state(float headingDeg) const;
  void reset();

 private:
  Eigen::Vector4d x_ = Eigen::Vector4d::Zero();
  Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity() * 1e3;
  bool init_ = false;
  double ageSinceMeasurementS_ = 0;
};

}  // namespace ballalgo

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

class PoseKalman {
 public:
  void predict(double dtS);
  void update(const PoseEstimate& meas, float headingDeg);
  PoseState state(float headingDeg) const;
  void reset();

 private:
  Eigen::Matrix4d x_ = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity() * 1e3;
  bool init_ = false;
};

}  // namespace ballalgo

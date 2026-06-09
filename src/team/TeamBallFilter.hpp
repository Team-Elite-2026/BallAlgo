#pragma once

#include "team/TeamTypes.hpp"

#include <Eigen/Dense>

namespace ballalgo {

class TeamBallFilter {
 public:
  void predict(double dtS);
  void update(const TeamBallObservation& observation);
  void reset();
  FusedBallFieldState state() const;

 private:
  Eigen::Vector4d x_ = Eigen::Vector4d::Zero();
  Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity() * 1e6;
  bool init_ = false;
  double ageSinceMeasurementS_ = 1e9;
};

}  // namespace ballalgo

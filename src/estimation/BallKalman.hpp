#pragma once

#include <Eigen/Dense>

namespace ballalgo {

struct BallState {
  bool visible = false;
  float xM = 0, yM = 0, vx = 0, vy = 0;
};

class BallKalman {
 public:
  void predict(double dtS);
  void update(double xM, double yM, bool found);
  BallState state() const;
  void reset();

 private:
  Eigen::Vector4d x_ = Eigen::Vector4d::Zero();
  Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity();
  bool init_ = false;
  bool visible_ = false;
};

}  // namespace ballalgo

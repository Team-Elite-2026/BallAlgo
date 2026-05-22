#include "estimation/BallKalman.hpp"

#include "config.hpp"

namespace ballalgo {

void BallKalman::predict(double dtS) {
  if (!init_ || dtS <= 0) return;
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dtS;
  F(1, 3) = dtS;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = Q(1, 1) = config::kBallKfProcessPosVar * dtS;
  Q(2, 2) = Q(3, 3) = config::kBallKfProcessVelVar * dtS;
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void BallKalman::update(double xM, double yM, bool found) {
  if (!found) {
    visible_ = init_;
    return;
  }
  if (!init_) {
    x_ << xM, yM, 0, 0;
    P_ = Eigen::Matrix4d::Identity();
    init_ = true;
  } else {
    Eigen::Matrix<double, 2, 4> H;
    H << 1, 0, 0, 0, 0, 1, 0, 0;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * config::kBallKfMeasVar;
    Eigen::Vector2d z(xM, yM);
    Eigen::Vector2d y = z - H * x_;
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
  }
  visible_ = true;
}

BallState BallKalman::state() const {
  BallState s;
  s.visible = visible_ && init_;
  if (!init_) return s;
  s.xM = static_cast<float>(x_(0));
  s.yM = static_cast<float>(x_(1));
  s.vx = static_cast<float>(x_(2));
  s.vy = static_cast<float>(x_(3));
  return s;
}

void BallKalman::reset() {
  init_ = false;
  visible_ = false;
}

}  // namespace ballalgo

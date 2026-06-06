#include "estimation/BallKalman.hpp"

#include "config.hpp"

#include <cmath>

namespace ballalgo {

void BallKalman::predict(double dtS) {
  if (!init_ || dtS <= 0) return;
  ageSinceMeasurementS_ += dtS;
  // Spec Step 1 (ball): constant-velocity with viscous friction. The velocity
  // states decay by gamma each camera frame, normalised to this dt so the model
  // is frame-rate independent:  v_{k+1} = gamma^(dt/dt_cam) * v_k.
  const double gammaStep =
      std::pow(static_cast<double>(config::kBallKfFrictionGamma),
               dtS / static_cast<double>(config::kBallKfCamDtS));
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dtS;
  F(1, 3) = dtS;
  F(2, 2) = gammaStep;
  F(3, 3) = gammaStep;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = Q(1, 1) = config::kBallKfProcessPosVar * dtS;
  Q(2, 2) = Q(3, 3) = config::kBallKfProcessVelVar * dtS;
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void BallKalman::update(double xM, double yM, bool found) {
  if (!found) return;
  if (!init_) {
    x_ << xM, yM, 0, 0;
    P_ = Eigen::Matrix4d::Identity();
    // High initial uncertainty on the (unobserved) velocity states.
    P_(2, 2) = P_(3, 3) = config::kBallKfVelInitVar;
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
  ageSinceMeasurementS_ = 0;
}

BallState BallKalman::state() const {
  BallState s;
  s.visible = init_ && ageSinceMeasurementS_ <= config::kBallMaxStaleS;
  if (!init_) return s;
  s.xM = static_cast<float>(x_(0));
  s.yM = static_cast<float>(x_(1));
  s.vx = static_cast<float>(x_(2));
  s.vy = static_cast<float>(x_(3));
  return s;
}

void BallKalman::reset() {
  init_ = false;
  ageSinceMeasurementS_ = 0;
  x_.setZero();
  P_ = Eigen::Matrix4d::Identity();
}

}  // namespace ballalgo

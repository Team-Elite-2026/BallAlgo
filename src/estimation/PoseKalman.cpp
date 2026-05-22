#include "estimation/PoseKalman.hpp"

#include "config.hpp"
#include "vision/VisionMath.hpp"

namespace ballalgo {

void PoseKalman::predict(double dtS) {
  if (!init_ || dtS <= 0) return;
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dtS;
  F(1, 3) = dtS;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = Q(1, 1) = config::kPoseKfProcessPosVar * dtS;
  Q(2, 2) = Q(3, 3) = config::kPoseKfProcessVelVar * dtS;
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void PoseKalman::update(const PoseEstimate& meas, float headingDeg) {
  (void)headingDeg;
  if (meas.valid) {
    if (!init_) {
      x_ << meas.xMm, meas.yMm, 0, 0;
      P_ = Eigen::Matrix4d::Identity() * 100;
      init_ = true;
    } else {
      Eigen::Matrix<double, 2, 4> H;
      H << 1, 0, 0, 0, 0, 1, 0, 0;
      Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * config::kPoseKfMeasPosVar;
      Eigen::Vector2d z(meas.xMm, meas.yMm);
      Eigen::Vector2d y = z - H * x_;
      Eigen::Matrix2d S = H * P_ * H.transpose() + R;
      Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();
      x_ = x_ + K * y;
      P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
    }
  }
}

PoseState PoseKalman::state(float headingDeg) const {
  PoseState s;
  s.valid = init_;
  if (!init_) return s;
  s.xMm = static_cast<float>(x_(0));
  s.yMm = static_cast<float>(x_(1));
  s.vxMmS = static_cast<float>(x_(2));
  s.vyMmS = static_cast<float>(x_(3));
  fieldVelToBody(s.vxMmS, s.vyMmS, headingDeg, s.vxBody, s.vyBody);
  return s;
}

void PoseKalman::reset() {
  init_ = false;
  x_.setZero();
  P_ = Eigen::Matrix4d::Identity() * 1e3;
}

}  // namespace ballalgo

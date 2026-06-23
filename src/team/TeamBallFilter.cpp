#include "team/TeamBallFilter.hpp"

#include "params.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {
namespace {

constexpr double kBallAccelSigmaMmS2 = 2200.0;
constexpr double kInitialVelocityVar = 600000.0;

}  // namespace

void TeamBallFilter::predict(double dtS) {
  if (!init_ || dtS <= 0.0) return;
  ageSinceMeasurementS_ += dtS;

  const double dt2 = dtS * dtS;
  const double dt3 = dt2 * dtS;
  const double dt4 = dt2 * dt2;
  const double q = kBallAccelSigmaMmS2 * kBallAccelSigmaMmS2;

  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dtS;
  F(1, 3) = dtS;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = 0.25 * dt4 * q;
  Q(0, 2) = 0.5 * dt3 * q;
  Q(1, 1) = 0.25 * dt4 * q;
  Q(1, 3) = 0.5 * dt3 * q;
  Q(2, 0) = 0.5 * dt3 * q;
  Q(2, 2) = dt2 * q;
  Q(3, 1) = 0.5 * dt3 * q;
  Q(3, 3) = dt2 * q;

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void TeamBallFilter::update(const TeamBallObservation& observation) {
  if (!observation.valid) return;

  const double sigmaX = std::max(1.0f, observation.sigmaXMm);
  const double sigmaY = std::max(1.0f, observation.sigmaYMm);

  if (!init_) {
    x_ << observation.xMm, observation.yMm, 0.0, 0.0;
    P_.setZero();
    P_(0, 0) = sigmaX * sigmaX;
    P_(1, 1) = sigmaY * sigmaY;
    P_(2, 2) = kInitialVelocityVar;
    P_(3, 3) = kInitialVelocityVar;
    init_ = true;
    ageSinceMeasurementS_ = 0.0;
    return;
  }

  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  Eigen::Vector2d z(observation.xMm, observation.yMm);
  Eigen::Vector2d y = z - H * x_;

  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = sigmaX * sigmaX;
  R(1, 1) = sigmaY * sigmaY;

  const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();
  x_ += K * y;
  P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
  ageSinceMeasurementS_ = 0.0;
}

void TeamBallFilter::reset() {
  x_.setZero();
  P_ = Eigen::Matrix4d::Identity() * 1e6;
  init_ = false;
  ageSinceMeasurementS_ = 1e9;
}

FusedBallFieldState TeamBallFilter::state() const {
  FusedBallFieldState result;
  if (!init_ || ageSinceMeasurementS_ > params::get().ballMaxStaleS) return result;

  result.valid = true;
  result.xMm = static_cast<float>(x_(0));
  result.yMm = static_cast<float>(x_(1));
  result.vxMmS = static_cast<float>(x_(2));
  result.vyMmS = static_cast<float>(x_(3));
  result.sigmaXMm = static_cast<float>(std::sqrt(std::max(0.0, P_(0, 0))));
  result.sigmaYMm = static_cast<float>(std::sqrt(std::max(0.0, P_(1, 1))));
  return result;
}

}  // namespace ballalgo

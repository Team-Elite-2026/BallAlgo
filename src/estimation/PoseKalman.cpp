#include "estimation/PoseKalman.hpp"

#include "config.hpp"
#include "vision/VisionMath.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

namespace {

// Body->world rotation matching the production convention used by ballFieldMm /
// fieldVelToBody (heading 0 = +y, 90 = +x, clockwise):
//   v_world_x = cos(h)*v_body_x - sin(h)*v_body_y
//   v_world_y = sin(h)*v_body_x + cos(h)*v_body_y
void bodyVelToWorld(double vxBody, double vyBody, float headingDeg, double& vxWorld,
                    double& vyWorld) {
  const double h = headingDeg * M_PI / 180.0;
  const double c = std::cos(h);
  const double s = std::sin(h);
  vxWorld = c * vxBody - s * vyBody;
  vyWorld = s * vxBody + c * vyBody;
}

// Constant-velocity F and DWNA process noise Q for a given accel-variance.
void cvPredict(Eigen::Vector4d& x, Eigen::Matrix4d& P, double dtS, double accelVar) {
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dtS;
  F(1, 3) = dtS;
  const double dt2 = dtS * dtS;
  const double dt3 = dt2 * dtS;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  // Per-axis DWNA block: pos/pos = sa*dt^3/3, pos/vel = sa*dt^2/2, vel/vel = sa*dt.
  Q(0, 0) = accelVar * dt3 / 3.0;
  Q(0, 2) = Q(2, 0) = accelVar * dt2 / 2.0;
  Q(2, 2) = accelVar * dtS;
  Q(1, 1) = accelVar * dt3 / 3.0;
  Q(1, 3) = Q(3, 1) = accelVar * dt2 / 2.0;
  Q(3, 3) = accelVar * dtS;
  x = F * x;
  P = F * P * F.transpose() + Q;
}

}  // namespace

void PoseKalman::predict(double dtS) {
  if (!init_ || dtS <= 0) return;
  ageSinceMeasurementS_ += dtS;
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = dtS;
  F(1, 3) = dtS;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = Q(1, 1) = config::kPoseKfProcessPosVar * dtS;
  Q(2, 2) = Q(3, 3) = config::kPoseKfProcessVelVar * dtS;
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;
}

void PoseKalman::predictMouse(double vxBodyMmS, double vyBodyMmS, float headingDeg, double dtS) {
  if (!init_ || dtS <= 0) return;
  ageSinceMeasurementS_ += dtS;

  // Dead-reckoning propagation (DWNA constant-velocity model).
  cvPredict(x_, P_, dtS, config::kPoseKfMouseAccelVar);

  // Measurement update: the heading-rotated mouse velocity observes [vx, vy].
  // The time-varying rotation by the IMU heading is what makes this an EKF.
  double zx = 0;
  double zy = 0;
  bodyVelToWorld(vxBodyMmS, vyBodyMmS, headingDeg, zx, zy);

  Eigen::Matrix<double, 2, 4> H;
  H.setZero();
  H(0, 2) = 1.0;
  H(1, 3) = 1.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * config::kPoseKfMouseMeasVar;
  Eigen::Vector2d z(zx, zy);
  Eigen::Vector2d y = z - H * x_;
  Eigen::Matrix2d S = H * P_ * H.transpose() + R;
  Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
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
    ageSinceMeasurementS_ = 0;
  }
}

void PoseKalman::updateGoalBearing(double measBearingRad, double goalXMm, double goalYMm,
                                   float headingDeg, double certainty) {
  if (!init_) return;

  const double xr = x_(0);
  const double yr = x_(1);
  const double dxF = goalXMm - xr;
  const double dyF = goalYMm - yr;
  const double r2 = dxF * dxF + dyF * dyF;
  if (r2 < 1.0) return;  // robot essentially on the goal post; bearing undefined

  // Predicted body-frame bearing. We rotate the field displacement into the
  // body frame with the SAME world->body convention the rest of the codebase
  // uses (the inverse of ballFieldMm / matching fieldVelToBody), then take
  // atan2(bodyX, bodyY). Computing h this way (rather than the literal spec
  // atan2(dx,dy)-theta) keeps the residual consistent with the camera bearing,
  // which is produced via polarToBodyXY in the same body frame.
  const double h = headingDeg * M_PI / 180.0;
  const double c = std::cos(h);
  const double s = std::sin(h);
  const double bx = c * dxF + s * dyF;
  const double by = -s * dxF + c * dyF;
  const double hx = std::atan2(bx, by);

  // Innovation, wrapped to (-pi, pi] to handle the 360-0 edge case.
  double y = measBearingRad - hx;
  while (y > M_PI) y -= 2.0 * M_PI;
  while (y < -M_PI) y += 2.0 * M_PI;

  // Analytic Jacobian of hx wrt [x, y, vx, vy]. With the rotation above,
  // d(hx)/dxr = -dyF/r^2 and d(hx)/dyr = dxF/r^2 (the rotation terms cancel).
  Eigen::Matrix<double, 1, 4> H;
  H << -dyF / r2, dxF / r2, 0.0, 0.0;

  // Occlusion-scaled measurement variance: R = base + penalty*(1-cert)^2.
  const double cert = std::clamp(certainty, 0.0, 1.0);
  const double oneMinusC = 1.0 - cert;
  const double R = config::kGoalBearingBaseVarRad2 +
                   config::kGoalBearingPenaltyRad2 * oneMinusC * oneMinusC;

  const double S = (H * P_ * H.transpose())(0, 0) + R;
  if (S < 1e-9) return;
  const Eigen::Vector4d K = P_ * H.transpose() / S;
  x_ = x_ + K * y;
  P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
}

PoseState PoseKalman::state(float headingDeg) const {
  PoseState s;
  s.valid = init_ && ageSinceMeasurementS_ <= config::kPoseMaxStaleS;
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
  ageSinceMeasurementS_ = 0;
  x_.setZero();
  P_ = Eigen::Matrix4d::Identity() * 1e3;
}

}  // namespace ballalgo

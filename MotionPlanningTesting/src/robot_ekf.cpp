#include "robot_ekf.hpp"
#include <stdexcept>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

RobotEKF::RobotEKF(double sigma_a, double r_mouse, double r_lidar)
    : sigma_a2_(sigma_a * sigma_a),
      r_lidar_(r_lidar),
      initialized_(false)
{
    R_mouse_ = Eigen::Matrix2d::Identity() * r_mouse;
    x_.setZero();
    P_.setIdentity();
}

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------

void RobotEKF::init(double x, double y) {
    x_ << x, y, 0.0, 0.0;

    P_.setZero();
    P_(0, 0) = r_lidar_;   // x position uncertainty = LiDAR accuracy
    P_(1, 1) = r_lidar_;   // y position uncertainty
    P_(2, 2) = 1.0;        // vx: unknown at startup
    P_(3, 3) = 1.0;        // vy: unknown at startup

    initialized_ = true;
}

// ---------------------------------------------------------------------------
// Prediction step  (constant-velocity, DWNA process noise)
//
// State transition:
//   [ x  ]   [1  0  dt  0 ] [ x  ]
//   [ y  ] = [0  1  0  dt] [ y  ]
//   [ vx ]   [0  0  1   0] [ vx ]
//   [ vy ]   [0  0  0   1] [ vy ]
//
// Process noise Q (Discrete White Noise Acceleration model, σ_a²):
//   Captures un-modelled acceleration as a zero-mean Gaussian disturbance.
//   Q_xx = σ_a²·dt³/3,  Q_xv = σ_a²·dt²/2,  Q_vv = σ_a²·dt  (per axis)
// ---------------------------------------------------------------------------

void RobotEKF::predict(double dt) {
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    const double dt2  = dt * dt;
    const double dt3  = dt2 * dt;

    Eigen::Matrix4d Q;
    Q.setZero();
    // x-axis block
    Q(0, 0) = sigma_a2_ * dt3 / 3.0;
    Q(0, 2) = sigma_a2_ * dt2 / 2.0;
    Q(2, 0) = Q(0, 2);
    Q(2, 2) = sigma_a2_ * dt;
    // y-axis block (same structure)
    Q(1, 1) = sigma_a2_ * dt3 / 3.0;
    Q(1, 3) = sigma_a2_ * dt2 / 2.0;
    Q(3, 1) = Q(1, 3);
    Q(3, 3) = sigma_a2_ * dt;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

// ---------------------------------------------------------------------------
// Mouse sensor update  (~1 kHz)
//
// The mouse sensor measures body-frame velocity.  To fuse it with the world-
// frame velocity states, we rotate the observation to world frame using the
// IMU heading, then apply a standard Kalman measurement update.
//
// Observation model (world-frame velocity, linear in state given heading θ):
//   z = [vx_world_meas, vy_world_meas]ᵀ
//   h(x) = H·x,  H = [0  0  1  0]
//                     [0  0  0  1]
//
// The rotation by the time-varying heading θ is what makes this an EKF —
// the Jacobian H is recomputed at every step from the current IMU reading.
// (Equivalently, the raw body-frame measurement has H = R_wb(θ)·[…] which
//  is clearly heading-dependent; transforming to world frame first yields the
//  simpler H above while preserving the EKF character.)
// ---------------------------------------------------------------------------

void RobotEKF::updateMouse(double vx_body, double vy_body,
                            double heading_deg, double dt)
{
    if (!initialized_) return;

    predict(dt);

    // Rotate body-frame mouse velocity to world frame.
    // Body-to-world rotation for heading convention (0=+y, 90=+x, CW):
    //   vx_world =  cos(θ)·vx_body + sin(θ)·vy_body
    //   vy_world = −sin(θ)·vx_body + cos(θ)·vy_body
    const double theta = heading_deg * kDeg2Rad;
    const double c     = std::cos(theta);
    const double s     = std::sin(theta);

    Eigen::Vector2d z;
    z(0) =  c * vx_body + s * vy_body;
    z(1) = -s * vx_body + c * vy_body;

    // H: observe [vx, vy] part of state [x, y, vx, vy]
    Eigen::Matrix<double, 2, 4> H;
    H.setZero();
    H(0, 2) = 1.0;
    H(1, 3) = 1.0;

    const Eigen::Matrix2d              S = H * P_ * H.transpose() + R_mouse_;
    const Eigen::Matrix<double, 4, 2>  K = P_ * H.transpose() * S.inverse();

    x_ += K * (z - H * x_);
    P_  = (Eigen::Matrix4d::Identity() - K * H) * P_;
}

// ---------------------------------------------------------------------------
// LiDAR position snap  (~15 Hz)
//
// LiDAR has absolute authority over x/y.  Rather than applying a Kalman gain
// (which would only partially correct the position), we hard-reset x/y and
// restore the position rows/cols of P to reflect LiDAR accuracy.
// The velocity estimate and its covariance are left untouched.
// ---------------------------------------------------------------------------

void RobotEKF::updateLidar(double x, double y) {
    if (!initialized_) {
        init(x, y);
        return;
    }

    x_(0) = x;
    x_(1) = y;

    // Reset position uncertainty to LiDAR accuracy; zero cross-terms with velocity.
    P_(0, 0) = r_lidar_;
    P_(1, 1) = r_lidar_;
    P_(0, 1) = P_(1, 0) = 0.0;
    P_(0, 2) = P_(2, 0) = 0.0;
    P_(0, 3) = P_(3, 0) = 0.0;
    P_(1, 2) = P_(2, 1) = 0.0;
    P_(1, 3) = P_(3, 1) = 0.0;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

double RobotEKF::getX()  const { return x_(0); }
double RobotEKF::getY()  const { return x_(1); }
double RobotEKF::getVx() const { return x_(2); }
double RobotEKF::getVy() const { return x_(3); }

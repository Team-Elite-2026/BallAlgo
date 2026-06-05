#include "ball_kalman.hpp"

// ---------------------------------------------------------------------------
// State transition: constant-velocity model
//
//   [ x  ]   [1  0  dt  0 ] [ x  ]
//   [ y  ] = [0  1  0  dt] [ y  ]
//   [ vx ]   [0  0  1   0] [ vx ]
//   [ vy ]   [0  0  0   1] [ vy ]
// ---------------------------------------------------------------------------
Eigen::MatrixXd BallKalman::makeA(double dt) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(4, 4);
    A(0, 2) = dt;
    A(1, 3) = dt;
    return A;
}

BallKalman::BallKalman(double process_noise, double measurement_noise)
    : initialized_(false)
{
    // Observation: measures [x, y] from state [x, y, vx, vy]
    C_ = Eigen::MatrixXd::Zero(2, 4);
    C_(0, 0) = 1.0;
    C_(1, 1) = 1.0;

    // Process noise Q: position terms scale with dt^2/dt, velocity terms with dt.
    // We use a simple diagonal here — tune process_noise to balance lag vs. noise.
    Q_ = Eigen::MatrixXd::Zero(4, 4);
    Q_(0, 0) = process_noise * 0.25;   // x position
    Q_(1, 1) = process_noise * 0.25;   // y position
    Q_(2, 2) = process_noise * 1.0;    // vx  (higher: velocity can change quickly)
    Q_(3, 3) = process_noise * 1.0;    // vy

    // Measurement noise R: camera pixel/position uncertainty
    R_ = Eigen::MatrixXd::Identity(2, 2) * measurement_noise;

    // Initial covariance P0: reflect high uncertainty in velocity at startup
    P0_ = Eigen::MatrixXd::Zero(4, 4);
    P0_(0, 0) = 10.0;
    P0_(1, 1) = 10.0;
    P0_(2, 2) = 100.0;
    P0_(3, 3) = 100.0;

    // Construct the filter with a placeholder A (rebuilt on each update with real dt)
    kf_ = KalmanFilter(1.0, makeA(1.0), C_, Q_, R_, P0_);
}

void BallKalman::init(double x, double y) {
    Eigen::VectorXd x0(4);
    x0 << x, y, 0.0, 0.0;
    kf_.init(0.0, x0);
    initialized_ = true;
}

void BallKalman::update(double x, double y, double dt) {
    if (!initialized_) {
        init(x, y);
        return;
    }

    Eigen::VectorXd measurement(2);
    measurement << x, y;

    kf_.update(measurement, dt, makeA(dt));
}

double BallKalman::getX()  const { return kf_.state()(0); }
double BallKalman::getY()  const { return kf_.state()(1); }
double BallKalman::getVx() const { return kf_.state()(2); }
double BallKalman::getVy() const { return kf_.state()(3); }

/**
 * Kalman filter implementation using Eigen.
 * Source: https://github.com/hmartiro/kalman-cpp
 * Author: Hayk Martirosyan (2014)
 */

#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
  /**
   * @param dt   Fixed time step (seconds). Ignored when using the variable-dt update overload.
   * @param A    State transition matrix (n x n)
   * @param C    Observation matrix (m x n)
   * @param Q    Process noise covariance (n x n)
   * @param R    Measurement noise covariance (m x m)
   * @param P    Initial estimate error covariance (n x n)
   */
  KalmanFilter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  KalmanFilter();

  void init();
  void init(double t0, const Eigen::VectorXd& x0);

  // Update with a constant dt (uses A set at construction time).
  void update(const Eigen::VectorXd& y);

  // Update with a new dt and dynamics matrix (use when dt is variable).
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  Eigen::VectorXd state() const { return x_hat; }
  double time() const { return t; }

private:
  Eigen::MatrixXd A, C, Q, R, P, K, P0;
  int m, n;
  double t0, t;
  double dt;
  bool initialized;
  Eigen::MatrixXd I;
  Eigen::VectorXd x_hat, x_hat_new;
};

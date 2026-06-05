#pragma once

#include "kalman.hpp"

/**
 * Kalman filter for ball state estimation.
 *
 * State:       [x, y, vx, vy]
 * Measurement: [x, y]  (camera provides position only; velocity is estimated)
 *
 * Coordinate system: 0° = front of robot, 90° = right (see CLAUDE.md).
 * x = lateral axis (positive right), y = forward axis (positive forward).
 */
class BallKalman {
public:
    /**
     * @param process_noise      Scales Q; higher = trust the model less, react faster to motion changes.
     * @param measurement_noise  Scales R; higher = trust camera less, smoother output.
     */
    explicit BallKalman(double process_noise = 5.0, double measurement_noise = 2.0);

    /**
     * Seed the filter with a known starting position.
     * Must be called before the first update(), otherwise the first measurement
     * is used automatically.
     */
    void init(double x, double y);

    /**
     * Feed one camera measurement.
     * @param x   Ball x position (field coords, pixels or metres — consistent units).
     * @param y   Ball y position.
     * @param dt  Seconds elapsed since the last call (camera frame period).
     */
    void update(double x, double y, double dt);

    double getX()  const;
    double getY()  const;
    double getVx() const;
    double getVy() const;

    bool isInitialized() const { return initialized_; }

private:
    KalmanFilter kf_;
    bool initialized_;

    // C and P0 are dt-independent; A is rebuilt each update.
    Eigen::MatrixXd C_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P0_;

    static Eigen::MatrixXd makeA(double dt);
};

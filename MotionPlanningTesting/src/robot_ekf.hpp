/**
 * Extended Kalman Filter for robot position/velocity estimation.
 *
 * State vector: [x, y, vx, vy]  — world-frame position (m) and velocity (m/s)
 *
 * Sensor inputs and authority:
 *   Mouse sensor (~1 kHz): body-frame lateral/forward velocity.
 *                          Rotated to world frame via IMU heading for the velocity update.
 *   LiDAR         (~15 Hz): absolute x/y position — HARD SNAP on each frame.
 *   IMU heading  (continuous): degrees, 0=facing forward (+y), 90=facing right (+x), clockwise.
 *                              Treated as a known external parameter, not an EKF state.
 *
 * Field coordinate system (CLAUDE.md):
 *   origin  = centre of field
 *   +x      = right (east)
 *   +y      = forward (north)
 *   heading = 0° → facing +y; 90° → facing +x; measured clockwise
 *
 * Mouse sensor body frame:
 *   vx_body = lateral velocity  (positive = robot-right)
 *   vy_body = forward velocity  (positive = robot-forward)
 *
 * Body-to-world rotation (heading θ in radians):
 *   vx_world =  cos(θ)·vx_body + sin(θ)·vy_body
 *   vy_world = −sin(θ)·vx_body + cos(θ)·vy_body
 *
 * Process model: constant-velocity (DWNA — Discrete White Noise Acceleration).
 * EKF measurement model: the heading-dependent rotation makes H time-varying,
 *   so we recompute it at every mouse update — the defining characteristic of an EKF.
 */

#pragma once
#include <Eigen/Dense>
#include <cmath>

class RobotEKF {
public:
    /**
     * @param sigma_a   Acceleration process noise (m/s²).  Drives the DWNA Q matrix —
     *                  higher = allow the model to deviate more between sensor updates.
     * @param r_mouse   Mouse velocity measurement noise variance (m²/s²).
     *                  Lower = trust the mouse sensor more.
     * @param r_lidar   Position variance to restore after a LiDAR snap (m²).
     *                  Reflects LiDAR absolute accuracy (~0.05 m → 2.5e-3 m²).
     */
    RobotEKF(double sigma_a = 0.5,
             double r_mouse = 1e-4,
             double r_lidar = 2.5e-3);

    /** Seed the filter with a known position and zero velocity. */
    void init(double x, double y);

    /**
     * Mouse-sensor + predict update — call at ~1 kHz.
     *
     * Propagates the position estimate forward by dt using the current velocity estimate,
     * then corrects the velocity estimate from the (heading-rotated) mouse observation.
     *
     * @param vx_body     Lateral velocity in robot frame (m/s, +right).
     * @param vy_body     Forward velocity in robot frame (m/s, +forward).
     * @param heading_deg IMU heading in degrees (0 = facing +y, 90 = facing +x, clockwise).
     * @param dt          Time since last call (seconds).
     */
    void updateMouse(double vx_body, double vy_body,
                     double heading_deg, double dt);

    /**
     * LiDAR position snap — call at ~15 Hz.
     *
     * Hard-resets x/y to the LiDAR reading and restores position covariance to r_lidar.
     * Velocity estimate and velocity covariance are preserved.
     * If the filter has not been initialised yet, this call also initialises it.
     *
     * @param x  Measured x position (field frame, m).
     * @param y  Measured y position (field frame, m).
     */
    void updateLidar(double x, double y);

    double getX()  const;   ///< Estimated x position (m)
    double getY()  const;   ///< Estimated y position (m)
    double getVx() const;   ///< Estimated world-frame x velocity (m/s)
    double getVy() const;   ///< Estimated world-frame y velocity (m/s)

    bool isInitialized() const { return initialized_; }

    /** Full 4×4 state covariance matrix — useful for diagnostics. */
    const Eigen::Matrix4d& covariance() const { return P_; }

private:
    void predict(double dt);

    Eigen::Vector4d x_;       // [x, y, vx_world, vy_world]
    Eigen::Matrix4d P_;       // state covariance
    double          sigma_a2_;// σ_a² — scales DWNA process noise Q
    Eigen::Matrix2d R_mouse_; // mouse velocity measurement noise
    double          r_lidar_; // position variance after LiDAR snap
    bool            initialized_;

    static constexpr double kDeg2Rad = M_PI / 180.0;
};

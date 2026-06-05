/**
 * Cubic Hermite spline over an A* path.
 *
 * Each A* grid node is a Hermite control point.
 *   - Boundary tangents : set directly from the robot's actual velocity vector.
 *     T(0) = (vx_start, vy_start) and T(end) = (vx_end, vy_end) so the
 *     spline derivative matches the robot's physical velocity.  A robot in
 *     motion cannot change velocity direction instantaneously — enforcing
 *     S'(0) = v_start is the only way to produce a physically realisable path.
 *   - Interior tangents : Catmull-Rom central difference, scaled to the
 *     average of adjacent chord lengths (C1 continuity, no overshoot).
 *
 * The heading at each sampled point is the robot ORIENTATION (facing angle),
 * linearly interpolated between the heading-schedule values at the two
 * bounding A* nodes. For a holonomic robot, orientation and direction of
 * travel are independent — the spline gives position, the heading schedule
 * gives facing angle. The spline derivative (direction of travel) is
 * available via evaluate() but is NOT stored in SplinePoint::heading.
 *
 * Usage:
 *   auto spline = HermiteSpline::build(astar_result, samples, vx0, vy0, vxN, vyN);
 *   // spline[i].x/y  — world position (cm)
 *   // spline[i].heading — robot facing angle (degrees, same convention as A*)
 */

#pragma once
#include "astar.hpp"
#include <vector>

struct SplinePoint {
    double x;        // world x (cm), +x = right
    double y;        // world y (cm), +y = forward
    double heading;  // degrees, 0° = facing +y, 90° = facing +x, clockwise
};

// Evaluated spline state and derivatives at a single s value.
// All positional quantities in metres; angular in radians; kappa in 1/m.
struct SplineDerivState {
    double x, y;             // position (m)
    double theta;            // orientation, radians, unwrapped
    double dx_ds, dy_ds;     // ∂x/∂s, ∂y/∂s  (m / s-unit)
    double dtheta_ds;        // ∂θ/∂s          (rad / s-unit)
    double d2x_ds2, d2y_ds2; // ∂²x/∂s²        (m / s-unit²)
    double d2theta_ds2;      // ∂²θ/∂s²        (rad / s-unit²)
    double kappa;            // curvature       (1/m)
};

// Internal per-node storage used by HermiteSplineData::evalState().
struct HermiteNode {
    double wx, wy;    // world position (cm)
    double theta_rad; // orientation, radians, unwrapped across path
    double ptx, pty;  // position tangent w.r.t. local segment t  (cm)
    double pttheta;   // theta tangent   w.r.t. local segment t  (rad)
};

// Full spline representation: discrete samples (for visualisation/output)
// plus the raw node data needed for continuous analytical evaluation.
struct HermiteSplineData {
    std::vector<SplinePoint> samples;  // cm / degrees — backward-compatible
    std::vector<HermiteNode> nodes;    // one entry per A* node
    int num_segments = 0;

    // Evaluate position, heading, first/second derivatives, and curvature
    // at global parameter s ∈ [0, 1].  s is clamped to [0, 1].
    SplineDerivState evalState(double s) const;
};

class HermiteSpline {
public:
    static constexpr int DEFAULT_SAMPLES = 5; // output points per A* segment

    /**
     * Build the full spline from an A* result.
     *
     * Returns (path.size()-1)*samples_per_segment + 1 points.
     * Points at indices 0, S, 2S, ... (S = samples_per_segment) coincide
     * exactly with the original A* grid nodes.
     *
     * @param vx_start  Robot's current world-x velocity (cm/s).  Sets S'(0)
     *                  so the path leaves the start in the robot's actual
     *                  direction of travel.  Pass 0 when starting from rest.
     * @param vy_start  Robot's current world-y velocity (cm/s).
     * @param vx_end    Desired world-x velocity at the end pose (cm/s).
     * @param vy_end    Desired world-y velocity at the end pose (cm/s).
     */
    static std::vector<SplinePoint> build(const AStarResult& result,
                                          int    samples_per_segment = DEFAULT_SAMPLES,
                                          double vx_start = 0.0,
                                          double vy_start = 0.0,
                                          double vx_end   = 0.0,
                                          double vy_end   = 0.0);

    /**
     * Like build(), but also stores per-node data so the spline can be
     * evaluated analytically at any s ∈ [0,1] via HermiteSplineData::evalState().
     *
     * Theta is Hermite-interpolated (cubic) using Catmull-Rom tangents at
     * interior nodes and one-sided differences at the endpoints.
     */
    static HermiteSplineData buildData(const AStarResult& result,
                                       int    samples_per_segment = DEFAULT_SAMPLES,
                                       double vx_start = 0.0,
                                       double vy_start = 0.0,
                                       double vx_end   = 0.0,
                                       double vy_end   = 0.0);

    /**
     * Evaluate one cubic Hermite segment at t ∈ [0, 1].
     *
     * Returns position (x, y) via Hermite interpolation.
     * Returns heading = direction of TRAVEL (atan2 of spline derivative) —
     * useful for non-holonomic robots. For holonomic orientation control use
     * the heading values stored by build() instead.
     *
     * @param t0_scale  Tangent magnitude at the start node.
     * @param t1_scale  Tangent magnitude at the end node.
     */
    static SplinePoint evaluate(double x0, double y0, double h0_deg,
                                double x1, double y1, double h1_deg,
                                double t0_scale, double t1_scale,
                                double t);
};

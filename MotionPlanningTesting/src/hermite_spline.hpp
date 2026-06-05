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

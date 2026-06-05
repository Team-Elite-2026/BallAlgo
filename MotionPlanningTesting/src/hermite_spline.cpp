#include "hermite_spline.hpp"
#include <cmath>
#include <algorithm>

// ── Cubic Hermite basis polynomials and their derivatives ────────────────────
//
//  p(t) = h00·P0 + h10·T0 + h01·P1 + h11·T1   (position)
//  v(t) = d/dt p(t)                              (1st derivative)
//  a(t) = d²/dt² p(t)                            (2nd derivative)
//
// where T0, T1 are the tangent vectors at P0 and P1.

static double h00 (double t) { return  2*t*t*t - 3*t*t + 1; }
static double h10 (double t) { return    t*t*t - 2*t*t + t; }
static double h01 (double t) { return -2*t*t*t + 3*t*t;     }
static double h11 (double t) { return    t*t*t -   t*t;     }

static double dh00(double t) { return  6*t*t - 6*t;     }
static double dh10(double t) { return  3*t*t - 4*t + 1; }
static double dh01(double t) { return -6*t*t + 6*t;     }
static double dh11(double t) { return  3*t*t - 2*t;     }

static double ddh00(double t) { return  12*t - 6; }
static double ddh10(double t) { return   6*t - 4; }
static double ddh01(double t) { return -12*t + 6; }
static double ddh11(double t) { return   6*t - 2; }

// ── Single-segment evaluation ─────────────────────────────────────────────────

SplinePoint HermiteSpline::evaluate(double x0, double y0, double h0_deg,
                                    double x1, double y1, double h1_deg,
                                    double t0_scale, double t1_scale,
                                    double t) {
    const double h0 = h0_deg * (M_PI / 180.0);
    const double h1 = h1_deg * (M_PI / 180.0);

    // Tangent vectors: direction from heading, magnitude from per-node scale.
    // Heading convention: 0°=+y, 90°=+x, CW  →  tx = sin(h),  ty = cos(h)
    const double tx0 = std::sin(h0) * t0_scale;
    const double ty0 = std::cos(h0) * t0_scale;
    const double tx1 = std::sin(h1) * t1_scale;
    const double ty1 = std::cos(h1) * t1_scale;

    // Position
    const double x = h00(t)*x0 + h10(t)*tx0 + h01(t)*x1 + h11(t)*tx1;
    const double y = h00(t)*y0 + h10(t)*ty0 + h01(t)*y1 + h11(t)*ty1;

    // Velocity (spline derivative w.r.t. t)
    const double vx = dh00(t)*x0 + dh10(t)*tx0 + dh01(t)*x1 + dh11(t)*tx1;
    const double vy = dh00(t)*y0 + dh10(t)*ty0 + dh01(t)*y1 + dh11(t)*ty1;

    // Heading from velocity direction (same convention as A*)
    double heading = std::atan2(vx, vy) * (180.0 / M_PI);
    if (heading <   0.0) heading += 360.0;
    if (heading >= 360.0) heading -= 360.0;

    return {x, y, heading};
}

// ── Full spline build ─────────────────────────────────────────────────────────

std::vector<SplinePoint> HermiteSpline::build(const AStarResult& result,
                                              int    samples_per_segment,
                                              double vx_start,
                                              double vy_start,
                                              double vx_end,
                                              double vy_end) {
    if (!result.found || result.path.empty()) return {};

    const auto& path    = result.path;
    const auto& heading = result.heading;
    const size_t n      = path.size();

    // World coordinates for each A* node
    std::vector<double> wx(n), wy(n);
    for (size_t i = 0; i < n; ++i)
        AStar::gridToWorld(path[i], wx[i], wy[i]);

    if (n == 1) return {{wx[0], wy[0], heading[0]}};

    // Per-node tangent scale = average of adjacent segment chord lengths.
    // Using the same scale on both sides of each interior node gives C1
    // continuity (velocity is continuous at the join).
    std::vector<double> scale(n);
    for (size_t i = 0; i < n; ++i) {
        const double prev = (i > 0)
            ? std::hypot(wx[i] - wx[i-1], wy[i] - wy[i-1]) : 0.0;
        const double next = (i + 1 < n)
            ? std::hypot(wx[i+1] - wx[i], wy[i+1] - wy[i]) : 0.0;

        if      (i == 0)     scale[i] = next;
        else if (i == n - 1) scale[i] = prev;
        else                 scale[i] = (prev + next) * 0.5;
    }

    // Position tangents.
    // Boundary nodes use the caller-supplied velocity vectors directly so that
    // S'(0) = v_start and S'(end) = v_end.  A robot already in motion cannot
    // snap to a new direction — the spline must leave the start in the robot's
    // actual direction of travel and arrive at the end with the desired velocity.
    // Interior nodes use Catmull-Rom central differences (C1 continuity).
    std::vector<double> ptx(n), pty(n);

    ptx[0] = vx_start;
    pty[0] = vy_start;

    ptx[n - 1] = vx_end;
    pty[n - 1] = vy_end;

    for (size_t i = 1; i + 1 < n; ++i) {
        // Central difference: direction from P_{i-1} to P_{i+1}
        const double dx  = wx[i+1] - wx[i-1];
        const double dy  = wy[i+1] - wy[i-1];
        const double len = std::hypot(dx, dy);
        if (len < 1e-9) { ptx[i] = pty[i] = 0.0; continue; }
        ptx[i] = (dx / len) * scale[i];
        pty[i] = (dy / len) * scale[i];
    }

    std::vector<SplinePoint> spline;
    spline.reserve((n - 1) * samples_per_segment + 1);

    spline.push_back({wx[0], wy[0], heading[0]});

    for (size_t i = 0; i + 1 < n; ++i) {
        double h_delta = heading[i+1] - heading[i];
        while (h_delta >  180.0) h_delta -= 360.0;
        while (h_delta < -180.0) h_delta += 360.0;

        for (int s = 1; s <= samples_per_segment; ++s) {
            const double t = static_cast<double>(s) / samples_per_segment;

            // Position: Hermite basis with Catmull-Rom tangents
            const double x = h00(t)*wx[i]   + h10(t)*ptx[i] +
                             h01(t)*wx[i+1] + h11(t)*ptx[i+1];
            const double y = h00(t)*wy[i]   + h10(t)*pty[i] +
                             h01(t)*wy[i+1] + h11(t)*pty[i+1];

            // Orientation: linear heading schedule (independent of travel direction)
            double h = heading[i] + t * h_delta;
            while (h <   0.0) h += 360.0;
            while (h >= 360.0) h -= 360.0;

            spline.push_back({x, y, h});
        }
    }

    return spline;
}

// ── HermiteSplineData::evalState ──────────────────────────────────────────────

SplineDerivState HermiteSplineData::evalState(double s) const {
    if (nodes.empty() || num_segments == 0)
        return {};

    s = std::max(0.0, std::min(1.0, s));

    // Map s → segment index and local t ∈ [0,1].
    // Chain rule: d/ds = d/dt * N,  d²/ds² = d²/dt² * N²
    const int    N   = num_segments;
    const double sN  = s * N;
    const int    seg = std::min(static_cast<int>(sN), N - 1);
    const double t   = sN - seg;

    const HermiteNode& n0 = nodes[seg];
    const HermiteNode& n1 = nodes[seg + 1];

    // Hermite basis values
    const double p00 =  h00(t), p10 =  h10(t), p01 =  h01(t), p11 =  h11(t);
    const double d00 = dh00(t), d10 = dh10(t), d01 = dh01(t), d11 = dh11(t);
    const double e00 = ddh00(t), e10 = ddh10(t), e01 = ddh01(t), e11 = ddh11(t);

    // ── position (cm) → convert to metres at the end
    const double x_cm  = p00*n0.wx + p10*n0.ptx + p01*n1.wx + p11*n1.ptx;
    const double y_cm  = p00*n0.wy + p10*n0.pty + p01*n1.wy + p11*n1.pty;

    // ── theta (rad, unwrapped)
    const double theta = p00*n0.theta_rad + p10*n0.pttheta
                       + p01*n1.theta_rad + p11*n1.pttheta;

    // ── first derivatives w.r.t. t
    const double dxdt      = d00*n0.wx    + d10*n0.ptx    + d01*n1.wx    + d11*n1.ptx;
    const double dydt      = d00*n0.wy    + d10*n0.pty    + d01*n1.wy    + d11*n1.pty;
    const double dthetadt  = d00*n0.theta_rad + d10*n0.pttheta
                           + d01*n1.theta_rad + d11*n1.pttheta;

    // ── second derivatives w.r.t. t
    const double d2xdt2     = e00*n0.wx    + e10*n0.ptx    + e01*n1.wx    + e11*n1.ptx;
    const double d2ydt2     = e00*n0.wy    + e10*n0.pty    + e01*n1.wy    + e11*n1.pty;
    const double d2thetadt2 = e00*n0.theta_rad + e10*n0.pttheta
                            + e01*n1.theta_rad + e11*n1.pttheta;

    // ── chain rule: derivatives w.r.t. s = derivatives w.r.t. t × N (or N²)
    // Convert positional values cm → m (* 0.01)
    const double CM_TO_M = 0.01;
    const double dx_ds     = dxdt  * N * CM_TO_M;
    const double dy_ds     = dydt  * N * CM_TO_M;
    const double dtheta_ds = dthetadt * N;

    const double d2x_ds2     = d2xdt2     * N * N * CM_TO_M;
    const double d2y_ds2     = d2ydt2     * N * N * CM_TO_M;
    const double d2theta_ds2 = d2thetadt2 * N * N;

    // ── curvature κ = |x'y'' − y'x''| / (x'² + y'²)^(3/2)  (1/m, uses m-based derivs)
    const double denom_sq  = dx_ds*dx_ds + dy_ds*dy_ds;
    const double denom_1p5 = denom_sq > 1e-18 ? std::pow(denom_sq, 1.5) : 0.0;
    const double kappa     = denom_1p5 > 1e-18
                           ? std::abs(dx_ds*d2y_ds2 - dy_ds*d2x_ds2) / denom_1p5
                           : 0.0;

    return { x_cm * CM_TO_M, y_cm * CM_TO_M, theta,
             dx_ds, dy_ds, dtheta_ds,
             d2x_ds2, d2y_ds2, d2theta_ds2,
             kappa };
}

// ── HermiteSpline::buildData ──────────────────────────────────────────────────

HermiteSplineData HermiteSpline::buildData(const AStarResult& result,
                                            int    samples_per_segment,
                                            double vx_start,
                                            double vy_start,
                                            double vx_end,
                                            double vy_end) {
    HermiteSplineData data;

    if (!result.found || result.path.empty()) return data;

    const auto& path    = result.path;
    const auto& heading = result.heading;
    const size_t n      = path.size();

    // World coordinates for each A* node (cm)
    std::vector<double> wx(n), wy(n);
    for (size_t i = 0; i < n; ++i)
        AStar::gridToWorld(path[i], wx[i], wy[i]);

    if (n == 1) {
        const double th = heading[0] * (M_PI / 180.0);
        data.nodes.push_back({wx[0], wy[0], th, 0.0, 0.0, 0.0});
        data.num_segments = 0;
        data.samples.push_back({wx[0], wy[0], heading[0]});
        return data;
    }

    // Per-node tangent scale = average of adjacent chord lengths (matches build())
    std::vector<double> scale(n);
    for (size_t i = 0; i < n; ++i) {
        const double prev = (i > 0)     ? std::hypot(wx[i]-wx[i-1], wy[i]-wy[i-1]) : 0.0;
        const double next = (i+1 < n)   ? std::hypot(wx[i+1]-wx[i], wy[i+1]-wy[i]) : 0.0;
        if      (i == 0)     scale[i] = next;
        else if (i == n - 1) scale[i] = prev;
        else                 scale[i] = (prev + next) * 0.5;
    }

    // Position tangents (same as build())
    std::vector<double> ptx(n), pty(n);
    ptx[0] = vx_start;  pty[0] = vy_start;
    ptx[n-1] = vx_end;  pty[n-1] = vy_end;
    for (size_t i = 1; i + 1 < n; ++i) {
        const double dx  = wx[i+1] - wx[i-1];
        const double dy  = wy[i+1] - wy[i-1];
        const double len = std::hypot(dx, dy);
        if (len < 1e-9) { ptx[i] = pty[i] = 0.0; continue; }
        ptx[i] = (dx / len) * scale[i];
        pty[i] = (dy / len) * scale[i];
    }

    // Theta: convert heading (degrees) to radians, then unwrap across path
    std::vector<double> theta_rad(n);
    theta_rad[0] = heading[0] * (M_PI / 180.0);
    for (size_t i = 1; i < n; ++i) {
        double delta = heading[i] * (M_PI / 180.0) - theta_rad[i-1];
        while (delta >  M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        theta_rad[i] = theta_rad[i-1] + delta;
    }

    // Theta tangents: Catmull-Rom central difference for interior nodes,
    // one-sided finite difference at boundaries.
    std::vector<double> pttheta(n);
    pttheta[0]     = theta_rad[1] - theta_rad[0];          // forward difference
    pttheta[n - 1] = theta_rad[n-1] - theta_rad[n-2];      // backward difference
    for (size_t i = 1; i + 1 < n; ++i)
        pttheta[i] = (theta_rad[i+1] - theta_rad[i-1]) * 0.5;  // central difference

    // Build HermiteNode array
    data.nodes.resize(n);
    for (size_t i = 0; i < n; ++i)
        data.nodes[i] = { wx[i], wy[i], theta_rad[i], ptx[i], pty[i], pttheta[i] };

    data.num_segments = static_cast<int>(n) - 1;

    // Build discrete samples (identical logic to build())
    data.samples.reserve((n - 1) * samples_per_segment + 1);
    data.samples.push_back({wx[0], wy[0], heading[0]});

    for (size_t i = 0; i + 1 < n; ++i) {
        double h_delta = heading[i+1] - heading[i];
        while (h_delta >  180.0) h_delta -= 360.0;
        while (h_delta < -180.0) h_delta += 360.0;

        for (int s = 1; s <= samples_per_segment; ++s) {
            const double t  = static_cast<double>(s) / samples_per_segment;
            const double xp = h00(t)*wx[i] + h10(t)*ptx[i]
                            + h01(t)*wx[i+1] + h11(t)*ptx[i+1];
            const double yp = h00(t)*wy[i] + h10(t)*pty[i]
                            + h01(t)*wy[i+1] + h11(t)*pty[i+1];
            double hd = heading[i] + t * h_delta;
            while (hd <   0.0) hd += 360.0;
            while (hd >= 360.0) hd -= 360.0;
            data.samples.push_back({xp, yp, hd});
        }
    }

    return data;
}

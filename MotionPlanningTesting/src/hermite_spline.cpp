#include "hermite_spline.hpp"
#include <cmath>

// ── Cubic Hermite basis polynomials and their derivatives ────────────────────
//
//  p(t) = h00·P0 + h10·T0 + h01·P1 + h11·T1   (position)
//  v(t) = d/dt p(t)                              (velocity → heading)
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

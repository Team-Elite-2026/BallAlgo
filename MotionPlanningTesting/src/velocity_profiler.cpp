#include "velocity_profiler.hpp"
#include <algorithm>
#include <cmath>

// ── Internal motor helpers ─────────────────────────────────────────────────────

static float maxWheelVelocity(const ProfilerConfig& cfg) {
    const float omega_noload  = (cfg.datasheet_no_load_rpm * 2.0f * M_PI) / 60.0f;
    const float omega_loaded  = omega_noload - cfg.load_comp_omega;
    return omega_loaded * cfg.r_wheel;  // m/s
}

// ── Path velocity ceiling ─────────────────────────────────────────────────────
// Returns the maximum ṡ at this spline state imposed by:
//   (a) motor back-EMF / wheel speed limits
//   (b) lateral traction limit
//
// All positional spline derivatives must be in metres.

static float calcCeiling(const SplineDerivState& st, const ProfilerConfig& cfg) {
    const float max_wv = maxWheelVelocity(cfg);
    float s_dot_lim = 9999.0f;

    // (a) Each wheel's contribution to the path velocity
    for (int i = 0; i < 4; ++i) {
        const float proj_trans = std::sin(cfg.alpha_wheel[i]) * (float)st.dx_ds
                               + std::cos(cfg.alpha_wheel[i]) * (float)st.dy_ds;
        const float proj_rot   = -cfg.R_chassis * (float)st.dtheta_ds;
        const float K_i        = proj_trans + proj_rot;
        if (std::abs(K_i) > 0.001f)
            s_dot_lim = std::min(s_dot_lim, max_wv / std::abs(K_i));
    }

    // (b) Centripetal traction: v²·κ ≤ a_max_grip  →  ṡ ≤ √(a_grip/κ) / ‖(dx,dy)/ds‖
    if (std::abs(st.kappa) > 0.001f) {
        const float v_curve    = std::sqrt(cfg.a_max_grip_accel / (float)std::abs(st.kappa));
        const float path_ratio = std::sqrt((float)(st.dx_ds*st.dx_ds + st.dy_ds*st.dy_ds));
        if (path_ratio > 0.001f)
            s_dot_lim = std::min(s_dot_lim, v_curve / path_ratio);
    }

    return s_dot_lim;
}

// ── Dynamic acceleration limits ────────────────────────────────────────────────
// Returns {max_s_ddot_accel, max_s_ddot_decel} at the current state.
//
// Derived from:
//   wheel_vel   = K_i · ṡ
//   wheel_accel = K_i · ṡ̈ + C_i · ṡ²
//
// Voltage budget:  -V_bus ≤ kS·sign(ω) + kV·ω + kA·α_wheel ≤ V_bus
// After eliminating the kS and kV terms already consumed by the current
// operating point, the remaining headroom bounds ṡ̈.

struct DynLimits { float accel; float decel; };

static DynLimits calcDynLimits(const SplineDerivState& st,
                                float s_dot,
                                float vx, float vy, float omega,
                                const ProfilerConfig& cfg) {
    DynLimits lim{ 999.0f, 999.0f };
    const float s_dot_sq = s_dot * s_dot;
    const float EPS = 0.001f;

    for (int i = 0; i < 4; ++i) {
        // ── Current wheel angular velocity
        const float v_wheel   = std::sin(cfg.alpha_wheel[i]) * vx
                              + std::cos(cfg.alpha_wheel[i]) * vy
                              - cfg.R_chassis * omega;
        const float w_motor   = v_wheel / cfg.r_wheel;

        // ── Voltage headroom after back-EMF and static friction
        const float v_emf     = cfg.kV * w_motor;
        const float friction  = (w_motor >  EPS) ?  cfg.kS
                              : (w_motor < -EPS) ? -cfg.kS : 0.0f;
        const float v_pos     =  cfg.V_bus - friction - v_emf;
        const float v_neg     = -cfg.V_bus - friction - v_emf;

        // Voltage headroom → wheel linear acceleration limits
        const float a_wh_pos  = (v_pos * cfg.r_wheel) / cfg.kA;
        const float a_wh_neg  = (v_neg * cfg.r_wheel) / cfg.kA;

        // ── Geometric coefficients  (K_i·ṡ̈ + C_i·ṡ² = wheel_accel)
        const float K_i = std::sin(cfg.alpha_wheel[i]) * (float)st.dx_ds
                        + std::cos(cfg.alpha_wheel[i]) * (float)st.dy_ds
                        - cfg.R_chassis * (float)st.dtheta_ds;

        const float C_i = std::sin(cfg.alpha_wheel[i]) * (float)st.d2x_ds2
                        + std::cos(cfg.alpha_wheel[i]) * (float)st.d2y_ds2
                        - cfg.R_chassis * (float)st.d2theta_ds2;

        const float curve_term = C_i * s_dot_sq;  // centripetal / path-curvature demand

        if (std::abs(K_i) > EPS) {
            // Solve:  a_wh_neg ≤ K_i·ṡ̈ + curve_term ≤ a_wh_pos
            const float b1 = (a_wh_pos - curve_term) / K_i;
            const float b2 = (a_wh_neg - curve_term) / K_i;
            const float local_max = std::max(b1, b2);
            const float local_min = std::min(b1, b2);

            lim.accel = (local_max > 0.0f)
                      ? std::min(lim.accel, local_max) : 0.0f;
            lim.decel = (local_min < 0.0f)
                      ? std::min(lim.decel, std::abs(local_min)) : 0.0f;
        } else {
            // Wheel doesn't contribute to ṡ̈ — check if curve term alone overloads it
            if (curve_term > a_wh_pos || curve_term < a_wh_neg) {
                lim.accel = 0.0f;
                lim.decel = 0.0f;
            }
        }
    }

    // ── Traction limit on tangential acceleration
    // a_tangential ≤ √(a_grip² − a_lateral²)
    const float path_ratio = std::sqrt((float)(st.dx_ds*st.dx_ds + st.dy_ds*st.dy_ds));
    if (path_ratio > EPS) {
        const float v_sq      = vx*vx + vy*vy;
        const float a_lat     = std::min(v_sq * (float)std::abs(st.kappa),
                                         cfg.a_max_grip_accel);
        const float a_tang    = std::sqrt(std::max(0.0f,
                                    cfg.a_max_grip_accel * cfg.a_max_grip_accel
                                  - a_lat * a_lat));
        const float s_ddot_tr = a_tang / path_ratio;
        lim.accel = std::min(lim.accel, s_ddot_tr);
        lim.decel = std::min(lim.decel, s_ddot_tr);
    }

    return lim;
}

// ── VelocityProfiler::compute ─────────────────────────────────────────────────

std::vector<ProfilePoint> VelocityProfiler::compute(const HermiteSplineData& spline,
                                                     const ProfilerConfig& cfg) {
    if (spline.num_segments == 0 || spline.nodes.empty())
        return {};

    const int   N   = std::max(cfg.num_steps, 2);
    const float ds  = 1.0f / (N - 1);
    const float EPS = 1e-6f;

    // ── Pre-evaluate spline states at every s-grid point
    std::vector<SplineDerivState> states(N);
    for (int i = 0; i < N; ++i)
        states[i] = spline.evalState(static_cast<double>(i) / (N - 1));

    // ── Pre-compute hard velocity ceiling
    std::vector<float> ceiling(N);
    for (int i = 0; i < N; ++i)
        ceiling[i] = calcCeiling(states[i], cfg);

    // ── Convert boundary conditions to s-domain
    auto pathRatio = [&](int idx) -> float {
        return std::sqrt((float)(states[idx].dx_ds * states[idx].dx_ds
                               + states[idx].dy_ds * states[idx].dy_ds));
    };

    const float pr0 = pathRatio(0);
    float s_dot_start = (pr0 > EPS) ? cfg.v_start_mps / pr0 : 0.0f;
    s_dot_start = std::min(s_dot_start, ceiling[0]);

    const float prN = pathRatio(N - 1);
    float s_dot_end = (prN > EPS) ? cfg.v_end_mps / prN : 0.0f;
    s_dot_end = std::min(s_dot_end, ceiling[N - 1]);

    float s_ddot_start = (pr0 > EPS) ? cfg.a_start_mps2 / pr0 : 0.0f;

    // ── Forward pass ─────────────────────────────────────────────────────────
    // At each step: ramp s_ddot toward the dynamic accel limit (bounded by J_max),
    // then integrate with the kinematic equation ṡ_{i+1}² = ṡ_i² + 2·ṡ̈·ds.
    // The forward pass never applies negative acceleration — the backward pass
    // is responsible for the deceleration profile.

    std::vector<float> fwd(N);
    {
        float s_dot  = s_dot_start;
        float s_ddot = s_ddot_start;

        for (int i = 0; i < N; ++i) {
            fwd[i] = s_dot;
            if (i == N - 1) break;

            const auto& st = states[i];
            const float vx    = (float)st.dx_ds    * s_dot;
            const float vy    = (float)st.dy_ds    * s_dot;
            const float omega = (float)st.dtheta_ds * s_dot;

            const DynLimits dl = calcDynLimits(st, s_dot, vx, vy, omega, cfg);

            // Jerk limit: d(ṡ̈)/dt = J_max  →  Δṡ̈ = J_max · dt = J_max · ds / ṡ
            const float sdot_safe = std::max(s_dot, EPS);
            const float d_sdot   = cfg.J_max * ds / sdot_safe;

            // Track the target acceleration, ramp toward it with ±J_max
            const float target = dl.accel;
            s_ddot += (s_ddot < target) ? d_sdot : -d_sdot;
            s_ddot  = std::clamp(s_ddot, 0.0f, target);  // fwd pass: non-negative only

            // Kinematic step
            const float v2 = s_dot * s_dot + 2.0f * s_ddot * ds;
            s_dot = std::sqrt(std::max(0.0f, v2));
            s_dot = std::min(s_dot, ceiling[i + 1]);
        }
    }

    // ── Backward pass ─────────────────────────────────────────────────────────
    // Integrate from s=1 backward: asks "what is the maximum velocity at s_i
    // that still allows decelerating to bwd[i+1] in one ds step?"
    // Jerk is also limited symmetrically.

    std::vector<float> bwd(N);
    {
        float s_dot       = s_dot_end;
        float s_ddot_decel = 0.0f;

        bwd[N - 1] = s_dot;

        for (int i = N - 2; i >= 0; --i) {
            // Evaluate limits at the point we are integrating back toward (index i+1)
            const auto& st = states[i + 1];
            const float vx    = (float)st.dx_ds    * s_dot;
            const float vy    = (float)st.dy_ds    * s_dot;
            const float omega = (float)st.dtheta_ds * s_dot;

            const DynLimits dl = calcDynLimits(st, s_dot, vx, vy, omega, cfg);

            const float sdot_safe = std::max(s_dot, EPS);
            const float d_sdot   = cfg.J_max * ds / sdot_safe;

            // Ramp deceleration magnitude toward the physical limit
            s_ddot_decel += d_sdot;
            s_ddot_decel  = std::clamp(s_ddot_decel, 0.0f, dl.decel);

            // Propagate backward: what velocity at i reaches s_dot at i+1?
            const float v2 = s_dot * s_dot + 2.0f * s_ddot_decel * ds;
            s_dot = std::sqrt(std::max(0.0f, v2));
            s_dot = std::min(s_dot, ceiling[i]);

            bwd[i] = s_dot;
        }
    }

    // ── Intersection + output ─────────────────────────────────────────────────

    std::vector<ProfilePoint> result(N);

    for (int i = 0; i < N; ++i) {
        const float s_dot = std::min({ fwd[i], bwd[i], ceiling[i] });

        // ṡ̈ from finite difference of the final s_dot profile (forward difference,
        // backward at the last point).  Uses the kinematic relation:
        //   ṡ_{i+1}² = ṡ_i² + 2·ṡ̈·ds  →  ṡ̈ = (ṡ_{i+1}² − ṡ_i²) / (2·ds)
        float s_ddot;
        if (i < N - 1) {
            const float s_dot_next = std::min({ fwd[i+1], bwd[i+1], ceiling[i+1] });
            s_ddot = (s_dot_next * s_dot_next - s_dot * s_dot) / (2.0f * ds);
        } else {
            s_ddot = 0.0f;
        }

        const auto& st = states[i];

        // Physical velocities: v = (d·/ds) · ṡ
        const float vx    = (float)st.dx_ds     * s_dot;
        const float vy    = (float)st.dy_ds     * s_dot;
        const float omega = (float)st.dtheta_ds * s_dot;

        // Physical accelerations: a = (d²·/ds²)·ṡ² + (d·/ds)·ṡ̈
        const float ax    = (float)st.d2x_ds2     * s_dot * s_dot + (float)st.dx_ds     * s_ddot;
        const float ay    = (float)st.d2y_ds2     * s_dot * s_dot + (float)st.dy_ds     * s_ddot;
        const float alpha = (float)st.d2theta_ds2 * s_dot * s_dot + (float)st.dtheta_ds * s_ddot;

        result[i] = {
            static_cast<float>(i) / (N - 1),
            s_dot, s_ddot,
            vx, vy, omega,
            ax, ay, alpha
        };
    }

    return result;
}

/**
 * S-curve velocity profiler for a parametric Hermite spline.
 *
 * Operates on the path parameter s ∈ [0, 1] as the integration variable.
 * Produces the time-optimal ṡ(s) profile subject to:
 *   - Motor back-EMF voltage limits (per wheel)
 *   - Traction / lateral-grip limits
 *   - Maximum path jerk J_max (time domain)
 *
 * Algorithm (three passes over the s grid):
 *   1. Pre-compute ṡ_ceiling(s):  hard cap from motor speed and grip.
 *   2. Forward pass:  jerk-limited acceleration from ṡ_start, capped by ceiling.
 *   3. Backward pass: jerk-limited deceleration from ṡ_end, capped by ceiling.
 *   4. ṡ_final(s) = min(forward, backward, ceiling).
 *
 * Output: one ProfilePoint per s-grid step with physical velocities and
 * accelerations derived from ṡ and ṡ̈ via the spline chain rule.
 */

#pragma once
#include "hermite_spline.hpp"
#include <array>
#include <vector>

// ── Configuration ─────────────────────────────────────────────────────────────

struct ProfilerConfig {
    // ── Boundary conditions (physical SI units) ──────────────────────────────
    float v_start_mps  = 0.0f;   // initial robot speed  (m/s)
    float v_end_mps    = 0.0f;   // desired final speed  (m/s)
    float a_start_mps2 = 0.0f;   // initial path-tangent acceleration (m/s²)

    // ── Jerk limit (time domain) ──────────────────────────────────────────────
    // Maximum d(ṡ̈)/dt in s-units/s³.  Tune together with a_max_grip_accel.
    float J_max = 10.0f;

    // ── Motor model ───────────────────────────────────────────────────────────
    float datasheet_no_load_rpm = 1620.0f;  // motor datasheet no-load speed
    float load_comp_omega       =   50.0f;  // empirical load deduction (rad/s)
    float r_wheel               =  0.025f;  // wheel radius (m)
    float R_chassis             =  0.090f;  // chassis rotation radius (m)
    float kS                    =  0.50f;   // static-friction voltage (V)
    float kV                    =  0.08f;   // back-EMF constant (V·s/rad)
    float kA                    =  0.02f;   // acceleration constant (V·s²/rad)
    float V_bus                 = 12.0f;    // battery voltage (V)

    // Wheel angles from robot-forward (rad).  Convention: 0 = +y (forward).
    // Matches the holonomic layout in PipelineContext §Step 3.
    std::array<float,4> alpha_wheel = { -2.5307f, -0.6109f, 0.6109f, 2.5307f };

    // ── Traction ─────────────────────────────────────────────────────────────
    float a_max_grip_accel = 1.5f;   // total grip budget (m/s²)

    // ── Integration resolution ────────────────────────────────────────────────
    int num_steps = 500;    // number of uniform s steps
};

// ── Output ────────────────────────────────────────────────────────────────────

struct ProfilePoint {
    float s;         // path parameter [0, 1]
    float s_dot;     // path speed   ṡ  (1/s)
    float s_ddot;    // path accel   ṡ̈  (1/s²)

    // Physical velocities (global frame, m/s and rad/s)
    float vx, vy, omega;

    // Physical accelerations (global frame, m/s² and rad/s²)
    float ax, ay, alpha;
};

// ── Profiler ──────────────────────────────────────────────────────────────────

class VelocityProfiler {
public:
    static std::vector<ProfilePoint> compute(const HermiteSplineData& spline,
                                             const ProfilerConfig& cfg);
};

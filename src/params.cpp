#include "params.hpp"

#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

namespace ballalgo::params {

namespace {

Params g;

bool matchFloat(const std::string& json, const std::string& key, float& out) {
    std::regex re("\"" + key + "\"\\s*:\\s*([+-]?[0-9]*\\.?[0-9]+(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re)) return false;
    out = std::stof(m[1]);
    return true;
}

bool matchDouble(const std::string& json, const std::string& key, double& out) {
    std::regex re("\"" + key + "\"\\s*:\\s*([+-]?[0-9]*\\.?[0-9]+(?:[eE][+-]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(json, m, re)) return false;
    out = std::stod(m[1]);
    return true;
}

bool matchInt(const std::string& json, const std::string& key, int& out) {
    std::regex re("\"" + key + "\"\\s*:\\s*([+-]?[0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re)) return false;
    out = std::stoi(m[1]);
    return true;
}

bool matchUint64(const std::string& json, const std::string& key, uint64_t& out) {
    std::regex re("\"" + key + "\"\\s*:\\s*([0-9]+)");
    std::smatch m;
    if (!std::regex_search(json, m, re)) return false;
    out = std::stoull(m[1]);
    return true;
}

}  // namespace

bool load(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "[PARAMS] " << path << " not found — using built-in defaults\n";
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    const std::string json = ss.str();

    bool ok = true;
    auto warn = [&](bool found, const char* key) {
        if (!found) { std::cerr << "[PARAMS] missing key: " << key << "\n"; ok = false; }
    };

    // Motor model
    warn(matchFloat(json, "motor_no_load_rpm",     g.motorNoLoadRpm),     "motor_no_load_rpm");
    warn(matchFloat(json, "motor_load_comp_omega", g.motorLoadCompOmega), "motor_load_comp_omega");
    warn(matchFloat(json, "wheel_radius_m",        g.wheelRadiusM),       "wheel_radius_m");
    warn(matchFloat(json, "chassis_radius_m",      g.chassisRadiusM),     "chassis_radius_m");
    warn(matchFloat(json, "motor_ks",              g.motorKs),            "motor_ks");
    warn(matchFloat(json, "motor_kv",              g.motorKv),            "motor_kv");
    warn(matchFloat(json, "motor_ka",              g.motorKa),            "motor_ka");
    warn(matchFloat(json, "motor_vbus",            g.motorVBus),          "motor_vbus");
    warn(matchFloat(json, "amax_grip_accel",       g.aMaxGripAccel),      "amax_grip_accel");
    warn(matchFloat(json, "profiler_jmax",         g.profilerJMax),       "profiler_jmax");
    warn(matchFloat(json, "wheel_angle_0",         g.wheelAngle0),        "wheel_angle_0");
    warn(matchFloat(json, "wheel_angle_1",         g.wheelAngle1),        "wheel_angle_1");
    warn(matchFloat(json, "wheel_angle_2",         g.wheelAngle2),        "wheel_angle_2");
    warn(matchFloat(json, "wheel_angle_3",         g.wheelAngle3),        "wheel_angle_3");
    warn(matchFloat(json, "vmax_x",               g.vMaxX),              "vmax_x");
    warn(matchFloat(json, "vmax_y",               g.vMaxY),              "vmax_y");
    warn(matchFloat(json, "amax_x",               g.aMaxX),              "amax_x");
    warn(matchFloat(json, "amax_y",               g.aMaxY),              "amax_y");
    warn(matchFloat(json, "omega_max_rad_s",       g.omegaMaxRadS),       "omega_max_rad_s");

    // Ball Kalman filter
    warn(matchFloat(json,  "ball_kf_meas_var",        g.ballKfMeasVar),       "ball_kf_meas_var");
    warn(matchFloat(json,  "ball_kf_process_pos_var", g.ballKfProcessPosVar), "ball_kf_process_pos_var");
    warn(matchFloat(json,  "ball_kf_process_vel_var", g.ballKfProcessVelVar), "ball_kf_process_vel_var");
    warn(matchFloat(json,  "ball_kf_friction_gamma",  g.ballKfFrictionGamma), "ball_kf_friction_gamma");
    warn(matchFloat(json,  "ball_kf_vel_init_var",    g.ballKfVelInitVar),    "ball_kf_vel_init_var");
    warn(matchDouble(json, "ball_max_stale_s",        g.ballMaxStaleS),       "ball_max_stale_s");

    // Pose Kalman filter
    warn(matchFloat(json,  "pose_kf_process_pos_var", g.poseKfProcessPosVar), "pose_kf_process_pos_var");
    warn(matchFloat(json,  "pose_kf_process_vel_var", g.poseKfProcessVelVar), "pose_kf_process_vel_var");
    warn(matchFloat(json,  "pose_kf_meas_pos_var",    g.poseKfMeasPosVar),    "pose_kf_meas_pos_var");
    warn(matchDouble(json, "pose_max_stale_s",        g.poseMaxStaleS),       "pose_max_stale_s");
    warn(matchFloat(json,  "pose_kf_mouse_accel_var", g.poseKfMouseAccelVar), "pose_kf_mouse_accel_var");
    warn(matchFloat(json,  "pose_kf_mouse_meas_var",  g.poseKfMouseMeasVar),  "pose_kf_mouse_meas_var");

    // Goal bearing EKF
    warn(matchFloat(json, "goal_bearing_base_var_rad2", g.goalBearingBaseVarRad2), "goal_bearing_base_var_rad2");
    warn(matchFloat(json, "goal_bearing_penalty_rad2",  g.goalBearingPenaltyRad2), "goal_bearing_penalty_rad2");
    warn(matchFloat(json, "goal_certainty_threshold",   g.goalCertaintyThreshold), "goal_certainty_threshold");

    // Strategy
    warn(matchFloat(json,  "ball_prediction_damping",          g.ballPredictionDamping),        "ball_prediction_damping");
    warn(matchFloat(json,  "strike_offset_m",                  g.strikeOffsetM),                "strike_offset_m");
    warn(matchFloat(json,  "strike_intercept_max_time_s",      g.strikeInterceptMaxTimeS),      "strike_intercept_max_time_s");
    warn(matchInt(json,    "strike_intercept_max_iterations",  g.strikeInterceptMaxIterations), "strike_intercept_max_iterations");
    warn(matchFloat(json,  "strike_intercept_converge_s",      g.strikeInterceptConvergeS),     "strike_intercept_converge_s");
    warn(matchInt(json,    "dribbler_active_power",            g.dribblerActivePower),          "dribbler_active_power");
    warn(matchFloat(json,  "dribbler_capture_dist_mm",         g.dribblerCaptureDistMm),        "dribbler_capture_dist_mm");
    warn(matchFloat(json,  "kick_aim_tolerance_deg",           g.kickAimToleranceDeg),          "kick_aim_tolerance_deg");
    warn(matchFloat(json,  "kick_min_goal_forward_mm",         g.kickMinGoalForwardMm),         "kick_min_goal_forward_mm");
    warn(matchUint64(json, "kick_cooldown_us",                 g.kickCooldownUs),               "kick_cooldown_us");
    warn(matchFloat(json,  "defense_goal_line_y_min_cm",       g.defenseGoalLineYMinCm),        "defense_goal_line_y_min_cm");
    warn(matchFloat(json,  "defense_goal_line_x_max_cm",       g.defenseGoalLineXMaxCm),        "defense_goal_line_x_max_cm");
    warn(matchFloat(json,  "defense_goal_line_quadratic_c",    g.defenseGoalLineQuadraticC),    "defense_goal_line_quadratic_c");
    warn(matchFloat(json,  "defense_future_ball_max_time_s",   g.defenseFutureBallMaxTimeS),    "defense_future_ball_max_time_s");
    warn(matchFloat(json,  "defense_impact_margin_s",          g.defenseImpactMarginS),         "defense_impact_margin_s");
    warn(matchFloat(json,  "role_switch_margin_cm",            g.roleSwitchMarginCm),           "role_switch_margin_cm");
    warn(matchInt(json,    "role_confirm_samples",             g.roleConfirmSamples),           "role_confirm_samples");
    warn(matchFloat(json,  "astar_ball_clear_mm",              g.astarBallClearMm),             "astar_ball_clear_mm");
    warn(matchFloat(json,  "command_goal_position_tolerance_mm", g.commandGoalPositionToleranceMm), "command_goal_position_tolerance_mm");
    warn(matchFloat(json,  "command_goal_heading_tolerance_deg", g.commandGoalHeadingToleranceDeg), "command_goal_heading_tolerance_deg");
    warn(matchFloat(json,  "offense_intake_offset_mm",         g.offenseIntakeOffsetMm),        "offense_intake_offset_mm");
    warn(matchFloat(json,  "offense_boundary_inset_x_mm",      g.offenseBoundaryInsetXMm),      "offense_boundary_inset_x_mm");
    warn(matchFloat(json,  "offense_boundary_inset_y_mm",      g.offenseBoundaryInsetYMm),      "offense_boundary_inset_y_mm");

    if (!ok) std::cerr << "[PARAMS] some keys missing; built-in defaults used for those\n";
    return ok;
}

const Params& get() { return g; }

}  // namespace ballalgo::params

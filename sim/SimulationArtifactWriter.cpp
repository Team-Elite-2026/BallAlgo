#include "sim/SimulationArtifactWriter.hpp"

#include "config.hpp"
#include "sim/SimulationMath.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <string>

namespace ballalgo::sim {

namespace {

void ensureOutputParentExists(const std::filesystem::path& outputPath) {
  const auto parent = outputPath.parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
}

template <typename WaypointContainer>
void writeWaypoints(std::ofstream& out, const WaypointContainer& waypoints, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t index = 0; index < waypoints.size(); ++index) {
    const auto& waypoint = waypoints[index];
    out << pad << "{\"index\": " << index << ", \"x_mm\": " << waypoint.xMm << ", \"y_mm\": "
        << waypoint.yMm << ", \"theta_deg\": " << waypoint.thetaDeg << "}";
    out << (index + 1 == waypoints.size() ? "\n" : ",\n");
  }
}

void writePathSamples(std::ofstream& out, const std::vector<PathSample>& path, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t index = 0; index < path.size(); ++index) {
    const auto& sample = path[index];
    out << pad << "{\"index\": " << index << ", \"x_mm\": " << sample.xMm << ", \"y_mm\": "
        << sample.yMm << ", \"theta_deg\": " << sample.thetaDeg << ", \"s_mm\": " << sample.sMm
        << "}";
    out << (index + 1 == path.size() ? "\n" : ",\n");
  }
}

void writeProfileSamples(std::ofstream& out, const std::vector<ProfileSample>& profile, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t index = 0; index < profile.size(); ++index) {
    const auto& sample = profile[index];
    out << pad << "{\"index\": " << index << ", \"s_mm\": " << sample.sMm
        << ", \"v_cap_mps\": " << sample.vCap << ", \"v_mps\": " << sample.v
        << ", \"a_mps2\": " << sample.a << ", \"phi_deg\": "
        << (sample.phi * 180.0 / M_PI) << ", \"kappa_per_m\": " << sample.kappa << "}";
    out << (index + 1 == profile.size() ? "\n" : ",\n");
  }
}

void writePlannedActions(std::ofstream& out, const std::vector<MotionAction>& actions, int indent,
                         double dtMs) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t index = 0; index < actions.size(); ++index) {
    const auto& action = actions[index];
    out << pad << "{\"index\": " << index << ", \"t_s\": "
        << (dtMs * static_cast<double>(index) / 1000.0) << ", \"vx_body_mps\": " << action.vx
        << ", \"vy_body_mps\": " << action.vy << ", \"speed_body_mps\": "
        << std::hypot(static_cast<double>(action.vx), static_cast<double>(action.vy))
        << ", \"ax_body_mps2\": " << action.ax << ", \"ay_body_mps2\": " << action.ay
        << ", \"accel_body_mps2\": "
        << std::hypot(static_cast<double>(action.ax), static_cast<double>(action.ay))
        << ", \"omega_rad_s\": " << action.omega << ", \"alpha_rad_s2\": " << action.alpha
        << "}";
    out << (index + 1 == actions.size() ? "\n" : ",\n");
  }
}

void writeTraceSamples(std::ofstream& out, const std::vector<SimSample>& samples, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t index = 0; index < samples.size(); ++index) {
    const auto& sample = samples[index];
    out << pad << "{\"step\": " << sample.step << ", \"replan_index\": " << sample.replanIndex
        << ", \"local_action_index\": " << sample.localActionIndex << ", \"t_s\": "
        << sample.timeS << ", \"applied_dt_s\": " << sample.appliedDtS << ", \"x_mm\": "
        << sample.xMm << ", \"y_mm\": " << sample.yMm << ", \"x_cm\": " << sample.xCm
        << ", \"y_cm\": " << sample.yCm << ", \"heading_deg\": " << sample.headingDeg
        << ", \"vx_body_mps\": " << sample.vxBody << ", \"vy_body_mps\": " << sample.vyBody
        << ", \"speed_body_mps\": " << sample.speedBody << ", \"ax_body_mps2\": "
        << sample.axBody << ", \"ay_body_mps2\": " << sample.ayBody
        << ", \"accel_body_mps2\": " << sample.accelBody << ", \"omega_rad_s\": "
        << sample.omega << ", \"alpha_rad_s2\": " << sample.alpha << ", \"vx_field_mps\": "
        << sample.vxField << ", \"vy_field_mps\": " << sample.vyField
        << ", \"speed_field_mps\": " << sample.speedField << "}";
    out << (index + 1 == samples.size() ? "\n" : ",\n");
  }
}

void writeExecutedActions(std::ofstream& out, const std::vector<ExecutedActionSample>& actions,
                          int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t index = 0; index < actions.size(); ++index) {
    const auto& action = actions[index];
    out << pad << "{\"global_index\": " << action.globalIndex << ", \"replan_index\": "
        << action.replanIndex << ", \"local_action_index\": " << action.localActionIndex
        << ", \"t_s\": " << action.timeS << ", \"applied_dt_s\": " << action.appliedDtS
        << ", \"vx_body_mps\": " << action.vxBody << ", \"vy_body_mps\": " << action.vyBody
        << ", \"speed_body_mps\": " << action.speedBody << ", \"ax_body_mps2\": "
        << action.axBody << ", \"ay_body_mps2\": " << action.ayBody
        << ", \"accel_body_mps2\": " << action.accelBody << ", \"omega_rad_s\": "
        << action.omega << ", \"alpha_rad_s2\": " << action.alpha << ", \"vx_field_mps\": "
        << action.vxField << ", \"vy_field_mps\": " << action.vyField
        << ", \"speed_field_mps\": " << action.speedField << "}";
    out << (index + 1 == actions.size() ? "\n" : ",\n");
  }
}

void writePlannerInputs(std::ofstream& out, const PlannerInputSnapshot& inputs, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  out << pad << "\"planner_inputs\": {\n";
  out << pad << "  \"heading_deg\": " << inputs.headingDeg << ",\n";
  out << pad << "  \"goal_deg\": " << inputs.goalDeg << ",\n";
  out << pad << "  \"pose\": {\n";
  out << pad << "    \"x_mm\": " << inputs.pose.xMm << ",\n";
  out << pad << "    \"y_mm\": " << inputs.pose.yMm << ",\n";
  out << pad << "    \"x_cm\": " << fieldMmToCenteredCm(inputs.pose.xMm, config::kFieldWidthMm)
      << ",\n";
  out << pad << "    \"y_cm\": " << fieldMmToCenteredCm(inputs.pose.yMm, config::kFieldHeightMm)
      << ",\n";
  out << pad << "    \"vx_mm_s\": " << inputs.pose.vxMmS << ",\n";
  out << pad << "    \"vy_mm_s\": " << inputs.pose.vyMmS << ",\n";
  out << pad << "    \"vx_body_mps\": " << inputs.pose.vxBody << ",\n";
  out << pad << "    \"vy_body_mps\": " << inputs.pose.vyBody << "\n";
  out << pad << "  },\n";
  out << pad << "  \"ball_body\": {\n";
  out << pad << "    \"visible\": " << (inputs.ballBody.visible ? "true" : "false") << ",\n";
  out << pad << "    \"x_m\": " << inputs.ballBody.xM << ",\n";
  out << pad << "    \"y_m\": " << inputs.ballBody.yM << ",\n";
  out << pad << "    \"vx_m_s\": " << inputs.ballBody.vx << ",\n";
  out << pad << "    \"vy_m_s\": " << inputs.ballBody.vy << "\n";
  out << pad << "  },\n";
  out << pad << "  \"ball_field\": {\n";
  out << pad << "    \"x_mm\": " << inputs.ballField.xMm << ",\n";
  out << pad << "    \"y_mm\": " << inputs.ballField.yMm << ",\n";
  out << pad << "    \"x_cm\": " << fieldMmToCenteredCm(inputs.ballField.xMm, config::kFieldWidthMm)
      << ",\n";
  out << pad << "    \"y_cm\": "
      << fieldMmToCenteredCm(inputs.ballField.yMm, config::kFieldHeightMm) << ",\n";
  out << pad << "    \"vx_mm_s\": " << inputs.ballField.vxMmS << ",\n";
  out << pad << "    \"vy_mm_s\": " << inputs.ballField.vyMmS << "\n";
  out << pad << "  },\n";
  out << pad << "  \"goal_target\": {\n";
  out << pad << "    \"x_mm\": " << inputs.goalTargetField.xMm << ",\n";
  out << pad << "    \"y_mm\": " << inputs.goalTargetField.yMm << ",\n";
  out << pad << "    \"x_cm\": "
      << fieldMmToCenteredCm(inputs.goalTargetField.xMm, config::kFieldWidthMm) << ",\n";
  out << pad << "    \"y_cm\": "
      << fieldMmToCenteredCm(inputs.goalTargetField.yMm, config::kFieldHeightMm) << "\n";
  out << pad << "  }\n";
  out << pad << "}";
}

void writeReplans(std::ofstream& out, const std::vector<ReplanResult>& replans, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  for (size_t replanIndex = 0; replanIndex < replans.size(); ++replanIndex) {
    const ReplanResult& replan = replans[replanIndex];
    const PlannedChunk& chunk =
        replan.productionRoute ? replan.productionDebug.chunk : replan.poseDebug.chunk;
    const std::vector<Waypoint3>& waypoints =
        replan.productionRoute ? replan.productionDebug.waypoints : replan.poseDebug.waypoints;
    const std::vector<PathSample>& path =
        replan.productionRoute ? replan.productionDebug.path : replan.poseDebug.path;
    const std::vector<ProfileSample>& profile =
        replan.productionRoute ? replan.productionDebug.profile : replan.poseDebug.profile;

    out << pad << "{\n";
    out << pad << "  \"replan_index\": " << replan.replanIndex << ",\n";
    out << pad << "  \"plan_time_s\": " << replan.planTimeS << ",\n";
    out << pad << "  \"end_time_s\": " << replan.endTimeS << ",\n";
    out << pad << "  \"executed_action_count\": " << replan.executedActionCount << ",\n";
    const PoseState& startPose =
        replan.productionRoute ? replan.productionDebug.startPose : replan.poseDebug.startPose;
    const double startHeadingDeg = replan.productionRoute
                                       ? replan.productionDebug.startHeadingDeg
                                       : replan.poseDebug.startHeadingDeg;
    out << pad << "  \"start_pose\": {\n";
    out << pad << "    \"x_mm\": " << startPose.xMm << ",\n";
    out << pad << "    \"y_mm\": " << startPose.yMm << ",\n";
    out << pad << "    \"x_cm\": " << fieldMmToCenteredCm(startPose.xMm, config::kFieldWidthMm)
        << ",\n";
    out << pad << "    \"y_cm\": " << fieldMmToCenteredCm(startPose.yMm, config::kFieldHeightMm)
        << ",\n";
    out << pad << "    \"heading_deg\": " << startHeadingDeg << ",\n";
    out << pad << "    \"vx_mm_s\": " << startPose.vxMmS << ",\n";
    out << pad << "    \"vy_mm_s\": " << startPose.vyMmS << "\n";
    out << pad << "  },\n";
    out << pad << "  \"target_field\": {\n";
    out << pad << "    \"x_mm\": " << replan.targetXMm << ",\n";
    out << pad << "    \"y_mm\": " << replan.targetYMm << ",\n";
    out << pad << "    \"x_cm\": " << fieldMmToCenteredCm(replan.targetXMm, config::kFieldWidthMm)
        << ",\n";
    out << pad << "    \"y_cm\": "
        << fieldMmToCenteredCm(replan.targetYMm, config::kFieldHeightMm) << ",\n";
    out << pad << "    \"heading_deg\": " << replan.targetHeadingDeg << ",\n";
    out << pad << "    \"within_tolerance\": "
        << (replan.withinTargetTolerance ? "true" : "false") << "\n";
    out << pad << "  },\n";

    if (replan.productionRoute) {
      writePlannerInputs(out, replan.plannerInputs, indent + 2);
      out << ",\n";
      out << pad << "  \"debug\": {\n";
      out << pad << "    \"kind\": \"production_ball_plan\",\n";
      out << pad << "    \"full_planner\": "
          << (replan.productionDebug.fullPlanner ? "true" : "false") << ",\n";
      out << pad << "    \"used_center_fallback\": "
          << (replan.productionDebug.usedCenterFallback ? "true" : "false") << ",\n";
      out << pad << "    \"used_body_chase_fallback\": "
          << (replan.productionDebug.usedBodyChaseFallback ? "true" : "false") << ",\n";
      out << pad << "    \"used_strike_pose_plan\": "
          << (replan.productionDebug.usedStrikePosePlan ? "true" : "false") << ",\n";
      out << pad << "    \"goal_deg\": " << replan.productionDebug.goalDeg << ",\n";
      out << pad << "    \"target_error_mm\": " << replan.productionDebug.targetErrMm << ",\n";
      out << pad << "    \"within_target_tolerance\": "
          << (replan.productionDebug.withinTargetTolerance ? "true" : "false") << ",\n";
      out << pad << "    \"strike_target_body\": {\n";
      out << pad << "      \"x_m\": " << replan.productionDebug.strikeTargetBodyXM << ",\n";
      out << pad << "      \"y_m\": " << replan.productionDebug.strikeTargetBodyYM << "\n";
      out << pad << "    },\n";
      out << pad << "    \"ball_field\": {\n";
      out << pad << "      \"x_mm\": " << replan.productionDebug.ballFieldXMm << ",\n";
      out << pad << "      \"y_mm\": " << replan.productionDebug.ballFieldYMm << "\n";
      out << pad << "    }\n";
      out << pad << "  },\n";
    } else {
      out << pad << "  \"debug\": {\n";
      out << pad << "    \"kind\": \"pose_target\",\n";
      out << pad << "    \"within_tolerance\": "
          << (replan.poseDebug.withinTolerance ? "true" : "false") << ",\n";
      out << pad << "    \"position_error_mm\": " << replan.poseDebug.posErrMm << ",\n";
      out << pad << "    \"heading_error_deg\": " << replan.poseDebug.headingErrDeg << "\n";
      out << pad << "  },\n";
    }

    out << pad << "  \"chunk\": {\n";
    out << pad << "    \"trajectory_id\": " << chunk.trajectoryId << ",\n";
    out << pad << "    \"start_time_pi_us\": " << chunk.startTimePi << ",\n";
    out << pad << "    \"dt_ms\": " << chunk.dtMs << "\n";
    out << pad << "  },\n";
    out << pad << "  \"waypoints\": [\n";
    writeWaypoints(out, waypoints, indent + 4);
    out << pad << "  ],\n";
    out << pad << "  \"path\": [\n";
    writePathSamples(out, path, indent + 4);
    out << pad << "  ],\n";
    out << pad << "  \"profile\": [\n";
    writeProfileSamples(out, profile, indent + 4);
    out << pad << "  ],\n";
    out << pad << "  \"actions\": [\n";
    writePlannedActions(out, chunk.actions, indent + 4, chunk.dtMs);
    out << pad << "  ],\n";
    out << pad << "  \"executed_trace\": [\n";
    writeTraceSamples(out, replan.executedTrace, indent + 4);
    out << pad << "  ]\n";
    out << pad << "}";
    out << (replanIndex + 1 == replans.size() ? "\n" : ",\n");
  }
}

}  // namespace

void writeArtifact(const std::filesystem::path& outputPath, const InputOptions& options,
                   const PoseState& startPose, const CommandedPoseGoal& poseGoal,
                   const FieldBallState& ballField, const GoalFieldTarget& goalTarget,
                   const SimulationResult& simulation) {
  ensureOutputParentExists(outputPath);
  std::ofstream out(outputPath);
  out << std::fixed << std::setprecision(6);

  const SimSample& finalSample = simulation.trace.back();
  const ReplanResult* firstReplan =
      simulation.replans.empty() ? nullptr : &simulation.replans.front();
  const ReplanResult* lastReplan =
      simulation.replans.empty() ? nullptr : &simulation.replans.back();
  const double finalPoseGoalErrMm =
      std::hypot(finalSample.xMm - poseGoal.xMm, finalSample.yMm - poseGoal.yMm);
  const double finalHeadingErrDeg =
      std::fabs(wrapAngleDeg(poseGoal.headingDeg - finalSample.headingDeg));
  const double finalTargetHeadingErrDeg =
      lastReplan == nullptr ? 0.0
                            : std::fabs(wrapAngleDeg(lastReplan->targetHeadingDeg - finalSample.headingDeg));
  const double finalTargetErrMm =
      lastReplan == nullptr
          ? 0.0
          : std::hypot(finalSample.xMm - lastReplan->targetXMm,
                       finalSample.yMm - lastReplan->targetYMm);
  const double finalBallDistanceMm =
      std::hypot(finalSample.xMm - ballField.xMm, finalSample.yMm - ballField.yMm);
  const double summaryPositionErrMm =
      simulation.mode == SimulationMode::ProductionBallPlan ? finalTargetErrMm : finalPoseGoalErrMm;
  const double summaryHeadingErrDeg =
      simulation.mode == SimulationMode::ProductionBallPlan ? finalTargetHeadingErrDeg : finalHeadingErrDeg;

  out << "{\n";
  out << "  \"schema_version\": 3,\n";
  out << "  \"label\": \"" << escapeJson(options.label) << "\",\n";
  out << "  \"mode\": \"" << modeName(simulation.mode) << "\",\n";
  out << "  \"field\": {\n";
  out << "    \"width_mm\": " << config::kFieldWidthMm << ",\n";
  out << "    \"height_mm\": " << config::kFieldHeightMm << "\n";
  out << "  },\n";
  out << "  \"input\": {\n";
  out << "    \"start\": {\n";
  out << "      \"x_cm\": " << options.startXCm << ",\n";
  out << "      \"y_cm\": " << options.startYCm << ",\n";
  out << "      \"heading_deg\": " << options.startHeadingDeg << ",\n";
  out << "      \"x_mm\": " << startPose.xMm << ",\n";
  out << "      \"y_mm\": " << startPose.yMm << ",\n";
  out << "      \"vx_mm_s\": " << startPose.vxMmS << ",\n";
  out << "      \"vy_mm_s\": " << startPose.vyMmS << ",\n";
  out << "      \"vx_body_mps\": " << startPose.vxBody << ",\n";
  out << "      \"vy_body_mps\": " << startPose.vyBody << "\n";
  out << "    },\n";
  out << "    \"ball\": {\n";
  out << "      \"specified\": " << (options.ballSpecified ? "true" : "false") << ",\n";
  out << "      \"x_cm\": " << options.ballXCm << ",\n";
  out << "      \"y_cm\": " << options.ballYCm << ",\n";
  out << "      \"vx_cm_s\": " << options.ballVxCmS << ",\n";
  out << "      \"vy_cm_s\": " << options.ballVyCmS << ",\n";
  out << "      \"x_mm\": " << ballField.xMm << ",\n";
  out << "      \"y_mm\": " << ballField.yMm << ",\n";
  out << "      \"vx_mm_s\": " << ballField.vxMmS << ",\n";
  out << "      \"vy_mm_s\": " << ballField.vyMmS << "\n";
  out << "    },\n";
  out << "    \"goal\": {\n";
  out << "      \"specified\": " << (options.goalSpecified ? "true" : "false") << ",\n";
  out << "      \"x_cm\": " << options.goalXCm << ",\n";
  out << "      \"y_cm\": " << options.goalYCm << ",\n";
  out << "      \"heading_deg\": " << options.goalHeadingDeg << ",\n";
  out << "      \"x_mm\": " << poseGoal.xMm << ",\n";
  out << "      \"y_mm\": " << poseGoal.yMm << "\n";
  out << "    },\n";
  out << "    \"goal_target\": {\n";
  out << "      \"specified\": " << (options.goalTargetSpecified ? "true" : "false") << ",\n";
  out << "      \"x_cm\": " << options.goalTargetXCm << ",\n";
  out << "      \"y_cm\": " << options.goalTargetYCm << ",\n";
  out << "      \"x_mm\": " << goalTarget.xMm << ",\n";
  out << "      \"y_mm\": " << goalTarget.yMm << "\n";
  out << "    },\n";
  out << "    \"control_hz\": " << options.controlHz << ",\n";
  out << "    \"max_replans\": " << options.maxReplans << ",\n";
  out << "    \"max_sim_time_s\": " << options.maxSimTimeS << "\n";
  out << "  },\n";
  out << "  \"summary\": {\n";
  out << "    \"reached_goal\": " << (simulation.reachedGoal ? "true" : "false") << ",\n";
  out << "    \"max_replans_hit\": " << (simulation.maxReplansHit ? "true" : "false") << ",\n";
  out << "    \"max_time_hit\": " << (simulation.maxTimeHit ? "true" : "false") << ",\n";
  out << "    \"replan_count\": " << simulation.replans.size() << ",\n";
  out << "    \"control_interval_s\": " << simulation.controlIntervalS << ",\n";
  out << "    \"final_position_error_mm\": " << summaryPositionErrMm << ",\n";
  out << "    \"final_heading_error_deg\": " << summaryHeadingErrDeg << ",\n";
  out << "    \"final_pose_goal_error_mm\": " << finalPoseGoalErrMm << ",\n";
  out << "    \"final_pose_goal_heading_error_deg\": " << finalHeadingErrDeg << ",\n";
  out << "    \"final_target_error_mm\": " << finalTargetErrMm << ",\n";
  out << "    \"final_target_heading_error_deg\": " << finalTargetHeadingErrDeg << ",\n";
  out << "    \"final_ball_distance_mm\": " << finalBallDistanceMm << ",\n";
  out << "    \"executed_action_count\": " << simulation.executedActions.size() << ",\n";
  out << "    \"action_count\": " << simulation.executedActions.size() << ",\n";
  out << "    \"sim_duration_s\": " << finalSample.timeS << "\n";
  out << "  },\n";

  if (firstReplan != nullptr) {
    const PlannedChunk& chunk = firstReplan->productionRoute
                                    ? firstReplan->productionDebug.chunk
                                    : firstReplan->poseDebug.chunk;
    const std::vector<Waypoint3>& waypoints = firstReplan->productionRoute
                                                  ? firstReplan->productionDebug.waypoints
                                                  : firstReplan->poseDebug.waypoints;
    const std::vector<PathSample>& path = firstReplan->productionRoute
                                              ? firstReplan->productionDebug.path
                                              : firstReplan->poseDebug.path;
    const std::vector<ProfileSample>& profile = firstReplan->productionRoute
                                                    ? firstReplan->productionDebug.profile
                                                    : firstReplan->poseDebug.profile;
    out << "  \"chunk\": {\n";
    out << "    \"trajectory_id\": " << chunk.trajectoryId << ",\n";
    out << "    \"start_time_pi_us\": " << chunk.startTimePi << ",\n";
    out << "    \"dt_ms\": " << chunk.dtMs << "\n";
    out << "  },\n";
    out << "  \"waypoints\": [\n";
    writeWaypoints(out, waypoints, 4);
    out << "  ],\n";
    out << "  \"path\": [\n";
    writePathSamples(out, path, 4);
    out << "  ],\n";
    out << "  \"profile\": [\n";
    writeProfileSamples(out, profile, 4);
    out << "  ],\n";
  } else {
    out << "  \"chunk\": {\"trajectory_id\": 0, \"start_time_pi_us\": 0, \"dt_ms\": "
        << config::kChunkDtMs << "},\n";
    out << "  \"waypoints\": [],\n";
    out << "  \"path\": [],\n";
    out << "  \"profile\": [],\n";
  }

  if (firstReplan != nullptr) {
    const PlannedChunk& firstChunk = firstReplan->productionRoute
                                         ? firstReplan->productionDebug.chunk
                                         : firstReplan->poseDebug.chunk;
    out << "  \"actions\": [\n";
    writePlannedActions(out, firstChunk.actions, 4, firstChunk.dtMs);
    out << "  ],\n";
  } else {
    out << "  \"actions\": [],\n";
  }
  out << "  \"executed_actions\": [\n";
  writeExecutedActions(out, simulation.executedActions, 4);
  out << "  ],\n";
  out << "  \"sim_trace\": [\n";
  writeTraceSamples(out, simulation.trace, 4);
  out << "  ],\n";
  out << "  \"executed_trace\": [\n";
  writeTraceSamples(out, simulation.trace, 4);
  out << "  ],\n";
  out << "  \"replans\": [\n";
  writeReplans(out, simulation.replans, 4);
  out << "  ]\n";
  out << "}\n";
}

}  // namespace ballalgo::sim

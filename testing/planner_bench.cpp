#include "config.hpp"
#include "motion/MotionPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace ballalgo::testing {

namespace {

enum class SimulationMode { SingleChunk, RollingReplan };

struct InputOptions {
  float startXCm = 0;
  float startYCm = 0;
  float startHeadingDeg = 0;
  float goalXCm = 0;
  float goalYCm = 0;
  float goalHeadingDeg = 0;
  float startVxMmS = 0;
  float startVyMmS = 0;
  std::string outputPath;
  std::string label = "planner_bench";
  SimulationMode mode = SimulationMode::RollingReplan;
  float controlHz = config::kChunkPublishHz;
  int maxReplans = 250;
  float maxSimTimeS = 8.f;
};

struct SimSample {
  int step = 0;
  int replanIndex = -1;
  int localActionIndex = -1;
  double timeS = 0;
  double xMm = 0;
  double yMm = 0;
  double xCm = 0;
  double yCm = 0;
  double headingDeg = 0;
  double vxBody = 0;
  double vyBody = 0;
  double speedBody = 0;
  double axBody = 0;
  double ayBody = 0;
  double accelBody = 0;
  double omega = 0;
  double alpha = 0;
  double vxField = 0;
  double vyField = 0;
  double speedField = 0;
};

struct ExecutedActionSample {
  int globalIndex = 0;
  int replanIndex = 0;
  int localActionIndex = 0;
  double timeS = 0;
  double vxBody = 0;
  double vyBody = 0;
  double speedBody = 0;
  double axBody = 0;
  double ayBody = 0;
  double accelBody = 0;
  double omega = 0;
  double alpha = 0;
  double vxField = 0;
  double vyField = 0;
  double speedField = 0;
};

struct ReplanResult {
  int replanIndex = 0;
  double planTimeS = 0;
  double endTimeS = 0;
  int executedActionCount = 0;
  CommandedPosePlanDebug debug;
  std::vector<SimSample> executedTrace;
};

struct SimulationResult {
  SimulationMode mode = SimulationMode::RollingReplan;
  double controlIntervalS = 0;
  int actionsPerCycle = 0;
  bool reachedGoal = false;
  bool maxReplansHit = false;
  bool maxTimeHit = false;
  std::vector<ReplanResult> replans;
  std::vector<SimSample> trace;
  std::vector<ExecutedActionSample> executedActions;
};

bool parseFloatArg(const char* text, float& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

bool parseIntArg(const char* text, int& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text || end == nullptr || *end != '\0') return false;
  value = static_cast<int>(parsed);
  return true;
}

bool parseStringArg(const char* text, std::string& value) {
  if (text == nullptr) return false;
  value = text;
  return true;
}

bool parseModeArg(const char* text, SimulationMode& mode) {
  if (text == nullptr) return false;
  const std::string value = text;
  if (value == "single_chunk") {
    mode = SimulationMode::SingleChunk;
    return true;
  }
  if (value == "rolling_replan") {
    mode = SimulationMode::RollingReplan;
    return true;
  }
  return false;
}

const char* modeName(SimulationMode mode) {
  switch (mode) {
    case SimulationMode::SingleChunk:
      return "single_chunk";
    case SimulationMode::RollingReplan:
      return "rolling_replan";
  }
  return "unknown";
}

float centeredCmToFieldMm(float centeredCm, float axisLimitMm) {
  return centeredCm * 10.f + 0.5f * axisLimitMm;
}

double fieldMmToCenteredCm(double positionMm, double axisLimitMm) {
  return (positionMm - 0.5 * axisLimitMm) * 0.1;
}

std::string escapeJson(const std::string& input) {
  std::ostringstream out;
  for (char ch : input) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

bool parseArgs(int argc, char** argv, InputOptions& options) {
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    auto requireFloat = [&](float& value) {
      if (index + 1 >= argc) return false;
      ++index;
      return parseFloatArg(argv[index], value);
    };
    auto requireInt = [&](int& value) {
      if (index + 1 >= argc) return false;
      ++index;
      return parseIntArg(argv[index], value);
    };
    auto requireString = [&](std::string& value) {
      if (index + 1 >= argc) return false;
      ++index;
      return parseStringArg(argv[index], value);
    };
    auto requireMode = [&](SimulationMode& mode) {
      if (index + 1 >= argc) return false;
      ++index;
      return parseModeArg(argv[index], mode);
    };

    if (arg == "--start-x-cm") {
      if (!requireFloat(options.startXCm)) return false;
      continue;
    }
    if (arg == "--start-y-cm") {
      if (!requireFloat(options.startYCm)) return false;
      continue;
    }
    if (arg == "--start-heading-deg") {
      if (!requireFloat(options.startHeadingDeg)) return false;
      continue;
    }
    if (arg == "--goal-x-cm") {
      if (!requireFloat(options.goalXCm)) return false;
      continue;
    }
    if (arg == "--goal-y-cm") {
      if (!requireFloat(options.goalYCm)) return false;
      continue;
    }
    if (arg == "--goal-heading-deg") {
      if (!requireFloat(options.goalHeadingDeg)) return false;
      continue;
    }
    if (arg == "--start-vx-mm-s") {
      if (!requireFloat(options.startVxMmS)) return false;
      continue;
    }
    if (arg == "--start-vy-mm-s") {
      if (!requireFloat(options.startVyMmS)) return false;
      continue;
    }
    if (arg == "--output") {
      if (!requireString(options.outputPath)) return false;
      continue;
    }
    if (arg == "--label") {
      if (!requireString(options.label)) return false;
      continue;
    }
    if (arg == "--mode") {
      if (!requireMode(options.mode)) return false;
      continue;
    }
    if (arg == "--control-hz") {
      if (!requireFloat(options.controlHz)) return false;
      continue;
    }
    if (arg == "--max-replans") {
      if (!requireInt(options.maxReplans)) return false;
      continue;
    }
    if (arg == "--max-sim-time-s") {
      if (!requireFloat(options.maxSimTimeS)) return false;
      continue;
    }
    return false;
  }
  return !options.outputPath.empty();
}

void printUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << "\n"
            << "  --start-x-cm <value> --start-y-cm <value> --start-heading-deg <value>\n"
            << "  --goal-x-cm <value> --goal-y-cm <value> --goal-heading-deg <value>\n"
            << "  --output <artifact.json> [--label <name>]\n"
            << "  [--mode single_chunk|rolling_replan]\n"
            << "  [--start-vx-mm-s <value>] [--start-vy-mm-s <value>]\n"
            << "  [--control-hz <value>] [--max-replans <count>] [--max-sim-time-s <value>]\n";
}

double wrapAngleDeg(double angleDeg) {
  double wrapped = std::fmod(angleDeg + 180.0, 360.0);
  if (wrapped < 0) wrapped += 360.0;
  return wrapped - 180.0;
}

bool withinGoalTolerance(double xMm, double yMm, double headingDeg, const CommandedPoseGoal& goal) {
  const double posErrMm = std::hypot(goal.xMm - xMm, goal.yMm - yMm);
  const double headingErrDeg = std::fabs(wrapAngleDeg(goal.headingDeg - headingDeg));
  return posErrMm <= config::kCommandGoalPositionToleranceMm &&
         headingErrDeg <= config::kCommandGoalHeadingToleranceDeg;
}

SimSample makeInitialSample(const PoseState& pose, float headingDeg) {
  return {
      0,
      -1,
      -1,
      0.0,
      pose.xMm,
      pose.yMm,
      fieldMmToCenteredCm(pose.xMm, config::kFieldWidthMm),
      fieldMmToCenteredCm(pose.yMm, config::kFieldHeightMm),
      headingDeg,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
  };
}

SimSample simulateOneAction(const MotionAction& action, int globalStep, int replanIndex,
                            int localActionIndex, double timeS, double xMm, double yMm,
                            double headingDeg) {
  return {
      globalStep,
      replanIndex,
      localActionIndex,
      timeS,
      xMm,
      yMm,
      fieldMmToCenteredCm(xMm, config::kFieldWidthMm),
      fieldMmToCenteredCm(yMm, config::kFieldHeightMm),
      headingDeg,
      action.vx,
      action.vy,
      std::hypot(static_cast<double>(action.vx), static_cast<double>(action.vy)),
      action.ax,
      action.ay,
      std::hypot(static_cast<double>(action.ax), static_cast<double>(action.ay)),
      action.omega,
      action.alpha,
      0.0,
      0.0,
      0.0,
  };
}

void applyActionKinematics(const MotionAction& action, double dtS, double& xMm, double& yMm,
                           double& headingDeg, double& vxField, double& vyField) {
  const double headingRad = headingDeg * M_PI / 180.0;
  const double c = std::cos(headingRad);
  const double s = std::sin(headingRad);
  vxField = c * action.vx - s * action.vy;
  vyField = s * action.vx + c * action.vy;
  xMm += vxField * dtS * 1000.0;
  yMm += vyField * dtS * 1000.0;
  headingDeg += action.omega * dtS * 180.0 / M_PI;
}

void applyActionKinematics(const MotionAction& action, double dtS, float& xMm, float& yMm,
                           double& headingDeg, double& vxField, double& vyField) {
  double xMmDouble = xMm;
  double yMmDouble = yMm;
  applyActionKinematics(action, dtS, xMmDouble, yMmDouble, headingDeg, vxField, vyField);
  xMm = static_cast<float>(xMmDouble);
  yMm = static_cast<float>(yMmDouble);
}

ExecutedActionSample makeExecutedActionSample(const SimSample& sample) {
  return {
      sample.step - 1,
      sample.replanIndex,
      sample.localActionIndex,
      sample.timeS,
      sample.vxBody,
      sample.vyBody,
      sample.speedBody,
      sample.axBody,
      sample.ayBody,
      sample.accelBody,
      sample.omega,
      sample.alpha,
      sample.vxField,
      sample.vyField,
      sample.speedField,
  };
}

SimulationResult simulateSingleChunk(MotionPlanner& planner, const PoseState& startPose,
                                     float startHeadingDeg, const CommandedPoseGoal& goal) {
  SimulationResult result;
  result.mode = SimulationMode::SingleChunk;
  result.controlIntervalS =
      static_cast<double>(config::kChunkDtMs * config::kChunkMaxActions) / 1000.0;
  result.actionsPerCycle = config::kChunkMaxActions;
  result.trace.push_back(makeInitialSample(startPose, startHeadingDeg));

  ReplanResult replan;
  replan.replanIndex = 0;
  replan.planTimeS = 0.0;
  replan.debug = planner.debugPlanToPose(startPose, goal, startHeadingDeg);

  double xMm = startPose.xMm;
  double yMm = startPose.yMm;
  double headingDeg = startHeadingDeg;
  const double dtS = static_cast<double>(replan.debug.chunk.dtMs) / 1000.0;

  for (size_t index = 0; index < replan.debug.chunk.actions.size(); ++index) {
    const MotionAction& action = replan.debug.chunk.actions[index];
    double vxField = 0.0;
    double vyField = 0.0;
    applyActionKinematics(action, dtS, xMm, yMm, headingDeg, vxField, vyField);
    SimSample sample = simulateOneAction(action, static_cast<int>(result.trace.size()), 0,
                                         static_cast<int>(index),
                                         dtS * static_cast<double>(index + 1), xMm, yMm,
                                         headingDeg);
    sample.vxField = vxField;
    sample.vyField = vyField;
    sample.speedField = std::hypot(vxField, vyField);
    replan.executedTrace.push_back(sample);
    result.trace.push_back(sample);
    result.executedActions.push_back(makeExecutedActionSample(sample));
  }

  replan.executedActionCount = static_cast<int>(replan.executedTrace.size());
  replan.endTimeS = result.trace.back().timeS;
  result.replans.push_back(replan);
  result.reachedGoal = withinGoalTolerance(xMm, yMm, headingDeg, goal);
  return result;
}

SimulationResult simulateRollingReplan(MotionPlanner& planner, const PoseState& startPose,
                                       float startHeadingDeg, const CommandedPoseGoal& goal,
                                       float controlHz, int maxReplans, float maxSimTimeS) {
  SimulationResult result;
  result.mode = SimulationMode::RollingReplan;
  result.controlIntervalS = 1.0 / std::max(1e-3f, controlHz);
  result.trace.push_back(makeInitialSample(startPose, startHeadingDeg));

  PoseState currentPose = startPose;
  currentPose.valid = true;
  double currentHeadingDeg = startHeadingDeg;
  double currentTimeS = 0.0;

  for (int replanIndex = 0; replanIndex < maxReplans; ++replanIndex) {
    ReplanResult replan;
    replan.replanIndex = replanIndex;
    replan.planTimeS = currentTimeS;
    replan.debug = planner.debugPlanToPose(currentPose, goal, static_cast<float>(currentHeadingDeg));
    const int maxActionsInChunk = static_cast<int>(replan.debug.chunk.actions.size());
    const double dtS = static_cast<double>(replan.debug.chunk.dtMs) / 1000.0;
    result.actionsPerCycle =
        std::max(1, static_cast<int>(std::llround(result.controlIntervalS / dtS)));

    if (replan.debug.withinTolerance) {
      replan.endTimeS = currentTimeS;
      result.replans.push_back(replan);
      result.reachedGoal = true;
      break;
    }

    const int actionsToExecute = std::min(maxActionsInChunk, result.actionsPerCycle);
    for (int localActionIndex = 0; localActionIndex < actionsToExecute; ++localActionIndex) {
      const MotionAction& action =
          replan.debug.chunk.actions[static_cast<size_t>(localActionIndex)];
      double vxField = 0.0;
      double vyField = 0.0;
      applyActionKinematics(action, dtS, currentPose.xMm, currentPose.yMm, currentHeadingDeg,
                            vxField, vyField);
      currentTimeS += dtS;
      currentPose.vxMmS = static_cast<float>(vxField * 1000.0);
      currentPose.vyMmS = static_cast<float>(vyField * 1000.0);
      currentPose.vxBody = action.vx;
      currentPose.vyBody = action.vy;

      SimSample sample =
          simulateOneAction(action, static_cast<int>(result.trace.size()), replanIndex,
                            localActionIndex, currentTimeS, currentPose.xMm, currentPose.yMm,
                            currentHeadingDeg);
      sample.vxField = vxField;
      sample.vyField = vyField;
      sample.speedField = std::hypot(vxField, vyField);
      replan.executedTrace.push_back(sample);
      result.trace.push_back(sample);
      result.executedActions.push_back(makeExecutedActionSample(sample));
    }

    replan.executedActionCount = static_cast<int>(replan.executedTrace.size());
    replan.endTimeS = currentTimeS;
    result.replans.push_back(replan);

    if (withinGoalTolerance(currentPose.xMm, currentPose.yMm, currentHeadingDeg, goal)) {
      result.reachedGoal = true;
      break;
    }
    if (currentTimeS >= static_cast<double>(maxSimTimeS)) {
      result.maxTimeHit = true;
      break;
    }
    if (replanIndex == maxReplans - 1) {
      result.maxReplansHit = true;
    }
  }

  return result;
}

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
        << sample.timeS << ", \"x_mm\": " << sample.xMm << ", \"y_mm\": " << sample.yMm
        << ", \"x_cm\": " << sample.xCm << ", \"y_cm\": " << sample.yCm
        << ", \"heading_deg\": " << sample.headingDeg << ", \"vx_body_mps\": "
        << sample.vxBody << ", \"vy_body_mps\": " << sample.vyBody << ", \"speed_body_mps\": "
        << sample.speedBody << ", \"ax_body_mps2\": " << sample.axBody
        << ", \"ay_body_mps2\": " << sample.ayBody << ", \"accel_body_mps2\": "
        << sample.accelBody << ", \"omega_rad_s\": " << sample.omega
        << ", \"alpha_rad_s2\": " << sample.alpha << ", \"vx_field_mps\": "
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
        << ", \"t_s\": " << action.timeS << ", \"vx_body_mps\": " << action.vxBody
        << ", \"vy_body_mps\": " << action.vyBody << ", \"speed_body_mps\": "
        << action.speedBody << ", \"ax_body_mps2\": " << action.axBody
        << ", \"ay_body_mps2\": " << action.ayBody << ", \"accel_body_mps2\": "
        << action.accelBody << ", \"omega_rad_s\": " << action.omega
        << ", \"alpha_rad_s2\": " << action.alpha << ", \"vx_field_mps\": "
        << action.vxField << ", \"vy_field_mps\": " << action.vyField
        << ", \"speed_field_mps\": " << action.speedField << "}";
    out << (index + 1 == actions.size() ? "\n" : ",\n");
  }
}

void writeArtifact(const std::filesystem::path& outputPath, const InputOptions& options,
                   const PoseState& startPose, const CommandedPoseGoal& goal,
                   const SimulationResult& simulation) {
  ensureOutputParentExists(outputPath);
  std::ofstream out(outputPath);
  out << std::fixed << std::setprecision(6);

  const SimSample& finalSample = simulation.trace.back();
  const double finalPosErrMm = std::hypot(finalSample.xMm - goal.xMm, finalSample.yMm - goal.yMm);
  const double finalHeadingErrDeg = std::fabs(wrapAngleDeg(goal.headingDeg - finalSample.headingDeg));
  const ReplanResult* firstReplan = simulation.replans.empty() ? nullptr : &simulation.replans.front();

  out << "{\n";
  out << "  \"schema_version\": 2,\n";
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
  out << "      \"vy_mm_s\": " << startPose.vyMmS << "\n";
  out << "    },\n";
  out << "    \"goal\": {\n";
  out << "      \"x_cm\": " << options.goalXCm << ",\n";
  out << "      \"y_cm\": " << options.goalYCm << ",\n";
  out << "      \"heading_deg\": " << options.goalHeadingDeg << ",\n";
  out << "      \"x_mm\": " << goal.xMm << ",\n";
  out << "      \"y_mm\": " << goal.yMm << "\n";
  out << "    },\n";
  out << "    \"control_hz\": " << options.controlHz << ",\n";
  out << "    \"max_replans\": " << options.maxReplans << ",\n";
  out << "    \"max_sim_time_s\": " << options.maxSimTimeS << "\n";
  out << "  },\n";
  out << "  \"summary\": {\n";
  out << "    \"pose_valid\": " << (startPose.valid ? "true" : "false") << ",\n";
  out << "    \"reached_goal\": " << (simulation.reachedGoal ? "true" : "false") << ",\n";
  out << "    \"max_replans_hit\": " << (simulation.maxReplansHit ? "true" : "false") << ",\n";
  out << "    \"max_time_hit\": " << (simulation.maxTimeHit ? "true" : "false") << ",\n";
  out << "    \"replan_count\": " << simulation.replans.size() << ",\n";
  out << "    \"control_interval_s\": " << simulation.controlIntervalS << ",\n";
  out << "    \"actions_per_cycle\": " << simulation.actionsPerCycle << ",\n";
  out << "    \"final_position_error_mm\": " << finalPosErrMm << ",\n";
  out << "    \"final_heading_error_deg\": " << finalHeadingErrDeg << ",\n";
  out << "    \"executed_action_count\": " << simulation.executedActions.size() << ",\n";
  out << "    \"sim_duration_s\": " << finalSample.timeS << "\n";
  out << "  },\n";

  if (firstReplan != nullptr) {
    out << "  \"chunk\": {\n";
    out << "    \"trajectory_id\": " << firstReplan->debug.chunk.trajectoryId << ",\n";
    out << "    \"start_time_pi_us\": " << firstReplan->debug.chunk.startTimePi << ",\n";
    out << "    \"dt_ms\": " << firstReplan->debug.chunk.dtMs << "\n";
    out << "  },\n";

    out << "  \"waypoints\": [\n";
    writeWaypoints(out, firstReplan->debug.waypoints, 4);
    out << "  ],\n";

    out << "  \"path\": [\n";
    writePathSamples(out, firstReplan->debug.path, 4);
    out << "  ],\n";

    out << "  \"profile\": [\n";
    writeProfileSamples(out, firstReplan->debug.profile, 4);
    out << "  ],\n";
  } else {
    out << "  \"chunk\": {\"trajectory_id\": 0, \"start_time_pi_us\": 0, \"dt_ms\": "
        << config::kChunkDtMs << "},\n";
    out << "  \"waypoints\": [],\n";
    out << "  \"path\": [],\n";
    out << "  \"profile\": [],\n";
  }

  out << "  \"actions\": [\n";
  writeExecutedActions(out, simulation.executedActions, 4);
  out << "  ],\n";

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
  for (size_t replanIndex = 0; replanIndex < simulation.replans.size(); ++replanIndex) {
    const ReplanResult& replan = simulation.replans[replanIndex];
    out << "    {\n";
    out << "      \"replan_index\": " << replan.replanIndex << ",\n";
    out << "      \"plan_time_s\": " << replan.planTimeS << ",\n";
    out << "      \"end_time_s\": " << replan.endTimeS << ",\n";
    out << "      \"executed_action_count\": " << replan.executedActionCount << ",\n";
    out << "      \"start_pose\": {\n";
    out << "        \"x_mm\": " << replan.debug.startPose.xMm << ",\n";
    out << "        \"y_mm\": " << replan.debug.startPose.yMm << ",\n";
    out << "        \"x_cm\": "
        << fieldMmToCenteredCm(replan.debug.startPose.xMm, config::kFieldWidthMm) << ",\n";
    out << "        \"y_cm\": "
        << fieldMmToCenteredCm(replan.debug.startPose.yMm, config::kFieldHeightMm) << ",\n";
    out << "        \"heading_deg\": " << replan.debug.startHeadingDeg << ",\n";
    out << "        \"vx_mm_s\": " << replan.debug.startPose.vxMmS << ",\n";
    out << "        \"vy_mm_s\": " << replan.debug.startPose.vyMmS << "\n";
    out << "      },\n";
    out << "      \"summary\": {\n";
    out << "        \"within_tolerance\": " << (replan.debug.withinTolerance ? "true" : "false")
        << ",\n";
    out << "        \"initial_position_error_mm\": " << replan.debug.posErrMm << ",\n";
    out << "        \"initial_heading_error_deg\": " << replan.debug.headingErrDeg << ",\n";
    out << "        \"waypoint_count\": " << replan.debug.waypoints.size() << ",\n";
    out << "        \"path_sample_count\": " << replan.debug.path.size() << ",\n";
    out << "        \"profile_sample_count\": " << replan.debug.profile.size() << ",\n";
    out << "        \"planned_action_count\": " << replan.debug.chunk.actions.size() << "\n";
    out << "      },\n";
    out << "      \"chunk\": {\n";
    out << "        \"trajectory_id\": " << replan.debug.chunk.trajectoryId << ",\n";
    out << "        \"start_time_pi_us\": " << replan.debug.chunk.startTimePi << ",\n";
    out << "        \"dt_ms\": " << replan.debug.chunk.dtMs << "\n";
    out << "      },\n";
    out << "      \"waypoints\": [\n";
    writeWaypoints(out, replan.debug.waypoints, 8);
    out << "      ],\n";
    out << "      \"path\": [\n";
    writePathSamples(out, replan.debug.path, 8);
    out << "      ],\n";
    out << "      \"profile\": [\n";
    writeProfileSamples(out, replan.debug.profile, 8);
    out << "      ],\n";
    out << "      \"actions\": [\n";
    writePlannedActions(out, replan.debug.chunk.actions, 8, replan.debug.chunk.dtMs);
    out << "      ],\n";
    out << "      \"executed_trace\": [\n";
    writeTraceSamples(out, replan.executedTrace, 8);
    out << "      ]\n";
    out << "    }";
    out << (replanIndex + 1 == simulation.replans.size() ? "\n" : ",\n");
  }
  out << "  ]\n";
  out << "}\n";
}

}  // namespace

}  // namespace ballalgo::testing

int main(int argc, char** argv) {
  using namespace ballalgo;
  using namespace ballalgo::testing;

  InputOptions options;
  if (!parseArgs(argc, argv, options)) {
    printUsage(argv[0]);
    return 2;
  }

  PoseState startPose;
  startPose.valid = true;
  startPose.xMm = centeredCmToFieldMm(options.startXCm, config::kFieldWidthMm);
  startPose.yMm = centeredCmToFieldMm(options.startYCm, config::kFieldHeightMm);
  startPose.vxMmS = options.startVxMmS;
  startPose.vyMmS = options.startVyMmS;

  CommandedPoseGoal goal;
  goal.xMm = centeredCmToFieldMm(options.goalXCm, config::kFieldWidthMm);
  goal.yMm = centeredCmToFieldMm(options.goalYCm, config::kFieldHeightMm);
  goal.headingDeg = options.goalHeadingDeg;

  MotionPlanner planner;
  SimulationResult simulation =
      options.mode == SimulationMode::SingleChunk
          ? simulateSingleChunk(planner, startPose, options.startHeadingDeg, goal)
          : simulateRollingReplan(planner, startPose, options.startHeadingDeg, goal,
                                  options.controlHz, options.maxReplans, options.maxSimTimeS);

  const std::filesystem::path outputPath = options.outputPath;
  writeArtifact(outputPath, options, startPose, goal, simulation);

  const SimSample& finalSample = simulation.trace.back();
  const double finalErrMm = std::hypot(finalSample.xMm - goal.xMm, finalSample.yMm - goal.yMm);

  std::cout << "planner bench wrote " << outputPath << "\n";
  std::cout << "mode=" << modeName(simulation.mode) << " start=(" << options.startXCm << ", "
            << options.startYCm << ") cm goal=(" << options.goalXCm << ", " << options.goalYCm
            << ") cm final_error_mm=" << finalErrMm
            << " replans=" << simulation.replans.size()
            << " executed_actions=" << simulation.executedActions.size() << "\n";
  return 0;
}

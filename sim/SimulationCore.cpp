#include "sim/SimulationCore.hpp"

#include "config.hpp"
#include "sim/SimulationArtifactWriter.hpp"
#include "sim/SimulationCli.hpp"
#include "sim/SimulationMath.hpp"
#include "sim/SimulationRunner.hpp"

#include <cmath>
#include <filesystem>
#include <ostream>
#include <vector>

namespace ballalgo::sim {

namespace {

constexpr double kSimWhiteLineFieldWidthCm = 158.0;
constexpr double kSimGoalDepthCm = 7.4;

double hardcodedGoalCenterXCm(GoalIdentity goalIdentity) {
  // The goal target is the center of the 60 cm-wide goal mouth. The goal sits
  // just outside the 158 cm white-line field width, so the target lies half a
  // goal depth beyond the field edge.
  const double centerOffsetCm = (kSimWhiteLineFieldWidthCm * 0.5) + (kSimGoalDepthCm * 0.5);
  switch (goalIdentity) {
    case GoalIdentity::Blue:
      return -centerOffsetCm;
    case GoalIdentity::Yellow:
      return centerOffsetCm;
    case GoalIdentity::Unspecified:
      return 0.0;
  }
  return 0.0;
}

PoseState buildStartPose(const InputOptions& options) {
  PoseState startPose;
  startPose.valid = true;
  startPose.xMm = centeredCmToFieldMm(options.startXCm, config::kFieldWidthMm);
  startPose.yMm = centeredCmToFieldMm(options.startYCm, config::kFieldHeightMm);
  startPose.vxMmS = options.startVxMmS;
  startPose.vyMmS = options.startVyMmS;
  fieldVelocityToBodyMps(startPose.vxMmS, startPose.vyMmS, options.startHeadingDeg,
                         startPose.vxBody, startPose.vyBody);
  return startPose;
}

CommandedPoseGoal buildPoseGoal(const InputOptions& options) {
  CommandedPoseGoal goal;
  goal.xMm = centeredCmToFieldMm(options.goalXCm, config::kFieldWidthMm);
  goal.yMm = centeredCmToFieldMm(options.goalYCm, config::kFieldHeightMm);
  goal.headingDeg = options.goalHeadingDeg;
  return goal;
}

FieldBallState buildFieldBallState(const InputOptions& options) {
  FieldBallState ballField;
  ballField.xMm = centeredCmToFieldMm(options.ballXCm, config::kFieldWidthMm);
  ballField.yMm = centeredCmToFieldMm(options.ballYCm, config::kFieldHeightMm);
  ballField.vxMmS = options.ballVxCmS * 10.0;
  ballField.vyMmS = options.ballVyCmS * 10.0;
  return ballField;
}

GoalFieldTarget buildGoalTarget(const InputOptions& options) {
  GoalFieldTarget goalTarget;
  if (options.goalIdentity != GoalIdentity::Unspecified) {
    goalTarget.xMm =
        centeredCmToFieldMm(hardcodedGoalCenterXCm(options.goalIdentity), config::kFieldWidthMm);
    // The goal mouth is centered on the field midline.
    goalTarget.yMm = centeredCmToFieldMm(0.0, config::kFieldHeightMm);
    return goalTarget;
  }
  goalTarget.xMm = centeredCmToFieldMm(options.goalTargetXCm, config::kFieldWidthMm);
  goalTarget.yMm = centeredCmToFieldMm(options.goalTargetYCm, config::kFieldHeightMm);
  return goalTarget;
}

SimulationResult runSimulation(MotionPlanner& planner, const InputOptions& options,
                               const PoseState& startPose, const CommandedPoseGoal& poseGoal,
                               const FieldBallState& ballField, const GoalFieldTarget& goalTarget) {
  if (options.mode == SimulationMode::ProductionBallPlan) {
    return simulateProductionBallPlan(planner, startPose, options.startHeadingDeg, ballField,
                                      goalTarget, options.controlHz, options.maxReplans,
                                      options.maxSimTimeS);
  }
  if (options.mode == SimulationMode::SingleChunk) {
    return simulateSingleChunk(planner, startPose, options.startHeadingDeg, poseGoal);
  }
  return simulatePoseTarget(planner, startPose, options.startHeadingDeg, poseGoal,
                            options.controlHz, options.maxReplans, options.maxSimTimeS);
}

}  // namespace

int runBallalgoSimMain(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
  InputOptions options;
  if (!parseArgs(args, options)) {
    const char* argv0 = args.empty() ? "ballalgo_sim" : args.front().c_str();
    printUsage(argv0, err);
    return 2;
  }
  if (!validateOptions(options, err)) return 2;

  const PoseState startPose = buildStartPose(options);
  const CommandedPoseGoal poseGoal = buildPoseGoal(options);
  const FieldBallState ballField = buildFieldBallState(options);
  const GoalFieldTarget goalTarget = buildGoalTarget(options);

  MotionPlanner planner;
  const SimulationResult simulation =
      runSimulation(planner, options, startPose, poseGoal, ballField, goalTarget);

  const std::filesystem::path outputPath = options.outputPath;
  writeArtifact(outputPath, options, startPose, poseGoal, ballField, goalTarget, simulation);

  const SimSample& finalSample = simulation.trace.back();
  const double finalTargetErrMm =
      simulation.replans.empty()
          ? 0.0
          : std::hypot(finalSample.xMm - simulation.replans.back().targetXMm,
                       finalSample.yMm - simulation.replans.back().targetYMm);
  out << "ballalgo sim wrote " << outputPath << "\n";
  out << "mode=" << modeName(simulation.mode) << " start=(" << options.startXCm << ", "
      << options.startYCm << ") cm final_target_error_mm=" << finalTargetErrMm
      << " replans=" << simulation.replans.size()
      << " executed_actions=" << simulation.executedActions.size() << "\n";
  return 0;
}

}  // namespace ballalgo::sim

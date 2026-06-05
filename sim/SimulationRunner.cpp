#include "sim/SimulationRunner.hpp"

#include "config.hpp"
#include "sim/SimulationMath.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo::sim {

SimulationResult simulateSingleChunk(MotionPlanner& planner, const PoseState& startPose,
                                     double startHeadingDeg, const CommandedPoseGoal& goal) {
  SimulationResult result;
  result.mode = SimulationMode::SingleChunk;
  result.controlIntervalS =
      static_cast<double>(config::kChunkDtMs * config::kChunkMaxActions) / 1000.0;
  result.trace.push_back(makeInitialSample(startPose, startHeadingDeg));

  ReplanResult replan;
  replan.replanIndex = 0;
  replan.planTimeS = 0.0;
  replan.poseDebug = planner.debugPlanToPose(startPose, goal, static_cast<float>(startHeadingDeg));
  replan.targetXMm = goal.xMm;
  replan.targetYMm = goal.yMm;
  replan.targetHeadingDeg = goal.headingDeg;
  replan.withinTargetTolerance = replan.poseDebug.withinTolerance;

  PoseState currentPose = startPose;
  double currentHeadingDeg = startHeadingDeg;
  double currentTimeS = 0.0;
  const double actionDtS = static_cast<double>(replan.poseDebug.chunk.dtMs) / 1000.0;
  const double fullChunkDurationS =
      actionDtS * static_cast<double>(replan.poseDebug.chunk.actions.size());
  replan.executedActionCount =
      executeControlSlice(replan.poseDebug.chunk.actions, actionDtS, fullChunkDurationS,
                          replan.replanIndex, currentPose, currentHeadingDeg, currentTimeS,
                          result.trace, result.executedActions, replan.executedTrace);
  replan.endTimeS = currentTimeS;
  result.replans.push_back(replan);
  result.reachedGoal =
      withinPoseGoalTolerance(currentPose.xMm, currentPose.yMm, currentHeadingDeg, goal);
  return result;
}

SimulationResult simulatePoseTarget(MotionPlanner& planner, const PoseState& startPose,
                                    double startHeadingDeg, const CommandedPoseGoal& goal,
                                    double controlHz, int maxReplans, double maxSimTimeS) {
  SimulationResult result;
  result.mode = SimulationMode::PoseTarget;
  result.controlIntervalS = 1.0 / std::max(1e-3, controlHz);
  result.trace.push_back(makeInitialSample(startPose, startHeadingDeg));

  PoseState currentPose = startPose;
  currentPose.valid = true;
  double currentHeadingDeg = startHeadingDeg;
  double currentTimeS = 0.0;

  for (int replanIndex = 0; replanIndex < maxReplans; ++replanIndex) {
    ReplanResult replan;
    replan.replanIndex = replanIndex;
    replan.planTimeS = currentTimeS;
    replan.poseDebug =
        planner.debugPlanToPose(currentPose, goal, static_cast<float>(currentHeadingDeg));
    replan.targetXMm = goal.xMm;
    replan.targetYMm = goal.yMm;
    replan.targetHeadingDeg = goal.headingDeg;
    replan.withinTargetTolerance = replan.poseDebug.withinTolerance;

    if (replan.poseDebug.withinTolerance) {
      replan.endTimeS = currentTimeS;
      result.replans.push_back(replan);
      result.reachedGoal = true;
      break;
    }

    const double actionDtS = static_cast<double>(replan.poseDebug.chunk.dtMs) / 1000.0;
    replan.executedActionCount =
        executeControlSlice(replan.poseDebug.chunk.actions, actionDtS, result.controlIntervalS,
                            replan.replanIndex, currentPose, currentHeadingDeg, currentTimeS,
                            result.trace, result.executedActions, replan.executedTrace);
    replan.endTimeS = currentTimeS;
    result.replans.push_back(replan);

    if (withinPoseGoalTolerance(currentPose.xMm, currentPose.yMm, currentHeadingDeg, goal)) {
      result.reachedGoal = true;
      break;
    }
    if (currentTimeS >= maxSimTimeS) {
      result.maxTimeHit = true;
      break;
    }
    if (replanIndex == maxReplans - 1) result.maxReplansHit = true;
  }

  return result;
}

SimulationResult simulateProductionBallPlan(MotionPlanner& planner, const PoseState& startPose,
                                            double startHeadingDeg, const FieldBallState& ballField,
                                            const GoalFieldTarget& goalTarget, double controlHz,
                                            int maxReplans, double maxSimTimeS) {
  SimulationResult result;
  result.mode = SimulationMode::ProductionBallPlan;
  result.controlIntervalS = 1.0 / std::max(1e-3, controlHz);
  result.trace.push_back(makeInitialSample(startPose, startHeadingDeg));

  PoseState currentPose = startPose;
  currentPose.valid = true;
  FieldBallState currentBall = ballField;
  double currentHeadingDeg = startHeadingDeg;
  double currentTimeS = 0.0;

  for (int replanIndex = 0; replanIndex < maxReplans; ++replanIndex) {
    ReplanResult replan;
    replan.replanIndex = replanIndex;
    replan.planTimeS = currentTimeS;
    replan.productionRoute = true;
    replan.plannerInputs =
        makeProductionPlannerInputs(currentPose, currentHeadingDeg, currentBall, goalTarget);
    const FieldTarget plannerGoalFieldTarget{
        static_cast<float>(replan.plannerInputs.goalTargetField.xMm),
        static_cast<float>(replan.plannerInputs.goalTargetField.yMm)};
    replan.productionDebug =
        planner.debugPlan(currentPose, replan.plannerInputs.ballBody,
                          static_cast<float>(replan.plannerInputs.goalDeg),
                          static_cast<float>(currentHeadingDeg), true,
                          &plannerGoalFieldTarget);
    replan.targetXMm = replan.productionDebug.targetXMm;
    replan.targetYMm = replan.productionDebug.targetYMm;
    replan.targetHeadingDeg = replan.productionDebug.targetHeadingDeg;
    replan.withinTargetTolerance = replan.productionDebug.withinTargetTolerance;

    if (replan.productionDebug.withinTargetTolerance) {
      replan.endTimeS = currentTimeS;
      result.replans.push_back(replan);
      result.reachedGoal = true;
      break;
    }

    const double actionDtS = static_cast<double>(replan.productionDebug.chunk.dtMs) / 1000.0;
    replan.executedActionCount =
        executeControlSlice(replan.productionDebug.chunk.actions, actionDtS, result.controlIntervalS,
                            replan.replanIndex, currentPose, currentHeadingDeg, currentTimeS,
                            result.trace, result.executedActions, replan.executedTrace);
    currentBall.xMm += currentBall.vxMmS * result.controlIntervalS;
    currentBall.yMm += currentBall.vyMmS * result.controlIntervalS;
    replan.endTimeS = currentTimeS;
    result.replans.push_back(replan);

    const double targetErrMm =
        std::hypot(currentPose.xMm - replan.targetXMm, currentPose.yMm - replan.targetYMm);
    if (targetErrMm <= config::kCommandGoalPositionToleranceMm) {
      result.reachedGoal = true;
      break;
    }
    if (currentTimeS >= maxSimTimeS) {
      result.maxTimeHit = true;
      break;
    }
    if (replanIndex == maxReplans - 1) result.maxReplansHit = true;
  }

  return result;
}

}  // namespace ballalgo::sim

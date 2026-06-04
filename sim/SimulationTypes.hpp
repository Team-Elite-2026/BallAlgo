#pragma once

#include "config.hpp"
#include "motion/MotionPlanner.hpp"

#include <string>
#include <vector>

namespace ballalgo::sim {

enum class SimulationMode { ProductionBallPlan, PoseTarget, SingleChunk };

enum class GoalIdentity { Unspecified, Blue, Yellow };

struct InputOptions {
  float startXCm = 0;
  float startYCm = 0;
  float startHeadingDeg = 0;
  float startVxMmS = 0;
  float startVyMmS = 0;
  bool goalSpecified = false;
  bool goalXSpecified = false;
  bool goalYSpecified = false;
  bool goalHeadingSpecified = false;
  float goalXCm = 0;
  float goalYCm = 0;
  float goalHeadingDeg = 0;
  bool ballSpecified = false;
  bool ballXSpecified = false;
  bool ballYSpecified = false;
  float ballXCm = 0;
  float ballYCm = 0;
  float ballVxCmS = 0;
  float ballVyCmS = 0;
  bool goalTargetSpecified = false;
  bool goalTargetXSpecified = false;
  bool goalTargetYSpecified = false;
  float goalTargetXCm = 0;
  float goalTargetYCm = 0;
  GoalIdentity goalIdentity = GoalIdentity::Unspecified;
  std::string outputPath;
  std::string label = "ballalgo_sim";
  SimulationMode mode = SimulationMode::ProductionBallPlan;
  float controlHz = config::kChunkPublishHz;
  int maxReplans = 250;
  float maxSimTimeS = 8.f;
};

struct FieldBallState {
  double xMm = 0;
  double yMm = 0;
  double vxMmS = 0;
  double vyMmS = 0;
};

struct GoalFieldTarget {
  double xMm = 0;
  double yMm = 0;
};

struct PlannerInputSnapshot {
  PoseState pose;
  double headingDeg = 0;
  BallState ballBody;
  FieldBallState ballField;
  GoalFieldTarget goalTargetField;
  double goalDeg = 0;
};

struct SimSample {
  int step = 0;
  int replanIndex = -1;
  int localActionIndex = -1;
  double timeS = 0;
  double appliedDtS = 0;
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
  double appliedDtS = 0;
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
  PlannerInputSnapshot plannerInputs;
  bool productionRoute = false;
  BallPlanDebug productionDebug;
  CommandedPosePlanDebug poseDebug;
  double targetXMm = 0;
  double targetYMm = 0;
  double targetHeadingDeg = 0;
  bool withinTargetTolerance = false;
  std::vector<SimSample> executedTrace;
};

struct SimulationResult {
  SimulationMode mode = SimulationMode::ProductionBallPlan;
  double controlIntervalS = 0;
  bool reachedGoal = false;
  bool maxReplansHit = false;
  bool maxTimeHit = false;
  std::vector<ReplanResult> replans;
  std::vector<SimSample> trace;
  std::vector<ExecutedActionSample> executedActions;
};

const char* modeName(SimulationMode mode);

}  // namespace ballalgo::sim

#include "FoxGloveSim/FoxgloveTelemetryPublisher.hpp"
#include "config.hpp"
#include "motion/MotionPlanner.hpp"
#include "motion/OffensePose.hpp"
#include "vision/VisionMath.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

using namespace ballalgo;

namespace {

constexpr double kDefaultPublishHz = 2.0;

volatile std::sig_atomic_t gKeepRunning = 1;

enum class GoalMode {
  Blue,
  Yellow,
  CustomPoint,
};

struct RuntimeOptions {
  PoseState pose;
  float headingDeg = 0;
  float robotVxFieldMps = 0;
  float robotVyFieldMps = 0;
  bool robotPoseSet = false;

  float ballXMm = 0;
  float ballYMm = 0;
  float ballVxFieldMps = 0;
  float ballVyFieldMps = 0;
  bool ballPositionSet = false;

  GoalMode goalMode = GoalMode::Yellow;
  float customGoalXMm = 0;
  float customGoalYMm = 0;
  bool customGoalSet = false;

  double publishHz = kDefaultPublishHz;
  double durationS = 0.0;
  std::string foxgloveConfigPath = config::kFoxgloveConfigPath;
};

void handleSignal(int) { gKeepRunning = 0; }

bool parseFloatArg(const char* text, float& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

bool parseDoubleArg(const char* text, double& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  value = std::strtod(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

float centeredCmToFieldMm(float centeredCm, float axisLimitMm) {
  return centeredCm * 10.f + 0.5f * axisLimitMm;
}

int fieldMmToCenteredCm(float fieldMm, float axisLimitMm) {
  return static_cast<int>(std::lround((fieldMm - 0.5f * axisLimitMm) * 0.1f));
}

void setRobotPoseFieldMm(RuntimeOptions& options, float xMm, float yMm, float headingDeg) {
  options.pose.valid = true;
  options.pose.xMm = xMm;
  options.pose.yMm = yMm;
  options.headingDeg = headingDeg;
  options.robotPoseSet = true;
}

void setBallFieldMm(RuntimeOptions& options, float xMm, float yMm) {
  options.ballXMm = xMm;
  options.ballYMm = yMm;
  options.ballPositionSet = true;
}

void updateRobotVelocity(PoseState& pose, float headingDeg, float vxFieldMps, float vyFieldMps) {
  pose.vxMmS = vxFieldMps * 1000.f;
  pose.vyMmS = vyFieldMps * 1000.f;
  fieldVelToBody(pose.vxMmS, pose.vyMmS, headingDeg, pose.vxBody, pose.vyBody);
}

void fieldOffsetToBody(float dxMm, float dyMm, float headingDeg, float& xM, float& yM) {
  const float headingRad = headingDeg * static_cast<float>(M_PI / 180.0);
  const float c = std::cos(headingRad);
  const float s = std::sin(headingRad);
  xM = (c * dxMm + s * dyMm) / 1000.f;
  yM = (-s * dxMm + c * dyMm) / 1000.f;
}

void fieldVelocityToBody(float vxFieldMps, float vyFieldMps, float headingDeg, float& vxBodyMps,
                         float& vyBodyMps) {
  fieldVelToBody(vxFieldMps * 1000.f, vyFieldMps * 1000.f, headingDeg, vxBodyMps, vyBodyMps);
}

BallState makeBallState(const RuntimeOptions& options) {
  BallState ball;
  ball.visible = options.robotPoseSet && options.ballPositionSet;
  if (!ball.visible) return ball;

  fieldOffsetToBody(options.ballXMm - options.pose.xMm, options.ballYMm - options.pose.yMm,
                    options.headingDeg, ball.xM, ball.yM);

  const float relVxFieldMps = options.ballVxFieldMps - options.robotVxFieldMps;
  const float relVyFieldMps = options.ballVyFieldMps - options.robotVyFieldMps;
  fieldVelocityToBody(relVxFieldMps, relVyFieldMps, options.headingDeg, ball.vx, ball.vy);
  return ball;
}

FieldTarget makeGoalFieldTarget(const RuntimeOptions& options) {
  if (options.goalMode == GoalMode::Blue) {
    return {config::kBlueGoalXMm, config::kBlueGoalYMm};
  }
  if (options.goalMode == GoalMode::Yellow) {
    return {config::kYellowGoalXMm, config::kYellowGoalYMm};
  }
  return {options.customGoalXMm, options.customGoalYMm};
}

float goalAngleDeg(const RuntimeOptions& options, const FieldTarget& goalFieldTarget) {
  float bodyX = 0;
  float bodyY = 0;
  fieldOffsetToBody(goalFieldTarget.xMm - options.pose.xMm, goalFieldTarget.yMm - options.pose.yMm,
                    options.headingDeg, bodyX, bodyY);
  return std::atan2(bodyY, bodyX) * 180.f / static_cast<float>(M_PI);
}

double ballAngleDeg(const BallState& ball) {
  return std::atan2(static_cast<double>(ball.yM), static_cast<double>(ball.xM)) * 180.0 / M_PI;
}

PlannerDebugSnapshot makePlannerSnapshot(const BallPlanDebug& debug) {
  PlannerDebugSnapshot snapshot;
  snapshot.valid = true;
  snapshot.commandedGoalMode = false;
  snapshot.withinTolerance = debug.withinTargetTolerance;
  snapshot.usedCenterFallback = debug.usedCenterFallback;
  snapshot.usedBodyChaseFallback = debug.usedBodyChaseFallback;
  snapshot.usedStrikePosePlan = debug.usedStrikePosePlan;
  snapshot.trajectoryId = debug.chunk.trajectoryId;
  snapshot.startTimePi = debug.chunk.startTimePi;
  snapshot.dtMs = debug.chunk.dtMs;
  snapshot.targetXMm = debug.targetXMm;
  snapshot.targetYMm = debug.targetYMm;
  snapshot.targetHeadingDeg = debug.targetHeadingDeg;
  snapshot.path = debug.path;
  snapshot.trajectorySpeedProfile = debug.trajectorySpeedProfile;
  return snapshot;
}

const char* goalModeName(const RuntimeOptions& options) {
  switch (options.goalMode) {
    case GoalMode::Blue:
      return "blue";
    case GoalMode::Yellow:
      return "yellow";
    case GoalMode::CustomPoint:
      return "custom";
  }
  return "unknown";
}

void printUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << "\n"
      << "  --robot-pose x_mm y_mm heading_deg\n"
      << "  --robot-pose-centered-cm x_cm y_cm heading_deg\n"
      << "  --ball-position x_mm y_mm\n"
      << "  --ball-position-centered-cm x_cm y_cm\n"
      << "  [--robot-velocity-field-mps vx vy]\n"
      << "  [--ball-velocity-field-mps vx vy]\n"
      << "  [--goal blue|yellow]\n"
      << "  [--goal-point x_mm y_mm]\n"
      << "  [--goal-point-centered-cm x_cm y_cm]\n"
      << "  [--publish-hz hz]\n"
      << "  [--duration-s seconds]\n"
      << "  [--config foxglove.conf]\n";
}

bool parseArgs(int argc, char** argv, RuntimeOptions& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--robot-pose") {
      float xMm = 0;
      float yMm = 0;
      float headingDeg = 0;
      if (i + 3 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], xMm) || !parseFloatArg(argv[i + 2], yMm) ||
          !parseFloatArg(argv[i + 3], headingDeg)) {
        return false;
      }
      setRobotPoseFieldMm(options, xMm, yMm, headingDeg);
      i += 3;
      continue;
    }
    if (arg == "--robot-pose-centered-cm") {
      float xCm = 0;
      float yCm = 0;
      float headingDeg = 0;
      if (i + 3 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], xCm) || !parseFloatArg(argv[i + 2], yCm) ||
          !parseFloatArg(argv[i + 3], headingDeg)) {
        return false;
      }
      setRobotPoseFieldMm(options, centeredCmToFieldMm(xCm, config::kFieldWidthMm),
                          centeredCmToFieldMm(yCm, config::kFieldHeightMm), headingDeg);
      i += 3;
      continue;
    }
    if (arg == "--ball-position") {
      float xMm = 0;
      float yMm = 0;
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], xMm) || !parseFloatArg(argv[i + 2], yMm)) {
        return false;
      }
      setBallFieldMm(options, xMm, yMm);
      i += 2;
      continue;
    }
    if (arg == "--ball-position-centered-cm") {
      float xCm = 0;
      float yCm = 0;
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], xCm) || !parseFloatArg(argv[i + 2], yCm)) {
        return false;
      }
      setBallFieldMm(options, centeredCmToFieldMm(xCm, config::kFieldWidthMm),
                     centeredCmToFieldMm(yCm, config::kFieldHeightMm));
      i += 2;
      continue;
    }
    if (arg == "--robot-velocity-field-mps") {
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.robotVxFieldMps) ||
          !parseFloatArg(argv[i + 2], options.robotVyFieldMps)) {
        return false;
      }
      i += 2;
      continue;
    }
    if (arg == "--ball-velocity-field-mps") {
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.ballVxFieldMps) ||
          !parseFloatArg(argv[i + 2], options.ballVyFieldMps)) {
        return false;
      }
      i += 2;
      continue;
    }
    if (arg == "--goal") {
      if (i + 1 >= argc) return false;
      const std::string goal = argv[i + 1];
      if (goal == "blue") {
        options.goalMode = GoalMode::Blue;
      } else if (goal == "yellow") {
        options.goalMode = GoalMode::Yellow;
      } else {
        return false;
      }
      ++i;
      continue;
    }
    if (arg == "--goal-point") {
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.customGoalXMm) ||
          !parseFloatArg(argv[i + 2], options.customGoalYMm)) {
        return false;
      }
      options.goalMode = GoalMode::CustomPoint;
      options.customGoalSet = true;
      i += 2;
      continue;
    }
    if (arg == "--goal-point-centered-cm") {
      float xCm = 0;
      float yCm = 0;
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], xCm) || !parseFloatArg(argv[i + 2], yCm)) {
        return false;
      }
      options.customGoalXMm = centeredCmToFieldMm(xCm, config::kFieldWidthMm);
      options.customGoalYMm = centeredCmToFieldMm(yCm, config::kFieldHeightMm);
      options.goalMode = GoalMode::CustomPoint;
      options.customGoalSet = true;
      i += 2;
      continue;
    }
    if (arg == "--publish-hz") {
      if (i + 1 >= argc) return false;
      if (!parseDoubleArg(argv[i + 1], options.publishHz)) return false;
      ++i;
      continue;
    }
    if (arg == "--duration-s") {
      if (i + 1 >= argc) return false;
      if (!parseDoubleArg(argv[i + 1], options.durationS)) return false;
      ++i;
      continue;
    }
    if (arg == "--config") {
      if (i + 1 >= argc) return false;
      options.foxgloveConfigPath = argv[i + 1];
      ++i;
      continue;
    }
    return false;
  }

  if (!options.robotPoseSet || !options.ballPositionSet) return false;
  if (options.goalMode == GoalMode::CustomPoint && !options.customGoalSet) return false;
  if (options.publishHz <= 0.0) return false;
  if (options.durationS < 0.0) return false;

  updateRobotVelocity(options.pose, options.headingDeg, options.robotVxFieldMps,
                      options.robotVyFieldMps);
  return true;
}

void printScenarioSummary(const RuntimeOptions& options, const FieldTarget& goalFieldTarget) {
  std::cout << "ballalgo planner-case starting\n";
  std::cout << "robot start: field=(" << options.pose.xMm << ", " << options.pose.yMm
            << ") mm centered=("
            << fieldMmToCenteredCm(options.pose.xMm, config::kFieldWidthMm) << ", "
            << fieldMmToCenteredCm(options.pose.yMm, config::kFieldHeightMm)
            << ") cm heading=" << options.headingDeg << " deg\n";
  std::cout << "robot velocity: (" << options.robotVxFieldMps << ", "
            << options.robotVyFieldMps << ") field m/s\n";
  std::cout << "ball start: field=(" << options.ballXMm << ", " << options.ballYMm
            << ") mm centered=("
            << fieldMmToCenteredCm(options.ballXMm, config::kFieldWidthMm) << ", "
            << fieldMmToCenteredCm(options.ballYMm, config::kFieldHeightMm)
            << ") cm velocity=(" << options.ballVxFieldMps << ", " << options.ballVyFieldMps
            << ") field m/s\n";
  std::cout << "goal: " << goalModeName(options) << " field=(" << goalFieldTarget.xMm << ", "
            << goalFieldTarget.yMm << ") mm centered=("
            << fieldMmToCenteredCm(goalFieldTarget.xMm, config::kFieldWidthMm) << ", "
            << fieldMmToCenteredCm(goalFieldTarget.yMm, config::kFieldHeightMm)
            << ") cm\n";
  std::cout << "publish-hz=" << options.publishHz << " duration-s=" << options.durationS
            << " foxglove-config=" << options.foxgloveConfigPath << "\n";
}

void printPlanSummary(const BallPlanDebug& debug) {
  std::cout << "planner result: mode=" << offensePoseStateName(debug.offensePose.state)
            << " target=(" << debug.targetXMm << ", " << debug.targetYMm << ") mm heading="
            << debug.targetHeadingDeg << " deg"
            << " path-samples=" << debug.path.size()
            << " speed-samples=" << debug.trajectorySpeedProfile.size()
            << " used-strike=" << (debug.usedStrikePosePlan ? "true" : "false")
            << " used-center-fallback=" << (debug.usedCenterFallback ? "true" : "false")
            << " used-body-fallback=" << (debug.usedBodyChaseFallback ? "true" : "false")
            << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  RuntimeOptions options;
  if (!parseArgs(argc, argv, options)) {
    printUsage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  const FieldTarget goalFieldTarget = makeGoalFieldTarget(options);
  printScenarioSummary(options, goalFieldTarget);

  FoxgloveTelemetryPublisher foxglove(options.foxgloveConfigPath);
  MotionPlanner planner;

  const auto startTime = std::chrono::steady_clock::now();
  auto lastTick = startTime;
  auto nextTick = startTime;
  const auto period =
      std::chrono::duration<double>(1.0 / std::max(options.publishHz, 0.1));

  bool printedPlan = false;
  unsigned long loopCount = 0;

  while (gKeepRunning) {
    const auto now = std::chrono::steady_clock::now();
    const double dtS = std::chrono::duration<double>(now - lastTick).count();
    lastTick = now;

    BallState ball = makeBallState(options);
    const float goalDeg = goalAngleDeg(options, goalFieldTarget);
    const BallPlanDebug debug =
        planner.debugPlan(options.pose, ball, goalDeg, options.headingDeg, true, &goalFieldTarget);
    if (!printedPlan) {
      printPlanSummary(debug);
      printedPlan = true;
    }

    FoxgloveTelemetryFrame frame;
    frame.dtS = dtS > 0.0 ? dtS : (1.0 / options.publishHz);
    frame.loopCount = ++loopCount;
    frame.headingDeg = options.headingDeg;
    frame.pose = options.pose;
    frame.ball = ball;
    frame.visionBallAngleDeg = ballAngleDeg(ball);
    frame.visionBallDistance = std::hypot(static_cast<double>(ball.xM), static_cast<double>(ball.yM));
    frame.planner = makePlannerSnapshot(debug);
    foxglove.publish(frame);

    if (options.durationS > 0.0) {
      const double elapsedS = std::chrono::duration<double>(now - startTime).count();
      if (elapsedS >= options.durationS) break;
    }

    nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(nextTick);
  }

  std::cout << "ballalgo planner-case exiting after " << loopCount << " publish iterations\n";
  return 0;
}

#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"

namespace ballalgo {

enum class OffensePoseState {
  Invalid = 0,
  NormalStrike,
  CollectBallNearEnemyGoalLine,
  CollectBallBehindRobotNearOurGoalLine,
  CollectBallAtBoundary,
  CollectBallInCorner,
};

struct OffensePoseResult {
  bool valid = false;
  OffensePoseState state = OffensePoseState::Invalid;
  bool usesDirectBallPose = false;
  bool usesTerminalVelocity = false;
  bool ballInFrontOfRobot = false;
  bool ballPastEnemyGoalLine = false;
  bool ballPastOurGoalLine = false;
  bool ballAtOrBeyondBoundary = false;
  float currentBallFieldXMm = 0;
  float currentBallFieldYMm = 0;
  float predictedBallBodyXM = 0;
  float predictedBallBodyYM = 0;
  float predictedBallBodyVXMps = 0;
  float predictedBallBodyVYMps = 0;
  float predictedBallFieldXMm = 0;
  float predictedBallFieldYMm = 0;
  float targetXMm = 0;
  float targetYMm = 0;
  float targetHeadingDeg = 0;
  float targetVxFieldMps = 0;
  float targetVyFieldMps = 0;
  float targetOmegaRadS = 0;
  float strikeTargetBodyXM = 0;
  float strikeTargetBodyYM = 0;
};

const char* offensePoseStateName(OffensePoseState state);

OffensePoseResult computeOffensePose(const PoseState& pose, const BallState& ball, float goalDeg,
                                     float headingDeg, bool hasGoalFieldTarget,
                                     float goalFieldXMm, float goalFieldYMm,
                                     float interceptTimeS);

}  // namespace ballalgo

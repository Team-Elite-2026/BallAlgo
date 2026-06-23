#include "motion/OffensePose.hpp"

#include "config.hpp"
#include "params.hpp"
#include "motion/StrikePose.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

namespace {

float minPoseXMm() {
  return params::get().offenseBoundaryInsetXMm;
}

float maxPoseXMm() {
  return config::kFieldWidthMm - params::get().offenseBoundaryInsetXMm;
}

float minPoseYMm() {
  return params::get().offenseBoundaryInsetYMm;
}

float maxPoseYMm() {
  return config::kFieldHeightMm - params::get().offenseBoundaryInsetYMm;
}

bool ballPastLeftBoundary(float ballXMm) {
  return ballXMm - config::kOffenseBallRadiusMm <= params::get().offenseBoundaryInsetXMm;
}

bool ballPastRightBoundary(float ballXMm) {
  return ballXMm + config::kOffenseBallRadiusMm >=
         config::kFieldWidthMm - params::get().offenseBoundaryInsetXMm;
}

bool ballPastBottomBoundary(float ballYMm) {
  return ballYMm - config::kOffenseBallRadiusMm <= params::get().offenseBoundaryInsetYMm;
}

bool ballPastTopBoundary(float ballYMm) {
  return ballYMm + config::kOffenseBallRadiusMm >=
         config::kFieldHeightMm - params::get().offenseBoundaryInsetYMm;
}

void clampPoseToSafeBox(float& xMm, float& yMm) {
  xMm = std::clamp(xMm, minPoseXMm(), maxPoseXMm());
  yMm = std::clamp(yMm, minPoseYMm(), maxPoseYMm());
}

bool poseOutsideSafeBox(float xMm, float yMm) {
  return xMm < minPoseXMm() || xMm > maxPoseXMm() || yMm < minPoseYMm() ||
         yMm > maxPoseYMm();
}

float headingBetweenPointsDeg(float fromX, float fromY, float toX, float toY) {
  return std::atan2(toX - fromX, toY - fromY) * 180.f / static_cast<float>(M_PI);
}

float wrapAngleDeg(float angleDeg) {
  float wrapped = std::fmod(angleDeg + 180.f, 360.f);
  if (wrapped < 0) wrapped += 360.f;
  return wrapped - 180.f;
}

void fieldPointToBodyMeters(float fieldXMm, float fieldYMm, const PoseState& pose, float headingDeg,
                            float& bodyXM, float& bodyYM) {
  const float headingRad = headingDeg * static_cast<float>(M_PI / 180.0);
  const float c = std::cos(headingRad);
  const float s = std::sin(headingRad);
  const float dxMm = fieldXMm - pose.xMm;
  const float dyMm = fieldYMm - pose.yMm;
  bodyXM = (c * dxMm + s * dyMm) / 1000.f;
  bodyYM = (-s * dxMm + c * dyMm) / 1000.f;
}

bool ballAtOrBeyondBoundary(float ballXMm, float ballYMm) {
  return ballPastLeftBoundary(ballXMm) || ballPastRightBoundary(ballXMm) ||
         ballPastBottomBoundary(ballYMm) || ballPastTopBoundary(ballYMm);
}

bool ballInCornerBand(float ballXMm, float ballYMm) {
  const bool nearLeft = ballPastLeftBoundary(ballXMm);
  const bool nearRight = ballPastRightBoundary(ballXMm);
  const bool nearBottom = ballPastBottomBoundary(ballYMm);
  const bool nearTop = ballPastTopBoundary(ballYMm);
  return (nearLeft || nearRight) && (nearBottom || nearTop);
}

bool attackRunsPositiveY(float goalFieldYMm) {
  return goalFieldYMm >= config::kFieldHeightMm * 0.5f;
}

float enemyGoalLineYMm(bool attackPositiveY) {
  return attackPositiveY ? config::kOffenseHighGoalLineYMm
                         : config::kOffenseLowGoalLineYMm;
}

float ourGoalLineYMm(bool attackPositiveY) {
  return attackPositiveY ? config::kOffenseLowGoalLineYMm
                         : config::kOffenseHighGoalLineYMm;
}

OffensePoseState classifyOffensePoseState(float ballFieldXMm, float ballFieldYMm,
                                          bool attackPositiveY) {
  if (ballInCornerBand(ballFieldXMm, ballFieldYMm)) {
    return OffensePoseState::CollectBallInCorner;
  }
  if ((attackPositiveY && ballFieldYMm >= enemyGoalLineYMm(true)) ||
      (!attackPositiveY && ballFieldYMm <= enemyGoalLineYMm(false))) {
    return OffensePoseState::CollectBallNearEnemyGoalLine;
  }
  if ((attackPositiveY && ballFieldYMm <= ourGoalLineYMm(true)) ||
      (!attackPositiveY && ballFieldYMm >= ourGoalLineYMm(false))) {
    return OffensePoseState::CollectBallBehindRobotNearOurGoalLine;
  }
  if (ballAtOrBeyondBoundary(ballFieldXMm, ballFieldYMm)) {
    return OffensePoseState::CollectBallAtBoundary;
  }
  return OffensePoseState::NormalStrike;
}

void setTargetFacingBall(OffensePoseResult& result, const PoseState& pose, float headingDeg,
                         float targetXMm, float targetYMm) {
  clampPoseToSafeBox(targetXMm, targetYMm);
  result.targetXMm = targetXMm;
  result.targetYMm = targetYMm;
  result.targetHeadingDeg = headingBetweenPointsDeg(result.targetXMm, result.targetYMm,
                                                    result.predictedBallFieldXMm,
                                                    result.predictedBallFieldYMm);
  fieldPointToBodyMeters(result.targetXMm, result.targetYMm, pose, headingDeg,
                         result.strikeTargetBodyXM, result.strikeTargetBodyYM);
}

void setTerminalVelocityHold(OffensePoseResult& result) {
  result.usesTerminalVelocity = true;
  result.targetVxFieldMps = 0.f;
  result.targetVyFieldMps = 0.f;
  result.targetOmegaRadS = 0.f;
}

void setForwardSafeStrikeTarget(OffensePoseResult& result, const PoseState& pose,
                                float headingDeg, bool attackPositiveY) {
  const float attackSignY = attackPositiveY ? 1.f : -1.f;
  float targetXMm = result.predictedBallFieldXMm;
  float targetYMm =
      result.predictedBallFieldYMm - attackSignY * params::get().offenseIntakeOffsetMm;
  clampPoseToSafeBox(targetXMm, targetYMm);
  result.targetXMm = targetXMm;
  result.targetYMm = targetYMm;
  result.targetHeadingDeg = attackPositiveY ? 0.f : 180.f;
  fieldPointToBodyMeters(result.targetXMm, result.targetYMm, pose, headingDeg,
                         result.strikeTargetBodyXM, result.strikeTargetBodyYM);
}

void fillDirectBallTarget(OffensePoseResult& result, const PoseState& pose, float headingDeg,
                          bool attackPositiveY) {
  const float strikeOffsetMm = params::get().offenseIntakeOffsetMm;
  const float attackSignY = attackPositiveY ? 1.f : -1.f;
  result.usesDirectBallPose = true;

  if (result.state == OffensePoseState::CollectBallInCorner) {
    const bool nearLeft = ballPastLeftBoundary(result.predictedBallFieldXMm);
    const bool nearBottom = ballPastBottomBoundary(result.predictedBallFieldYMm);
    const float targetXMm = nearLeft ? minPoseXMm() : maxPoseXMm();
    const float targetYMm = nearBottom ? minPoseYMm() : maxPoseYMm();
    setTargetFacingBall(result, pose, headingDeg, targetXMm, targetYMm);
    return;
  }

  if (result.state == OffensePoseState::CollectBallAtBoundary) {
    const float leftLineXMm = params::get().offenseBoundaryInsetXMm;
    const float rightLineXMm = config::kFieldWidthMm - params::get().offenseBoundaryInsetXMm;
    const float bottomLineYMm = params::get().offenseBoundaryInsetYMm;
    const float topLineYMm = config::kFieldHeightMm - params::get().offenseBoundaryInsetYMm;

    const float leftDistMm = std::fabs(result.predictedBallFieldXMm - leftLineXMm);
    const float rightDistMm = std::fabs(result.predictedBallFieldXMm - rightLineXMm);
    const float bottomDistMm = std::fabs(result.predictedBallFieldYMm - bottomLineYMm);
    const float topDistMm = std::fabs(result.predictedBallFieldYMm - topLineYMm);

    if (leftDistMm <= rightDistMm && leftDistMm <= bottomDistMm &&
        leftDistMm <= topDistMm) {
      setTargetFacingBall(result, pose, headingDeg,
                          result.predictedBallFieldXMm + strikeOffsetMm,
                          result.predictedBallFieldYMm);
    } else if (rightDistMm <= bottomDistMm && rightDistMm <= topDistMm) {
      setTargetFacingBall(result, pose, headingDeg,
                          result.predictedBallFieldXMm - strikeOffsetMm,
                          result.predictedBallFieldYMm);
    } else if (bottomDistMm <= topDistMm) {
      setTargetFacingBall(result, pose, headingDeg, result.predictedBallFieldXMm,
                          result.predictedBallFieldYMm + strikeOffsetMm);
    } else {
      setTargetFacingBall(result, pose, headingDeg, result.predictedBallFieldXMm,
                          result.predictedBallFieldYMm - strikeOffsetMm);
    }
    return;
  }

  if (result.state == OffensePoseState::CollectBallNearEnemyGoalLine) {
    setTargetFacingBall(result, pose, headingDeg, result.predictedBallFieldXMm,
                        result.predictedBallFieldYMm - attackSignY * strikeOffsetMm);
    return;
  }

  if (result.state == OffensePoseState::CollectBallBehindRobotNearOurGoalLine) {
    setTargetFacingBall(result, pose, headingDeg, result.predictedBallFieldXMm,
                        result.predictedBallFieldYMm + attackSignY * strikeOffsetMm);
    return;
  }
}

}  // namespace

const char* offensePoseStateName(OffensePoseState state) {
  switch (state) {
    case OffensePoseState::Invalid:
      return "invalid";
    case OffensePoseState::NormalStrike:
      return "normal_strike";
    case OffensePoseState::CollectBallNearEnemyGoalLine:
      return "collect_ball_beyond_enemy_goal_line";
    case OffensePoseState::CollectBallBehindRobotNearOurGoalLine:
      return "collect_ball_behind_our_goal_line";
    case OffensePoseState::CollectBallAtBoundary:
      return "collect_ball_at_boundary";
    case OffensePoseState::CollectBallInCorner:
      return "collect_ball_in_corner";
  }
  return "unknown";
}

OffensePoseResult computeOffensePose(const PoseState& pose, const BallState& ball, float goalDeg,
                                     float headingDeg, bool hasGoalFieldTarget,
                                     float goalFieldXMm, float goalFieldYMm,
                                     float interceptTimeS) {
  OffensePoseResult result;
  if (!pose.valid || !ball.visible) return result;

  result.valid = true;
  ballFieldMm(pose.xMm, pose.yMm, ball.xM, ball.yM, headingDeg, result.currentBallFieldXMm,
              result.currentBallFieldYMm);

  const PredictedBallPose predicted =
      predictBallBody(ball.xM, ball.yM, ball.vx, ball.vy, std::max(0.f, interceptTimeS));
  result.predictedBallBodyXM = predicted.xM;
  result.predictedBallBodyYM = predicted.yM;
  result.predictedBallBodyVXMps = predicted.vx;
  result.predictedBallBodyVYMps = predicted.vy;
  ballFieldMm(pose.xMm, pose.yMm, predicted.xM, predicted.yM, headingDeg,
              result.predictedBallFieldXMm, result.predictedBallFieldYMm);
  const bool attackPositiveY = !hasGoalFieldTarget || attackRunsPositiveY(goalFieldYMm);
  setTerminalVelocityHold(result);
  result.ballInFrontOfRobot = result.predictedBallBodyXM >= 0.f;
  result.ballPastEnemyGoalLine =
      attackPositiveY ? (result.predictedBallFieldYMm >= enemyGoalLineYMm(true))
                      : (result.predictedBallFieldYMm <= enemyGoalLineYMm(false));
  result.ballPastOurGoalLine =
      attackPositiveY ? (result.predictedBallFieldYMm <= ourGoalLineYMm(true))
                      : (result.predictedBallFieldYMm >= ourGoalLineYMm(false));
  result.ballAtOrBeyondBoundary =
      ballAtOrBeyondBoundary(result.predictedBallFieldXMm, result.predictedBallFieldYMm);
  result.state = classifyOffensePoseState(result.predictedBallFieldXMm,
                                          result.predictedBallFieldYMm, attackPositiveY);

  if (result.state != OffensePoseState::NormalStrike) {
    fillDirectBallTarget(result, pose, headingDeg, attackPositiveY);
    return result;
  }

  if (hasGoalFieldTarget) {
    const float goalDxMm = goalFieldXMm - result.predictedBallFieldXMm;
    const float goalDyMm = goalFieldYMm - result.predictedBallFieldYMm;
    const float goalDistMm = std::hypot(goalDxMm, goalDyMm);
    if (goalDistMm > 1e-3f) {
      const float targetXMm =
          result.predictedBallFieldXMm - params::get().offenseIntakeOffsetMm * (goalDxMm / goalDistMm);
      const float targetYMm =
          result.predictedBallFieldYMm - params::get().offenseIntakeOffsetMm * (goalDyMm / goalDistMm);
      if (poseOutsideSafeBox(targetXMm, targetYMm)) {
        setForwardSafeStrikeTarget(result, pose, headingDeg, attackPositiveY);
        return result;
      }
      result.targetXMm = targetXMm;
      result.targetYMm = targetYMm;
      clampPoseToSafeBox(result.targetXMm, result.targetYMm);
      fieldPointToBodyMeters(result.targetXMm, result.targetYMm, pose, headingDeg,
                             result.strikeTargetBodyXM, result.strikeTargetBodyYM);
      result.targetHeadingDeg =
          headingBetweenPointsDeg(result.targetXMm, result.targetYMm, goalFieldXMm, goalFieldYMm);
      return result;
    }
  }
  const float goalRad = goalDeg * static_cast<float>(M_PI / 180.0);
  const float fallbackTargetBodyXM =
      predicted.xM - params::get().offenseIntakeOffsetMm / 1000.f * std::cos(goalRad);
  const float fallbackTargetBodyYM =
      predicted.yM - params::get().offenseIntakeOffsetMm / 1000.f * std::sin(goalRad);
  ballFieldMm(pose.xMm, pose.yMm, fallbackTargetBodyXM, fallbackTargetBodyYM, headingDeg,
              result.targetXMm, result.targetYMm);
  if (poseOutsideSafeBox(result.targetXMm, result.targetYMm)) {
    setForwardSafeStrikeTarget(result, pose, headingDeg, attackPositiveY);
    return result;
  }
  result.strikeTargetBodyXM = fallbackTargetBodyXM;
  result.strikeTargetBodyYM = fallbackTargetBodyYM;
  clampPoseToSafeBox(result.targetXMm, result.targetYMm);
  result.targetHeadingDeg = wrapAngleDeg(headingDeg + goalDeg);
  return result;
}

}  // namespace ballalgo

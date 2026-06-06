#include "motion/DefensePose.hpp"

#include "config.hpp"
#include "motion/MotionLimits.hpp"
#include "motion/StrikePose.hpp"

#include <algorithm>
#include <cmath>

namespace ballalgo {

namespace {

constexpr float kMmPerCm = 10.f;
constexpr float kDefenseArcCenterXCm = 40.f;
constexpr float kDefenseArcCenterYCm = 25.f;

enum class DefenseGoalLineSegment {
  TopLine = 0,
  SideLine,
  Arc,
};

float signNonZero(float value) { return value < 0.f ? -1.f : 1.f; }

float clampMagnitude(float value, float limit) {
  return std::clamp(value, -limit, limit);
}

float targetHeadingFromBallGoal(float ballRelXCm, float ballRelYCm) {
  return std::atan2(ballRelYCm, ballRelXCm) * 180.f / static_cast<float>(M_PI);
}

void fieldBallFromBody(const PoseState& pose, const BallState& ball, float headingDeg,
                       float& ballXMm, float& ballYMm, float& ballVxMps, float& ballVyMps) {
  ballFieldMm(pose.xMm, pose.yMm, ball.xM, ball.yM, headingDeg, ballXMm, ballYMm);

  const float headingRad = headingDeg * static_cast<float>(M_PI / 180.0);
  const float c = std::cos(headingRad);
  const float s = std::sin(headingRad);
  ballVxMps = c * ball.vx - s * ball.vy + pose.vxMmS / 1000.f;
  ballVyMps = s * ball.vx + c * ball.vy + pose.vyMmS / 1000.f;
}

void goalLinePointFromBall(float ballRelXCm, float ballRelYCm, float& targetRelXCm,
                           float& targetRelYCm, DefenseGoalLineSegment& segment) {
  const float absX = std::max(std::fabs(ballRelXCm), 1e-3f);
  const float m = ballRelYCm / absX;
  if (m >= 1.f) {
    segment = DefenseGoalLineSegment::TopLine;
    targetRelXCm = config::kDefenseGoalLineYMinCm / m;
    targetRelYCm = config::kDefenseGoalLineYMinCm;
  } else if (m <= -5.f / 11.f) {
    segment = DefenseGoalLineSegment::SideLine;
    targetRelXCm = config::kDefenseGoalLineXMaxCm;
    targetRelYCm = config::kDefenseGoalLineXMaxCm * m;
  } else {
    segment = DefenseGoalLineSegment::Arc;
    const float a = 1.f + m * m;
    const float b = -2.f * (config::kDefenseGoalLineYMinCm + 25.f * m);
    const float c = config::kDefenseGoalLineQuadraticC;
    const float disc = std::max(0.f, b * b - 4.f * a * c);
    targetRelXCm = (-b + std::sqrt(disc)) / (2.f * a);
    targetRelYCm = m * targetRelXCm;
  }
  targetRelXCm *= signNonZero(ballRelXCm);
}

void goalLineTangentUnitFromTarget(float targetRelXCm, float targetRelYCm,
                                   DefenseGoalLineSegment segment, float& pathUx,
                                   float& pathUy) {
  if (segment == DefenseGoalLineSegment::TopLine) {
    pathUx = 1.f;
    pathUy = 0.f;
    return;
  }
  if (segment == DefenseGoalLineSegment::SideLine) {
    pathUx = 0.f;
    pathUy = 1.f;
    return;
  }

  const float centerXCm = signNonZero(targetRelXCm) * kDefenseArcCenterXCm;
  const float radialX = targetRelXCm - centerXCm;
  const float radialY = targetRelYCm - kDefenseArcCenterYCm;
  const float radialLen = std::hypot(radialX, radialY);
  if (radialLen <= 1e-5f) {
    pathUx = 0.f;
    pathUy = 1.f;
    return;
  }

  pathUx = -radialY / radialLen;
  pathUy = radialX / radialLen;
}

DefensePoseResult blockPoseForBall(float ballXMm, float ballYMm, float ballVxMps,
                                   float ballVyMps,
                                   const DefenseFieldTarget& defendedGoal) {
  DefensePoseResult result;
  const float ballRelXCm = (ballXMm - defendedGoal.xMm) / kMmPerCm;
  const float ballRelYCm = (ballYMm - defendedGoal.yMm) / kMmPerCm;

  float targetRelXCm = 0.f;
  float targetRelYCm = 0.f;
  DefenseGoalLineSegment unusedSegment = DefenseGoalLineSegment::TopLine;
  goalLinePointFromBall(ballRelXCm, ballRelYCm, targetRelXCm, targetRelYCm, unusedSegment);

  result.valid = true;
  result.targetXMm = defendedGoal.xMm + targetRelXCm * kMmPerCm;
  result.targetYMm = defendedGoal.yMm + targetRelYCm * kMmPerCm;
  result.targetHeadingDeg = targetHeadingFromBallGoal(ballRelXCm, ballRelYCm);
  result.futureBallXMm = ballXMm;
  result.futureBallYMm = ballYMm;
  result.futureBallVxMps = ballVxMps;
  result.futureBallVyMps = ballVyMps;
  result.initialTargetXMm = result.targetXMm;
  result.initialTargetYMm = result.targetYMm;
  result.initialTargetHeadingDeg = result.targetHeadingDeg;
  return result;
}

float impactTimeForDampedTravel(float distanceMm, float approachSpeedMps) {
  const float distanceM = distanceMm / 1000.f;
  const float dtCam = 1.f / static_cast<float>(config::kCameraFps);
  const float gamma = config::kBallPredictionDamping;
  if (distanceM <= 1e-6f) return 0.f;
  if (approachSpeedMps <= 1e-6f) return -1.f;
  if (std::fabs(1.f - gamma) <= 1e-5f) return distanceM / approachSpeedMps;

  const float maxDistanceM = approachSpeedMps * dtCam / (1.f - gamma);
  if (maxDistanceM < distanceM) return -1.f;

  const float inside = 1.f - distanceM * (1.f - gamma) / (approachSpeedMps * dtCam);
  if (inside <= 0.f) return -1.f;
  const float samples = std::log(inside) / std::log(gamma);
  return samples * dtCam;
}

void maybeApplyInterceptVelocity(DefensePoseResult& result,
                                 const DefenseFieldTarget& defendedGoal,
                                 const std::vector<Waypoint3>& initialWaypoints,
                                 float pathTimeS) {
  result.pathTimeS = pathTimeS;
  if (pathTimeS <= 0.f) return;

  const float ballGoalX = result.futureBallXMm - defendedGoal.xMm;
  const float ballGoalY = result.futureBallYMm - defendedGoal.yMm;
  const float ballGoalLen = std::hypot(ballGoalX, ballGoalY);
  if (ballGoalLen <= 1e-3f) return;

  const float uxBallGoalX = ballGoalX / ballGoalLen;
  const float uxBallGoalY = ballGoalY / ballGoalLen;
  result.scoreDistanceMm =
      std::hypot(result.futureBallXMm - result.initialTargetXMm,
                 result.futureBallYMm - result.initialTargetYMm);
  result.approachSpeedMps =
      -(result.futureBallVxMps * uxBallGoalX + result.futureBallVyMps * uxBallGoalY);
  if (result.approachSpeedMps <= 0.f) return;

  const float dtCam = 1.f / static_cast<float>(config::kCameraFps);
  const float gamma = config::kBallPredictionDamping;
  result.maxApproachDistanceMm =
      result.approachSpeedMps * dtCam / std::max(1e-5f, 1.f - gamma) * 1000.f;
  if (result.maxApproachDistanceMm < result.scoreDistanceMm) return;

  result.impactTimeS = impactTimeForDampedTravel(result.scoreDistanceMm, result.approachSpeedMps);
  if (result.impactTimeS < 0.f || result.impactTimeS > pathTimeS + config::kDefenseImpactMarginS) {
    return;
  }

  if (initialWaypoints.size() < 2) return;
  const Waypoint3& previous = initialWaypoints[initialWaypoints.size() - 2];
  const Waypoint3& target = initialWaypoints.back();
  const float approachPhiGlobal =
      std::atan2(target.yMm - previous.yMm, target.xMm - previous.xMm);
  const float localEndPhi =
      approachPhiGlobal - previous.thetaDeg * static_cast<float>(M_PI / 180.0);
  const float vMax = motion::vMaxDir(localEndPhi, config::kVMaxX, config::kVMaxY);

  const float targetRelX = result.initialTargetXMm - defendedGoal.xMm;
  const float targetRelY = result.initialTargetYMm - defendedGoal.yMm;
  const float targetRelLen = std::hypot(targetRelX, targetRelY);
  if (targetRelLen <= 1e-3f) return;

  const float rayUx = targetRelX / targetRelLen;
  const float rayUy = targetRelY / targetRelLen;
  const float targetRelXCm = targetRelX / kMmPerCm;
  const float targetRelYCm = targetRelY / kMmPerCm;
  DefenseGoalLineSegment targetSegment = DefenseGoalLineSegment::Arc;
  if (std::fabs(targetRelYCm - config::kDefenseGoalLineYMinCm) <= 1e-3f) {
    targetSegment = DefenseGoalLineSegment::TopLine;
  } else if (std::fabs(std::fabs(targetRelXCm) - config::kDefenseGoalLineXMaxCm) <= 1e-3f) {
    targetSegment = DefenseGoalLineSegment::SideLine;
  }

  float pathUx = 0.f;
  float pathUy = 0.f;
  goalLineTangentUnitFromTarget(targetRelXCm, targetRelYCm, targetSegment, pathUx, pathUy);
  const float denom = rayUy * pathUx - rayUx * pathUy;
  if (std::fabs(denom) <= 1e-5f) return;

  const float ballGoalXM = ballGoalX / 1000.f;
  const float ballGoalYM = ballGoalY / 1000.f;
  const float ballGoalLenSqM = ballGoalXM * ballGoalXM + ballGoalYM * ballGoalYM;
  const float targetRelLenM = targetRelLen / 1000.f;
  const float omegaRay =
      (result.futureBallVxMps * ballGoalYM - result.futureBallVyMps * ballGoalXM) /
      std::max(1e-6f, ballGoalLenSqM);
  float vScalar = omegaRay * targetRelLenM / denom;
  vScalar = clampMagnitude(vScalar, vMax);

  result.targetVxFieldMps = vScalar * pathUx;
  result.targetVyFieldMps = vScalar * pathUy;

  const float dRelX = (result.futureBallXMm - result.initialTargetXMm) / 1000.f;
  const float dRelY = (result.futureBallYMm - result.initialTargetYMm) / 1000.f;
  const float dRelLenSq = dRelX * dRelX + dRelY * dRelY;
  if (dRelLenSq > 1e-6f) {
    const float vRelX = result.futureBallVxMps - result.targetVxFieldMps;
    const float vRelY = result.futureBallVyMps - result.targetVyFieldMps;
    result.targetOmegaRadS =
        clampMagnitude((dRelY * vRelX - dRelX * vRelY) / dRelLenSq, config::kOmegaMaxRadS);
  }

  result.usesInterceptVelocity = true;
}

}  // namespace

DefensePoseResult computeDefensePose(const PoseState& pose, const BallState& ball,
                                      float headingDeg,
                                      const DefenseFieldTarget& defendedGoal,
                                      const std::vector<Waypoint3>& initialWaypoints,
                                      float pathTimeS) {
  DefensePoseResult result;
  if (!pose.valid || !ball.visible) return result;

  float ballXMm = 0.f;
  float ballYMm = 0.f;
  float ballVxMps = 0.f;
  float ballVyMps = 0.f;
  fieldBallFromBody(pose, ball, headingDeg, ballXMm, ballYMm, ballVxMps, ballVyMps);
  result = blockPoseForBall(ballXMm, ballYMm, ballVxMps, ballVyMps, defendedGoal);

  const float safeTimeS = std::min(pathTimeS, config::kDefenseFutureBallMaxTimeS);
  if (safeTimeS > 0.f) {
    const float bodyHeadingRad = headingDeg * static_cast<float>(M_PI / 180.0);
    const float c = std::cos(-bodyHeadingRad);
    const float s = std::sin(-bodyHeadingRad);
    const float relVxMps = ballVxMps - pose.vxMmS / 1000.f;
    const float relVyMps = ballVyMps - pose.vyMmS / 1000.f;
    const float ballBodyVx = c * relVxMps - s * relVyMps;
    const float ballBodyVy = s * relVxMps + c * relVyMps;
    const PredictedBallPose future =
        predictBallBody(ball.xM, ball.yM, ballBodyVx, ballBodyVy, safeTimeS);
    ballFieldMm(pose.xMm, pose.yMm, future.xM, future.yM, headingDeg, ballXMm, ballYMm);
    const float headingRad = headingDeg * static_cast<float>(M_PI / 180.0);
    ballVxMps = std::cos(headingRad) * future.vx - std::sin(headingRad) * future.vy +
                pose.vxMmS / 1000.f;
    ballVyMps = std::sin(headingRad) * future.vx + std::cos(headingRad) * future.vy +
                pose.vyMmS / 1000.f;
    result = blockPoseForBall(ballXMm, ballYMm, ballVxMps, ballVyMps, defendedGoal);
  }

  maybeApplyInterceptVelocity(result, defendedGoal, initialWaypoints, pathTimeS);
  return result;
}

}  // namespace ballalgo

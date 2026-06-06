#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "motion/AStar3D.hpp"

#include <vector>

namespace ballalgo {

struct DefenseFieldTarget {
  float xMm = 0;
  float yMm = 0;
};

struct DefensePoseResult {
  bool valid = false;
  bool usesInterceptVelocity = false;
  float targetXMm = 0;
  float targetYMm = 0;
  float targetHeadingDeg = 0;
  float targetVxFieldMps = 0;
  float targetVyFieldMps = 0;
  float targetOmegaRadS = 0;
  float initialTargetXMm = 0;
  float initialTargetYMm = 0;
  float initialTargetHeadingDeg = 0;
  float futureBallXMm = 0;
  float futureBallYMm = 0;
  float futureBallVxMps = 0;
  float futureBallVyMps = 0;
  float pathTimeS = 0;
  float impactTimeS = 0;
  float scoreDistanceMm = 0;
  float approachSpeedMps = 0;
  float maxApproachDistanceMm = 0;
};

DefensePoseResult computeDefensePose(const PoseState& pose, const BallState& ball,
                                      float headingDeg,
                                      const DefenseFieldTarget& defendedGoal,
                                      const std::vector<Waypoint3>& initialWaypoints,
                                      float pathTimeS);

}  // namespace ballalgo

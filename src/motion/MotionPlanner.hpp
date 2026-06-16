#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "motion/AStar3D.hpp"
#include "motion/DefensePose.hpp"
#include "motion/OffensePose.hpp"
#include "motion/HermiteSpline.hpp"
#include "motion/VelocityProfile.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

struct PlannedChunk {
  uint64_t trajectoryId = 0;
  uint64_t startTimePi = 0;
  uint16_t dtMs = 0;
  uint8_t kick = 0;
  uint8_t dribblerPower = 0;
  std::vector<MotionAction> actions;
};

struct TrajectorySpeedSample {
  float progress01 = 0;
  float speedMps = 0;
};

struct CommandedPoseGoal {
  float xMm = 0;
  float yMm = 0;
  float headingDeg = 0;
};

struct FieldTarget {
  float xMm = 0;
  float yMm = 0;
};

struct CommandedPosePlanDebug {
  PoseState startPose;
  CommandedPoseGoal goal;
  float startHeadingDeg = 0;
  float posErrMm = 0;
  float headingErrDeg = 0;
  bool withinTolerance = false;
  PlannedChunk chunk;
  std::vector<Waypoint3> waypoints;
  std::vector<PathSample> path;
  std::vector<ProfileSample> profile;
  std::vector<TrajectorySpeedSample> trajectorySpeedProfile;
};

struct BallPlanDebug {
  PoseState startPose;
  BallState ball;
  OffensePoseResult offensePose;
  float goalDeg = 0;
  bool usedGoalFieldTarget = false;
  float goalFieldXMm = 0;
  float goalFieldYMm = 0;
  float startHeadingDeg = 0;
  bool fullPlanner = false;
  bool usedCenterFallback = false;
  bool usedBodyChaseFallback = false;
  bool usedStrikePosePlan = false;
  float interceptTimeS = 0;
  int interceptIterations = 0;
  float strikeTargetBodyXM = 0;
  float strikeTargetBodyYM = 0;
  float predictedBallBodyXM = 0;
  float predictedBallBodyYM = 0;
  float predictedBallBodyVXMps = 0;
  float predictedBallBodyVYMps = 0;
  float ballFieldXMm = 0;
  float ballFieldYMm = 0;
  float targetXMm = 0;
  float targetYMm = 0;
  float targetHeadingDeg = 0;
  float targetVxFieldMps = 0;
  float targetVyFieldMps = 0;
  float targetOmegaRadS = 0;
  float targetErrMm = 0;
  bool withinTargetTolerance = false;
  PlannedChunk chunk;
  std::vector<Waypoint3> waypoints;
  std::vector<PathSample> path;
  std::vector<ProfileSample> profile;
  std::vector<TrajectorySpeedSample> trajectorySpeedProfile;
};

struct DefensePlanDebug {
  PoseState startPose;
  BallState ball;
  DefenseFieldTarget defendedGoal;
  DefensePoseResult defensePose;
  float startHeadingDeg = 0;
  float targetErrMm = 0;
  bool withinTargetTolerance = false;
  PlannedChunk chunk;
  std::vector<Waypoint3> waypoints;
  std::vector<PathSample> path;
  std::vector<ProfileSample> profile;
  std::vector<TrajectorySpeedSample> trajectorySpeedProfile;
};

class MotionPlanner {
 public:
  MotionPlanner();
  PlannedChunk plan(const PoseState& pose, const BallState& ball, float goalDeg,
                    float headingDeg, bool fullPlanner);
  PlannedChunk plan(const PoseState& pose, const BallState& ball, float goalDeg,
                    float headingDeg, bool fullPlanner, const FieldTarget* goalFieldTarget);
  BallPlanDebug debugPlan(const PoseState& pose, const BallState& ball, float goalDeg,
                          float headingDeg, bool fullPlanner);
  BallPlanDebug debugPlan(const PoseState& pose, const BallState& ball, float goalDeg,
                          float headingDeg, bool fullPlanner, const FieldTarget* goalFieldTarget);
  PlannedChunk planToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                          float headingDeg);
  CommandedPosePlanDebug debugPlanToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                                         float headingDeg);
  PlannedChunk planDefense(const PoseState& pose, const BallState& ball, float headingDeg,
                           const DefenseFieldTarget& defendedGoal);
  DefensePlanDebug debugPlanDefense(const PoseState& pose, const BallState& ball,
                                    float headingDeg,
                                    const DefenseFieldTarget& defendedGoal);
  uint64_t nextTrajId();

 private:
  AStar3D astar_;
  HermiteSpline spline_;
  VelocityProfile profiler_;
  uint64_t trajId_ = 0;
};

}  // namespace ballalgo

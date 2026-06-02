#pragma once

#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "motion/AStar3D.hpp"
#include "motion/HermiteSpline.hpp"
#include "motion/VelocityProfile.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

struct PlannedChunk {
  uint64_t trajectoryId;
  uint64_t startTimePi;
  uint16_t dtMs;
  std::vector<MotionAction> actions;
};

struct CommandedPoseGoal {
  float xMm = 0;
  float yMm = 0;
  float headingDeg = 0;
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
};

class MotionPlanner {
 public:
  MotionPlanner();
  PlannedChunk plan(const PoseState& pose, const BallState& ball, float goalDeg,
                    float headingDeg, bool fullPlanner);
  PlannedChunk planToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                          float headingDeg);
  CommandedPosePlanDebug debugPlanToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                                         float headingDeg);
  uint64_t nextTrajId();

 private:
  AStar3D astar_;
  HermiteSpline spline_;
  VelocityProfile profiler_;
  uint64_t trajId_ = 0;
};

}  // namespace ballalgo

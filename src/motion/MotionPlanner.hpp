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

class MotionPlanner {
 public:
  MotionPlanner();
  PlannedChunk plan(const PoseState& pose, const BallState& ball, float goalDeg,
                    float headingDeg, bool fullPlanner);
  PlannedChunk planToPose(const PoseState& pose, const CommandedPoseGoal& goal,
                          float headingDeg);
  uint64_t nextTrajId();

 private:
  AStar3D astar_;
  HermiteSpline spline_;
  VelocityProfile profiler_;
  uint64_t trajId_ = 0;
};

}  // namespace ballalgo

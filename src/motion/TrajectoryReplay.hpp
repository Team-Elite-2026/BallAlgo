#pragma once

#include "estimation/PoseKalman.hpp"
#include "motion/HermiteSpline.hpp"
#include "motion/MotionPlanner.hpp"
#include "motion/VelocityProfile.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ballalgo {

struct CircularObstacle {
  bool enabled = false;
  float xMm = 0;
  float yMm = 0;
  float clearMm = 0;
};

struct ReplayChunk {
  uint64_t trajectoryId = 0;
  uint32_t startDelayMs = 0;
  uint16_t dtMs = 0;
  uint8_t kick = 0;
  uint8_t dribblerPower = 0;
  std::vector<MotionAction> actions;
};

struct TrajectoryReplayArtifact {
  std::string caseName;
  std::string caseKind;
  PoseState startPose;
  float startHeadingDeg = 0;
  bool hasGoalPose = false;
  CommandedPoseGoal goalPose;
  CircularObstacle obstacle;
  std::vector<PathSample> path;
  std::vector<TrajectorySpeedSample> trajectorySpeedProfile;
  std::vector<ReplayChunk> chunks;
};

struct TrajectoryTargetSample {
  bool valid = false;
  bool active = false;
  bool graceHold = false;
  bool stopChunk = false;
  bool scheduled = false;
  uint64_t trajectoryId = 0;
  uint64_t startTimePiUs = 0;
  uint16_t dtMs = 0;
  uint16_t numActions = 0;
  uint16_t actionIndex = 0;
  float progress01 = 0;
  uint8_t kick = 0;
  uint8_t dribblerPower = 0;
  MotionAction globalAction{};
  float vxBodyTargetMps = 0;
  float vyBodyTargetMps = 0;
  float axBodyTargetMps2 = 0;
  float ayBodyTargetMps2 = 0;
};

bool writeTrajectoryReplayArtifact(const std::string& path, const TrajectoryReplayArtifact& artifact,
                                   std::string* error = nullptr);
bool loadTrajectoryReplayArtifact(const std::string& path, TrajectoryReplayArtifact& artifact,
                                  std::string* error = nullptr);

TrajectoryTargetSample sampleTrajectoryTarget(const ReplayChunk& chunk, uint64_t startTimePiUs,
                                              uint64_t queryPiUs, float headingDeg);
TrajectoryTargetSample sampleTrajectoryTarget(const std::vector<MotionAction>& actions,
                                              uint64_t trajectoryId, uint64_t startTimePiUs,
                                              uint16_t dtMs, float headingDeg, uint64_t queryPiUs,
                                              uint8_t kick = 0, uint8_t dribblerPower = 0);

}  // namespace ballalgo

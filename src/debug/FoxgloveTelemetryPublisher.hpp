#pragma once

#include "debug/FoxgloveConfig.hpp"
#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "motion/ActionChunkPublisher.hpp"

#include <cstdint>
#include <string>

namespace ballalgo {

struct FoxgloveTelemetryFrame {
  double dtS = 0.0;
  unsigned long loopCount = 0;
  unsigned long frameGrabFailures = 0;
  float headingDeg = 0;
  PoseState pose;
  BallState ball;
  double visionBallAngleDeg = -5.0;
  double visionBallDistance = -5.0;
  PlannerDebugSnapshot planner;
};

class FoxgloveTelemetryPublisher {
 public:
  explicit FoxgloveTelemetryPublisher(const std::string& configPath);
  ~FoxgloveTelemetryPublisher();

  bool enabled() const { return config_.enabled && socketFd_ >= 0; }
  const FoxgloveConfig& config() const { return config_; }
  void publish(const FoxgloveTelemetryFrame& frame);

 private:
  bool openSocket();
  bool due(double hz, uint64_t nowNs, uint64_t& lastNs) const;
  void sendDatagram(const std::string& payload);

  FoxgloveConfig config_;
  int socketFd_ = -1;
  std::string socketPath_;

  uint64_t lastPoseNs_ = 0;
  uint64_t lastBallNs_ = 0;
  uint64_t lastVelocityNs_ = 0;
  uint64_t lastPathNs_ = 0;
  uint64_t lastLogNs_ = 0;

  bool havePrevRobotVel_ = false;
  float prevRobotVxBody_ = 0;
  float prevRobotVyBody_ = 0;
  float prevRobotVxField_ = 0;
  float prevRobotVyField_ = 0;

  bool havePrevHeading_ = false;
  float prevHeadingDeg_ = 0;
  float prevOmegaDegS_ = 0;

  uint64_t lastPathTrajectoryId_ = 0;
};

}  // namespace ballalgo

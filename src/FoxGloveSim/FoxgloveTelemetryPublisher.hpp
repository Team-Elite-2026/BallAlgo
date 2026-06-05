#pragma once

#include "FoxGloveSim/FoxgloveConfig.hpp"
#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "motion/ActionChunkPublisher.hpp"

#include <cstdint>
#include <string>
#include <vector>

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

  // Camera — populated in main.cpp only when streamCamera is enabled.
  std::vector<uint8_t> cameraJpegBytes;
  bool ballPxFound = false;
  int ballPxCx = 0;
  int ballPxCy = 0;
};

class FoxgloveTelemetryPublisher {
 public:
  explicit FoxgloveTelemetryPublisher(const std::string& configPath);
  ~FoxgloveTelemetryPublisher();

  // Returns true when configured enabled AND currently connected to the sidecar.
  bool connected() const { return config_.enabled && socketFd_ >= 0; }
  const FoxgloveConfig& config() const { return config_; }
  void publish(const FoxgloveTelemetryFrame& frame);

 private:
  bool connectSocket();
  void closeSocket();
  bool due(double hz, uint64_t nowNs, uint64_t& lastNs) const;
  void sendFrame(const std::string& payload);
  void noteSocketError(const std::string& error);
  void maybeReportSocketHealth(uint64_t nowNs);

  FoxgloveConfig config_;
  int socketFd_ = -1;
  std::string socketPath_;
  std::string lastSocketError_;
  bool reportedConnected_ = false;
  uint64_t lastSocketReportNs_ = 0;
  uint64_t framesSent_ = 0;

  uint64_t lastPoseNs_ = 0;
  uint64_t lastBallNs_ = 0;
  uint64_t lastVelocityNs_ = 0;
  uint64_t lastPathNs_ = 0;
  uint64_t lastLogNs_ = 0;
  uint64_t lastCameraNs_ = 0;

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

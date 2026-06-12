#pragma once

#include <string>

namespace ballalgo {

struct FoxgloveConfig {
  bool enabled = true;
  bool websocketEnabled = true;
  bool recordMcap = true;
  bool streamPaths = true;
  bool streamPose = true;
  bool streamBall = true;
  bool streamVelocity = true;
  bool streamTrajectory = true;
  bool streamLidar = false;
  bool streamCamera = false;
  bool streamLogs = true;

  double pathsHz = 15.0;
  double poseHz = 30.0;
  double ballHz = 30.0;
  double velocityHz = 30.0;
  double trajectoryHz = 30.0;
  double logsHz = 1.0;
  double cameraHz = 10.0;

  std::string socketPath = "/tmp/ballalgo_foxglove.sock";
  std::string websocketHost = "0.0.0.0";
  int websocketPort = 8765;
  std::string recordDir = "foxglove_sim/recordings";

  static FoxgloveConfig loadFromFile(const std::string& path);
};

}  // namespace ballalgo

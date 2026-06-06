#include "FoxGloveSim/FoxgloveConfig.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <string>

namespace ballalgo {

namespace {

std::string trim(std::string value) {
  auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
  value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
  return value;
}

bool parseBool(const std::string& value, bool& out) {
  const std::string lowered = [&]() {
    std::string copy = value;
    std::transform(copy.begin(), copy.end(), copy.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    return copy;
  }();
  if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on") {
    out = true;
    return true;
  }
  if (lowered == "0" || lowered == "false" || lowered == "no" || lowered == "off") {
    out = false;
    return true;
  }
  return false;
}

bool parseDouble(const std::string& value, double& out) {
  char* end = nullptr;
  out = std::strtod(value.c_str(), &end);
  return end != value.c_str() && end != nullptr && *end == '\0';
}

bool parseInt(const std::string& value, int& out) {
  char* end = nullptr;
  long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || end == nullptr || *end != '\0') return false;
  out = static_cast<int>(parsed);
  return true;
}

}  // namespace

FoxgloveConfig FoxgloveConfig::loadFromFile(const std::string& path) {
  FoxgloveConfig cfg;
  std::ifstream input(path);
  if (!input) return cfg;

  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    line = trim(line);
    if (line.empty()) continue;

    const auto eq = line.find('=');
    if (eq == std::string::npos) continue;

    const std::string key = trim(line.substr(0, eq));
    const std::string value = trim(line.substr(eq + 1));

    if (key == "enabled") {
      parseBool(value, cfg.enabled);
    } else if (key == "websocket_enabled") {
      parseBool(value, cfg.websocketEnabled);
    } else if (key == "record_mcap") {
      parseBool(value, cfg.recordMcap);
    } else if (key == "stream_paths") {
      parseBool(value, cfg.streamPaths);
    } else if (key == "stream_pose") {
      parseBool(value, cfg.streamPose);
    } else if (key == "stream_ball") {
      parseBool(value, cfg.streamBall);
    } else if (key == "stream_velocity") {
      parseBool(value, cfg.streamVelocity);
    } else if (key == "stream_lidar") {
      parseBool(value, cfg.streamLidar);
    } else if (key == "stream_camera") {
      parseBool(value, cfg.streamCamera);
    } else if (key == "stream_logs") {
      parseBool(value, cfg.streamLogs);
    } else if (key == "paths_hz") {
      parseDouble(value, cfg.pathsHz);
    } else if (key == "pose_hz") {
      parseDouble(value, cfg.poseHz);
    } else if (key == "ball_hz") {
      parseDouble(value, cfg.ballHz);
    } else if (key == "velocity_hz") {
      parseDouble(value, cfg.velocityHz);
    } else if (key == "logs_hz") {
      parseDouble(value, cfg.logsHz);
    } else if (key == "socket_path") {
      cfg.socketPath = value;
    } else if (key == "websocket_host") {
      cfg.websocketHost = value;
    } else if (key == "websocket_port") {
      parseInt(value, cfg.websocketPort);
    } else if (key == "record_dir") {
      cfg.recordDir = value;
    } else if (key == "camera_hz") {
      parseDouble(value, cfg.cameraHz);
    }
  }

  return cfg;
}

}  // namespace ballalgo

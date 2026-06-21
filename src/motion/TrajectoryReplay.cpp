#include "motion/TrajectoryReplay.hpp"

#include "vision/VisionMath.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ballalgo {

namespace {

constexpr const char* kReplayMagic = "BALLALGO_TRAJECTORY_REPLAY";
constexpr int kReplayVersion = 1;
constexpr uint64_t kGraceHoldUs = 20'000;

void setError(std::string* error, const std::string& message) {
  if (error != nullptr) *error = message;
}

bool parseBoolToken(const std::string& token, bool& value) {
  if (token == "0" || token == "false") {
    value = false;
    return true;
  }
  if (token == "1" || token == "true") {
    value = true;
    return true;
  }
  return false;
}

bool parseUint32Token(const std::string& token, uint32_t& value) {
  try {
    value = static_cast<uint32_t>(std::stoul(token));
    return true;
  } catch (...) {
    return false;
  }
}

bool parseUint8Token(const std::string& token, uint8_t& value) {
  try {
    value = static_cast<uint8_t>(std::stoul(token));
    return true;
  } catch (...) {
    return false;
  }
}

void rotateFieldVectorToBody(float xField, float yField, float headingDeg, float& xBody,
                             float& yBody) {
  // Heading is clockwise-positive (0 deg = field +y = robot forward), matching the
  // Teensy executor which rotates with theta = -CompassSensor offset. Negate here
  // so the body-frame telemetry stays in the same frame the robot actually drives.
  const float headingRad = -headingDeg * static_cast<float>(M_PI / 180.0);
  const float c = std::cos(headingRad);
  const float s = std::sin(headingRad);
  xBody = c * xField + s * yField;
  yBody = -s * xField + c * yField;
}

TrajectoryTargetSample sampleTrajectoryTargetImpl(const std::vector<MotionAction>& actions,
                                                  uint64_t trajectoryId, uint64_t startTimePiUs,
                                                  uint16_t dtMs, float headingDeg,
                                                  uint64_t queryPiUs, uint8_t kick,
                                                  uint8_t dribblerPower) {
  TrajectoryTargetSample sample;
  sample.trajectoryId = trajectoryId;
  sample.startTimePiUs = startTimePiUs;
  sample.dtMs = dtMs;
  sample.numActions = static_cast<uint16_t>(actions.size());
  sample.kick = kick;
  sample.dribblerPower = dribblerPower;
  sample.stopChunk = actions.empty() || dtMs == 0;
  sample.scheduled = queryPiUs < startTimePiUs;

  if (sample.scheduled) return sample;

  if (sample.stopChunk) {
    sample.valid = true;
    sample.progress01 = 1.0f;
    return sample;
  }

  const uint64_t dtUs = static_cast<uint64_t>(dtMs) * 1000u;
  if (dtUs == 0) return sample;

  const uint64_t elapsedUs = queryPiUs - startTimePiUs;
  uint32_t actionIndex = static_cast<uint32_t>(elapsedUs / dtUs);
  if (actionIndex >= actions.size()) {
    const uint32_t graceSlots = std::max<uint32_t>(1u, static_cast<uint32_t>(kGraceHoldUs / dtUs));
    const uint32_t graceLimit = static_cast<uint32_t>(actions.size()) + graceSlots;
    if (actionIndex >= graceLimit) return sample;
    actionIndex = static_cast<uint32_t>(actions.size() - 1);
    sample.graceHold = true;
  } else {
    sample.active = true;
  }

  sample.valid = true;
  sample.actionIndex = static_cast<uint16_t>(actionIndex);
  sample.globalAction = actions[actionIndex];
  if (sample.graceHold) {
    sample.globalAction.ax = 0.0f;
    sample.globalAction.ay = 0.0f;
    sample.globalAction.alpha = 0.0f;
  }
  sample.progress01 =
      actions.size() <= 1 ? 1.0f
                          : static_cast<float>(actionIndex) /
                                static_cast<float>(std::max<size_t>(1, actions.size() - 1));

  rotateFieldVectorToBody(sample.globalAction.vx, sample.globalAction.vy, headingDeg,
                          sample.vxBodyTargetMps, sample.vyBodyTargetMps);
  rotateFieldVectorToBody(sample.globalAction.ax, sample.globalAction.ay, headingDeg,
                          sample.axBodyTargetMps2, sample.ayBodyTargetMps2);
  return sample;
}

}  // namespace

bool writeTrajectoryReplayArtifact(const std::string& path, const TrajectoryReplayArtifact& artifact,
                                   std::string* error) {
  std::ofstream out(path);
  if (!out) {
    setError(error, "failed to open artifact for writing: " + path);
    return false;
  }

  out.setf(std::ios::fixed);
  out.precision(6);

  out << kReplayMagic << ' ' << kReplayVersion << '\n';
  out << "case_name " << artifact.caseName << '\n';
  out << "case_kind " << artifact.caseKind << '\n';
  out << "start_pose " << (artifact.startPose.valid ? 1 : 0) << ' ' << artifact.startPose.xMm
      << ' ' << artifact.startPose.yMm << ' ' << artifact.startPose.vxMmS << ' '
      << artifact.startPose.vyMmS << ' ' << artifact.startPose.vxBody << ' '
      << artifact.startPose.vyBody << '\n';
  out << "start_heading_deg " << artifact.startHeadingDeg << '\n';
  out << "goal_pose " << (artifact.hasGoalPose ? 1 : 0) << ' ' << artifact.goalPose.xMm << ' '
      << artifact.goalPose.yMm << ' ' << artifact.goalPose.headingDeg << '\n';
  out << "obstacle " << (artifact.obstacle.enabled ? 1 : 0) << ' ' << artifact.obstacle.xMm
      << ' ' << artifact.obstacle.yMm << ' ' << artifact.obstacle.clearMm << '\n';

  out << "path_count " << artifact.path.size() << '\n';
  for (const auto& sample : artifact.path) {
    out << "path " << sample.xMm << ' ' << sample.yMm << ' ' << sample.thetaDeg << ' '
        << sample.sMm << '\n';
  }

  out << "profile_count " << artifact.trajectorySpeedProfile.size() << '\n';
  for (const auto& sample : artifact.trajectorySpeedProfile) {
    out << "profile " << sample.progress01 << ' ' << sample.speedMps << '\n';
  }

  out << "chunk_count " << artifact.chunks.size() << '\n';
  for (const auto& chunk : artifact.chunks) {
    out << "chunk " << chunk.trajectoryId << ' ' << chunk.startDelayMs << ' ' << chunk.dtMs
        << ' ' << static_cast<int>(chunk.kick) << ' ' << static_cast<int>(chunk.dribblerPower)
        << ' ' << chunk.actions.size() << '\n';
    for (const auto& action : chunk.actions) {
      out << "action " << action.vx << ' ' << action.vy << ' ' << action.omega << ' '
          << action.ax << ' ' << action.ay << ' ' << action.alpha << '\n';
    }
    out << "endchunk\n";
  }

  if (!out.good()) {
    setError(error, "failed while writing artifact: " + path);
    return false;
  }
  return true;
}

bool loadTrajectoryReplayArtifact(const std::string& path, TrajectoryReplayArtifact& artifact,
                                  std::string* error) {
  std::ifstream in(path);
  if (!in) {
    setError(error, "failed to open artifact for reading: " + path);
    return false;
  }

  std::string line;
  if (!std::getline(in, line)) {
    setError(error, "artifact is empty: " + path);
    return false;
  }

  {
    std::istringstream header(line);
    std::string magic;
    int version = 0;
    if (!(header >> magic >> version) || magic != kReplayMagic || version != kReplayVersion) {
      setError(error, "artifact header is invalid: " + path);
      return false;
    }
  }

  artifact = {};
  size_t expectedPathCount = 0;
  size_t expectedProfileCount = 0;
  size_t expectedChunkCount = 0;
  size_t pathCount = 0;
  size_t profileCount = 0;
  size_t chunkCount = 0;
  ReplayChunk* currentChunk = nullptr;

  auto parseExpectedCount = [&](std::istringstream& iss, size_t& outCount,
                                const char* label) -> bool {
    std::string token;
    if (!(iss >> token)) {
      setError(error, std::string("missing count for ") + label);
      return false;
    }
    try {
      outCount = static_cast<size_t>(std::stoul(token));
      return true;
    } catch (...) {
      setError(error, std::string("invalid count for ") + label);
      return false;
    }
  };

  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    std::string tag;
    if (!(iss >> tag)) continue;

    if (tag == "case_name") {
      if (!(iss >> artifact.caseName)) {
        setError(error, "missing case_name value");
        return false;
      }
      continue;
    }
    if (tag == "case_kind") {
      if (!(iss >> artifact.caseKind)) {
        setError(error, "missing case_kind value");
        return false;
      }
      continue;
    }
    if (tag == "start_pose") {
      std::string validToken;
      if (!(iss >> validToken) || !parseBoolToken(validToken, artifact.startPose.valid) ||
          !(iss >> artifact.startPose.xMm >> artifact.startPose.yMm >> artifact.startPose.vxMmS >>
            artifact.startPose.vyMmS >> artifact.startPose.vxBody >>
            artifact.startPose.vyBody)) {
        setError(error, "invalid start_pose line");
        return false;
      }
      continue;
    }
    if (tag == "start_heading_deg") {
      if (!(iss >> artifact.startHeadingDeg)) {
        setError(error, "invalid start_heading_deg line");
        return false;
      }
      continue;
    }
    if (tag == "goal_pose") {
      std::string enabledToken;
      if (!(iss >> enabledToken) || !parseBoolToken(enabledToken, artifact.hasGoalPose) ||
          !(iss >> artifact.goalPose.xMm >> artifact.goalPose.yMm >> artifact.goalPose.headingDeg)) {
        setError(error, "invalid goal_pose line");
        return false;
      }
      continue;
    }
    if (tag == "obstacle") {
      std::string enabledToken;
      if (!(iss >> enabledToken) || !parseBoolToken(enabledToken, artifact.obstacle.enabled) ||
          !(iss >> artifact.obstacle.xMm >> artifact.obstacle.yMm >> artifact.obstacle.clearMm)) {
        setError(error, "invalid obstacle line");
        return false;
      }
      continue;
    }
    if (tag == "path_count") {
      if (!parseExpectedCount(iss, expectedPathCount, "path_count")) return false;
      artifact.path.clear();
      artifact.path.reserve(expectedPathCount);
      continue;
    }
    if (tag == "path") {
      PathSample sample{};
      if (!(iss >> sample.xMm >> sample.yMm >> sample.thetaDeg >> sample.sMm)) {
        setError(error, "invalid path line");
        return false;
      }
      artifact.path.push_back(sample);
      ++pathCount;
      continue;
    }
    if (tag == "profile_count") {
      if (!parseExpectedCount(iss, expectedProfileCount, "profile_count")) return false;
      artifact.trajectorySpeedProfile.clear();
      artifact.trajectorySpeedProfile.reserve(expectedProfileCount);
      continue;
    }
    if (tag == "profile") {
      TrajectorySpeedSample sample{};
      if (!(iss >> sample.progress01 >> sample.speedMps)) {
        setError(error, "invalid profile line");
        return false;
      }
      artifact.trajectorySpeedProfile.push_back(sample);
      ++profileCount;
      continue;
    }
    if (tag == "chunk_count") {
      if (!parseExpectedCount(iss, expectedChunkCount, "chunk_count")) return false;
      artifact.chunks.clear();
      artifact.chunks.reserve(expectedChunkCount);
      continue;
    }
    if (tag == "chunk") {
      ReplayChunk chunk{};
      std::string kickToken;
      std::string dribblerToken;
      std::string numActionsToken;
      if (!(iss >> chunk.trajectoryId >> chunk.startDelayMs >> chunk.dtMs >> kickToken >>
            dribblerToken >> numActionsToken) ||
          !parseUint8Token(kickToken, chunk.kick) ||
          !parseUint8Token(dribblerToken, chunk.dribblerPower)) {
        setError(error, "invalid chunk line");
        return false;
      }
      uint32_t numActions = 0;
      if (!parseUint32Token(numActionsToken, numActions)) {
        setError(error, "invalid chunk action count");
        return false;
      }
      chunk.actions.reserve(numActions);
      artifact.chunks.push_back(std::move(chunk));
      currentChunk = &artifact.chunks.back();
      ++chunkCount;
      continue;
    }
    if (tag == "action") {
      if (currentChunk == nullptr) {
        setError(error, "action line appeared outside a chunk");
        return false;
      }
      MotionAction action{};
      if (!(iss >> action.vx >> action.vy >> action.omega >> action.ax >> action.ay >>
            action.alpha)) {
        setError(error, "invalid action line");
        return false;
      }
      currentChunk->actions.push_back(action);
      continue;
    }
    if (tag == "endchunk") {
      currentChunk = nullptr;
      continue;
    }

    setError(error, "unknown artifact line tag: " + tag);
    return false;
  }

  if (currentChunk != nullptr) {
    setError(error, "artifact ended before endchunk");
    return false;
  }
  if (pathCount != expectedPathCount) {
    setError(error, "path_count did not match the number of path lines");
    return false;
  }
  if (profileCount != expectedProfileCount) {
    setError(error, "profile_count did not match the number of profile lines");
    return false;
  }
  if (chunkCount != expectedChunkCount) {
    setError(error, "chunk_count did not match the number of chunk blocks");
    return false;
  }

  std::sort(artifact.chunks.begin(), artifact.chunks.end(),
            [](const ReplayChunk& a, const ReplayChunk& b) {
              if (a.startDelayMs != b.startDelayMs) return a.startDelayMs < b.startDelayMs;
              return a.trajectoryId < b.trajectoryId;
            });
  return true;
}

TrajectoryTargetSample sampleTrajectoryTarget(const ReplayChunk& chunk, uint64_t startTimePiUs,
                                              uint64_t queryPiUs, float headingDeg) {
  return sampleTrajectoryTargetImpl(chunk.actions, chunk.trajectoryId, startTimePiUs, chunk.dtMs,
                                    headingDeg, queryPiUs, chunk.kick, chunk.dribblerPower);
}

TrajectoryTargetSample sampleTrajectoryTarget(const std::vector<MotionAction>& actions,
                                              uint64_t trajectoryId, uint64_t startTimePiUs,
                                              uint16_t dtMs, float headingDeg, uint64_t queryPiUs,
                                              uint8_t kick, uint8_t dribblerPower) {
  return sampleTrajectoryTargetImpl(actions, trajectoryId, startTimePiUs, dtMs, headingDeg,
                                    queryPiUs, kick, dribblerPower);
}

}  // namespace ballalgo

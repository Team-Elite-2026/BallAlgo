#include "config.hpp"
#include "motion/MotionPlanner.hpp"
#include "motion/TrajectoryReplay.hpp"
#include "vision/VisionMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ballalgo;

namespace {

struct PlannerCaseOptions {
  bool generate = false;
  bool inspect = false;
  std::string caseName = "trajectory_case";
  std::string outputPath;
  std::string inspectPath;
  PoseState startPose;
  float startHeadingDeg = 0;
  CommandedPoseGoal goalPose;
  bool goalProvided = false;
};

bool parseFloatArg(const char* text, float& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

bool parseArgs(int argc, char** argv, PlannerCaseOptions& options) {
  options.startPose.valid = true;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--case-name") {
      if (i + 1 >= argc) return false;
      options.caseName = argv[++i];
      continue;
    }
    if (arg == "--output") {
      if (i + 1 >= argc) return false;
      options.outputPath = argv[++i];
      options.generate = true;
      continue;
    }
    if (arg == "--inspect-artifact") {
      if (i + 1 >= argc) return false;
      options.inspectPath = argv[++i];
      options.inspect = true;
      continue;
    }
    if (arg == "--start") {
      if (i + 3 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.startPose.xMm) ||
          !parseFloatArg(argv[i + 2], options.startPose.yMm) ||
          !parseFloatArg(argv[i + 3], options.startHeadingDeg)) {
        return false;
      }
      i += 3;
      continue;
    }
    if (arg == "--start-velocity-field") {
      if (i + 2 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.startPose.vxMmS) ||
          !parseFloatArg(argv[i + 2], options.startPose.vyMmS)) {
        return false;
      }
      i += 2;
      continue;
    }
    if (arg == "--goal") {
      if (i + 3 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.goalPose.xMm) ||
          !parseFloatArg(argv[i + 2], options.goalPose.yMm) ||
          !parseFloatArg(argv[i + 3], options.goalPose.headingDeg)) {
        return false;
      }
      options.goalProvided = true;
      i += 3;
      continue;
    }
    if (arg == "--start-invalid") {
      options.startPose.valid = false;
      continue;
    }
    return false;
  }
  return (options.generate && options.goalProvided && !options.outputPath.empty()) ||
         (options.inspect && !options.inspectPath.empty());
}

void printUsage(const char* argv0) {
  std::cerr
      << "Usage:\n"
      << "  " << argv0
      << " --case-name name --output path.traj --start x_mm y_mm heading_deg"
      << " --goal x_mm y_mm heading_deg [--start-velocity-field vx_mm_s vy_mm_s]\n"
      << "  " << argv0 << " --inspect-artifact path.traj\n";
}

void fillStartBodyVelocity(PoseState& pose, float headingDeg) {
  fieldVelToBody(pose.vxMmS, pose.vyMmS, headingDeg, pose.vxBody, pose.vyBody);
}

int inspectArtifact(const std::string& path) {
  TrajectoryReplayArtifact artifact;
  std::string error;
  if (!loadTrajectoryReplayArtifact(path, artifact, &error)) {
    std::cerr << error << "\n";
    return 1;
  }

  std::cout << "case_name=" << artifact.caseName << "\n";
  std::cout << "case_kind=" << artifact.caseKind << "\n";
  std::cout << "start_pose=(" << artifact.startPose.xMm << ", " << artifact.startPose.yMm
            << ") heading=" << artifact.startHeadingDeg << " valid="
            << (artifact.startPose.valid ? "true" : "false") << "\n";
  if (artifact.hasGoalPose) {
    std::cout << "goal_pose=(" << artifact.goalPose.xMm << ", " << artifact.goalPose.yMm
              << ") heading=" << artifact.goalPose.headingDeg << "\n";
  }
  std::cout << "path_samples=" << artifact.path.size() << " profile_samples="
            << artifact.trajectorySpeedProfile.size() << " chunks=" << artifact.chunks.size()
            << "\n";
  for (const auto& chunk : artifact.chunks) {
    std::cout << "chunk traj=" << chunk.trajectoryId << " start_delay_ms=" << chunk.startDelayMs
              << " dt_ms=" << chunk.dtMs << " actions=" << chunk.actions.size() << "\n";
    const uint64_t absoluteStart = 1000000 + static_cast<uint64_t>(chunk.startDelayMs) * 1000u;
    const uint64_t lastSampleTime =
        absoluteStart + static_cast<uint64_t>(std::max<size_t>(1, chunk.actions.size())) *
                            static_cast<uint64_t>(std::max<uint16_t>(1, chunk.dtMs)) * 1000u;
    const uint64_t sampleStepUs =
        static_cast<uint64_t>(std::max<uint16_t>(20, chunk.dtMs * 5)) * 1000u;
    for (uint64_t query = absoluteStart; query <= lastSampleTime; query += sampleStepUs) {
      const auto sample = sampleTrajectoryTarget(chunk, absoluteStart, query,
                                                 artifact.startHeadingDeg);
      if (!sample.valid) continue;
      std::cout << "  sample t_ms=" << (query - absoluteStart) / 1000
                << " idx=" << sample.actionIndex << " vx=" << sample.globalAction.vx
                << " vy=" << sample.globalAction.vy << " omega=" << sample.globalAction.omega
                << " grace=" << (sample.graceHold ? "true" : "false") << "\n";
    }
  }
  return 0;
}

int generatePlannerCase(const PlannerCaseOptions& options) {
  MotionPlanner planner;
  PoseState startPose = options.startPose;
  fillStartBodyVelocity(startPose, options.startHeadingDeg);

  const auto debug = planner.debugPlanToPose(startPose, options.goalPose, options.startHeadingDeg);
  TrajectoryReplayArtifact artifact;
  artifact.caseName = options.caseName;
  artifact.caseKind = "planner_goal";
  artifact.startPose = startPose;
  artifact.startHeadingDeg = options.startHeadingDeg;
  artifact.hasGoalPose = true;
  artifact.goalPose = options.goalPose;
  artifact.path = debug.path;
  artifact.trajectorySpeedProfile = debug.trajectorySpeedProfile;

  ReplayChunk chunk;
  chunk.trajectoryId = debug.chunk.trajectoryId;
  chunk.startDelayMs = 0;
  chunk.dtMs = debug.chunk.dtMs;
  chunk.actions = debug.chunk.actions;
  artifact.chunks.push_back(std::move(chunk));

  std::string error;
  if (!writeTrajectoryReplayArtifact(options.outputPath, artifact, &error)) {
    std::cerr << error << "\n";
    return 1;
  }

  std::cout << "wrote " << options.outputPath << "\n";
  std::cout << "trajectory_id=" << artifact.chunks.front().trajectoryId << " actions="
            << artifact.chunks.front().actions.size() << " dt_ms="
            << artifact.chunks.front().dtMs << " path_samples=" << artifact.path.size()
            << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  PlannerCaseOptions options;
  if (!parseArgs(argc, argv, options)) {
    printUsage(argv[0]);
    return 2;
  }
  if (options.inspect) return inspectArtifact(options.inspectPath);
  return generatePlannerCase(options);
}

#pragma once

#include "sim/SimulationTypes.hpp"

#include <filesystem>

namespace ballalgo::sim {

void writeArtifact(const std::filesystem::path& outputPath, const InputOptions& options,
                   const PoseState& startPose, const CommandedPoseGoal& poseGoal,
                   const FieldBallState& ballField, const GoalFieldTarget& goalTarget,
                   const SimulationResult& simulation);

}  // namespace ballalgo::sim

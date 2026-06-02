#pragma once

#include "sim/SimulationTypes.hpp"

namespace ballalgo::sim {

SimulationResult simulateSingleChunk(MotionPlanner& planner, const PoseState& startPose,
                                     double startHeadingDeg, const CommandedPoseGoal& goal);
SimulationResult simulatePoseTarget(MotionPlanner& planner, const PoseState& startPose,
                                    double startHeadingDeg, const CommandedPoseGoal& goal,
                                    double controlHz, int maxReplans, double maxSimTimeS);
SimulationResult simulateProductionBallPlan(MotionPlanner& planner, const PoseState& startPose,
                                            double startHeadingDeg, const FieldBallState& ballField,
                                            const GoalFieldTarget& goalTarget, double controlHz,
                                            int maxReplans, double maxSimTimeS);

}  // namespace ballalgo::sim

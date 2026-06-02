#pragma once

#include "sim/SimulationTypes.hpp"

#include <string>
#include <vector>

namespace ballalgo::sim {

float centeredCmToFieldMm(float centeredCm, float axisLimitMm);
double fieldMmToCenteredCm(double positionMm, double axisLimitMm);
double wrapAngleDeg(double angleDeg);
std::string escapeJson(const std::string& input);

void bodyVelocityToField(double vxBody, double vyBody, double headingDeg, double& vxField,
                         double& vyField);
void fieldPointToBodyMeters(double fieldXMm, double fieldYMm, const PoseState& pose,
                            double headingDeg, float& bodyXM, float& bodyYM);
void fieldVelocityToBodyMps(double vxMmS, double vyMmS, double headingDeg, float& vxBodyMps,
                            float& vyBodyMps);
double bodyAngleDegFromFieldPoint(double fieldXMm, double fieldYMm, const PoseState& pose,
                                  double headingDeg);

SimSample makeInitialSample(const PoseState& pose, double headingDeg);
SimSample makeTraceSample(const MotionAction& action, int globalStep, int replanIndex,
                          int localActionIndex, double timeS, double appliedDtS, double xMm,
                          double yMm, double headingDeg, double vxField, double vyField);
ExecutedActionSample makeExecutedActionSample(const SimSample& sample);

void applyActionKinematics(const MotionAction& action, double dtS, float& xMm, float& yMm,
                           double& headingDeg, double& vxField, double& vyField);
bool withinPoseGoalTolerance(double xMm, double yMm, double headingDeg,
                             const CommandedPoseGoal& goal);

PlannerInputSnapshot makeProductionPlannerInputs(const PoseState& pose, double headingDeg,
                                                 const FieldBallState& ballField,
                                                 const GoalFieldTarget& goalTarget);
int executeControlSlice(const std::vector<MotionAction>& actions, double actionDtS,
                        double controlIntervalS, int replanIndex, PoseState& currentPose,
                        double& currentHeadingDeg, double& currentTimeS,
                        std::vector<SimSample>& globalTrace,
                        std::vector<ExecutedActionSample>& executedActions,
                        std::vector<SimSample>& localTrace);

}  // namespace ballalgo::sim

#include "sim/SimulationMath.hpp"

#include "config.hpp"
#include "vision/VisionMath.hpp"

#include <cmath>
#include <sstream>

namespace ballalgo::sim {

const char* modeName(SimulationMode mode) {
  switch (mode) {
    case SimulationMode::ProductionBallPlan:
      return "production_ball_plan";
    case SimulationMode::PoseTarget:
      return "pose_target";
    case SimulationMode::SingleChunk:
      return "single_chunk";
  }
  return "unknown";
}

float centeredCmToFieldMm(float centeredCm, float axisLimitMm) {
  return centeredCm * 10.f + 0.5f * axisLimitMm;
}

double fieldMmToCenteredCm(double positionMm, double axisLimitMm) {
  return (positionMm - 0.5 * axisLimitMm) * 0.1;
}

double wrapAngleDeg(double angleDeg) {
  double wrapped = std::fmod(angleDeg + 180.0, 360.0);
  if (wrapped < 0) wrapped += 360.0;
  return wrapped - 180.0;
}

std::string escapeJson(const std::string& input) {
  std::ostringstream out;
  for (char ch : input) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

void bodyVelocityToField(double vxBody, double vyBody, double headingDeg, double& vxField,
                         double& vyField) {
  const double headingRad = headingDeg * M_PI / 180.0;
  const double c = std::cos(headingRad);
  const double s = std::sin(headingRad);
  vxField = c * vxBody - s * vyBody;
  vyField = s * vxBody + c * vyBody;
}

void fieldPointToBodyMeters(double fieldXMm, double fieldYMm, const PoseState& pose,
                            double headingDeg, float& bodyXM, float& bodyYM) {
  const double headingRad = headingDeg * M_PI / 180.0;
  const double c = std::cos(headingRad);
  const double s = std::sin(headingRad);
  const double dxMm = fieldXMm - pose.xMm;
  const double dyMm = fieldYMm - pose.yMm;
  bodyXM = static_cast<float>((c * dxMm + s * dyMm) / 1000.0);
  bodyYM = static_cast<float>((-s * dxMm + c * dyMm) / 1000.0);
}

void fieldVelocityToBodyMps(double vxMmS, double vyMmS, double headingDeg, float& vxBodyMps,
                            float& vyBodyMps) {
  fieldVelToBody(static_cast<float>(vxMmS), static_cast<float>(vyMmS),
                 static_cast<float>(headingDeg), vxBodyMps, vyBodyMps);
}

double bodyAngleDegFromFieldPoint(double fieldXMm, double fieldYMm, const PoseState& pose,
                                  double headingDeg) {
  float bodyXM = 0;
  float bodyYM = 0;
  fieldPointToBodyMeters(fieldXMm, fieldYMm, pose, headingDeg, bodyXM, bodyYM);
  return std::atan2(bodyYM, bodyXM) * 180.0 / M_PI;
}

SimSample makeInitialSample(const PoseState& pose, double headingDeg) {
  return {
      0,
      -1,
      -1,
      0.0,
      0.0,
      pose.xMm,
      pose.yMm,
      fieldMmToCenteredCm(pose.xMm, config::kFieldWidthMm),
      fieldMmToCenteredCm(pose.yMm, config::kFieldHeightMm),
      headingDeg,
      pose.vxBody,
      pose.vyBody,
      std::hypot(static_cast<double>(pose.vxBody), static_cast<double>(pose.vyBody)),
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      pose.vxMmS / 1000.0,
      pose.vyMmS / 1000.0,
      std::hypot(static_cast<double>(pose.vxMmS), static_cast<double>(pose.vyMmS)) / 1000.0,
  };
}

SimSample makeTraceSample(const MotionAction& action, int globalStep, int replanIndex,
                          int localActionIndex, double timeS, double appliedDtS, double xMm,
                          double yMm, double headingDeg, double vxField, double vyField) {
  return {
      globalStep,
      replanIndex,
      localActionIndex,
      timeS,
      appliedDtS,
      xMm,
      yMm,
      fieldMmToCenteredCm(xMm, config::kFieldWidthMm),
      fieldMmToCenteredCm(yMm, config::kFieldHeightMm),
      headingDeg,
      action.vx,
      action.vy,
      std::hypot(static_cast<double>(action.vx), static_cast<double>(action.vy)),
      action.ax,
      action.ay,
      std::hypot(static_cast<double>(action.ax), static_cast<double>(action.ay)),
      action.omega,
      action.alpha,
      vxField,
      vyField,
      std::hypot(vxField, vyField),
  };
}

ExecutedActionSample makeExecutedActionSample(const SimSample& sample) {
  return {
      sample.step - 1,
      sample.replanIndex,
      sample.localActionIndex,
      sample.timeS,
      sample.appliedDtS,
      sample.vxBody,
      sample.vyBody,
      sample.speedBody,
      sample.axBody,
      sample.ayBody,
      sample.accelBody,
      sample.omega,
      sample.alpha,
      sample.vxField,
      sample.vyField,
      sample.speedField,
  };
}

void applyActionKinematics(const MotionAction& action, double dtS, float& xMm, float& yMm,
                           double& headingDeg, double& vxField, double& vyField) {
  bodyVelocityToField(action.vx, action.vy, headingDeg, vxField, vyField);
  xMm += static_cast<float>(vxField * dtS * 1000.0);
  yMm += static_cast<float>(vyField * dtS * 1000.0);
  headingDeg += action.omega * dtS * 180.0 / M_PI;
}

bool withinPoseGoalTolerance(double xMm, double yMm, double headingDeg,
                             const CommandedPoseGoal& goal) {
  const double posErrMm = std::hypot(goal.xMm - xMm, goal.yMm - yMm);
  const double headingErrDeg = std::fabs(wrapAngleDeg(goal.headingDeg - headingDeg));
  return posErrMm <= config::kCommandGoalPositionToleranceMm &&
         headingErrDeg <= config::kCommandGoalHeadingToleranceDeg;
}

PlannerInputSnapshot makeProductionPlannerInputs(const PoseState& pose, double headingDeg,
                                                 const FieldBallState& ballField,
                                                 const GoalFieldTarget& goalTarget) {
  PlannerInputSnapshot snapshot;
  snapshot.pose = pose;
  snapshot.headingDeg = headingDeg;
  snapshot.ballField = ballField;
  snapshot.goalTargetField = goalTarget;
  snapshot.goalDeg = bodyAngleDegFromFieldPoint(goalTarget.xMm, goalTarget.yMm, pose, headingDeg);
  snapshot.ballBody.visible = true;
  fieldPointToBodyMeters(ballField.xMm, ballField.yMm, pose, headingDeg, snapshot.ballBody.xM,
                         snapshot.ballBody.yM);

  const double relVxMmS = ballField.vxMmS - pose.vxMmS;
  const double relVyMmS = ballField.vyMmS - pose.vyMmS;
  fieldVelocityToBodyMps(relVxMmS, relVyMmS, headingDeg, snapshot.ballBody.vx,
                         snapshot.ballBody.vy);
  return snapshot;
}

int executeControlSlice(const std::vector<MotionAction>& actions, double actionDtS,
                        double controlIntervalS, int replanIndex, PoseState& currentPose,
                        double& currentHeadingDeg, double& currentTimeS,
                        std::vector<SimSample>& globalTrace,
                        std::vector<ExecutedActionSample>& executedActions,
                        std::vector<SimSample>& localTrace) {
  double remainingS = controlIntervalS;
  int localActionIndex = 0;
  while (remainingS > 1e-9 && localActionIndex < static_cast<int>(actions.size())) {
    const MotionAction& action = actions[static_cast<size_t>(localActionIndex)];
    const double appliedDtS = std::min(actionDtS, remainingS);

    double vxField = 0.0;
    double vyField = 0.0;
    applyActionKinematics(action, appliedDtS, currentPose.xMm, currentPose.yMm, currentHeadingDeg,
                          vxField, vyField);

    currentTimeS += appliedDtS;
    currentPose.valid = true;
    currentPose.vxMmS = static_cast<float>(vxField * 1000.0);
    currentPose.vyMmS = static_cast<float>(vyField * 1000.0);
    currentPose.vxBody = action.vx;
    currentPose.vyBody = action.vy;

    SimSample sample =
        makeTraceSample(action, static_cast<int>(globalTrace.size()), replanIndex, localActionIndex,
                        currentTimeS, appliedDtS, currentPose.xMm, currentPose.yMm,
                        currentHeadingDeg, vxField, vyField);
    localTrace.push_back(sample);
    globalTrace.push_back(sample);
    executedActions.push_back(makeExecutedActionSample(sample));

    remainingS -= appliedDtS;
    ++localActionIndex;
  }
  return localActionIndex;
}

}  // namespace ballalgo::sim

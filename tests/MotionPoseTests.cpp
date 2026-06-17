#include "config.hpp"
#include "motion/DefensePose.hpp"
#include "motion/MotionPlanner.hpp"
#include "motion/OffensePose.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    std::exit(1);
  }
}

bool near(float a, float b, float epsilon = 1e-3f) {
  return std::fabs(a - b) <= epsilon;
}

ballalgo::PoseState makePose(float xMm, float yMm) {
  ballalgo::PoseState pose;
  pose.valid = true;
  pose.xMm = xMm;
  pose.yMm = yMm;
  return pose;
}

ballalgo::BallState makeBall(float xM, float yM) {
  ballalgo::BallState ball;
  ball.visible = true;
  ball.xM = xM;
  ball.yM = yM;
  return ball;
}

void testOffenseStraightAheadHeadingUsesFieldForwardConvention() {
  const ballalgo::PoseState pose =
      makePose(ballalgo::config::kFieldWidthMm * 0.5f, 1000.f);
  const ballalgo::BallState ball = makeBall(0.f, 0.3f);

  const ballalgo::OffensePoseResult result =
      ballalgo::computeOffensePose(pose, ball, 0.f, 0.f, true,
                                   ballalgo::config::kYellowGoalXMm,
                                   ballalgo::config::kYellowGoalYMm, 0.f);

  expect(result.valid, "offense pose should be valid");
  expect(result.state == ballalgo::OffensePoseState::NormalStrike,
         "straight-ahead offense case should stay in normal strike");
  expect(near(result.targetHeadingDeg, 0.f),
         "offense straight-ahead target heading should be 0 degrees");
}

void testOffenseGoalMouthUsesEnemyGoalLineState() {
  const ballalgo::PoseState pose =
      makePose(ballalgo::config::kYellowGoalXMm, 2230.f);
  const ballalgo::BallState ball = makeBall(0.f, 0.1f);

  const ballalgo::OffensePoseResult result =
      ballalgo::computeOffensePose(pose, ball, 0.f, 0.f, true,
                                   ballalgo::config::kYellowGoalXMm,
                                   ballalgo::config::kYellowGoalYMm, 0.f);

  expect(result.valid, "enemy goal-mouth offense pose should be valid");
  expect(result.state == ballalgo::OffensePoseState::CollectBallNearEnemyGoalLine,
         "enemy goal-mouth ball should use the near-enemy-goal-line state");
}

void testDefenseStraightAheadHeadingUsesFieldForwardConvention() {
  const ballalgo::PoseState pose =
      makePose(ballalgo::config::kBlueGoalXMm, 200.f);
  const ballalgo::BallState ball = makeBall(0.f, 0.6f);
  const ballalgo::DefenseFieldTarget defendedGoal{ballalgo::config::kBlueGoalXMm,
                                                  ballalgo::config::kBlueGoalYMm};

  const ballalgo::DefensePoseResult result =
      ballalgo::computeDefensePose(pose, ball, 0.f, defendedGoal, {}, 0.f);

  expect(result.valid, "defense pose should be valid");
  expect(near(result.targetHeadingDeg, 0.f),
         "defense straight-ahead target heading should be 0 degrees");
}

void testDefenseWideBallUsesSideLineGoalGeometry() {
  const ballalgo::PoseState pose =
      makePose(ballalgo::config::kBlueGoalXMm, ballalgo::config::kBlueGoalYMm);
  const ballalgo::BallState ball = makeBall(0.8f, 0.f);
  const ballalgo::DefenseFieldTarget defendedGoal{ballalgo::config::kBlueGoalXMm,
                                                  ballalgo::config::kBlueGoalYMm};

  const ballalgo::DefensePoseResult result =
      ballalgo::computeDefensePose(pose, ball, 0.f, defendedGoal, {}, 0.f);

  expect(result.valid, "wide defense pose should be valid");
  expect(near(result.targetXMm,
              defendedGoal.xMm + ballalgo::config::kDefenseGoalLineXMaxCm * 10.f),
         "wide defense target should land on the 55 cm side line");
  expect(near(result.targetYMm, defendedGoal.yMm),
         "wide defense target should stay level with the defended goal");
}

void testCommandedPosePlanKeepsExactStartAndGoalPose() {
  ballalgo::MotionPlanner planner;

  ballalgo::PoseState pose = makePose(913.f, 417.f);
  const float headingDeg = 17.f;
  ballalgo::CommandedPoseGoal goal;
  goal.xMm = 1262.f;
  goal.yMm = 1488.f;
  goal.headingDeg = 90.f;

  const ballalgo::CommandedPosePlanDebug debug =
      planner.debugPlanToPose(pose, goal, headingDeg);

  expect(!debug.path.empty(), "commanded pose plan should produce a path");
  expect(!debug.chunk.actions.empty(), "commanded pose plan should produce actions");
  expect(near(debug.path.front().xMm, pose.xMm),
         "planned path should start at the exact current x");
  expect(near(debug.path.front().yMm, pose.yMm),
         "planned path should start at the exact current y");
  expect(near(debug.path.front().thetaDeg, headingDeg),
         "planned path should start at the exact current heading");
  expect(near(debug.path.back().xMm, goal.xMm),
         "planned path should end at the exact commanded x");
  expect(near(debug.path.back().yMm, goal.yMm),
         "planned path should end at the exact commanded y");
  expect(near(debug.path.back().thetaDeg, goal.headingDeg),
         "planned path should end at the exact commanded heading");
}

void testCommandedPosePlanStillMovesWithinSameAstarCell() {
  ballalgo::MotionPlanner planner;

  ballalgo::PoseState pose = makePose(100.f, 100.f);
  const float headingDeg = 0.f;
  ballalgo::CommandedPoseGoal goal;
  goal.xMm = 149.f;
  goal.yMm = 149.f;
  goal.headingDeg = 44.f;

  const ballalgo::CommandedPosePlanDebug debug =
      planner.debugPlanToPose(pose, goal, headingDeg);

  expect(debug.posErrMm > ballalgo::config::kCommandGoalPositionToleranceMm,
         "repro case should begin outside the commanded-goal tolerance");
  expect(debug.path.size() >= 2,
         "same-cell commanded pose should still generate a non-degenerate path");
  expect(!debug.chunk.actions.empty(),
         "same-cell commanded pose should still generate an action chunk");
  bool hasMotion = false;
  for (const auto& action : debug.chunk.actions) {
    if (std::fabs(action.vx) > 1e-4f || std::fabs(action.vy) > 1e-4f ||
        std::fabs(action.omega) > 1e-4f) {
      hasMotion = true;
      break;
    }
  }
  expect(hasMotion, "same-cell commanded pose should not collapse into an all-stop chunk");
}

}  // namespace

int main() {
  testOffenseStraightAheadHeadingUsesFieldForwardConvention();
  testOffenseGoalMouthUsesEnemyGoalLineState();
  testDefenseStraightAheadHeadingUsesFieldForwardConvention();
  testDefenseWideBallUsesSideLineGoalGeometry();
  testCommandedPosePlanKeepsExactStartAndGoalPose();
  testCommandedPosePlanStillMovesWithinSameAstarCell();

  std::cout << "ballalgo_motion_tests passed\n";
  return 0;
}

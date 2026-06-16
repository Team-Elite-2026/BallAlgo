#include "config.hpp"
#include "motion/DefensePose.hpp"
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

}  // namespace

int main() {
  testOffenseStraightAheadHeadingUsesFieldForwardConvention();
  testOffenseGoalMouthUsesEnemyGoalLineState();
  testDefenseStraightAheadHeadingUsesFieldForwardConvention();
  testDefenseWideBallUsesSideLineGoalGeometry();

  std::cout << "ballalgo_motion_tests passed\n";
  return 0;
}

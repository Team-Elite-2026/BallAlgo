// Frame-convention guard for the trajectory replay body-frame projection.
//
// Convention (authoritative, see project coordinate notes + Teensy executor):
//   * heading 0 deg  = robot forward = field +y
//   * heading increases CLOCKWISE, so heading 90 deg => robot faces field +x
//   * body frame: +x = right, +y = forward
//   * sampleTrajectoryTarget() reports vxBodyTargetMps (right) / vyBodyTargetMps
//     (forward) by rotating the GLOBAL action velocity into the body frame.
//
// The Teensy executor rotates field->body with theta = -heading (CompassSensor
// offset is clockwise-positive). This test pins the Pi-side projection to the
// SAME convention so the debug telemetry frame cannot silently drift from the
// frame the robot actually drives in.

#include "motion/TrajectoryReplay.hpp"
#include "motion/VelocityProfile.hpp"

#include <cmath>
#include <iostream>

namespace {

int g_failures = 0;

void expectNear(float actual, float expected, const char* message, float epsilon = 1e-3f) {
  if (std::fabs(actual - expected) > epsilon) {
    std::cerr << "FAIL: " << message << " (expected " << expected << ", got " << actual << ")\n";
    ++g_failures;
  }
}

// Project a single global-frame velocity (vx,vy) into the body frame at the
// given heading, using the public replay sampler (which exercises the same
// rotateFieldVectorToBody used for /traj/target telemetry).
ballalgo::TrajectoryTargetSample bodyAt(float vxGlobal, float vyGlobal, float headingDeg) {
  std::vector<ballalgo::MotionAction> actions;
  ballalgo::MotionAction action{};
  action.vx = vxGlobal;
  action.vy = vyGlobal;
  actions.push_back(action);
  // trajId=1, startTimePiUs=0, dtMs=4, query at t=0 -> action index 0 (active).
  return ballalgo::sampleTrajectoryTarget(actions, /*trajectoryId=*/1, /*startTimePiUs=*/0,
                                          /*dtMs=*/4, headingDeg, /*queryPiUs=*/0);
}

void testHeadingZeroIsIdentity() {
  const auto s = bodyAt(0.f, 1.f, 0.f);
  expectNear(s.vxBodyTargetMps, 0.f, "heading 0: field +y -> body right");
  expectNear(s.vyBodyTargetMps, 1.f, "heading 0: field +y -> body forward");
}

void testHeading90FacesFieldPlusX() {
  // Robot rotated 90 deg CW now faces field +x.
  // A field +y command is to the robot's LEFT -> body right = -1, forward = 0.
  const auto fy = bodyAt(0.f, 1.f, 90.f);
  expectNear(fy.vxBodyTargetMps, -1.f, "heading 90: field +y -> body right (-1, left)");
  expectNear(fy.vyBodyTargetMps, 0.f, "heading 90: field +y -> body forward (0)");

  // A field +x command is straight ahead of the robot -> body forward = +1.
  const auto fx = bodyAt(1.f, 0.f, 90.f);
  expectNear(fx.vxBodyTargetMps, 0.f, "heading 90: field +x -> body right (0)");
  expectNear(fx.vyBodyTargetMps, 1.f, "heading 90: field +x -> body forward (+1)");
}

void testHeading45SplitsForward() {
  const float k = std::sqrt(0.5f);
  const auto s = bodyAt(0.f, 1.f, 45.f);
  expectNear(s.vxBodyTargetMps, -k, "heading 45: field +y -> body right (-sin45)");
  expectNear(s.vyBodyTargetMps, k, "heading 45: field +y -> body forward (cos45)");
}

}  // namespace

int main() {
  testHeadingZeroIsIdentity();
  testHeading90FacesFieldPlusX();
  testHeading45SplitsForward();

  if (g_failures != 0) {
    std::cerr << g_failures << " trajectory frame assertion(s) failed\n";
    return 1;
  }
  std::cout << "all trajectory frame tests passed\n";
  return 0;
}

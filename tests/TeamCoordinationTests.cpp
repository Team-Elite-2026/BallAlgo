#include "team/RoleArbiter.hpp"
#include "team/TeamBallFilter.hpp"
#include "team/TeamProto.hpp"

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

ballalgo::PeerTeamState makePeer(uint64_t nowUs, ballalgo::TeamRole role, bool ballVisible,
                                 float ballDistanceCm, ballalgo::ModeOverride modeOverride =
                                     ballalgo::ModeOverride::Auto) {
  ballalgo::PeerTeamState peer;
  peer.valid = true;
  peer.receivedTimeUs = nowUs;
  peer.state.robotId = 1;
  peer.state.roleClaim = role;
  peer.state.ballVisible = ballVisible;
  peer.state.ballDistanceCm = ballDistanceCm;
  peer.state.ballAgeMs = ballVisible ? 0u : ballalgo::config::kPeerStaleMs + 1u;
  peer.state.modeOverride = modeOverride;
  return peer;
}

void testProtoRoundTrip() {
  ballalgo::TeamState state;
  state.schemaVersion = 7;
  state.robotId = 0;
  state.seq = 44;
  state.monotonicTimeUs = 1234567;
  state.roleClaim = ballalgo::TeamRole::Defense;
  state.poseValid = true;
  state.poseXMm = 1100.5f;
  state.poseYMm = 2200.25f;
  state.headingDeg = 42.0f;
  state.ballVisible = true;
  state.ballDistanceCm = 87.5f;
  state.ballAbsXMm = 900.0f;
  state.ballAbsYMm = 1300.0f;
  state.ballSigmaXMm = 90.0f;
  state.ballSigmaYMm = 95.0f;
  state.ballAgeMs = 11;
  state.hasBall = true;
  state.startEnabled = true;
  state.goalIsBlue = false;
  state.modeOverride = ballalgo::ModeOverride::ManualDefense;

  const std::vector<uint8_t> encoded = ballalgo::encodeTeamState(state);
  ballalgo::TeamState decoded;
  expect(ballalgo::decodeTeamState(encoded, decoded), "protobuf decode should succeed");
  expect(decoded.schemaVersion == state.schemaVersion, "schema version should round-trip");
  expect(decoded.roleClaim == state.roleClaim, "role claim should round-trip");
  expect(decoded.modeOverride == state.modeOverride, "mode override should round-trip");
  expect(near(decoded.ballAbsXMm, state.ballAbsXMm), "ball abs x should round-trip");
  expect(decoded.startEnabled == state.startEnabled, "start flag should round-trip");

  const std::vector<uint8_t> framed = ballalgo::packTeamStateFrame(state);
  std::vector<uint8_t> partial(framed.begin(), framed.begin() + 2);
  std::vector<ballalgo::TeamState> out;
  expect(!ballalgo::unpackTeamStateFrames(partial, out), "partial frame should wait for more data");
  partial.insert(partial.end(), framed.begin() + 2, framed.end());
  expect(ballalgo::unpackTeamStateFrames(partial, out), "complete frame should parse");
  expect(out.size() == 1, "one framed packet should decode to one state");
  expect(out.front().seq == state.seq, "framed decode should preserve seq");
}

void testStalePacketRejectionHelper() {
  ballalgo::TeamState older;
  older.seq = 10;
  older.monotonicTimeUs = 1000;

  ballalgo::TeamState newer = older;
  newer.seq = 11;
  newer.monotonicTimeUs = 1100;

  expect(ballalgo::isNewerTeamState(newer, older), "newer seq should be accepted");
  expect(!ballalgo::isNewerTeamState(older, newer), "older seq should be rejected");
}

void testRoleWithinMargin() {
  const uint64_t nowUs = 1000000;
  ballalgo::RoleArbiter arbiter(0);
  ballalgo::PeerTeamState peer = makePeer(nowUs, ballalgo::TeamRole::Defense, true, 80.0f);
  for (int i = 0; i < 5; ++i) {
    expect(arbiter.update(nowUs + i, ballalgo::ModeOverride::Auto, true, 90.0f, peer) ==
               ballalgo::TeamRole::Offense,
           "offense should stay offense when distance margin is not met");
  }
}

void testRoleSwitchAfterThreeSamples() {
  const uint64_t nowUs = 2000000;
  ballalgo::RoleArbiter arbiter(1);
  ballalgo::PeerTeamState peer = makePeer(nowUs, ballalgo::TeamRole::Offense, true, 100.0f);
  expect(arbiter.update(nowUs, ballalgo::ModeOverride::Auto, true, 80.0f, peer) ==
             ballalgo::TeamRole::Defense,
         "first closer sample should not switch yet");
  expect(arbiter.update(nowUs + 1, ballalgo::ModeOverride::Auto, true, 80.0f, peer) ==
             ballalgo::TeamRole::Defense,
         "second closer sample should not switch yet");
  expect(arbiter.update(nowUs + 2, ballalgo::ModeOverride::Auto, true, 80.0f, peer) ==
             ballalgo::TeamRole::Offense,
         "third closer sample should switch to offense");
}

void testInvisibleOffenseHandoff() {
  const uint64_t nowUs = 3000000;
  ballalgo::RoleArbiter arbiter(1);
  ballalgo::PeerTeamState peer = makePeer(nowUs, ballalgo::TeamRole::Offense, false, 0.0f);
  arbiter.update(nowUs, ballalgo::ModeOverride::Auto, true, 70.0f, peer);
  arbiter.update(nowUs + 1, ballalgo::ModeOverride::Auto, true, 70.0f, peer);
  expect(arbiter.update(nowUs + 2, ballalgo::ModeOverride::Auto, true, 70.0f, peer) ==
             ballalgo::TeamRole::Offense,
         "visible defense robot should inherit offense after three fresh invisible-offense samples");
}

void testPeerTimeoutFallback() {
  const uint64_t nowUs = 4000000;
  ballalgo::RoleArbiter arbiter(1);
  ballalgo::PeerTeamState peer = makePeer(nowUs, ballalgo::TeamRole::Offense, true, 120.0f);
  arbiter.update(nowUs, ballalgo::ModeOverride::Auto, true, 80.0f, peer);
  arbiter.update(nowUs + 1, ballalgo::ModeOverride::Auto, true, 80.0f, peer);
  arbiter.update(nowUs + 2, ballalgo::ModeOverride::Auto, true, 80.0f, peer);

  peer.receivedTimeUs = nowUs - (ballalgo::config::kPeerStaleMs * 1000ull + 1ull);
  expect(arbiter.update(nowUs, ballalgo::ModeOverride::Auto, true, 80.0f, peer) ==
             ballalgo::TeamRole::Defense,
         "stale peer should force default fallback role");
}

void testSplitBrainRecovery() {
  const uint64_t nowUs = 5000000;
  ballalgo::RoleArbiter arbiter(1);
  ballalgo::PeerTeamState peer = makePeer(nowUs, ballalgo::TeamRole::Offense, true, 100.0f);
  arbiter.update(nowUs, ballalgo::ModeOverride::Auto, true, 70.0f, peer);
  arbiter.update(nowUs + 1, ballalgo::ModeOverride::Auto, true, 70.0f, peer);
  arbiter.update(nowUs + 2, ballalgo::ModeOverride::Auto, true, 70.0f, peer);
  expect(arbiter.currentRole() == ballalgo::TeamRole::Offense,
         "setup should move robot 1 into offense");

  peer.state.roleClaim = ballalgo::TeamRole::Offense;
  expect(arbiter.update(nowUs + 3, ballalgo::ModeOverride::Auto, true, 70.0f, peer) ==
             ballalgo::TeamRole::Defense,
         "split-brain offense/offense should reset to default tie-break role");
}

void testBallFusionLocalOnly() {
  ballalgo::TeamBallFilter filter;
  filter.update({true, 1000.0f, 500.0f, 50.0f, 50.0f});
  const ballalgo::FusedBallFieldState state = filter.state();
  expect(state.valid, "local-only update should initialize the team ball filter");
  expect(near(state.xMm, 1000.0f), "local-only update should preserve x");
  expect(near(state.yMm, 500.0f), "local-only update should preserve y");
}

void testBallFusionPeerOnly() {
  ballalgo::TeamBallFilter filter;
  filter.update({true, 400.0f, 800.0f, 70.0f, 70.0f});
  const ballalgo::FusedBallFieldState state = filter.state();
  expect(state.valid, "peer-only update should initialize the team ball filter");
  expect(near(state.xMm, 400.0f), "peer-only update should preserve x");
}

void testBallFusionUnequalCovariance() {
  ballalgo::TeamBallFilter filter;
  filter.update({true, 1000.0f, 0.0f, 40.0f, 40.0f});
  filter.update({true, 2000.0f, 0.0f, 300.0f, 300.0f});
  const ballalgo::FusedBallFieldState state = filter.state();
  expect(state.valid, "fused state should remain valid after two updates");
  expect(state.xMm < 1200.0f, "higher-confidence local measurement should dominate fused result");
}

void testBallFusionInvalidIgnored() {
  ballalgo::TeamBallFilter filter;
  filter.update({false, 0.0f, 0.0f, 0.0f, 0.0f});
  const ballalgo::FusedBallFieldState state = filter.state();
  expect(!state.valid, "invalid observations should be ignored");
}

}  // namespace

int main() {
  testProtoRoundTrip();
  testStalePacketRejectionHelper();
  testRoleWithinMargin();
  testRoleSwitchAfterThreeSamples();
  testInvisibleOffenseHandoff();
  testPeerTimeoutFallback();
  testSplitBrainRecovery();
  testBallFusionLocalOnly();
  testBallFusionPeerOnly();
  testBallFusionUnequalCovariance();
  testBallFusionInvalidIgnored();

  std::cout << "ballalgo_team_tests passed\n";
  return 0;
}

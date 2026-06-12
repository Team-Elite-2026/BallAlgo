#pragma once

#include "config.hpp"
#include "motion/Protocol.hpp"

#include <cstdint>

namespace ballalgo {

enum class TeamRole : uint8_t {
  Offense = 0,
  Defense = 1,
};

struct TeamState {
  uint32_t schemaVersion = 1;
  uint32_t robotId = 0;
  uint64_t seq = 0;
  uint64_t monotonicTimeUs = 0;
  TeamRole roleClaim = TeamRole::Offense;
  bool poseValid = false;
  float poseXMm = 0;
  float poseYMm = 0;
  float headingDeg = 0;
  bool ballVisible = false;
  float ballDistanceCm = 0;
  float ballAbsXMm = 0;
  float ballAbsYMm = 0;
  float ballSigmaXMm = 0;
  float ballSigmaYMm = 0;
  uint32_t ballAgeMs = 0;
  bool hasBall = false;
  bool startEnabled = false;
  bool goalIsBlue = true;
  ModeOverride modeOverride = ModeOverride::Auto;
};

struct PeerTeamState {
  TeamState state;
  bool valid = false;
  uint64_t receivedTimeUs = 0;
};

struct TeamBallObservation {
  bool valid = false;
  float xMm = 0;
  float yMm = 0;
  float sigmaXMm = 0;
  float sigmaYMm = 0;
};

struct FusedBallFieldState {
  bool valid = false;
  float xMm = 0;
  float yMm = 0;
  float vxMmS = 0;
  float vyMmS = 0;
  float sigmaXMm = 0;
  float sigmaYMm = 0;
};

inline TeamRole defaultRoleForRobotId(uint32_t robotId) {
  return robotId == 0 ? TeamRole::Offense : TeamRole::Defense;
}

inline TeamRole oppositeRole(TeamRole role) {
  return role == TeamRole::Offense ? TeamRole::Defense : TeamRole::Offense;
}

inline bool isPeerStateFresh(const PeerTeamState& peer, uint64_t nowUs) {
  if (!peer.valid || nowUs < peer.receivedTimeUs) return false;
  return (nowUs - peer.receivedTimeUs) <= static_cast<uint64_t>(config::kPeerStaleMs) * 1000ull;
}

inline bool isBallMeasurementFresh(const TeamState& state) {
  return state.ballVisible && state.ballAgeMs <= config::kPeerStaleMs;
}

inline bool isNewerTeamState(const TeamState& candidate, const TeamState& previous) {
  return candidate.seq > previous.seq || candidate.monotonicTimeUs > previous.monotonicTimeUs;
}

}  // namespace ballalgo

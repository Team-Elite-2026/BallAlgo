#include "team/RoleArbiter.hpp"

namespace ballalgo {
namespace {

bool overrideIsManual(ModeOverride overrideMode) {
  return overrideMode != ModeOverride::Auto;
}

TeamRole overrideRole(ModeOverride overrideMode, TeamRole fallback) {
  switch (overrideMode) {
    case ModeOverride::ManualOffense:
      return TeamRole::Offense;
    case ModeOverride::ManualDefense:
      return TeamRole::Defense;
    default:
      return fallback;
  }
}

}  // namespace

RoleArbiter::RoleArbiter(uint32_t robotId)
    : robotId_(robotId), currentRole_(defaultRoleForRobotId(robotId)) {}

void RoleArbiter::reset() {
  currentRole_ = defaultRole();
  betterSampleCount_ = 0;
}

TeamRole RoleArbiter::defaultRole() const {
  return defaultRoleForRobotId(robotId_);
}

TeamRole RoleArbiter::resolveManualOverrides(ModeOverride localOverride,
                                             const PeerTeamState& peer) const {
  const ModeOverride peerOverride =
      peer.valid ? peer.state.modeOverride : ModeOverride::Auto;
  const bool localManual = overrideIsManual(localOverride);
  const bool peerManual = overrideIsManual(peerOverride);

  if (!localManual && !peerManual) return currentRole_;

  if (localManual && !peerManual) return overrideRole(localOverride, currentRole_);
  if (!localManual && peerManual) return oppositeRole(overrideRole(peerOverride, currentRole_));

  const TeamRole localForced = overrideRole(localOverride, currentRole_);
  const TeamRole peerForced = overrideRole(peerOverride, currentRole_);
  if (localForced != peerForced) return localForced;
  return defaultRole();
}

bool RoleArbiter::peerIsCurrentOffense(const PeerTeamState& peer) const {
  return peer.state.roleClaim == TeamRole::Offense && currentRole_ == TeamRole::Defense;
}

bool RoleArbiter::shouldSwitchRoles(bool localBallVisible, float localBallDistanceCm,
                                    const PeerTeamState& peer) const {
  if (!peer.valid) return false;

  const bool peerBallVisible = isBallMeasurementFresh(peer.state);
  if (!localBallVisible && !peerBallVisible) return false;

  const bool peerOffense = peerIsCurrentOffense(peer);
  const float peerDistance = peer.state.ballDistanceCm;

  if (currentRole_ == TeamRole::Offense) {
    if (!peerBallVisible) return false;
    if (!localBallVisible) return true;
    return (localBallDistanceCm - peerDistance) >= config::kRoleSwitchMarginCm;
  }

  if (!localBallVisible) return false;
  if (!peerOffense) return false;
  if (!peerBallVisible) return true;
  return (peerDistance - localBallDistanceCm) >= config::kRoleSwitchMarginCm;
}

TeamRole RoleArbiter::update(uint64_t nowUs, ModeOverride localOverride, bool localBallVisible,
                             float localBallDistanceCm, const PeerTeamState& peer) {
  const TeamRole manualRole = resolveManualOverrides(localOverride, peer);
  if (overrideIsManual(localOverride) || (peer.valid && overrideIsManual(peer.state.modeOverride))) {
    currentRole_ = manualRole;
    betterSampleCount_ = 0;
    return currentRole_;
  }

  if (!isPeerStateFresh(peer, nowUs)) {
    currentRole_ = defaultRole();
    betterSampleCount_ = 0;
    return currentRole_;
  }

  if (peer.state.roleClaim == currentRole_) {
    currentRole_ = defaultRole();
    betterSampleCount_ = 0;
    return currentRole_;
  }

  if (shouldSwitchRoles(localBallVisible, localBallDistanceCm, peer)) {
    ++betterSampleCount_;
    if (betterSampleCount_ >= config::kRoleConfirmSamples) {
      currentRole_ = oppositeRole(currentRole_);
      betterSampleCount_ = 0;
    }
  } else {
    betterSampleCount_ = 0;
  }

  return currentRole_;
}

}  // namespace ballalgo

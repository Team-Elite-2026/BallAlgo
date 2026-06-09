#pragma once

#include "team/TeamTypes.hpp"

#include <cstdint>

namespace ballalgo {

class RoleArbiter {
 public:
  explicit RoleArbiter(uint32_t robotId);

  TeamRole currentRole() const { return currentRole_; }
  TeamRole update(uint64_t nowUs, ModeOverride localOverride, bool localBallVisible,
                  float localBallDistanceCm, const PeerTeamState& peer);
  void reset();

 private:
  TeamRole defaultRole() const;
  TeamRole resolveManualOverrides(ModeOverride localOverride, const PeerTeamState& peer) const;
  bool peerIsCurrentOffense(const PeerTeamState& peer) const;
  bool shouldSwitchRoles(bool localBallVisible, float localBallDistanceCm,
                         const PeerTeamState& peer) const;

  uint32_t robotId_ = 0;
  TeamRole currentRole_ = TeamRole::Offense;
  int betterSampleCount_ = 0;
};

}  // namespace ballalgo

#pragma once

#include "team/TeamTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ballalgo {

std::vector<uint8_t> encodeTeamState(const TeamState& state);
bool decodeTeamState(const uint8_t* data, size_t len, TeamState& out);
bool decodeTeamState(const std::vector<uint8_t>& data, TeamState& out);

std::vector<uint8_t> packTeamStateFrame(const TeamState& state);
bool unpackTeamStateFrames(std::vector<uint8_t>& buffer, std::vector<TeamState>& out);

}  // namespace ballalgo

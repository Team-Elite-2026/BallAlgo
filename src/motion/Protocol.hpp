#pragma once

#include "motion/VelocityProfile.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

constexpr uint32_t kProtocolMagic = 0xCEFAEDFEu;
constexpr uint8_t kMsgPing = 0x01;
constexpr uint8_t kMsgPong = 0x02;
constexpr uint8_t kMsgActionChunk = 0x03;

uint32_t crc32(const uint8_t* data, size_t len);
std::vector<uint8_t> packFrame(uint8_t type, const std::vector<uint8_t>& payload);
std::vector<uint8_t> packPong(uint64_t t0, uint64_t tPi);
std::vector<uint8_t> packActionChunk(uint64_t trajId, uint64_t startPi, uint16_t dtMs,
                                     const std::vector<MotionAction>& actions, int n,
                                     float vxMeas, float vyMeas, bool poseValid);
bool unpackFrames(std::vector<uint8_t>& buffer, std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& out);

}  // namespace ballalgo

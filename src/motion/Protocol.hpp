#pragma once

#include "motion/VelocityProfile.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ballalgo {

constexpr uint32_t kProtocolMagic = 0xCEFAEDFEu;
constexpr uint8_t kMsgPing = 0x01;
constexpr uint8_t kMsgPong = 0x02;
constexpr uint8_t kMsgActionChunk = 0x03;
constexpr uint8_t kMsgTelemetry = 0x04;

enum class ModeOverride : uint8_t {
  Auto = 0,
  ManualOffense = 1,
  ManualDefense = 2,
};

#pragma pack(push, 1)
struct TeensyTelemetryPayload {
  float headingDeg = 0;
  float mouseVxBodyMmS = 0;
  float mouseVyBodyMmS = 0;
  float omegaRadS = 0;
  uint8_t hasBall = 0;
  uint8_t startEnabled = 0;
  uint8_t goalIsBlue = 0;
  uint8_t modeOverride = 0;
  uint16_t serialLatencyUs = 0;
  uint16_t reserved = 0;
};
#pragma pack(pop)
static_assert(sizeof(TeensyTelemetryPayload) == 24,
              "TeensyTelemetryPayload must stay byte-identical across Pi and Teensy");

struct ProtocolFrame {
  uint8_t type = 0;
  std::vector<uint8_t> payload;
};

uint32_t crc32(const uint8_t* data, size_t len);
std::vector<uint8_t> packFrame(uint8_t type, const std::vector<uint8_t>& payload);
std::vector<uint8_t> packPong(uint64_t t0, uint64_t tPi);
std::vector<uint8_t> packActionChunk(uint64_t trajId, uint64_t startPi, uint16_t dtMs,
                                     const std::vector<MotionAction>& actions, int n,
                                     float vxMeas, float vyMeas, bool poseValid);
bool parseTeensyTelemetry(const uint8_t* data, size_t len, TeensyTelemetryPayload& out);
bool parseTeensyTelemetry(const std::vector<uint8_t>& payload, TeensyTelemetryPayload& out);
bool unpackFrames(std::vector<uint8_t>& buffer, std::vector<ProtocolFrame>& out);

}  // namespace ballalgo

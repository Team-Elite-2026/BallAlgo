#include "motion/ClockSync.hpp"

#include "motion/Protocol.hpp"

#include <chrono>
#include <cstring>

namespace ballalgo {

static uint64_t piTimeUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void ClockSync::processBuffer(RobotSerial& serial, std::vector<uint8_t>& rx) {
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> frames;
  unpackFrames(rx, frames);
  for (auto& [type, payload] : frames) {
    if (type == kMsgPing && payload.size() >= 8) {
      uint64_t t0;
      std::memcpy(&t0, payload.data(), 8);
      auto pong = packPong(t0, piTimeUs());
      serial.write(pong);
    }
  }
}

}  // namespace ballalgo

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

void ClockSync::processFrames(RobotSerial& serial, const std::vector<ProtocolFrame>& frames) {
  for (const auto& frame : frames) {
    if (frame.type == kMsgPing && frame.payload.size() >= 8) {
      uint64_t t0;
      std::memcpy(&t0, frame.payload.data(), 8);
      auto pong = packPong(t0, piTimeUs());
      serial.write(pong);
    }
  }
}

}  // namespace ballalgo

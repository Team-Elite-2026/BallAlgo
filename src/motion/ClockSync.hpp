#pragma once

#include "motion/Protocol.hpp"
#include "io/RobotSerial.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

class ClockSync {
 public:
  void processFrames(RobotSerial& serial, const std::vector<ProtocolFrame>& frames);
};

}  // namespace ballalgo

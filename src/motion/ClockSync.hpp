#pragma once

#include "io/RobotSerial.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

class ClockSync {
 public:
  void processBuffer(RobotSerial& serial, std::vector<uint8_t>& rx);
};

}  // namespace ballalgo

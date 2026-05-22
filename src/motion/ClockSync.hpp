#pragma once

#include "io/RobotSerial.hpp"

#include <cstdint>
#include <vector>

namespace ballalgo {

class ClockSync {
 public:
  void processBuffer(RobotSerial& serial, std::vector<uint8_t>& rx);
  int latencyUs() const { return latencyUs_; }

 private:
  int latencyUs_ = 0;
  float latencyEma_ = 0;
  bool haveEma_ = false;
};

}  // namespace ballalgo

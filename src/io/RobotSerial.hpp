#pragma once

#include "motion/Protocol.hpp"

#include <cstdint>
#include <sys/types.h>
#include <string>
#include <vector>

namespace ballalgo {

// Decoded high-frequency telemetry streamed from the Teensy over UART.
// Body-frame mouse velocities (mm/s, heading 0 = +y/forward, 90 = +x/right) and
// the gyro yaw rate (rad/s) feed the Step 1a dead-reckoning predict.
struct TeensyOdometry {
  float headingDeg = 0;
  float mouseVxBodyMmS = 0;  // lateral (+right)
  float mouseVyBodyMmS = 0;  // forward (+forward)
  float omegaRadS = 0;
  bool mouseFresh = false;
  bool telemetryFresh = false;
  bool hasBall = false;
  bool startEnabled = false;
  bool goalIsBlue = true;
  ModeOverride modeOverride = ModeOverride::Auto;
  uint16_t serialLatencyUs = 0;
};

class RobotSerial {
 public:
  bool open(const std::string& port, int baud);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  ssize_t readSome(std::vector<uint8_t>& out);
  bool write(const std::vector<uint8_t>& data);
  bool writeAscii(const std::string& s);
  bool poll();
  void takePendingFrames(std::vector<ProtocolFrame>& out);

  void pollHeading(float& headingDeg);
  void pollOdometry(TeensyOdometry& odo);

 private:
  bool writeAll(const uint8_t* data, size_t len);
  void handleFrame(const ProtocolFrame& frame);

  int fd_ = -1;
  std::vector<uint8_t> rxBuffer_;
  std::vector<ProtocolFrame> pendingFrames_;
  TeensyOdometry odoCache_;
};

}  // namespace ballalgo

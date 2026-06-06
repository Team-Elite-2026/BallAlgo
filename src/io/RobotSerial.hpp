#pragma once

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
  bool mouseFresh = false;  // set when a fresh vx or vy token arrived this poll
};

class RobotSerial {
 public:
  bool open(const std::string& port, int baud);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  ssize_t readSome(std::vector<uint8_t>& out);
  bool write(const std::vector<uint8_t>& data);
  bool writeAscii(const std::string& s);

  // Backward-compatible heading-only poll.
  void pollHeading(float& headingDeg);

  // Full ASCII telemetry poll. Tokens are numeric strings terminated by a tag
  // letter: 'h' heading(deg), 'x' mouse vx body(mm/s), 'y' mouse vy body(mm/s),
  // 'w' yaw rate(rad/s). odo.mouseFresh is true when new mouse data arrived.
  void pollOdometry(TeensyOdometry& odo);

 private:
  bool writeAll(const uint8_t* data, size_t len);
  void consumeAscii(TeensyOdometry& odo);

  int fd_ = -1;
  std::string headingBuf_;
  TeensyOdometry odoCache_;
};

}  // namespace ballalgo

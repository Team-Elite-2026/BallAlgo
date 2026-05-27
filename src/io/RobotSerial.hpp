#pragma once

#include <cstdint>
#include <sys/types.h>
#include <string>
#include <vector>

namespace ballalgo {

class RobotSerial {
 public:
  bool open(const std::string& port, int baud);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  ssize_t readSome(std::vector<uint8_t>& out);
  bool write(const std::vector<uint8_t>& data);
  bool writeAscii(const std::string& s);
  void pollHeading(float& headingDeg);

 private:
  int fd_ = -1;
  std::string headingBuf_;
};

}  // namespace ballalgo

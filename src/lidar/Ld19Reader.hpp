#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ballalgo {

struct LidarPoint {
  uint16_t distanceMm = 0;
  uint8_t intensity = 0;
  uint16_t angleCd = 0;
};

class Ld19Reader {
 public:
  explicit Ld19Reader(const std::string& port, int baud, double timeoutSec);
  ~Ld19Reader();

  bool isConnected() const { return fd_ >= 0; }
  std::vector<LidarPoint> pollPoints();
  void close();

 private:
  std::vector<LidarPoint> parseFrame(const uint8_t* frame, size_t len) const;
  static uint8_t crc8(const uint8_t* data, size_t len);
  static int angleStep(uint16_t start, uint16_t end, int lenMinusOne);

  int fd_ = -1;
  std::vector<uint8_t> buffer_;
};

}  // namespace ballalgo

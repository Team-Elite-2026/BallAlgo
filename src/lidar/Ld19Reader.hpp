#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ballalgo {

struct LidarPoint {
  uint16_t distanceMm  = 0;
  uint8_t  intensity   = 0;
  uint16_t angleCd     = 0;
  uint64_t timestampUs = 0;  // monotonic steady_clock microseconds; 0 = unset
};

class Ld19Reader {
 public:
  explicit Ld19Reader(const std::string& port, int baud, double timeoutSec);
  ~Ld19Reader();

  bool isConnected() const { return fd_ >= 0; }
  std::vector<LidarPoint> pollPoints();
  void close();

 private:
  std::vector<LidarPoint> parseFrame(const uint8_t* frame, size_t len);  // non-const: updates timestamp state
  static uint8_t crc8(const uint8_t* data, size_t len);
  static int angleStep(uint16_t start, uint16_t end, int lenMinusOne);

  int fd_ = -1;
  std::vector<uint8_t> buffer_;

  // LD19 hardware timestamp → monotonic wall-clock alignment
  bool     hwTimestampInited_    = false;
  uint64_t epochBaseUs_          = 0;   // steady_clock at first frame
  uint32_t hwBaseMs_             = 0;   // hardware ms at first frame
  uint32_t hwPrevMs_             = 0;   // for 30000 ms wrap detection
  uint32_t wrapCount_            = 0;
  uint64_t lastFrameTimestampUs_ = 0;   // monotonic guard against corrupt frames
};

}  // namespace ballalgo

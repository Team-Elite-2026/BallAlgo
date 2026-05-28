#pragma once

#include <optional>
#include <termios.h>

namespace ballalgo {

inline std::optional<speed_t> baudToSpeed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B1500000
    case 1500000:
      return B1500000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
#ifdef B2500000
    case 2500000:
      return B2500000;
#endif
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
#ifdef B3500000
    case 3500000:
      return B3500000;
#endif
#ifdef B4000000
    case 4000000:
      return B4000000;
#endif
    default:
      return std::nullopt;
  }
}

}  // namespace ballalgo

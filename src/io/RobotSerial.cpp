#include "io/RobotSerial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace ballalgo {

namespace {

speed_t baudToSpeed(int baud) {
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
    case 460800:
      return B460800;
    case 921600:
      return B921600;
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
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
    default:
      return B115200;
  }
}

}  // namespace

bool RobotSerial::open(const std::string& port, int baud) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[SERIAL] open failed " << port << "\n";
    return false;
  }
  termios tty{};
  tcgetattr(fd_, &tty);
  const speed_t spd = baudToSpeed(baud);
  cfsetospeed(&tty, spd);
  cfsetispeed(&tty, spd);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_lflag = tty.c_oflag = tty.c_iflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  tcsetattr(fd_, TCSANOW, &tty);
  std::cout << "[SERIAL] " << port << " @ " << baud << "\n";
  return true;
}

void RobotSerial::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t RobotSerial::readSome(std::vector<uint8_t>& out) {
  if (fd_ < 0) return 0;
  uint8_t buf[512];
  ssize_t n = ::read(fd_, buf, sizeof(buf));
  if (n > 0) out.insert(out.end(), buf, buf + n);
  return n;
}

bool RobotSerial::write(const std::vector<uint8_t>& data) {
  if (fd_ < 0) return false;
  ssize_t w = ::write(fd_, data.data(), data.size());
  return w == static_cast<ssize_t>(data.size());
}

bool RobotSerial::writeAscii(const std::string& s) {
  return write(std::vector<uint8_t>(s.begin(), s.end()));
}

void RobotSerial::pollHeading(float& headingDeg) {
  std::vector<uint8_t> chunk;
  readSome(chunk);
  for (uint8_t b : chunk) {
    char ch = static_cast<char>(b);
    if (ch == 'h') {
      if (!headingBuf_.empty()) {
        try {
          headingDeg = std::stof(headingBuf_);
        } catch (...) {
        }
      }
      headingBuf_.clear();
    } else if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '.') {
      headingBuf_ += ch;
    } else if (ch != '\xFE') {
      headingBuf_.clear();
    }
  }
}

}  // namespace ballalgo

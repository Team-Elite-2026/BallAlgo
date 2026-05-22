#include "io/RobotSerial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace ballalgo {

bool RobotSerial::open(const std::string& port, int baud) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[SERIAL] open failed " << port << "\n";
    return false;
  }
  termios tty{};
  tcgetattr(fd_, &tty);
  speed_t spd = B2000000;
  cfsetospeed(&tty, spd);
  cfsetispeed(&tty, spd);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_lflag = tty.c_oflag = tty.c_iflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  tcsetattr(fd_, TCSANOW, &tty);
  (void)baud;
  std::cout << "[SERIAL] " << port << "\n";
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

#include "io/RobotSerial.hpp"
#include "io/SerialBaud.hpp"

#include <fcntl.h>
#include <errno.h>
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
  const auto spd = baudToSpeed(baud);
  if (!spd) {
    std::cerr << "[SERIAL] unsupported baud rate " << baud << " for " << port << "\n";
    close();
    return false;
  }
  cfsetospeed(&tty, *spd);
  cfsetispeed(&tty, *spd);
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

bool RobotSerial::writeAll(const uint8_t* data, size_t len) {
  if (fd_ < 0) return false;
  size_t written = 0;
  while (written < len) {
    ssize_t chunk = ::write(fd_, data + written, len - written);
    if (chunk > 0) {
      written += static_cast<size_t>(chunk);
      continue;
    }
    if (chunk < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      ::usleep(100);
      continue;
    }
    return false;
  }
  return true;
}

bool RobotSerial::write(const std::vector<uint8_t>& data) {
  return writeAll(data.data(), data.size());
}

bool RobotSerial::writeAscii(const std::string& s) {
  return writeAll(reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

void RobotSerial::consumeAscii(TeensyOdometry& odo) {
  std::vector<uint8_t> chunk;
  readSome(chunk);
  auto flush = [&](char tag) {
    if (headingBuf_.empty()) return;
    float value = 0;
    try {
      value = std::stof(headingBuf_);
    } catch (...) {
      headingBuf_.clear();
      return;
    }
    switch (tag) {
      case 'h':
        odo.headingDeg = value;
        break;
      case 'x':
        odo.mouseVxBodyMmS = value;
        odo.mouseFresh = true;
        break;
      case 'y':
        odo.mouseVyBodyMmS = value;
        odo.mouseFresh = true;
        break;
      case 'w':
        odo.omegaRadS = value;
        break;
      default:
        break;
    }
    headingBuf_.clear();
  };
  for (uint8_t b : chunk) {
    char ch = static_cast<char>(b);
    if (ch == 'h' || ch == 'x' || ch == 'y' || ch == 'w') {
      flush(ch);
    } else if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '.') {
      headingBuf_ += ch;
    } else if (ch != '\xFE') {
      headingBuf_.clear();
    }
  }
}

void RobotSerial::pollHeading(float& headingDeg) {
  odoCache_.mouseFresh = false;
  odoCache_.headingDeg = headingDeg;
  consumeAscii(odoCache_);
  headingDeg = odoCache_.headingDeg;
}

void RobotSerial::pollOdometry(TeensyOdometry& odo) {
  odoCache_.mouseFresh = false;
  consumeAscii(odoCache_);
  odo = odoCache_;
}

}  // namespace ballalgo

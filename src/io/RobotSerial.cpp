#include "io/RobotSerial.hpp"
#include "io/SerialBaud.hpp"

#include <algorithm>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <utility>

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

bool RobotSerial::poll() {
  odoCache_.mouseFresh = false;
  odoCache_.telemetryFresh = false;

  std::vector<uint8_t> chunk;
  readSome(chunk);
  if (!chunk.empty()) rxBuffer_.insert(rxBuffer_.end(), chunk.begin(), chunk.end());

  std::vector<ProtocolFrame> frames;
  unpackFrames(rxBuffer_, frames);
  for (const auto& frame : frames) {
    handleFrame(frame);
  }
  return !frames.empty();
}

void RobotSerial::takePendingFrames(std::vector<ProtocolFrame>& out) {
  out = std::move(pendingFrames_);
  pendingFrames_.clear();
}

void RobotSerial::handleFrame(const ProtocolFrame& frame) {
  if (frame.type == kMsgTelemetry) {
    TeensyTelemetryPayload telemetry;
    if (!parseTeensyTelemetry(frame.payload, telemetry)) return;
    odoCache_.headingDeg = telemetry.headingDeg;
    odoCache_.mouseVxBodyMmS = telemetry.mouseVxBodyMmS;
    odoCache_.mouseVyBodyMmS = telemetry.mouseVyBodyMmS;
    odoCache_.omegaRadS = telemetry.omegaRadS;
    odoCache_.mouseFresh = true;
    odoCache_.telemetryFresh = true;
    odoCache_.hasBall = telemetry.hasBall != 0;
    odoCache_.startEnabled = telemetry.startEnabled != 0;
    odoCache_.goalIsBlue = telemetry.goalIsBlue != 0;
    switch (telemetry.modeOverride) {
      case 1:
        odoCache_.modeOverride = ModeOverride::ManualOffense;
        break;
      case 2:
        odoCache_.modeOverride = ModeOverride::ManualDefense;
        break;
      default:
        odoCache_.modeOverride = ModeOverride::Auto;
        break;
    }
    odoCache_.serialLatencyUs = telemetry.serialLatencyUs;
    return;
  }

  pendingFrames_.push_back(frame);
}

void RobotSerial::pollHeading(float& headingDeg) {
  odoCache_.headingDeg = headingDeg;
  poll();
  headingDeg = odoCache_.headingDeg;
}

void RobotSerial::pollOdometry(TeensyOdometry& odo) {
  poll();
  odo = odoCache_;
}

}  // namespace ballalgo

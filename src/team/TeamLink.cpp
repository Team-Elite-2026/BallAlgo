#include "team/TeamLink.hpp"

#include "config.hpp"
#include "team/TeamProto.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>

#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#endif

namespace ballalgo {
namespace {

constexpr uint64_t kBackoffMinUs = 200000;
constexpr uint64_t kBackoffMaxUs = 2000000;

bool isMajorStateChange(const TeamState& a, const TeamState& b) {
  return a.roleClaim != b.roleClaim || a.poseValid != b.poseValid || a.ballVisible != b.ballVisible ||
         a.hasBall != b.hasBall || a.startEnabled != b.startEnabled ||
         a.goalIsBlue != b.goalIsBlue || a.modeOverride != b.modeOverride;
}

#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
bool setNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return false;
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}
#endif

}  // namespace

TeamLink::TeamLink(uint32_t robotId, std::string peerBtAddress, uint8_t rfcommChannel)
    : robotId_(robotId),
      peerBtAddress_(std::move(peerBtAddress)),
      rfcommChannel_(rfcommChannel),
      serverMode_(robotId == 0),
      enabled_(rfcommChannel != 0 && (robotId == 0 || !peerBtAddress_.empty())) {}

TeamLink::~TeamLink() {
  closeConnection();
  closeListener();
}

bool TeamLink::connected() const {
  return connFd_ >= 0;
}

void TeamLink::publish(const TeamState& localState, uint64_t nowUs) {
#if !defined(__linux__) || !defined(BALLALGO_ENABLE_BLUETOOTH)
  (void)localState;
  (void)nowUs;
  return;
#else
  if (!enabled_) return;

  TeamState stamped = localState;
  stamped.seq = ++localSeq_;
  stamped.monotonicTimeUs = nowUs;

  const uint64_t sendIntervalUs = static_cast<uint64_t>(1000000.0 / config::kTeamPacketHz);
  const bool due = lastSendUs_ == 0 || (nowUs - lastSendUs_) >= sendIntervalUs;
  if (!due && haveLastSent_ && !isMajorStateChange(stamped, lastSentState_)) return;
  if (connFd_ < 0) return;

  if (!sendFrame(packTeamStateFrame(stamped))) {
    closeConnection();
    scheduleReconnect(nowUs);
    return;
  }

  lastSendUs_ = nowUs;
  lastSentState_ = stamped;
  haveLastSent_ = true;
#endif
}

void TeamLink::poll(uint64_t nowUs) {
#if !defined(__linux__) || !defined(BALLALGO_ENABLE_BLUETOOTH)
  (void)nowUs;
  return;
#else
  if (!enabled_) return;

  if (serverMode_) {
    ensureListener();
    acceptPeer(nowUs);
  } else {
    if (connectingFd_ >= 0) {
      finishClientConnect(nowUs);
    } else if (connFd_ < 0) {
      startClientConnect(nowUs);
    }
  }

  readPeer(nowUs);
#endif
}

void TeamLink::closeListener() {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (listenerFd_ >= 0) {
    ::close(listenerFd_);
    listenerFd_ = -1;
  }
#endif
}

void TeamLink::closeConnection() {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (connFd_ >= 0) {
    ::close(connFd_);
    connFd_ = -1;
  }
  if (connectingFd_ >= 0) {
    ::close(connectingFd_);
    connectingFd_ = -1;
  }
  rxBuffer_.clear();
#endif
}

void TeamLink::attachConnection(int fd) {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (connFd_ >= 0) ::close(connFd_);
  connFd_ = fd;
  if (connectingFd_ == fd) connectingFd_ = -1;
  rxBuffer_.clear();
  connectBackoffExp_ = 0;
  nextConnectAttemptUs_ = 0;
  std::cout << "[TEAM] peer connected on RFCOMM channel " << static_cast<int>(rfcommChannel_)
            << "\n";
#else
  (void)fd;
#endif
}

void TeamLink::scheduleReconnect(uint64_t nowUs) {
  if (serverMode_) return;
  const uint64_t delay = std::min(kBackoffMaxUs, kBackoffMinUs << std::min(connectBackoffExp_, 4));
  nextConnectAttemptUs_ = nowUs + delay;
  connectBackoffExp_ = std::min(connectBackoffExp_ + 1, 5);
}

void TeamLink::ensureListener() {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (listenerFd_ >= 0) return;

  listenerFd_ = ::socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  if (listenerFd_ < 0) {
    std::cerr << "[TEAM] failed to create RFCOMM listener\n";
    enabled_ = false;
    return;
  }

  sockaddr_rc localAddress{};
  bdaddr_t anyAddress = {{0, 0, 0, 0, 0, 0}};
  localAddress.rc_family = AF_BLUETOOTH;
  bacpy(&localAddress.rc_bdaddr, &anyAddress);
  localAddress.rc_channel = rfcommChannel_;

  if (::bind(listenerFd_, reinterpret_cast<sockaddr*>(&localAddress), sizeof(localAddress)) != 0 ||
      ::listen(listenerFd_, 1) != 0 || !setNonBlocking(listenerFd_)) {
    std::cerr << "[TEAM] failed to bind RFCOMM listener on channel "
              << static_cast<int>(rfcommChannel_) << "\n";
    closeListener();
    enabled_ = false;
  }
#endif
}

void TeamLink::acceptPeer(uint64_t nowUs) {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  (void)nowUs;
  if (listenerFd_ < 0 || connFd_ >= 0) return;

  sockaddr_rc remoteAddress{};
  socklen_t remoteLen = sizeof(remoteAddress);
  const int fd = ::accept(listenerFd_, reinterpret_cast<sockaddr*>(&remoteAddress), &remoteLen);
  if (fd < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cerr << "[TEAM] RFCOMM accept failed\n";
    }
    return;
  }

  if (!setNonBlocking(fd)) {
    ::close(fd);
    return;
  }

  attachConnection(fd);
#else
  (void)nowUs;
#endif
}

void TeamLink::startClientConnect(uint64_t nowUs) {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (serverMode_ || connFd_ >= 0 || connectingFd_ >= 0 || nowUs < nextConnectAttemptUs_) return;

  const int fd = ::socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  if (fd < 0) {
    scheduleReconnect(nowUs);
    return;
  }
  if (!setNonBlocking(fd)) {
    ::close(fd);
    scheduleReconnect(nowUs);
    return;
  }

  sockaddr_rc remoteAddress{};
  remoteAddress.rc_family = AF_BLUETOOTH;
  remoteAddress.rc_channel = rfcommChannel_;
  str2ba(peerBtAddress_.c_str(), &remoteAddress.rc_bdaddr);

  const int rc = ::connect(fd, reinterpret_cast<sockaddr*>(&remoteAddress), sizeof(remoteAddress));
  if (rc == 0) {
    attachConnection(fd);
    return;
  }
  if (errno == EINPROGRESS || errno == EALREADY) {
    connectingFd_ = fd;
    return;
  }

  ::close(fd);
  scheduleReconnect(nowUs);
#else
  (void)nowUs;
#endif
}

void TeamLink::finishClientConnect(uint64_t nowUs) {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (connectingFd_ < 0) return;

  int err = 0;
  socklen_t errLen = sizeof(err);
  if (::getsockopt(connectingFd_, SOL_SOCKET, SO_ERROR, &err, &errLen) != 0) {
    err = errno;
  }

  if (err == 0) {
    attachConnection(connectingFd_);
    return;
  }

  if (err == EINPROGRESS || err == EALREADY || err == EWOULDBLOCK) return;

  ::close(connectingFd_);
  connectingFd_ = -1;
  scheduleReconnect(nowUs);
#else
  (void)nowUs;
#endif
}

void TeamLink::readPeer(uint64_t nowUs) {
#if defined(__linux__) && defined(BALLALGO_ENABLE_BLUETOOTH)
  if (connFd_ < 0) return;

  uint8_t chunk[512];
  while (true) {
    const ssize_t n = ::read(connFd_, chunk, sizeof(chunk));
    if (n > 0) {
      rxBuffer_.insert(rxBuffer_.end(), chunk, chunk + n);
      continue;
    }
    if (n == 0) {
      closeConnection();
      scheduleReconnect(nowUs);
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
    closeConnection();
    scheduleReconnect(nowUs);
    return;
  }

  // Decode any complete frames currently buffered. A poll loop running faster
  // than the peer's send rate will frequently find no complete frame and an
  // empty buffer — that is the NORMAL idle state, not a desync. Real teardowns
  // are handled above: read()==0 (peer closed) and read() error. Do NOT tear
  // down here just because no frame was decoded this tick.
  std::vector<TeamState> states;
  unpackTeamStateFrames(rxBuffer_, states);

  for (const TeamState& state : states) {
    if (state.robotId == robotId_) continue;
    if (peerState_.valid && !isNewerTeamState(state, peerState_.state)) continue;
    peerState_.state = state;
    peerState_.valid = true;
    peerState_.receivedTimeUs = nowUs;
  }
#else
  (void)nowUs;
#endif
}

bool TeamLink::sendFrame(const std::vector<uint8_t>& data) {
#if !defined(__linux__) || !defined(BALLALGO_ENABLE_BLUETOOTH)
  (void)data;
  return false;
#else
  if (connFd_ < 0) return false;

  size_t written = 0;
  while (written < data.size()) {
    const ssize_t n = ::write(connFd_, data.data() + written, data.size() - written);
    if (n > 0) {
      written += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    return false;
  }
  return true;
#endif
}

}  // namespace ballalgo

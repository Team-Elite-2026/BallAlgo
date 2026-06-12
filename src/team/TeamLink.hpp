#pragma once

#include "team/TeamTypes.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ballalgo {

class TeamLink {
 public:
  TeamLink(uint32_t robotId, std::string peerBtAddress, uint8_t rfcommChannel);
  ~TeamLink();

  void poll(uint64_t nowUs);
  void publish(const TeamState& localState, uint64_t nowUs);

  bool enabled() const { return enabled_; }
  bool connected() const;
  const PeerTeamState& peerState() const { return peerState_; }

 private:
  void closeListener();
  void closeConnection();
  void attachConnection(int fd);
  void scheduleReconnect(uint64_t nowUs);
  void ensureListener();
  void acceptPeer(uint64_t nowUs);
  void startClientConnect(uint64_t nowUs);
  void finishClientConnect(uint64_t nowUs);
  void readPeer(uint64_t nowUs);
  bool sendFrame(const std::vector<uint8_t>& data);

  uint32_t robotId_ = 0;
  std::string peerBtAddress_;
  uint8_t rfcommChannel_ = 0;
  bool serverMode_ = false;
  bool enabled_ = false;
  uint64_t localSeq_ = 0;
  uint64_t lastSendUs_ = 0;
  bool haveLastSent_ = false;
  TeamState lastSentState_;
  PeerTeamState peerState_;
  std::vector<uint8_t> rxBuffer_;
  int listenerFd_ = -1;
  int connFd_ = -1;
  int connectingFd_ = -1;
  uint64_t nextConnectAttemptUs_ = 0;
  int connectBackoffExp_ = 0;
};

}  // namespace ballalgo

#include "motion/Protocol.hpp"

#include <cstring>

namespace ballalgo {

static uint32_t crc32Update(uint32_t crc, uint8_t b) {
  crc ^= b;
  for (int k = 0; k < 8; ++k) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
  return crc;
}

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) crc = crc32Update(crc, data[i]);
  return ~crc;
}

std::vector<uint8_t> packFrame(uint8_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> body;
  body.resize(7 + payload.size());
  std::memcpy(body.data(), &kProtocolMagic, 4);
  body[4] = type;
  uint16_t plen = static_cast<uint16_t>(payload.size());
  std::memcpy(body.data() + 5, &plen, 2);
  std::memcpy(body.data() + 7, payload.data(), payload.size());
  std::vector<uint8_t> frame = body;
  uint32_t c = crc32(body.data(), body.size());
  frame.insert(frame.end(), reinterpret_cast<uint8_t*>(&c),
               reinterpret_cast<uint8_t*>(&c) + 4);
  return frame;
}

std::vector<uint8_t> packPong(uint64_t t0, uint64_t tPi) {
  std::vector<uint8_t> p(16);
  std::memcpy(p.data(), &t0, 8);
  std::memcpy(p.data() + 8, &tPi, 8);
  return packFrame(kMsgPong, p);
}

std::vector<uint8_t> packActionChunk(uint64_t trajId, uint64_t startPi, uint16_t dtMs,
                                     const std::vector<MotionAction>& actions, int n,
                                     float vxMeas, float vyMeas, bool poseValid) {
  std::vector<uint8_t> pl;
  pl.resize(20 + 12 + n * 24);
  size_t off = 0;
  std::memcpy(pl.data() + off, &trajId, 8);
  off += 8;
  std::memcpy(pl.data() + off, &startPi, 8);
  off += 8;
  std::memcpy(pl.data() + off, &dtMs, 2);
  off += 2;
  uint16_t num = static_cast<uint16_t>(n);
  std::memcpy(pl.data() + off, &num, 2);
  off += 2;
  std::memcpy(pl.data() + off, &vxMeas, 4);
  off += 4;
  std::memcpy(pl.data() + off, &vyMeas, 4);
  off += 4;
  uint8_t pv = poseValid ? 1 : 0;
  pl[off++] = pv;
  off += 3;
  for (int i = 0; i < n; ++i) {
    const auto& a = actions[i];
    std::memcpy(pl.data() + off, &a.vx, 4);
    off += 4;
    std::memcpy(pl.data() + off, &a.vy, 4);
    off += 4;
    std::memcpy(pl.data() + off, &a.omega, 4);
    off += 4;
    std::memcpy(pl.data() + off, &a.ax, 4);
    off += 4;
    std::memcpy(pl.data() + off, &a.ay, 4);
    off += 4;
    std::memcpy(pl.data() + off, &a.alpha, 4);
    off += 4;
  }
  pl.resize(off);
  return packFrame(kMsgActionChunk, pl);
}

bool parseTeensyTelemetry(const uint8_t* data, size_t len, TeensyTelemetryPayload& out) {
  if (data == nullptr || len != sizeof(TeensyTelemetryPayload)) return false;
  std::memcpy(&out, data, sizeof(out));
  return true;
}

bool parseTeensyTelemetry(const std::vector<uint8_t>& payload, TeensyTelemetryPayload& out) {
  return parseTeensyTelemetry(payload.data(), payload.size(), out);
}

bool unpackFrames(std::vector<uint8_t>& buffer, std::vector<ProtocolFrame>& out) {
  out.clear();
  size_t i = 0;
  while (i + 11 <= buffer.size()) {
    uint32_t magic;
    std::memcpy(&magic, buffer.data() + i, 4);
    if (magic != kProtocolMagic) {
      ++i;
      continue;
    }
    uint8_t type = buffer[i + 4];
    uint16_t plen;
    std::memcpy(&plen, buffer.data() + i + 5, 2);
    size_t frameLen = 7 + plen + 4;
    if (i + frameLen > buffer.size()) break;
    uint32_t expect;
    std::memcpy(&expect, buffer.data() + i + 7 + plen, 4);
    if (crc32(buffer.data() + i, 7 + plen) != expect) {
      ++i;
      continue;
    }
    ProtocolFrame frame;
    frame.type = type;
    frame.payload.assign(buffer.begin() + i + 7, buffer.begin() + i + 7 + plen);
    out.push_back(std::move(frame));
    buffer.erase(buffer.begin(), buffer.begin() + i + frameLen);
    i = 0;
  }
  if (i > 0) buffer.erase(buffer.begin(), buffer.begin() + i);
  return !out.empty();
}

}  // namespace ballalgo

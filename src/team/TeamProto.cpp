#include "team/TeamProto.hpp"

#include <cstring>

namespace ballalgo {
namespace {

constexpr uint32_t kMaxTeamPacketBytes = 4096;

void appendVarint(std::vector<uint8_t>& out, uint64_t value) {
  while (value >= 0x80u) {
    out.push_back(static_cast<uint8_t>(value) | 0x80u);
    value >>= 7;
  }
  out.push_back(static_cast<uint8_t>(value));
}

void appendTag(std::vector<uint8_t>& out, uint32_t fieldNumber, uint8_t wireType) {
  appendVarint(out, (static_cast<uint64_t>(fieldNumber) << 3) | wireType);
}

void appendBool(std::vector<uint8_t>& out, uint32_t fieldNumber, bool value) {
  appendTag(out, fieldNumber, 0);
  appendVarint(out, value ? 1u : 0u);
}

void appendUInt32(std::vector<uint8_t>& out, uint32_t fieldNumber, uint32_t value) {
  appendTag(out, fieldNumber, 0);
  appendVarint(out, value);
}

void appendUInt64(std::vector<uint8_t>& out, uint32_t fieldNumber, uint64_t value) {
  appendTag(out, fieldNumber, 0);
  appendVarint(out, value);
}

void appendEnum(std::vector<uint8_t>& out, uint32_t fieldNumber, uint32_t value) {
  appendUInt32(out, fieldNumber, value);
}

void appendFloat(std::vector<uint8_t>& out, uint32_t fieldNumber, float value) {
  appendTag(out, fieldNumber, 5);
  const size_t oldSize = out.size();
  out.resize(oldSize + sizeof(value));
  std::memcpy(out.data() + oldSize, &value, sizeof(value));
}

bool readVarint(const uint8_t* data, size_t len, size_t& offset, uint64_t& value) {
  value = 0;
  int shift = 0;
  while (offset < len && shift < 64) {
    const uint8_t byte = data[offset++];
    value |= static_cast<uint64_t>(byte & 0x7Fu) << shift;
    if ((byte & 0x80u) == 0) return true;
    shift += 7;
  }
  return false;
}

bool readFixed32(const uint8_t* data, size_t len, size_t& offset, float& value) {
  if (offset + sizeof(value) > len) return false;
  std::memcpy(&value, data + offset, sizeof(value));
  offset += sizeof(value);
  return true;
}

bool skipField(const uint8_t* data, size_t len, size_t& offset, uint8_t wireType) {
  switch (wireType) {
    case 0: {
      uint64_t ignored = 0;
      return readVarint(data, len, offset, ignored);
    }
    case 1:
      if (offset + 8 > len) return false;
      offset += 8;
      return true;
    case 2: {
      uint64_t size = 0;
      if (!readVarint(data, len, offset, size)) return false;
      if (offset + size > len) return false;
      offset += static_cast<size_t>(size);
      return true;
    }
    case 5:
      if (offset + 4 > len) return false;
      offset += 4;
      return true;
    default:
      return false;
  }
}

}  // namespace

std::vector<uint8_t> encodeTeamState(const TeamState& state) {
  std::vector<uint8_t> out;
  out.reserve(128);

  appendUInt32(out, 1, state.schemaVersion);
  appendUInt32(out, 2, state.robotId);
  appendUInt64(out, 3, state.seq);
  appendUInt64(out, 4, state.monotonicTimeUs);
  appendEnum(out, 5, static_cast<uint32_t>(state.roleClaim));
  appendBool(out, 6, state.poseValid);
  appendFloat(out, 7, state.poseXMm);
  appendFloat(out, 8, state.poseYMm);
  appendFloat(out, 9, state.headingDeg);
  appendBool(out, 10, state.ballVisible);
  appendFloat(out, 11, state.ballDistanceCm);
  appendFloat(out, 12, state.ballAbsXMm);
  appendFloat(out, 13, state.ballAbsYMm);
  appendFloat(out, 14, state.ballSigmaXMm);
  appendFloat(out, 15, state.ballSigmaYMm);
  appendUInt32(out, 16, state.ballAgeMs);
  appendBool(out, 17, state.hasBall);
  appendBool(out, 18, state.startEnabled);
  appendBool(out, 19, state.goalIsBlue);
  appendEnum(out, 20, static_cast<uint32_t>(state.modeOverride));

  return out;
}

bool decodeTeamState(const uint8_t* data, size_t len, TeamState& out) {
  if (data == nullptr) return false;

  TeamState decoded;
  size_t offset = 0;
  while (offset < len) {
    uint64_t tag = 0;
    if (!readVarint(data, len, offset, tag)) return false;

    const uint32_t fieldNumber = static_cast<uint32_t>(tag >> 3);
    const uint8_t wireType = static_cast<uint8_t>(tag & 0x07u);
    uint64_t varintValue = 0;
    float floatValue = 0;

    switch (fieldNumber) {
      case 1:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.schemaVersion = static_cast<uint32_t>(varintValue);
        break;
      case 2:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.robotId = static_cast<uint32_t>(varintValue);
        break;
      case 3:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.seq = varintValue;
        break;
      case 4:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.monotonicTimeUs = varintValue;
        break;
      case 5:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.roleClaim = varintValue == 1 ? TeamRole::Defense : TeamRole::Offense;
        break;
      case 6:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.poseValid = varintValue != 0;
        break;
      case 7:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.poseXMm = floatValue;
        break;
      case 8:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.poseYMm = floatValue;
        break;
      case 9:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.headingDeg = floatValue;
        break;
      case 10:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.ballVisible = varintValue != 0;
        break;
      case 11:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.ballDistanceCm = floatValue;
        break;
      case 12:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.ballAbsXMm = floatValue;
        break;
      case 13:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.ballAbsYMm = floatValue;
        break;
      case 14:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.ballSigmaXMm = floatValue;
        break;
      case 15:
        if (wireType != 5 || !readFixed32(data, len, offset, floatValue)) return false;
        decoded.ballSigmaYMm = floatValue;
        break;
      case 16:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.ballAgeMs = static_cast<uint32_t>(varintValue);
        break;
      case 17:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.hasBall = varintValue != 0;
        break;
      case 18:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.startEnabled = varintValue != 0;
        break;
      case 19:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        decoded.goalIsBlue = varintValue != 0;
        break;
      case 20:
        if (wireType != 0 || !readVarint(data, len, offset, varintValue)) return false;
        switch (varintValue) {
          case 1:
            decoded.modeOverride = ModeOverride::ManualOffense;
            break;
          case 2:
            decoded.modeOverride = ModeOverride::ManualDefense;
            break;
          default:
            decoded.modeOverride = ModeOverride::Auto;
            break;
        }
        break;
      default:
        if (!skipField(data, len, offset, wireType)) return false;
        break;
    }
  }

  out = decoded;
  return true;
}

bool decodeTeamState(const std::vector<uint8_t>& data, TeamState& out) {
  return decodeTeamState(data.data(), data.size(), out);
}

std::vector<uint8_t> packTeamStateFrame(const TeamState& state) {
  const std::vector<uint8_t> payload = encodeTeamState(state);
  std::vector<uint8_t> framed;
  const uint32_t len = static_cast<uint32_t>(payload.size());
  framed.resize(4 + payload.size());
  std::memcpy(framed.data(), &len, sizeof(len));
  if (!payload.empty()) {
    std::memcpy(framed.data() + 4, payload.data(), payload.size());
  }
  return framed;
}

bool unpackTeamStateFrames(std::vector<uint8_t>& buffer, std::vector<TeamState>& out) {
  out.clear();

  while (buffer.size() >= 4) {
    uint32_t len = 0;
    std::memcpy(&len, buffer.data(), sizeof(len));
    if (len > kMaxTeamPacketBytes) {
      buffer.clear();
      return false;
    }
    if (buffer.size() < 4 + len) break;

    TeamState state;
    if (!decodeTeamState(buffer.data() + 4, len, state)) {
      buffer.clear();
      return false;
    }
    out.push_back(state);
    buffer.erase(buffer.begin(), buffer.begin() + 4 + len);
  }

  return !out.empty();
}

}  // namespace ballalgo

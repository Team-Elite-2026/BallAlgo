#include "motion/Protocol.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    std::exit(1);
  }
}

void testActionChunkCarriesKickAndDribblerBytes() {
  std::vector<ballalgo::MotionAction> actions(1);
  const std::vector<uint8_t> frame =
      ballalgo::packActionChunk(7, 99, 4, actions, 1, 0.0f, 0.0f, true, 1u, 123u);

  std::vector<uint8_t> buffer = frame;
  std::vector<ballalgo::ProtocolFrame> out;
  expect(ballalgo::unpackFrames(buffer, out), "packed action chunk should unpack");
  expect(out.size() == 1, "packed action chunk should yield one frame");
  expect(out.front().type == ballalgo::kMsgActionChunk,
         "unpacked frame should be an action chunk");
  expect(out.front().payload.size() >= 32,
         "action chunk payload should include actuator bytes");
  expect(out.front().payload[28] == 1u, "pose-valid byte should be set");
  expect(out.front().payload[29] == 1u, "kick byte should be serialized");
  expect(out.front().payload[30] == 123u, "dribbler byte should be serialized");
  expect(out.front().payload[31] == 0u, "padding byte should stay zero");
}

}  // namespace

int main() {
  testActionChunkCarriesKickAndDribblerBytes();
  std::cout << "ballalgo_protocol_tests passed\n";
  return 0;
}

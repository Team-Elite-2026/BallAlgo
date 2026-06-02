#include "sim/SimulationCore.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

bool hasModeFlag(const std::vector<std::string>& args) {
  for (size_t index = 1; index < args.size(); ++index) {
    if (args[index] == "--mode") return true;
  }
  return false;
}

std::vector<std::string> buildCompatArgs(int argc, char** argv) {
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(argc) + 2);
  for (int index = 0; index < argc; ++index) args.emplace_back(argv[index]);

  if (!hasModeFlag(args)) {
    args.emplace_back("--mode");
    args.emplace_back("pose_target");
  }
  return args;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args = buildCompatArgs(argc, argv);
  return ballalgo::sim::runBallalgoSimMain(args, std::cout, std::cerr);
}

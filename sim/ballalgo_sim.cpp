#include "sim/SimulationCore.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(argc));
  for (int index = 0; index < argc; ++index) args.emplace_back(argv[index]);
  return ballalgo::sim::runBallalgoSimMain(args, std::cout, std::cerr);
}

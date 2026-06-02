#pragma once

#include "sim/SimulationTypes.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace ballalgo::sim {

bool parseArgs(const std::vector<std::string>& args, InputOptions& options);
bool validateOptions(const InputOptions& options, std::ostream& err);
void printUsage(const char* argv0, std::ostream& err);

}  // namespace ballalgo::sim

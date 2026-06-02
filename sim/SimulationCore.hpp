#pragma once

#include <iosfwd>
#include <string>
#include <vector>

namespace ballalgo::sim {

int runBallalgoSimMain(const std::vector<std::string>& args, std::ostream& out,
                       std::ostream& err);

}  // namespace ballalgo::sim

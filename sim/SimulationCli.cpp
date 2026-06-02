#include "sim/SimulationCli.hpp"

#include <cstdlib>

namespace ballalgo::sim {

namespace {

bool parseFloatArg(const char* text, float& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

bool parseIntArg(const char* text, int& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  long parsed = std::strtol(text, &end, 10);
  if (end == text || end == nullptr || *end != '\0') return false;
  value = static_cast<int>(parsed);
  return true;
}

bool parseStringArg(const char* text, std::string& value) {
  if (text == nullptr) return false;
  value = text;
  return true;
}

bool parseModeArg(const char* text, SimulationMode& mode) {
  if (text == nullptr) return false;
  const std::string value = text;
  if (value == "production_ball_plan") {
    mode = SimulationMode::ProductionBallPlan;
    return true;
  }
  if (value == "pose_target" || value == "rolling_replan") {
    mode = SimulationMode::PoseTarget;
    return true;
  }
  if (value == "single_chunk") {
    mode = SimulationMode::SingleChunk;
    return true;
  }
  return false;
}

}  // namespace

bool parseArgs(const std::vector<std::string>& args, InputOptions& options) {
  for (size_t index = 1; index < args.size(); ++index) {
    const std::string& arg = args[index];

    auto requireFloat = [&](float& value) {
      if (index + 1 >= args.size()) return false;
      ++index;
      return parseFloatArg(args[index].c_str(), value);
    };
    auto requireInt = [&](int& value) {
      if (index + 1 >= args.size()) return false;
      ++index;
      return parseIntArg(args[index].c_str(), value);
    };
    auto requireString = [&](std::string& value) {
      if (index + 1 >= args.size()) return false;
      ++index;
      return parseStringArg(args[index].c_str(), value);
    };
    auto requireMode = [&](SimulationMode& mode) {
      if (index + 1 >= args.size()) return false;
      ++index;
      return parseModeArg(args[index].c_str(), mode);
    };

    if (arg == "--start-x-cm") {
      if (!requireFloat(options.startXCm)) return false;
      continue;
    }
    if (arg == "--start-y-cm") {
      if (!requireFloat(options.startYCm)) return false;
      continue;
    }
    if (arg == "--start-heading-deg") {
      if (!requireFloat(options.startHeadingDeg)) return false;
      continue;
    }
    if (arg == "--start-vx-mm-s") {
      if (!requireFloat(options.startVxMmS)) return false;
      continue;
    }
    if (arg == "--start-vy-mm-s") {
      if (!requireFloat(options.startVyMmS)) return false;
      continue;
    }
    if (arg == "--goal-x-cm") {
      if (!requireFloat(options.goalXCm)) return false;
      options.goalSpecified = true;
      options.goalXSpecified = true;
      continue;
    }
    if (arg == "--goal-y-cm") {
      if (!requireFloat(options.goalYCm)) return false;
      options.goalSpecified = true;
      options.goalYSpecified = true;
      continue;
    }
    if (arg == "--goal-heading-deg") {
      if (!requireFloat(options.goalHeadingDeg)) return false;
      options.goalSpecified = true;
      options.goalHeadingSpecified = true;
      continue;
    }
    if (arg == "--ball-x-cm") {
      if (!requireFloat(options.ballXCm)) return false;
      options.ballSpecified = true;
      options.ballXSpecified = true;
      continue;
    }
    if (arg == "--ball-y-cm") {
      if (!requireFloat(options.ballYCm)) return false;
      options.ballSpecified = true;
      options.ballYSpecified = true;
      continue;
    }
    if (arg == "--ball-vx-cm-s") {
      if (!requireFloat(options.ballVxCmS)) return false;
      options.ballSpecified = true;
      continue;
    }
    if (arg == "--ball-vy-cm-s") {
      if (!requireFloat(options.ballVyCmS)) return false;
      options.ballSpecified = true;
      continue;
    }
    if (arg == "--goal-target-x-cm") {
      if (!requireFloat(options.goalTargetXCm)) return false;
      options.goalTargetSpecified = true;
      options.goalTargetXSpecified = true;
      continue;
    }
    if (arg == "--goal-target-y-cm") {
      if (!requireFloat(options.goalTargetYCm)) return false;
      options.goalTargetSpecified = true;
      options.goalTargetYSpecified = true;
      continue;
    }
    if (arg == "--output") {
      if (!requireString(options.outputPath)) return false;
      continue;
    }
    if (arg == "--label") {
      if (!requireString(options.label)) return false;
      continue;
    }
    if (arg == "--mode") {
      if (!requireMode(options.mode)) return false;
      continue;
    }
    if (arg == "--control-hz") {
      if (!requireFloat(options.controlHz)) return false;
      continue;
    }
    if (arg == "--max-replans") {
      if (!requireInt(options.maxReplans)) return false;
      continue;
    }
    if (arg == "--max-sim-time-s") {
      if (!requireFloat(options.maxSimTimeS)) return false;
      continue;
    }
    return false;
  }
  return !options.outputPath.empty();
}

bool validateOptions(const InputOptions& options, std::ostream& err) {
  if (options.outputPath.empty()) {
    err << "error: --output is required\n";
    return false;
  }

  if (options.mode == SimulationMode::ProductionBallPlan) {
    if (!options.ballXSpecified || !options.ballYSpecified) {
      err << "error: production_ball_plan requires --ball-x-cm/--ball-y-cm inputs\n";
      return false;
    }
    if (!options.goalTargetXSpecified || !options.goalTargetYSpecified) {
      err << "error: production_ball_plan requires --goal-target-x-cm and --goal-target-y-cm\n";
      return false;
    }
    return true;
  }

  if (!options.goalXSpecified || !options.goalYSpecified || !options.goalHeadingSpecified) {
    err << "error: pose-target modes require --goal-x-cm, --goal-y-cm, and --goal-heading-deg\n";
    return false;
  }
  return true;
}

void printUsage(const char* argv0, std::ostream& err) {
  err << "Usage: " << argv0 << "\n"
      << "  --start-x-cm <value> --start-y-cm <value> --start-heading-deg <value>\n"
      << "  --output <artifact.json> [--label <name>]\n"
      << "  [--mode production_ball_plan|pose_target|single_chunk]\n"
      << "  [--start-vx-mm-s <value>] [--start-vy-mm-s <value>]\n"
      << "  [--control-hz <value>] [--max-replans <count>] [--max-sim-time-s <value>]\n"
      << "\n"
      << "Production route mode:\n"
      << "  --ball-x-cm <value> --ball-y-cm <value>\n"
      << "  [--ball-vx-cm-s <value>] [--ball-vy-cm-s <value>]\n"
      << "  --goal-target-x-cm <value> --goal-target-y-cm <value>\n"
      << "\n"
      << "Pose-target modes:\n"
      << "  --goal-x-cm <value> --goal-y-cm <value> --goal-heading-deg <value>\n";
}

}  // namespace ballalgo::sim

#include "camera/CameraCapture.hpp"
#include "config.hpp"
#include "FoxGloveSim/FoxgloveTelemetryPublisher.hpp"
#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/GpioLidar.hpp"
#include "io/RobotSerial.hpp"
#include "lidar/Ld19Reader.hpp"
#include "motion/ActionChunkPublisher.hpp"
#include "motion/MotionPipeline.hpp"
#include "vision/SectorTracker.hpp"
#include "vision/Thresholds.hpp"
#include "vision/VisionMath.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

using namespace ballalgo;

namespace {

constexpr double kDebugReportPeriodS = 1.0;

static std::string formatPerception(int ballAng, int ballDist, int blue, int yellow, int lx,
                                    int ly, int bvx, int bvy) {
  std::ostringstream ss;
  ss << ballAng << 'b' << ballDist << 'a' << blue << 'c' << yellow << 'd' << lx << 'e' << ly
     << 'f' << bvx << 'g' << bvy << 'i';
  return ss.str();
}

static int fieldMmToCenteredCm(float positionMm, float axisLimitMm) {
  return static_cast<int>(std::lround((positionMm - 0.5f * axisLimitMm) * 0.1f));
}

static float centeredCmToFieldMm(float centeredCm, float axisLimitMm) {
  return centeredCm * 10.f + 0.5f * axisLimitMm;
}

struct RuntimeOptions {
  bool commandGoalEnabled = false;
  CommandedPoseGoal commandGoal;
  bool commandGoalUsesCenteredCm = false;
};

bool parseFloatArg(const char* text, float& value) {
  if (text == nullptr) return false;
  char* end = nullptr;
  value = std::strtof(text, &end);
  return end != text && end != nullptr && *end == '\0';
}

bool parseArgs(int argc, char** argv, RuntimeOptions& options) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--command-goal") {
      if (i + 3 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], options.commandGoal.xMm) ||
          !parseFloatArg(argv[i + 2], options.commandGoal.yMm) ||
          !parseFloatArg(argv[i + 3], options.commandGoal.headingDeg)) {
        return false;
      }
      options.commandGoalEnabled = true;
      i += 3;
      continue;
    }
    if (arg == "--command-goal-centered-cm") {
      float xCm = 0;
      float yCm = 0;
      if (i + 3 >= argc) return false;
      if (!parseFloatArg(argv[i + 1], xCm) || !parseFloatArg(argv[i + 2], yCm) ||
          !parseFloatArg(argv[i + 3], options.commandGoal.headingDeg)) {
        return false;
      }
      options.commandGoal.xMm = centeredCmToFieldMm(xCm, config::kFieldWidthMm);
      options.commandGoal.yMm = centeredCmToFieldMm(yCm, config::kFieldHeightMm);
      options.commandGoalEnabled = true;
      options.commandGoalUsesCenteredCm = true;
      i += 3;
      continue;
    }
    return false;
  }
  return true;
}

void printUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " [--command-goal x_mm y_mm heading_deg]"
            << " [--command-goal-centered-cm x_cm y_cm heading_deg]\n";
}

}  // namespace

int main(int argc, char** argv) {
  RuntimeOptions options;
  if (!parseArgs(argc, argv, options)) {
    printUsage(argv[0]);
    return 2;
  }
  std::cout << "ballalgo C++ starting\n";
  if (options.commandGoalEnabled) {
    std::cout << "command-goal mode enabled"
              << (options.commandGoalUsesCenteredCm ? " [centered-cm input]" : " [field-mm input]")
              << ": target=("
              << fieldMmToCenteredCm(options.commandGoal.xMm, config::kFieldWidthMm) << ", "
              << fieldMmToCenteredCm(options.commandGoal.yMm, config::kFieldHeightMm)
              << ") centered-cm, field=(" << options.commandGoal.xMm << ", "
              << options.commandGoal.yMm << ") mm heading=" << options.commandGoal.headingDeg
              << " deg\n";
    if (!config::kEnableActionChunks) {
      std::cout << "warning: config::kEnableActionChunks is false, so no motion chunks will be sent\n";
    }
  }

  ThresholdsData thr;
  loadThresholds(config::kThresholdsJson, thr);

  CameraCapture camera;
  if (!camera.open()) {
    std::cerr << "ballalgo failed to open camera\n";
    return 1;
  }

  SectorTracker tracker;
  cv::Mat frame;
  cv::Mat roiMask;
  bool roiBuilt = false;
  auto buildRoi = [&]() {
    tracker.buildRoi(frame.size(), thr);
    roiBuilt = true;
    if (thr.hasMask) {
      roiMask = cv::Mat::zeros(frame.size(), CV_8UC1);
      cv::ellipse(roiMask, thr.maskCenter, thr.maskAxes, 0, 0, 360, 255, -1);
    } else {
      roiMask.release();
    }
  };
  if (camera.grab(frame)) buildRoi();

  GpioLidar gpio;
#ifdef BALLALGO_ENABLE_LIDAR
  if (config::kLidarPwmHoldLow) gpio.init(config::kLidarPwmGpio);
#endif

  std::unique_ptr<Ld19Reader> lidar;
#ifdef BALLALGO_ENABLE_LIDAR
  lidar = std::make_unique<Ld19Reader>(config::kLidarPort, config::kLidarBaud, 0.001);
#endif

  RobotSerial serial;
  if (config::kEnableSerial) serial.open(config::kSerialPort, config::kSerialBaud);

  MotionPipeline motion;
  ActionChunkPublisher actionChunkPublisher;
  FoxgloveTelemetryPublisher foxglove(config::kFoxgloveConfigPath);
  std::deque<LidarPoint> lidarWindow;
  std::vector<uint8_t> rxBuf;
  float headingDeg = 0;
  double debugAccumS = 0.0;
  unsigned long loopCount = 0;
  unsigned long frameGrabFailures = 0;

  auto lastTime = std::chrono::steady_clock::now();

  while (true) {
    ++loopCount;
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastTime).count();
    const uint64_t nowUs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count());
    lastTime = now;

    if (!camera.grab(frame)) {
      ++frameGrabFailures;
      debugAccumS += dt;
      if (debugAccumS >= kDebugReportPeriodS) {
        debugAccumS = 0.0;
        std::fprintf(stderr,
                     "[DBG] loop=%lu grab=fail failCount=%lu heading=%.2f lidarWindow=%zu\n",
                     loopCount, frameGrabFailures, headingDeg, lidarWindow.size());
        std::fflush(stderr);
      }
      continue;
    }
    if (!roiBuilt) buildRoi();

    cv::Mat maskedFrame;
    if (!roiMask.empty()) {
      maskedFrame = cv::Mat::zeros(frame.size(), frame.type());
      frame.copyTo(maskedFrame, roiMask);
    }
    const cv::Mat& displayFrame = maskedFrame.empty() ? frame : maskedFrame;

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat binBall, binY, binB;
    cv::inRange(hsv, thr.ball.lower, thr.ball.upper, binBall);
    cv::inRange(hsv, thr.yellowGoal.lower, thr.yellowGoal.upper, binY);
    cv::inRange(hsv, thr.blueGoal.lower, thr.blueGoal.upper, binB);
    if (!roiMask.empty()) {
      cv::bitwise_and(binBall, roiMask, binBall);
      cv::bitwise_and(binY, roiMask, binY);
      cv::bitwise_and(binB, roiMask, binB);
    }
    if (config::kUseMorph) {
      cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
      if (config::kBallCloseIters > 0) {
        cv::morphologyEx(binBall, binBall, cv::MORPH_CLOSE, k, cv::Point(-1, -1),
                         config::kBallCloseIters);
      }
      if (config::kBallDilateIters > 0) {
        cv::dilate(binBall, binBall, k, cv::Point(-1, -1), config::kBallDilateIters);
      }
      if (config::kMorphIters > 0) {
        cv::morphologyEx(binY, binY, cv::MORPH_OPEN, k, cv::Point(-1, -1), config::kMorphIters);
        cv::morphologyEx(binY, binY, cv::MORPH_CLOSE, k, cv::Point(-1, -1), config::kMorphIters);
        cv::morphologyEx(binB, binB, cv::MORPH_OPEN, k, cv::Point(-1, -1), config::kMorphIters);
        cv::morphologyEx(binB, binB, cv::MORPH_CLOSE, k, cv::Point(-1, -1), config::kMorphIters);
      }
    }

    auto ball = tracker.trackBall(binBall, thr.xoffset, thr.yoffset);
    auto goals = tracker.trackGoals(binY, binB, thr.xoffset, thr.yoffset);

    TeensyOdometry odo;
    odo.headingDeg = headingDeg;
    if (serial.isOpen()) {
      serial.pollOdometry(odo);
      headingDeg = odo.headingDeg;
    }

    // Step 1a: high-frequency dead-reckoning predict (mouse + IMU when fresh).
    motion.predictStep(odo, dt, nowUs);

    PoseState pose;
#ifdef BALLALGO_ENABLE_LIDAR
    if (lidar && lidar->isConnected()) {
      auto pts = lidar->pollPoints();
      if (!pts.empty()) {
        for (auto& p : pts) {
          lidarWindow.push_back(p);
          if (static_cast<int>(lidarWindow.size()) > config::kLidarPointsWindow) lidarWindow.pop_front();
        }
        std::vector<LidarPoint> window(lidarWindow.begin(), lidarWindow.end());
        pose = motion.updateLidar(window, headingDeg);  // Step 1a map snap
      } else {
        pose = motion.poseState(headingDeg);
      }
    }
#endif

    // Step 1b: dual-goal bearing-only correction of lateral (x,y) drift.
    {
      GoalBearingObs obs[2];
      auto fillObs = [&](GoalBearingObs& o, double angleDeg, double certainty, float gx, float gy) {
        if (angleDeg < 0) return;  // sentinel: goal not seen this frame
        double bodyX = 0;
        double bodyY = 0;
        polarToBodyXY(angleDeg, 1.0, bodyX, bodyY);
        o.bearingRad = std::atan2(bodyX, bodyY);
        o.goalXMm = gx;
        o.goalYMm = gy;
        o.certainty = certainty;
        o.valid = true;
      };
      fillObs(obs[0], goals.yellowAngle, goals.yellowCertainty, config::kYellowGoalXMm,
              config::kYellowGoalYMm);
      fillObs(obs[1], goals.blueAngle, goals.blueCertainty, config::kBlueGoalXMm,
              config::kBlueGoalYMm);
      motion.updateGoalBearings(obs, 2, headingDeg);
      pose = motion.poseState(headingDeg);
    }

    auto ballState = motion.updateBall(ball.angleDeg, ball.distCal, ball.found, dt);

    int lx = pose.valid ? fieldMmToCenteredCm(pose.xMm, config::kFieldWidthMm)
                        : config::kLostSentinel;
    int ly = pose.valid ? fieldMmToCenteredCm(pose.yMm, config::kFieldHeightMm)
                        : config::kLostSentinel;
    float goalDeg = goals.yellowAngle;
    if (goalDeg < 0) goalDeg = goals.blueAngle;

    bool offense = ball.found || ballState.visible;
    const CommandedPoseGoal* commandedGoal =
        options.commandGoalEnabled ? &options.commandGoal : nullptr;

    if (serial.isOpen()) {
      auto ascii = formatPerception(static_cast<int>(ball.angleDeg),
                                    static_cast<int>(ball.distCal),
                                    static_cast<int>(goals.blueAngle),
                                    static_cast<int>(goals.yellowAngle), lx, ly, ball.ballVxPx,
                                    ball.ballVyPx);
      serial.writeAscii(ascii);
    }
    actionChunkPublisher.publish(serial, rxBuf, pose, ballState, goalDeg, headingDeg, offense,
                                 commandedGoal);

    FoxgloveTelemetryFrame telemetry;
    telemetry.dtS = dt;
    telemetry.loopCount = loopCount;
    telemetry.frameGrabFailures = frameGrabFailures;
    telemetry.headingDeg = headingDeg;
    telemetry.pose = pose;
    telemetry.ball = ballState;
    telemetry.visionBallAngleDeg = ball.angleDeg;
    telemetry.visionBallDistance = ball.distCal;
    telemetry.planner = actionChunkPublisher.latestDebugSnapshot();
    if (foxglove.config().streamCamera && !frame.empty()) {
      cv::imencode(".jpg", displayFrame, telemetry.cameraJpegBytes,
                   {cv::IMWRITE_JPEG_QUALITY, 70});
      telemetry.ballPxFound = ball.found;
      if (ball.found && ball.distPx > 0) {
        // Invert angleAndDistance: angleDeg = atan2(midx,midy)*180/π+180
        // so midx = distPx*sin(θ), midy = distPx*cos(θ), then add offsets.
        const double theta = (ball.angleDeg - 180.0) * M_PI / 180.0;
        telemetry.ballPxCx =
            static_cast<int>(std::round(ball.distPx * std::sin(theta) + thr.xoffset));
        telemetry.ballPxCy =
            static_cast<int>(std::round(ball.distPx * std::cos(theta) + thr.yoffset));
      }
    }
    foxglove.publish(telemetry);

    debugAccumS += dt;
    if (debugAccumS >= kDebugReportPeriodS) {
      debugAccumS = 0.0;
      std::fprintf(stderr,
                   "[DBG] loop=%lu grab=ok failCount=%lu heading=%.2f ballFound=%d "
                   "ballAng=%.2f ballDist=%.2f blue=%.2f yellow=%.2f lidarWindow=%zu "
                   "poseValid=%d lx=%d ly=%d commandGoal=%d targetX=%.0f targetY=%.0f "
                   "targetLX=%d targetLY=%d "
                   "targetHeading=%.1f\n",
                   loopCount, frameGrabFailures, headingDeg, ball.found ? 1 : 0, ball.angleDeg,
                   ball.distCal, goals.blueAngle, goals.yellowAngle, lidarWindow.size(),
                   pose.valid ? 1 : 0, lx, ly, options.commandGoalEnabled ? 1 : 0,
                   options.commandGoal.xMm, options.commandGoal.yMm,
                   fieldMmToCenteredCm(options.commandGoal.xMm, config::kFieldWidthMm),
                   fieldMmToCenteredCm(options.commandGoal.yMm, config::kFieldHeightMm),
                   options.commandGoal.headingDeg);
      std::fflush(stderr);
    }
  }

  return 0;
}

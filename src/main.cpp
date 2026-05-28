#include "camera/CameraCapture.hpp"
#include "config.hpp"
#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/GpioLidar.hpp"
#include "io/RobotSerial.hpp"
#include "lidar/Ld19Reader.hpp"
#include "motion/ActionChunkPublisher.hpp"
#include "motion/MotionPipeline.hpp"
#include "vision/SectorTracker.hpp"
#include "vision/Thresholds.hpp"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <sstream>

using namespace ballalgo;

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

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  std::cout << "ballalgo C++ starting\n";

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
  std::deque<LidarPoint> lidarWindow;
  std::vector<uint8_t> rxBuf;
  float headingDeg = 0;

  auto lastTime = std::chrono::steady_clock::now();

  while (true) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastTime).count();
    lastTime = now;

    if (!camera.grab(frame)) continue;
    if (!roiBuilt) buildRoi();

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

    if (serial.isOpen()) serial.pollHeading(headingDeg);

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
        pose = motion.updateLidar(window, headingDeg, dt);
      } else {
        pose = motion.updateLidar({}, headingDeg, dt);
      }
    }
#endif

    auto ballState = motion.updateBall(ball.angleDeg, ball.distCal, ball.found, dt);

    int lx = pose.valid ? fieldMmToCenteredCm(pose.xMm, config::kFieldWidthMm)
                        : config::kLostSentinel;
    int ly = pose.valid ? fieldMmToCenteredCm(pose.yMm, config::kFieldHeightMm)
                        : config::kLostSentinel;
    float goalDeg = goals.yellowAngle;
    if (goalDeg < 0) goalDeg = goals.blueAngle;

    if (serial.isOpen()) {
      auto ascii = formatPerception(static_cast<int>(ball.angleDeg),
                                    static_cast<int>(ball.distCal),
                                    static_cast<int>(goals.blueAngle),
                                    static_cast<int>(goals.yellowAngle), lx, ly, ball.ballVxPx,
                                    ball.ballVyPx);
      serial.writeAscii(ascii);
      bool offense = ball.found || ballState.visible;
      actionChunkPublisher.publish(serial, rxBuf, pose, ballState, goalDeg, headingDeg, offense);
    }
  }

  return 0;
}

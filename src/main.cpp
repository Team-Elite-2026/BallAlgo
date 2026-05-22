#include "camera/CameraCapture.hpp"
#include "config.hpp"
#include "estimation/BallKalman.hpp"
#include "estimation/PoseKalman.hpp"
#include "io/GpioLidar.hpp"
#include "io/RobotSerial.hpp"
#include "lidar/Ld19Reader.hpp"
#include "motion/MotionPipeline.hpp"
#include "vision/SectorTracker.hpp"
#include "vision/Thresholds.hpp"

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <deque>
#include <iostream>
#include <sstream>

using namespace ballalgo;

static std::string formatPerception(int ballAng, int ballDist, int blue, int yellow, int lx,
                                    int ly, int bvx, int bvy) {
  std::ostringstream ss;
  ss << ballAng << 'b' << ballDist << 'a' << blue << 'c' << yellow << 'd' << lx << 'e' << ly
     << 'f' << bvx << 'g' << bvy << 'i';
  return ss.str();
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  std::cout << "ballalgo C++ starting\n";

  ThresholdsData thr;
  loadThresholds(config::kThresholdsJson, thr);

  CameraCapture camera;
  camera.open();

  SectorTracker tracker;
  cv::Mat frame;
  if (camera.grab(frame)) tracker.buildRoi(frame.size(), thr);

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
  std::deque<LidarPoint> lidarWindow;
  std::vector<uint8_t> rxBuf;
  float headingDeg = 0;

  auto lastTime = std::chrono::steady_clock::now();

  while (true) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastTime).count();
    lastTime = now;

    if (!camera.grab(frame)) continue;

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat binBall, binY, binB;
    cv::inRange(hsv, thr.ball.lower, thr.ball.upper, binBall);
    cv::inRange(hsv, thr.yellowGoal.lower, thr.yellowGoal.upper, binY);
    cv::inRange(hsv, thr.blueGoal.lower, thr.blueGoal.upper, binB);
    if (config::kUseMorph) {
      cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
      cv::morphologyEx(binBall, binBall, cv::MORPH_OPEN, k);
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

    int lx = pose.valid ? static_cast<int>(pose.xMm) : config::kLostSentinel;
    int ly = pose.valid ? static_cast<int>(pose.yMm) : config::kLostSentinel;
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
      motion.tickPublish(serial, rxBuf, pose, ballState, goalDeg, headingDeg, offense);
    }
  }

  return 0;
}

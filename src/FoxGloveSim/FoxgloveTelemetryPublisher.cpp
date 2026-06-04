#include "FoxGloveSim/FoxgloveTelemetryPublisher.hpp"

#include "config.hpp"
#include "motion/StrikePose.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace ballalgo {

namespace {

constexpr const char* kFrameId = "field";
constexpr double kNsPerS = 1e9;

// Uses system_clock (wall time) so MCAP timestamps match the Unix epoch
// expected by Foxglove Studio's timeline.
uint64_t nowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

float wrapAngleDeg(float angleDeg) {
  float wrapped = std::fmod(angleDeg + 180.f, 360.f);
  if (wrapped < 0.f) wrapped += 360.f;
  return wrapped - 180.f;
}

std::string escapeJson(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (char ch : text) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += ch;
        break;
    }
  }
  return out;
}

void bodyVelToField(float vxBody, float vyBody, float headingDeg, float& vxFieldMps,
                    float& vyFieldMps) {
  const float radians = headingDeg * static_cast<float>(M_PI) / 180.f;
  const float c = std::cos(radians);
  const float s = std::sin(radians);
  vxFieldMps = c * vxBody - s * vyBody;
  vyFieldMps = s * vxBody + c * vyBody;
}

struct JsonObjectState {
  bool first = true;
};

void addFieldPrefix(std::ostringstream& out, JsonObjectState& state, const char* key) {
  if (!state.first) out << ',';
  state.first = false;
  out << '"' << key << "\":";
}

void addBool(std::ostringstream& out, JsonObjectState& state, const char* key, bool value) {
  addFieldPrefix(out, state, key);
  out << (value ? "true" : "false");
}

void addNumber(std::ostringstream& out, JsonObjectState& state, const char* key, double value) {
  addFieldPrefix(out, state, key);
  out << value;
}

void addUnsigned(std::ostringstream& out, JsonObjectState& state, const char* key,
                 unsigned long value) {
  addFieldPrefix(out, state, key);
  out << value;
}

void addString(std::ostringstream& out, JsonObjectState& state, const char* key,
               const std::string& value) {
  addFieldPrefix(out, state, key);
  out << '"' << escapeJson(value) << '"';
}

std::string base64Encode(const uint8_t* data, size_t size) {
  static const char kChars[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((size + 2) / 3) * 4);
  for (size_t i = 0; i < size; i += 3) {
    const uint32_t n =
        (static_cast<uint32_t>(data[i]) << 16) |
        (i + 1 < size ? static_cast<uint32_t>(data[i + 1]) << 8 : 0u) |
        (i + 2 < size ? static_cast<uint32_t>(data[i + 2]) : 0u);
    out += kChars[(n >> 18) & 63];
    out += kChars[(n >> 12) & 63];
    out += (i + 1 < size) ? kChars[(n >> 6) & 63] : '=';
    out += (i + 2 < size) ? kChars[n & 63] : '=';
  }
  return out;
}

}  // namespace

FoxgloveTelemetryPublisher::FoxgloveTelemetryPublisher(const std::string& configPath)
    : config_(FoxgloveConfig::loadFromFile(configPath)), socketPath_(config_.socketPath) {
  if (config_.enabled) connectSocket();
}

FoxgloveTelemetryPublisher::~FoxgloveTelemetryPublisher() {
  closeSocket();
}

void FoxgloveTelemetryPublisher::closeSocket() {
  if (socketFd_ >= 0) {
    ::close(socketFd_);
    socketFd_ = -1;
  }
}

bool FoxgloveTelemetryPublisher::connectSocket() {
  closeSocket();
  socketFd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (socketFd_ < 0) return false;

  int flags = ::fcntl(socketFd_, F_GETFL, 0);
  if (flags >= 0) ::fcntl(socketFd_, F_SETFL, flags | O_NONBLOCK);

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", socketPath_.c_str());
  if (::connect(socketFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) return true;

  closeSocket();
  return false;
}

bool FoxgloveTelemetryPublisher::due(double hz, uint64_t nowNsValue, uint64_t& lastNs) const {
  if (hz <= 0.0) return false;
  if (lastNs == 0) {
    lastNs = nowNsValue;
    return true;
  }
  const uint64_t minDelta = static_cast<uint64_t>(kNsPerS / hz);
  if (nowNsValue - lastNs < minDelta) return false;
  lastNs = nowNsValue;
  return true;
}

void FoxgloveTelemetryPublisher::sendFrame(const std::string& payload) {
  if (payload.empty()) return;
  if (socketFd_ < 0 && !connectSocket()) return;

  const std::string framed = payload + "\n";
  const ssize_t sent = ::send(socketFd_, framed.data(), framed.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
  if (sent == static_cast<ssize_t>(framed.size())) return;

  if (sent < 0 &&
      (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS || errno == ENOENT ||
       errno == ECONNREFUSED || errno == ENOTCONN || errno == EPIPE)) {
    closeSocket();
    return;
  }

  if (sent >= 0 && sent < static_cast<ssize_t>(framed.size())) {
    closeSocket();
    return;
  }

  closeSocket();
}

void FoxgloveTelemetryPublisher::publish(const FoxgloveTelemetryFrame& frame) {
  // Gate on config only — socket state is managed by sendFrame so that
  // reconnection is attempted on every publish after a disconnect.
  if (!config_.enabled) return;

  const uint64_t timestampNs = nowNs();
  const bool sendPose = config_.streamPose && due(config_.poseHz, timestampNs, lastPoseNs_);
  const bool sendBall = config_.streamBall && due(config_.ballHz, timestampNs, lastBallNs_);
  const bool sendVelocity =
      config_.streamVelocity && due(config_.velocityHz, timestampNs, lastVelocityNs_);
  const bool sendPath = config_.streamPaths &&
                        frame.planner.valid &&
                        frame.planner.trajectoryId != lastPathTrajectoryId_ &&
                        due(config_.pathsHz, timestampNs, lastPathNs_);
  const bool sendLog = config_.streamLogs && due(config_.logsHz, timestampNs, lastLogNs_);
  const bool sendCamera = config_.streamCamera &&
                          !frame.cameraJpegBytes.empty() &&
                          due(config_.cameraHz, timestampNs, lastCameraNs_);

  if (!sendPose && !sendBall && !sendVelocity && !sendPath && !sendLog && !sendCamera) return;

  if (sendPath) lastPathTrajectoryId_ = frame.planner.trajectoryId;

  float robotVxFieldMps = 0.f;
  float robotVyFieldMps = 0.f;
  bodyVelToField(frame.pose.vxBody, frame.pose.vyBody, frame.headingDeg, robotVxFieldMps,
                 robotVyFieldMps);

  float robotAxBody = 0.f;
  float robotAyBody = 0.f;
  float robotAxField = 0.f;
  float robotAyField = 0.f;
  if (sendVelocity && havePrevRobotVel_ && frame.dtS > 1e-6) {
    robotAxBody = (frame.pose.vxBody - prevRobotVxBody_) / static_cast<float>(frame.dtS);
    robotAyBody = (frame.pose.vyBody - prevRobotVyBody_) / static_cast<float>(frame.dtS);
    robotAxField = (robotVxFieldMps - prevRobotVxField_) / static_cast<float>(frame.dtS);
    robotAyField = (robotVyFieldMps - prevRobotVyField_) / static_cast<float>(frame.dtS);
  }
  if (frame.pose.valid) {
    havePrevRobotVel_ = true;
    prevRobotVxBody_ = frame.pose.vxBody;
    prevRobotVyBody_ = frame.pose.vyBody;
    prevRobotVxField_ = robotVxFieldMps;
    prevRobotVyField_ = robotVyFieldMps;
  }

  float omegaDegS = 0.f;
  float alphaDegS2 = 0.f;
  if (sendVelocity && havePrevHeading_ && frame.dtS > 1e-6) {
    omegaDegS = wrapAngleDeg(frame.headingDeg - prevHeadingDeg_) / static_cast<float>(frame.dtS);
    alphaDegS2 = (omegaDegS - prevOmegaDegS_) / static_cast<float>(frame.dtS);
  }
  havePrevHeading_ = true;
  prevHeadingDeg_ = frame.headingDeg;
  prevOmegaDegS_ = omegaDegS;

  bool haveBallFieldPose = false;
  float ballXMm = 0.f;
  float ballYMm = 0.f;
  float ballVxFieldMps = 0.f;
  float ballVyFieldMps = 0.f;
  if (frame.pose.valid && frame.ball.visible) {
    ballFieldMm(frame.pose.xMm, frame.pose.yMm, frame.ball.xM, frame.ball.yM, frame.headingDeg,
                ballXMm, ballYMm);
    bodyVelToField(frame.ball.vx, frame.ball.vy, frame.headingDeg, ballVxFieldMps, ballVyFieldMps);
    haveBallFieldPose = true;
  }

  std::ostringstream out;
  out.precision(6);
  out << '{';
  JsonObjectState root;
  addNumber(out, root, "schema_version", 1);
  addUnsigned(out, root, "timestamp_ns", timestampNs);

  addFieldPrefix(out, root, "field");
  out << '{';
  JsonObjectState fieldState;
  addString(out, fieldState, "frame_id", kFrameId);
  addNumber(out, fieldState, "width_mm", config::kFieldWidthMm);
  addNumber(out, fieldState, "height_mm", config::kFieldHeightMm);
  out << '}';

  if (sendPose) {
    addFieldPrefix(out, root, "pose");
    out << '{';
    JsonObjectState state;
    addBool(out, state, "valid", frame.pose.valid);
    addNumber(out, state, "heading_deg", frame.headingDeg);
    addNumber(out, state, "x_mm", frame.pose.xMm);
    addNumber(out, state, "y_mm", frame.pose.yMm);
    addNumber(out, state, "vx_mm_s", frame.pose.vxMmS);
    addNumber(out, state, "vy_mm_s", frame.pose.vyMmS);
    addNumber(out, state, "vx_body_m_s", frame.pose.vxBody);
    addNumber(out, state, "vy_body_m_s", frame.pose.vyBody);
    out << '}';
  }

  if (sendBall) {
    addFieldPrefix(out, root, "ball");
    out << '{';
    JsonObjectState state;
    addBool(out, state, "visible", frame.ball.visible);
    addBool(out, state, "field_visible", haveBallFieldPose);
    addNumber(out, state, "body_x_m", frame.ball.xM);
    addNumber(out, state, "body_y_m", frame.ball.yM);
    addNumber(out, state, "body_vx_m_s", frame.ball.vx);
    addNumber(out, state, "body_vy_m_s", frame.ball.vy);
    addNumber(out, state, "field_x_mm", ballXMm);
    addNumber(out, state, "field_y_mm", ballYMm);
    addNumber(out, state, "field_vx_m_s", ballVxFieldMps);
    addNumber(out, state, "field_vy_m_s", ballVyFieldMps);
    addNumber(out, state, "vision_angle_deg", frame.visionBallAngleDeg);
    addNumber(out, state, "vision_dist_cal", frame.visionBallDistance);
    out << '}';
  }

  if (sendVelocity) {
    addFieldPrefix(out, root, "robot_twist");
    out << '{';
    JsonObjectState state;
    addNumber(out, state, "vx_body_m_s", frame.pose.vxBody);
    addNumber(out, state, "vy_body_m_s", frame.pose.vyBody);
    addNumber(out, state, "speed_body_m_s",
              std::hypot(frame.pose.vxBody, frame.pose.vyBody));
    addNumber(out, state, "vx_field_m_s", robotVxFieldMps);
    addNumber(out, state, "vy_field_m_s", robotVyFieldMps);
    addNumber(out, state, "speed_field_m_s",
              std::hypot(robotVxFieldMps, robotVyFieldMps));
    out << '}';

    addFieldPrefix(out, root, "robot_accel");
    out << '{';
    JsonObjectState accelState;
    addNumber(out, accelState, "ax_body_m_s2", robotAxBody);
    addNumber(out, accelState, "ay_body_m_s2", robotAyBody);
    addNumber(out, accelState, "magnitude_body_m_s2", std::hypot(robotAxBody, robotAyBody));
    addNumber(out, accelState, "ax_field_m_s2", robotAxField);
    addNumber(out, accelState, "ay_field_m_s2", robotAyField);
    addNumber(out, accelState, "magnitude_field_m_s2",
              std::hypot(robotAxField, robotAyField));
    out << '}';

    addFieldPrefix(out, root, "robot_angular");
    out << '{';
    JsonObjectState angularState;
    addNumber(out, angularState, "heading_deg", frame.headingDeg);
    addNumber(out, angularState, "omega_deg_s", omegaDegS);
    addNumber(out, angularState, "alpha_deg_s2", alphaDegS2);
    out << '}';

    addFieldPrefix(out, root, "ball_twist");
    out << '{';
    JsonObjectState ballTwistState;
    addBool(out, ballTwistState, "visible", frame.ball.visible);
    addNumber(out, ballTwistState, "vx_body_m_s", frame.ball.vx);
    addNumber(out, ballTwistState, "vy_body_m_s", frame.ball.vy);
    addNumber(out, ballTwistState, "speed_body_m_s",
              std::hypot(frame.ball.vx, frame.ball.vy));
    addNumber(out, ballTwistState, "vx_field_m_s", ballVxFieldMps);
    addNumber(out, ballTwistState, "vy_field_m_s", ballVyFieldMps);
    addNumber(out, ballTwistState, "speed_field_m_s",
              std::hypot(ballVxFieldMps, ballVyFieldMps));
    out << '}';
  }

  if (sendPath) {
    addFieldPrefix(out, root, "planner_path");
    out << '{';
    JsonObjectState state;
    addBool(out, state, "commanded_goal_mode", frame.planner.commandedGoalMode);
    addBool(out, state, "within_tolerance", frame.planner.withinTolerance);
    addBool(out, state, "used_center_fallback", frame.planner.usedCenterFallback);
    addBool(out, state, "used_body_chase_fallback", frame.planner.usedBodyChaseFallback);
    addBool(out, state, "used_strike_pose_plan", frame.planner.usedStrikePosePlan);
    addUnsigned(out, state, "trajectory_id", frame.planner.trajectoryId);
    addNumber(out, state, "dt_ms", frame.planner.dtMs);
    addNumber(out, state, "target_x_mm", frame.planner.targetXMm);
    addNumber(out, state, "target_y_mm", frame.planner.targetYMm);
    addNumber(out, state, "target_heading_deg", frame.planner.targetHeadingDeg);
    addFieldPrefix(out, state, "points");
    out << '[';
    bool firstPoint = true;
    for (const auto& sample : frame.planner.path) {
      if (!firstPoint) out << ',';
      firstPoint = false;
      out << '{'
          << "\"x_mm\":" << sample.xMm << ','
          << "\"y_mm\":" << sample.yMm << ','
          << "\"heading_deg\":" << sample.thetaDeg << ','
          << "\"s_mm\":" << sample.sMm << '}';
    }
    out << ']';
    out << '}';
  }

  if (sendLog) {
    addFieldPrefix(out, root, "log");
    out << '{';
    JsonObjectState state;
    addNumber(out, state, "level", 2);
    addString(out, state, "name", "ballalgo");
    addString(out, state, "message",
              "loop=" + std::to_string(frame.loopCount) + " heading=" +
                  std::to_string(frame.headingDeg) + " pose_valid=" +
                  std::string(frame.pose.valid ? "true" : "false") + " ball_visible=" +
                  std::string(frame.ball.visible ? "true" : "false") + " failures=" +
                  std::to_string(frame.frameGrabFailures));
    out << '}';
  }

  if (sendCamera) {
    addFieldPrefix(out, root, "camera");
    out << '{';
    JsonObjectState state;
    addString(out, state, "format", "jpeg");
    addString(out, state, "frame_id", "camera");
    addString(out, state, "data_b64",
              base64Encode(frame.cameraJpegBytes.data(), frame.cameraJpegBytes.size()));
    // Ball pixel annotation for the image overlay
    addFieldPrefix(out, state, "ball_px");
    out << '{';
    JsonObjectState bpx;
    addBool(out, bpx, "found", frame.ballPxFound);
    addNumber(out, bpx, "cx", frame.ballPxCx);
    addNumber(out, bpx, "cy", frame.ballPxCy);
    out << '}';
    out << '}';
  }

  out << '}';
  sendFrame(out.str());
}

}  // namespace ballalgo

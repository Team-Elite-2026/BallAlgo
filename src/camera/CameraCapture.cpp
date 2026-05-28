#include "camera/CameraCapture.hpp"

#include "config.hpp"

#include <iostream>

namespace ballalgo {

struct CameraCapture::Impl {
  cv::VideoCapture cap;
};

CameraCapture::CameraCapture() : impl_(std::make_unique<Impl>()) {}
CameraCapture::~CameraCapture() { close(); }

bool CameraCapture::open() {
#if defined(BALLALGO_HAS_LIBCAMERA)
  impl_->cap.open("/dev/video0", cv::CAP_V4L2);
  if (!impl_->cap.isOpened()) {
    impl_->cap.open("/dev/video0");
  }
#else
  constexpr int kDefaultCameraIndex = 0;
  impl_->cap.open(kDefaultCameraIndex, cv::CAP_V4L2);
  if (!impl_->cap.isOpened()) {
    impl_->cap.open(kDefaultCameraIndex);
  }
#endif
  if (!impl_->cap.isOpened()) {
#if defined(BALLALGO_STUB_BUILD)
    std::cerr << "[CAM] Open failed, using stub frames\n";
    return true;
#else
    std::cerr << "[CAM] Open failed\n";
    return false;
#endif
  }
  impl_->cap.set(cv::CAP_PROP_FRAME_WIDTH, config::kFrameWidth);
  impl_->cap.set(cv::CAP_PROP_FRAME_HEIGHT, config::kFrameHeight);
  impl_->cap.set(cv::CAP_PROP_FPS, config::kCameraFps);
  impl_->cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
  // Best-effort manual tuning to match the lidar branch's runtime camera setup.
  impl_->cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1.0);
  impl_->cap.set(cv::CAP_PROP_EXPOSURE, config::kExposureUs);
  impl_->cap.set(cv::CAP_PROP_GAIN, config::kCameraGain);
  impl_->cap.set(cv::CAP_PROP_BRIGHTNESS, config::kCameraBrightness);
  impl_->cap.set(cv::CAP_PROP_CONTRAST, config::kCameraContrast);
  impl_->cap.set(cv::CAP_PROP_SATURATION, config::kCameraSaturation);
  impl_->cap.set(cv::CAP_PROP_AUTO_WB, 0.0);
  impl_->cap.set(cv::CAP_PROP_WB_TEMPERATURE, config::kCameraWhiteBalance);
  impl_->cap.set(cv::CAP_PROP_AUTOFOCUS, 0.0);
  impl_->cap.set(cv::CAP_PROP_FOCUS, config::kCameraFocus);
#if defined(BALLALGO_HAS_LIBCAMERA)
  std::cout << "[CAM] V4L2 via libcamera stack " << config::kFrameWidth << "x" << config::kFrameHeight
            << "\n";
#else
  std::cout << "[CAM] Stub/dev capture\n";
#endif
  return true;
}

bool CameraCapture::grab(cv::Mat& bgrOut) {
  if (impl_->cap.isOpened()) {
    return impl_->cap.read(bgrOut) && !bgrOut.empty();
  }
#if defined(BALLALGO_STUB_BUILD)
  bgrOut = cv::Mat(config::kFrameHeight, config::kFrameWidth, CV_8UC3, cv::Scalar(30, 30, 30));
  return true;
#else
  bgrOut.release();
  return false;
#endif
}

void CameraCapture::close() {
  if (impl_->cap.isOpened()) impl_->cap.release();
}

}  // namespace ballalgo

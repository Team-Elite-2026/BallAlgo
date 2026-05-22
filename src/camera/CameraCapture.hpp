#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>

namespace ballalgo {

class CameraCapture {
 public:
  CameraCapture();
  ~CameraCapture();

  bool open();
  bool grab(cv::Mat& bgrOut);
  void close();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ballalgo

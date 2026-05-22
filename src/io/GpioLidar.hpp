#pragma once

namespace ballalgo {

class GpioLidar {
 public:
  bool init(int pwmGpioBcm);
  void cleanup();

 private:
  int lineFd_ = -1;
  int gpio_ = -1;
};

}  // namespace ballalgo

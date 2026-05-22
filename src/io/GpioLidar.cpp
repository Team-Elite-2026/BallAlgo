#include "io/GpioLidar.hpp"

#include <iostream>

#if defined(BALLALGO_HAS_GPIOD)
#include <gpiod.h>
#endif

namespace ballalgo {

bool GpioLidar::init(int pwmGpioBcm) {
  gpio_ = pwmGpioBcm;
#if defined(BALLALGO_HAS_GPIOD)
  struct gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip0");
  if (!chip) return false;
  struct gpiod_line* line = gpiod_chip_get_line(chip, pwmGpioBcm);
  if (!line || gpiod_line_request_output(line, "ballalgo_lidar_pwm", 0) < 0) {
    gpiod_chip_close(chip);
    return false;
  }
  lineFd_ = 1;
  gpiod_chip_close(chip);
  std::cout << "[GPIO] LD19 PWM pin " << pwmGpioBcm << " LOW\n";
  return true;
#else
  (void)pwmGpioBcm;
  std::cout << "[GPIO] stub (no libgpiod)\n";
  return true;
#endif
}

void GpioLidar::cleanup() {
#if defined(BALLALGO_HAS_GPIOD)
  (void)lineFd_;
#endif
  lineFd_ = -1;
}

}  // namespace ballalgo

#include "io/GpioLidar.hpp"

#include <iostream>

#if defined(BALLALGO_HAS_GPIOD)
#include <gpiod.h>
#endif

namespace ballalgo {

bool GpioLidar::init(int pwmGpioBcm) {
  gpio_ = pwmGpioBcm;
#if defined(BALLALGO_HAS_GPIOD)
  cleanup();

  chip_ = gpiod_chip_open("/dev/gpiochip0");
  if (!chip_) return false;

  auto* requestConfig = gpiod_request_config_new();
  auto* lineConfig = gpiod_line_config_new();
  auto* lineSettings = gpiod_line_settings_new();
  if (!requestConfig || !lineConfig || !lineSettings) {
    if (lineSettings) gpiod_line_settings_free(lineSettings);
    if (lineConfig) gpiod_line_config_free(lineConfig);
    if (requestConfig) gpiod_request_config_free(requestConfig);
    cleanup();
    return false;
  }

  const unsigned int offset = static_cast<unsigned int>(pwmGpioBcm);
  gpiod_request_config_set_consumer(requestConfig, "ballalgo_lidar_pwm");
  if (gpiod_line_settings_set_direction(lineSettings, GPIOD_LINE_DIRECTION_OUTPUT) < 0 ||
      gpiod_line_settings_set_output_value(lineSettings, GPIOD_LINE_VALUE_INACTIVE) < 0 ||
      gpiod_line_config_add_line_settings(lineConfig, &offset, 1, lineSettings) < 0) {
    gpiod_line_settings_free(lineSettings);
    gpiod_line_config_free(lineConfig);
    gpiod_request_config_free(requestConfig);
    cleanup();
    return false;
  }

  request_ = gpiod_chip_request_lines(chip_, requestConfig, lineConfig);
  gpiod_line_settings_free(lineSettings);
  gpiod_line_config_free(lineConfig);
  gpiod_request_config_free(requestConfig);
  if (!request_) {
    cleanup();
    return false;
  }

  lineFd_ = gpiod_line_request_get_fd(request_);
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
  if (request_) {
    gpiod_line_request_release(request_);
    request_ = nullptr;
  }
  if (chip_) {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
#endif
  lineFd_ = -1;
}

}  // namespace ballalgo

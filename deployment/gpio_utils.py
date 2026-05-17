import config as cfg


def setup_lidar_pwm_ground():
    if not cfg.LIDAR_PWM_HOLD_LOW:
        return None

    try:
        import RPi.GPIO as GPIO  # type: ignore
    except Exception as exc:
        print(f"[GPIO] RPi.GPIO unavailable, cannot hold GPIO{cfg.LIDAR_PWM_GPIO} LOW: {exc}")
        return None

    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cfg.LIDAR_PWM_GPIO, GPIO.OUT, initial=GPIO.LOW)
        print(f"[GPIO] Holding GPIO{cfg.LIDAR_PWM_GPIO} LOW for LD19 PWM")
        return GPIO
    except Exception as exc:
        print(f"[GPIO] Failed to configure GPIO{cfg.LIDAR_PWM_GPIO} LOW: {exc}")
        return None


def cleanup_gpio(gpio_handle):
    if gpio_handle is None:
        return
    try:
        gpio_handle.cleanup(cfg.LIDAR_PWM_GPIO)
    except Exception:
        pass

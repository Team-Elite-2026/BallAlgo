from py_lidar_bench import config as cfg


def setup_lidar_pwm_ground():
    if not cfg.LIDAR_PWM_HOLD_LOW:
        return None
    try:
        import RPi.GPIO as GPIO
    except Exception as exc:
        print(f"[GPIO] RPi.GPIO unavailable: {exc}")
        return None
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(cfg.LIDAR_PWM_GPIO, GPIO.OUT, initial=GPIO.LOW)
        return GPIO
    except Exception as exc:
        print(f"[GPIO] setup failed: {exc}")
        return None


def cleanup_gpio(gpio_handle):
    if gpio_handle is None:
        return
    try:
        gpio_handle.cleanup(cfg.LIDAR_PWM_GPIO)
    except Exception:
        pass

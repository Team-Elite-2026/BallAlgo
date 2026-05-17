import os


def _env_flag(name, default):
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


SERIAL_PORT = "/dev/serial0"  # Pi hardware UART
SERIAL_BAUD = 2000000
SERIAL_TIMEOUT = 0.01
ENABLE_SERIAL = True

# Set BALLALGO_ENABLE_LIDAR=0 on robots without LD19 hardware.
ENABLE_LIDAR = _env_flag("BALLALGO_ENABLE_LIDAR", True)
# LD19 wiring:
# - LD19 TX -> Pi RX on GPIO13 (UART4 RX on Pi 5/CM5)
# - LD19 PWM -> GPIO12 (held LOW to keep internal speed control active)
# Requires dtoverlay=uart4-pi5 in /boot/firmware/config.txt.
LIDAR_PORT = "/dev/ttyAMA4"
LIDAR_BAUD = 230400
LIDAR_TIMEOUT = 0.001
LIDAR_RX_GPIO = 13
LIDAR_PWM_GPIO = 12
LIDAR_PWM_HOLD_LOW = True

# Field dimensions in millimeters.
FIELD_WIDTH_MM = 3600.0
FIELD_HEIGHT_MM = 2400.0
LIDAR_YAW_OFFSET_DEG = 0.0

# Fallback heading (degrees) when Teensy UART is disabled or no 'h' message yet.
# Live heading is compassSensor.currentOffset() from the Teensy, sent as "{deg}h".
ROBOT_HEADING_DEG = 0.0

VIDEO_PATH = "output.mp4"
USE_JSON_THRESHOLDS = True
THRESHOLDS_JSON = "thresholds.json"
RESIZE_TO = (655, 600)

SAVE_ANNOTATED = True
OUT_VIDEO = "annotated_sector_detection3.mp4"
CSV_PATH = "sector_performance_data3.csv"

# HSV fallback (OpenCV ranges)
FALLBACK_LOWER = (5, 120, 120)
FALLBACK_UPPER = (25, 255, 255)

# Elliptical ROI fallback
ENABLE_MASK = True
MASK_CENTER = (330, 300)
MASK_AXES = (280, 230)

# Sectors
NUM_SECTORS = 12
SECTOR_ANGLE = 360 // NUM_SECTORS

# Processing knobs
USE_MORPH = True
MORPH_ITERS = 1
MIN_AREA_PIX = 5

# Prediction knobs
VEL_ALPHA = 0.7  # EMA for velocity
VEL_MAX = 40.0  # px/frame clamp
LOOKAHEAD_MIN = 1  # frames
LOOKAHEAD_MAX = 3  # frames
LOOKAHEAD_SPEED_THRESH = 12.0  # px/frame: above this, add more lookahead

# Stop as soon as we find a valid blob in a sector.
STOP_ON_FIRST_HIT = True

# Colors (BGR)
COLOR_UNSEARCHED = (0, 0, 255)  # red
COLOR_SEARCHED = (0, 165, 255)  # orange
COLOR_PREDICTED = (128, 0, 128)  # purple
COLOR_FOUND = (0, 255, 0)  # green

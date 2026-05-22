# BallAlgo bench tools (Python)

Not used in the competition loop. Production runtime is the C++ `ballalgo` binary.

## hsv_picker.py

Interactive HSV tuning; writes `../thresholds.json`.

```bash
cd tools
python3 hsv_picker.py
```

Requires: `python3-picamera2`, `python3-opencv`.

## lidar_visual.py

LD19 scan + wall localizer debug UI (no camera, no Teensy UART).

```bash
cd tools
python3 lidar_visual.py
```

Requires: `python3-opencv`, `pyserial`, optional `RPi.GPIO` for LD19 PWM pin.

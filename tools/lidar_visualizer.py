"""Compatibility entrypoint for the LD19 bench visualizer.

The legacy branch named this script ``lidar_visual.py``.  Keep that file intact
and expose the clearer visualizer name used by the Python runtime docs/tools.
"""

from lidar_visual import run


if __name__ == "__main__":
    run()

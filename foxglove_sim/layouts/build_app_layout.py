"""Generate a Foxglove *app-importable* layout (the format the desktop app's
'Import from file' reads — configById + a mosaic layout tree).

This is intentionally separate from build_layout.py, whose foxglove.layouts SDK
output ({version, content}) is the notebook/platform format and does NOT import
into the desktop app.

Output: ballalgo_app_layout.json

This layout is for the current legacy-control branch:
- one top-down field panel for the robot pose, ball pose, and live vectors
- one angle plot for graphing robot heading + ball angle relative to the robot
- one range plot for graphing ball distance

The old planner/path/trajectory panels were removed because those topics do not
exist on this branch anymore.
"""
from __future__ import annotations

import json
from pathlib import Path


LAYOUT_DIR = Path(__file__).resolve().parent
OUTPUT_PATH = LAYOUT_DIR / "ballalgo_app_layout.json"

BACKGROUND_COLOR = "#101215"

# Stable panel ids (the suffix after ! is arbitrary but must be unique).
FIELD_3D = "3D!field"
PLOT_ANGLES = "Plot!angles"
PLOT_RANGE = "Plot!range"
TAB_ROOT = "Tab!root"


def field_panel_config() -> dict:
    return {
        # Orthographic, straight down (phi=0), no azimuth rotation (thetaOffset=0)
        # => the rectangular field renders axis-aligned, never tilted/rotated.
        "cameraState": {
            "perspective": False,
            "distance": 3.2,
            "phi": 0,
            "thetaOffset": 0,
            "target": [0, 0, 0],
            "targetOffset": [0, 0, 0],
            "targetOrientation": [0, 0, 0, 1],
            "fovy": 45,
            "near": 0.01,
            "far": 5000,
        },
        "followMode": "follow-none",
        "followTf": "field",
        "scene": {
            "backgroundColor": BACKGROUND_COLOR,
            "transforms": {"enabled": False},
        },
        "transforms": {},
        "topics": {
            "/field/scene/static": {"visible": True},
            "/field/scene/live": {"visible": True},
            "/lidar/scan": {
                "visible": False,
                "colorMode": "flat",
                "flatColor": "#50c850",
                "pointSize": 3,
            },
        },
        "layers": {},
        "imageMode": {},
    }


def _plot_path(value: str, label: str, color: str) -> dict:
    return {
        "value": value,
        "enabled": True,
        "timestampMethod": "receiveTime",
        "label": label,
        "color": color,
    }


def angles_plot_config() -> dict:
    return {
        "paths": [
            _plot_path("/robot/angular.heading_deg", "robot heading", "#20d8ff"),
            _plot_path("/control/intent.ball_angle_deg", "ball angle rel robot", "#f79a0c"),
        ],
        "showLegend": True,
        "legendDisplay": "floating",
        "showPlotValuesInLegend": True,
        "isSynced": True,
        "xAxisVal": "timestamp",
        "sidebarDimension": 240,
        "followingViewWidth": 10,
    }


def range_plot_config() -> dict:
    return {
        "paths": [
            _plot_path("/control/intent.ball_distance_cm", "ball distance (cm)", "#4cf5ae"),
        ],
        "showLegend": True,
        "legendDisplay": "floating",
        "showPlotValuesInLegend": True,
        "isSynced": True,
        "xAxisVal": "timestamp",
        "sidebarDimension": 240,
        "followingViewWidth": 10,
    }


def build_layout() -> dict:
    return {
        "configById": {
            FIELD_3D: field_panel_config(),
            PLOT_ANGLES: angles_plot_config(),
            PLOT_RANGE: range_plot_config(),
            TAB_ROOT: {
                "activeTabIdx": 0,
                "tabs": [
                    {
                        "title": "Field",
                        "layout": {
                            "direction": "row",
                            "first": FIELD_3D,
                            "second": {
                                "direction": "column",
                                "first": PLOT_ANGLES,
                                "second": PLOT_RANGE,
                                "splitPercentage": 50,
                            },
                            "splitPercentage": 70,
                        },
                    }
                ],
            },
        },
        "globalVariables": {},
        "userNodes": {},
        "linkedGlobalVariables": [],
        "playbackConfig": {"speed": 1, "messageOrder": "receiveTime"},
        "layout": TAB_ROOT,
    }


def main() -> int:
    OUTPUT_PATH.write_text(json.dumps(build_layout(), indent=2) + "\n", encoding="utf-8")
    print(OUTPUT_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

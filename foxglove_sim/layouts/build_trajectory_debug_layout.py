from __future__ import annotations

import json
from pathlib import Path


LAYOUT_DIR = Path(__file__).resolve().parent
OUTPUT_PATH = LAYOUT_DIR / "trajectory_debug_layout.json"


def plot_series(value: str, label: str, color: str, *, right_axis: bool = False) -> dict:
    return {
        "value": value,
        "color": color,
        "label": label,
        "timestampMethod": "receiveTime",
        "showLine": True,
        "lineSize": 2.0,
        "useRightYAxis": right_axis,
    }


def plot_panel(title: str, paths: list[dict], *, y_axis_label: str,
               right_axis_label: str | None = None) -> dict:
    config = {
        "paths": paths,
        "showLegend": True,
        "legendDisplay": "floating",
        "showPlotValuesInLegend": True,
        "showXAxisLabels": True,
        "showYAxisLabels": True,
        "isSynced": True,
        "xAxisVal": "timestamp",
        "xAxisLabel": "Time (s)",
        "timeWindowMode": "sliding",
        "followingViewWidth": 10.0,
        "yAxisLabel": y_axis_label,
    }
    if right_axis_label is not None:
        config["rightYAxis"] = {
            "label": right_axis_label,
            "showTickLabels": True,
            "showGridLines": False,
        }
    return {
        "type": "panel",
        "panelType": "Plot",
        "title": title,
        "version": 1,
        "config": config,
    }


def raw_panel(title: str, topic: str) -> dict:
    return {
        "type": "panel",
        "panelType": "RawMessages",
        "title": title,
        "version": 1,
        "config": {
            "topicPath": topic,
            "expansion": "all",
            "fontSize": 12,
        },
    }


def log_panel() -> dict:
    return {
        "type": "panel",
        "panelType": "Log",
        "title": "Debug Log",
        "version": 1,
        "config": {
            "minLogLevel": 1,
            "topicToRender": "/debug/log",
            "fontSize": 12,
            "showLevel": True,
            "showDate": False,
            "showTime": True,
            "showTimezone": False,
            "showName": False,
            "showSourceLocation": False,
        },
    }


def field_panel() -> dict:
    return {
        "type": "panel",
        "panelType": "ThreeDee",
        "title": "Trajectory Field",
        "version": 1,
        "config": {
            "scene": {
                "backgroundColor": "#101215",
                "transforms": {
                    "visible": False,
                    "showLabel": False,
                    "axisSize": 0.2,
                    "lineWidth": 1.0,
                },
                "syncCamera": False,
            },
            "topics": {
                "/field/scene/static": {
                    "visible": True,
                    "showOutlines": False,
                    "computeVertexNormals": False,
                },
                "/field/scene/live": {
                    "visible": True,
                    "showOutlines": False,
                    "selectedIdVariable": "selected_id",
                    "computeVertexNormals": False,
                },
                "/planner/scene/path": {
                    "visible": True,
                    "showOutlines": False,
                    "computeVertexNormals": False,
                },
            },
            "syncedTopics": {
                "/field/scene/static": False,
                "/field/scene/live": True,
                "/planner/scene/path": True,
            },
            "cameraState": {
                "distance": 2.6,
                "perspective": False,
                "phi": 0.0,
                "target": [0.0, 0.0, 0.0],
                "thetaOffset": 0.0,
                "near": 0.05,
                "far": 50.0,
                "logDepth": False,
            },
            "followTf": "field",
            "fixedFrame": "field",
            "followMode": "follow-none",
        },
    }


def split(direction: str, items: list[dict]) -> dict:
    return {"type": "split", "direction": direction, "items": items}


def item(content: dict, proportion: float = 1.0) -> dict:
    return {"proportion": proportion, "content": content}


def build_layout() -> dict:
    target_vs_actual = plot_panel(
        "Target vs Measured Velocity",
        [
            plot_series("/traj/target.vx_body_target_m_s", "target vx body", "#4e98e2"),
            plot_series("/traj/target.vy_body_target_m_s", "target vy body", "#f5774d"),
            plot_series("/robot/twist.vx_body_m_s", "measured vx body", "#9ed3ff"),
            plot_series("/robot/twist.vy_body_m_s", "measured vy body", "#ffb68c"),
        ],
        y_axis_label="m/s",
    )
    angular_compare = plot_panel(
        "Target vs Measured Omega",
        [
            plot_series("/traj/target.omega_rad_s", "target omega", "#4cf5ae"),
            plot_series("/traj/teensy_raw.omega_rad_s", "measured omega", "#f7df71"),
        ],
        y_axis_label="rad/s",
    )
    velocity_error = plot_panel(
        "Velocity Tracking Error",
        [
            plot_series("/traj/error.vx_body_error_m_s", "vx body err", "#4e98e2"),
            plot_series("/traj/error.vy_body_error_m_s", "vy body err", "#f5774d"),
            plot_series("/traj/error.speed_body_error_m_s", "speed err", "#f7df71"),
            plot_series("/traj/error.omega_error_rad_s", "omega err", "#c84fe3", right_axis=True),
        ],
        y_axis_label="m/s",
        right_axis_label="rad/s",
    )
    accel_compare = plot_panel(
        "Target vs Measured Accel",
        [
            plot_series("/traj/target.ax_body_target_m_s2", "target ax", "#4e98e2"),
            plot_series("/traj/target.ay_body_target_m_s2", "target ay", "#f5774d"),
            plot_series("/robot/accel.ax_body_m_s2", "measured ax", "#9ed3ff"),
            plot_series("/robot/accel.ay_body_m_s2", "measured ay", "#ffb68c"),
        ],
        y_axis_label="m/s^2",
    )
    profile_panel = {
        "type": "panel",
        "panelType": "Plot",
        "title": "Trajectory Speed Profile",
        "version": 1,
        "config": {
            "paths": [
                plot_series("/planner/profile.samples[:].speed_m_s", "path speed", "#4cf5ae"),
            ],
            "showLegend": True,
            "legendDisplay": "floating",
            "showPlotValuesInLegend": True,
            "showXAxisLabels": True,
            "showYAxisLabels": True,
            "xAxisVal": "custom",
            "xAxisPath": {"value": "/planner/profile.samples[:].progress_01"},
            "timeRange": "latest",
            "xAxisLabel": "Trajectory Progress (0-1)",
            "yAxisLabel": "m/s",
        },
    }

    overview_right = split(
        "column",
        [
            item(split("row", [item(target_vs_actual), item(angular_compare)]), 1.3),
            item(split("row", [item(accel_compare), item(velocity_error)]), 1.3),
            item(split("row", [item(profile_panel), item(log_panel())]), 1.0),
        ],
    )
    overview = split("row", [item(field_panel(), 1.8), item(overview_right, 1.4)])

    raw_tab = split(
        "row",
        [
            item(
                split(
                    "column",
                    [
                        item(raw_panel("Target Topic", "/traj/target")),
                        item(raw_panel("Tracking Error Topic", "/traj/error")),
                    ],
                ),
                1.0,
            ),
            item(
                split(
                    "column",
                    [
                        item(raw_panel("Teensy Raw Topic", "/traj/teensy_raw")),
                        item(raw_panel("Session Info", "/session/info")),
                    ],
                ),
                1.0,
            ),
        ],
    )

    return {
        "version": 1,
        "content": {
            "type": "tabs",
            "selectedTabIndex": 0,
            "tabs": [
                {"title": "Trajectory Overview", "content": overview},
                {"title": "Trajectory Raw", "content": raw_tab},
            ],
        },
    }


def main() -> int:
    OUTPUT_PATH.write_text(json.dumps(build_layout(), indent=2) + "\n", encoding="utf-8")
    print(OUTPUT_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

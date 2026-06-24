from __future__ import annotations

import json
from pathlib import Path

import foxglove.layouts as fl


LAYOUT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = LAYOUT_DIR / "ballalgo_standard_layout.json"

FIELD_PANEL_TITLE = "Field Overview"
BACKGROUND_COLOR = "#101215"
PLOT_WINDOW_SECONDS = 10.0


def series(
    value: str,
    *,
    label: str,
    color: str,
    line_size: float = 2.0,
    use_right_y_axis: bool = False,
) -> fl.PlotSeries:
    return fl.PlotSeries(
        value=value,
        label=label,
        color=color,
        line_size=line_size,
        show_line=True,
        timestamp_method="receiveTime",
        use_right_y_axis=use_right_y_axis,
    )


def timestamp_plot(
    title: str,
    plot_series: list[fl.PlotSeries],
    *,
    y_axis_label: str | None = None,
    right_y_axis: fl.PlotRightAxis | None = None,
) -> fl.PlotPanel:
    return fl.PlotPanel(
        title=title,
        config=fl.PlotConfig(
            paths=plot_series,
            x_axis_val="timestamp",
            time_window_mode="sliding",
            following_view_width=PLOT_WINDOW_SECONDS,
            is_synced=True,
            show_legend=True,
            legend_display="floating",
            show_plot_values_in_legend=True,
            show_x_axis_labels=True,
            show_y_axis_labels=True,
            x_axis_label="Time (s)",
            y_axis_label=y_axis_label,
            right_y_axis=right_y_axis,
        ),
    )


def latest_xy_plot(
    title: str,
    plot_series: list[fl.PlotSeries],
    *,
    x_axis_path: str,
    x_axis_label: str,
    y_axis_label: str | None = None,
) -> fl.PlotPanel:
    return fl.PlotPanel(
        title=title,
        config=fl.PlotConfig(
            paths=plot_series,
            x_axis_val="custom",
            x_axis_path=fl.PlotXAxisPath(value=x_axis_path),
            time_range="latest",
            show_legend=True,
            legend_display="floating",
            show_plot_values_in_legend=True,
            show_x_axis_labels=True,
            show_y_axis_labels=True,
            x_axis_label=x_axis_label,
            y_axis_label=y_axis_label,
        ),
    )


def field_panel() -> fl.ThreeDeePanel:
    return fl.ThreeDeePanel(
        title=FIELD_PANEL_TITLE,
        config=fl.ThreeDeeConfig(
            fixed_frame="field",
            follow_tf="field",
            follow_mode="follow-none",
            scene=fl.BaseRendererSceneSettings(
                background_color=BACKGROUND_COLOR,
                sync_camera=False,
                transforms=fl.BaseRendererTransforms(
                    visible=False,
                    show_label=False,
                    axis_size=0.2,
                    line_width=1.0,
                ),
            ),
            camera_state=fl.ThreeDeeCameraState(
                perspective=False,
                distance=2.6,
                target=(0.0, 0.0, 0.0),
                theta_offset=0.0,
                phi=0.0,
                near=0.05,
                far=50.0,
                log_depth=False,
            ),
            topics={
                "/field/scene/static": fl.BaseRendererSceneUpdateTopicSettings(
                    visible=True,
                    show_outlines=False,
                    compute_vertex_normals=False,
                ),
                "/field/scene/live": fl.BaseRendererSceneUpdateTopicSettings(
                    visible=True,
                    show_outlines=False,
                    compute_vertex_normals=False,
                    selected_id_variable="selected_id",
                ),
                "/planner/scene/path": fl.BaseRendererSceneUpdateTopicSettings(
                    visible=True,
                    show_outlines=False,
                    compute_vertex_normals=False,
                ),
                "/lidar/scan": fl.BaseRendererPointCloudTopicSettings(
                    visible=True,
                    color_mode="flat",
                    flat_color="#50c850",
                    point_size=3.0,
                    point_shape="circle",
                    decay_time=0.15,
                ),
                "/robot/pose": fl.BaseRendererPoseTopicSettings(
                    visible=False,
                    type="arrow",
                ),
                "/ball/pose": fl.BaseRendererPoseTopicSettings(
                    visible=False,
                    type="arrow",
                ),
            },
            synced_topics={
                "/field/scene/static": False,
                "/field/scene/live": True,
                "/planner/scene/path": True,
                "/lidar/scan": True,
            },
        ),
    )


def lidar_panel() -> fl.ThreeDeePanel:
    """Robot position + compass heading arrow on the field, no point cloud."""
    return fl.ThreeDeePanel(
        title="Robot Position (Lidar Session)",
        config=fl.ThreeDeeConfig(
            fixed_frame="field",
            follow_tf="field",
            follow_mode="follow-none",
            scene=fl.BaseRendererSceneSettings(
                background_color="#1a1e22",
                sync_camera=False,
                transforms=fl.BaseRendererTransforms(
                    visible=False,
                    show_label=False,
                    axis_size=0.2,
                    line_width=1.0,
                ),
            ),
            camera_state=fl.ThreeDeeCameraState(
                perspective=False,
                distance=2.6,
                target=(0.0, 0.0, 0.0),
                theta_offset=0.0,
                phi=0.0,
                near=0.05,
                far=50.0,
                log_depth=False,
            ),
            topics={
                "/field/scene/static": fl.BaseRendererSceneUpdateTopicSettings(
                    visible=True,
                    show_outlines=False,
                    compute_vertex_normals=False,
                ),
                "/field/scene/live": fl.BaseRendererSceneUpdateTopicSettings(
                    visible=True,
                    show_outlines=False,
                    compute_vertex_normals=False,
                ),
            },
            synced_topics={
                "/field/scene/static": False,
                "/field/scene/live": True,
            },
        ),
    )


def acceleration_panel() -> fl.PlotPanel:
    return timestamp_plot(
        "Linear Acceleration",
        [
            series(
                "/robot/accel.ax_body_m_s2",
                label="ax body",
                color="#4e98e2",
            ),
            series(
                "/robot/accel.ay_body_m_s2",
                label="ay body",
                color="#f5774d",
            ),
            series(
                "/robot/accel.magnitude_body_m_s2",
                label="accel magnitude",
                color="#f7df71",
            ),
        ],
        y_axis_label="m/s^2",
    )


def angular_panel() -> fl.PlotPanel:
    return timestamp_plot(
        "Angular Kinematics",
        [
            series(
                "/robot/angular.heading_deg",
                label="heading",
                color="#c84fe3",
            ),
            series(
                "/robot/angular.omega_deg_s",
                label="omega",
                color="#4cf5ae",
            ),
            series(
                "/robot/angular.alpha_deg_s2",
                label="alpha",
                color="#f7df71",
            ),
        ],
        y_axis_label="deg / deg/s / deg/s^2",
    )


def robot_velocity_panel() -> fl.PlotPanel:
    return timestamp_plot(
        "Robot Velocity",
        [
            series(
                "/robot/twist.vx_body_m_s",
                label="vx body",
                color="#4e98e2",
            ),
            series(
                "/robot/twist.vy_body_m_s",
                label="vy body",
                color="#f5774d",
            ),
            series(
                "/robot/twist.speed_body_m_s",
                label="speed body",
                color="#f7df71",
            ),
        ],
        y_axis_label="m/s",
    )


def ball_velocity_panel() -> fl.PlotPanel:
    return timestamp_plot(
        "Ball Velocity",
        [
            series(
                "/ball/twist.vx_body_m_s",
                label="vx body",
                color="#4e98e2",
            ),
            series(
                "/ball/twist.vy_body_m_s",
                label="vy body",
                color="#f5774d",
            ),
            series(
                "/ball/twist.speed_body_m_s",
                label="speed body",
                color="#f7df71",
            ),
        ],
        y_axis_label="m/s",
    )


def trajectory_speed_profile_panel() -> fl.PlotPanel:
    return latest_xy_plot(
        "Trajectory Speed Profile",
        [
            series(
                "/planner/profile.samples[:].speed_m_s",
                label="path speed",
                color="#4cf5ae",
            ),
        ],
        x_axis_path="/planner/profile.samples[:].progress_01",
        x_axis_label="Trajectory Progress (0-1)",
        y_axis_label="m/s",
    )


def ball_range_panel() -> fl.PlotPanel:
    return timestamp_plot(
        "Ball Range",
        [
            series(
                "/ball/range.dist_cal_m",
                label="distance",
                color="#f7df71",
            ),
            series(
                "/ball/range.body_x_m",
                label="body x",
                color="#4e98e2",
            ),
            series(
                "/ball/range.body_y_m",
                label="body y",
                color="#f5774d",
            ),
            series(
                "/ball/range.angle_deg",
                label="angle",
                color="#c84fe3",
                use_right_y_axis=True,
            ),
        ],
        y_axis_label="m",
        right_y_axis=fl.PlotRightAxis(
            label="Angle (deg)",
            show_tick_labels=True,
            show_grid_lines=False,
        ),
    )


def robot_xy_trace_panel() -> fl.PlotPanel:
    """XY scatter of robot position over the full recording — shows the path the robot took."""
    return fl.PlotPanel(
        title="Robot Position Trace (XY)",
        config=fl.PlotConfig(
            paths=[
                fl.PlotSeries(
                    value="/robot/pose.pose.position.y",
                    label="robot path",
                    color="#4cf5ae",
                    line_size=1.5,
                    show_line=True,
                    timestamp_method="receiveTime",
                ),
            ],
            x_axis_val="custom",
            x_axis_path=fl.PlotXAxisPath(value="/robot/pose.pose.position.x"),
            time_range="all",
            show_legend=True,
            legend_display="floating",
            show_plot_values_in_legend=True,
            show_x_axis_labels=True,
            show_y_axis_labels=True,
            x_axis_label="Field X (m)",
            y_axis_label="Field Y (m)",
        ),
    )


def commanded_velocity_panel() -> fl.PlotPanel:
    return timestamp_plot(
        "Commanded vs Measured Velocity",
        [
            series("/traj/target.vx_body_target_m_s", label="cmd vx", color="#4e98e2"),
            series("/traj/target.vy_body_target_m_s", label="cmd vy", color="#f5774d"),
            series("/robot/twist.vx_body_m_s", label="actual vx", color="#9ed3ff"),
            series("/robot/twist.vy_body_m_s", label="actual vy", color="#ffb68c"),
        ],
        y_axis_label="m/s",
    )


def debug_log_panel() -> fl.LogPanel:
    return fl.LogPanel(
        title="Debug Log",
        config=fl.LogConfig(
            topic_to_render="/debug/log",
            min_log_level=1,
            font_size=12,
            show_level=True,
            show_time=True,
            show_date=False,
            show_timezone=False,
            show_name=False,
            show_source_location=False,
        ),
    )


def session_info_panel() -> fl.RawMessagesPanel:
    return fl.RawMessagesPanel(
        title="Session Info",
        config=fl.RawMessagesConfig(
            topic_path="/session/info",
            expansion="all",
            font_size=12,
        ),
    )


def camera_panel() -> fl.ImagePanel:
    return fl.ImagePanel(
        title="Front Camera",
        config=fl.ImageConfig(
            image_mode=fl.ImageModeConfig(
                image_topic="/camera/front/image",
                annotations={
                    "/camera/front/annotations": fl.ImageAnnotationSettings(
                        visible=True
                    )
                },
            ),
            synchronize=True,
            synced_topics={
                "/camera/front/image": True,
                "/camera/front/annotations": True,
            },
        ),
    )


def lidar_tab() -> fl.SplitContainer:
    """Field-frame lidar view + angular kinematics, mirroring tools/lidar_visual.py right panel."""
    return fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(proportion=2, content=lidar_panel()),
            fl.SplitItem(proportion=1, content=angular_panel()),
        ],
    )


def trajectory_tab() -> fl.SplitContainer:
    """Robot trajectory trace + commanded vs measured velocity for MCAP replay."""
    left_col = fl.SplitContainer(
        direction="column",
        items=[
            fl.SplitItem(proportion=1, content=field_panel()),
            fl.SplitItem(proportion=1, content=robot_xy_trace_panel()),
        ],
    )
    right_col = fl.SplitContainer(
        direction="column",
        items=[
            fl.SplitItem(proportion=1, content=commanded_velocity_panel()),
            fl.SplitItem(proportion=1, content=robot_velocity_panel()),
            fl.SplitItem(proportion=1, content=angular_panel()),
        ],
    )
    return fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(proportion=1.8, content=left_col),
            fl.SplitItem(proportion=1.4, content=right_col),
        ],
    )


def overview_tab() -> fl.SplitContainer:
    top_row = fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(content=acceleration_panel()),
            fl.SplitItem(content=angular_panel()),
        ],
    )
    middle_row = fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(content=debug_log_panel()),
            fl.SplitItem(content=robot_velocity_panel()),
            fl.SplitItem(content=ball_velocity_panel()),
        ],
    )
    bottom_row = fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(content=trajectory_speed_profile_panel()),
        ],
    )
    right_column = fl.SplitContainer(
        direction="column",
        items=[
            fl.SplitItem(proportion=2.45, content=top_row),
            fl.SplitItem(proportion=1.45, content=middle_row),
            fl.SplitItem(proportion=1.0, content=bottom_row),
        ],
    )
    return fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(proportion=2, content=field_panel()),
            fl.SplitItem(proportion=1.45, content=right_column),
        ],
    )


def camera_tab() -> fl.SplitContainer:
    right_column = fl.SplitContainer(
        direction="column",
        items=[
            fl.SplitItem(proportion=1.2, content=ball_range_panel()),
            fl.SplitItem(proportion=1.0, content=session_info_panel()),
        ],
    )
    return fl.SplitContainer(
        direction="row",
        items=[
            fl.SplitItem(proportion=2, content=camera_panel()),
            fl.SplitItem(proportion=1, content=right_column),
        ],
    )


def build_layout() -> fl.Layout:
    return fl.Layout(
        content=fl.TabContainer(
            selected_tab_index=0,
            tabs=[
                fl.TabItem(title="Overview", content=overview_tab()),
                fl.TabItem(title="Lidar", content=lidar_tab()),
                fl.TabItem(title="Trajectory", content=trajectory_tab()),
                fl.TabItem(title="Camera", content=camera_tab()),
            ],
        )
    )


def write_layout(output_path: Path = DEFAULT_OUTPUT) -> Path:
    layout = build_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    layout_dict = json.loads(layout.to_json())
    output_path.write_text(json.dumps(layout_dict, indent=2) + "\n", encoding="utf-8")
    return output_path


def main() -> int:
    output_path = write_layout()
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

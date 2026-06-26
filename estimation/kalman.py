from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

import numpy as np


def _param(params: Mapping[str, float | bool], key: str, default: float) -> float:
    value = params.get(key, default)
    return float(value)


@dataclass(slots=True)
class BallState:
    visible: bool = False
    x_m: float = 0.0
    y_m: float = 0.0
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0


@dataclass(slots=True)
class PoseEstimate:
    valid: bool = False
    x_mm: float = 0.0
    y_mm: float = 0.0
    should_fuse: bool = True


@dataclass(slots=True)
class PoseState:
    valid: bool = False
    x_mm: float = 0.0
    y_mm: float = 0.0
    vx_mm_s: float = 0.0
    vy_mm_s: float = 0.0
    vx_body_m_s: float = 0.0
    vy_body_m_s: float = 0.0


def polar_to_body_xy(angle_deg: float, distance_m: float) -> tuple[float, float]:
    radians = math.radians(angle_deg)
    return distance_m * math.cos(radians), distance_m * math.sin(radians)


def body_to_field_offset_mm(
    body_x_mm: float,
    body_y_mm: float,
    heading_deg: float,
) -> tuple[float, float]:
    h = math.radians(heading_deg)
    c = math.cos(h)
    s = math.sin(h)
    return c * body_x_mm - s * body_y_mm, s * body_x_mm + c * body_y_mm


def field_vel_to_body(
    vx_mm_s: float,
    vy_mm_s: float,
    heading_deg: float,
) -> tuple[float, float]:
    h = math.radians(heading_deg)
    c = math.cos(h)
    s = math.sin(h)
    return (
        (c * vx_mm_s + s * vy_mm_s) / 1000.0,
        (-s * vx_mm_s + c * vy_mm_s) / 1000.0,
    )


def body_vel_to_world(
    vx_body_mm_s: float,
    vy_body_mm_s: float,
    heading_deg: float,
) -> tuple[float, float]:
    h = math.radians(heading_deg)
    c = math.cos(h)
    s = math.sin(h)
    return (
        c * vx_body_mm_s - s * vy_body_mm_s,
        s * vx_body_mm_s + c * vy_body_mm_s,
    )


class BallKalman:
    """Constant-velocity ball filter ported from legacy BallKalman.cpp."""

    def __init__(self, params: Mapping[str, float | bool], camera_fps: float = 120.0):
        self.params = params
        self.camera_dt_s = 1.0 / camera_fps
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.p = np.eye(4, dtype=np.float64)
        self.initialized = False
        self.age_since_measurement_s = 0.0

    def predict(self, dt_s: float) -> None:
        if not self.initialized or dt_s <= 0:
            return
        self.age_since_measurement_s += dt_s
        gamma = _param(self.params, "ball_kf_friction_gamma", 0.95)
        gamma_step = math.pow(gamma, dt_s / self.camera_dt_s)
        f = np.eye(4, dtype=np.float64)
        f[0, 2] = dt_s
        f[1, 3] = dt_s
        f[2, 2] = gamma_step
        f[3, 3] = gamma_step
        q = np.zeros((4, 4), dtype=np.float64)
        q[0, 0] = q[1, 1] = _param(self.params, "ball_kf_process_pos_var", 0.5) * dt_s
        q[2, 2] = q[3, 3] = _param(self.params, "ball_kf_process_vel_var", 2.0) * dt_s
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

    def update(self, x_m: float, y_m: float, found: bool) -> None:
        if not found:
            return
        if not self.initialized:
            self.x[:, 0] = [x_m, y_m, 0.0, 0.0]
            self.p = np.eye(4, dtype=np.float64)
            self.p[2, 2] = self.p[3, 3] = _param(self.params, "ball_kf_vel_init_var", 4.0)
            self.initialized = True
        else:
            h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
            r = np.eye(2, dtype=np.float64) * _param(self.params, "ball_kf_meas_var", 0.02)
            z = np.array([[x_m], [y_m]], dtype=np.float64)
            innovation = z - h @ self.x
            s = h @ self.p @ h.T + r
            k = self.p @ h.T @ np.linalg.inv(s)
            self.x = self.x + k @ innovation
            self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        self.age_since_measurement_s = 0.0

    def state(self) -> BallState:
        max_stale = _param(self.params, "ball_max_stale_s", 0.22)
        visible = self.initialized and self.age_since_measurement_s <= max_stale
        if not self.initialized:
            return BallState()
        return BallState(
            visible=visible,
            x_m=float(self.x[0, 0]),
            y_m=float(self.x[1, 0]),
            vx_m_s=float(self.x[2, 0]),
            vy_m_s=float(self.x[3, 0]),
        )

    def reset(self) -> None:
        self.x.fill(0.0)
        self.p = np.eye(4, dtype=np.float64)
        self.initialized = False
        self.age_since_measurement_s = 0.0


class PoseKalman:
    """Robot pose filter ported from legacy PoseKalman.cpp."""

    def __init__(self, params: Mapping[str, float | bool]):
        self.params = params
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.p = np.eye(4, dtype=np.float64) * 1e3
        self.initialized = False
        self.age_since_measurement_s = 0.0

    def predict(self, dt_s: float) -> None:
        if not self.initialized or dt_s <= 0:
            return
        self.age_since_measurement_s += dt_s
        f = np.eye(4, dtype=np.float64)
        f[0, 2] = dt_s
        f[1, 3] = dt_s
        q = np.zeros((4, 4), dtype=np.float64)
        q[0, 0] = q[1, 1] = _param(self.params, "pose_kf_process_pos_var", 50.0) * dt_s
        q[2, 2] = q[3, 3] = _param(self.params, "pose_kf_process_vel_var", 200.0) * dt_s
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

    def predict_mouse(
        self,
        vx_body_mm_s: float,
        vy_body_mm_s: float,
        heading_deg: float,
        dt_s: float,
    ) -> None:
        if not self.initialized or dt_s <= 0:
            return
        self.age_since_measurement_s += dt_s
        self._cv_predict(dt_s, _param(self.params, "pose_kf_mouse_accel_var", 250000.0))
        zx, zy = body_vel_to_world(vx_body_mm_s, vy_body_mm_s, heading_deg)
        h = np.zeros((2, 4), dtype=np.float64)
        h[0, 2] = 1.0
        h[1, 3] = 1.0
        r = np.eye(2, dtype=np.float64) * _param(self.params, "pose_kf_mouse_meas_var", 2500.0)
        z = np.array([[zx], [zy]], dtype=np.float64)
        innovation = z - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ innovation
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p

    def update(self, meas: PoseEstimate, heading_deg: float = 0.0) -> None:
        del heading_deg
        if not meas.valid or not meas.should_fuse:
            return
        if not self.initialized:
            self.x[:, 0] = [meas.x_mm, meas.y_mm, 0.0, 0.0]
            self.p = np.eye(4, dtype=np.float64) * 100.0
            self.initialized = True
        else:
            h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
            r = np.eye(2, dtype=np.float64) * _param(self.params, "pose_kf_meas_pos_var", 400.0)
            z = np.array([[meas.x_mm], [meas.y_mm]], dtype=np.float64)
            innovation = z - h @ self.x
            s = h @ self.p @ h.T + r
            k = self.p @ h.T @ np.linalg.inv(s)
            self.x = self.x + k @ innovation
            self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p
        self.age_since_measurement_s = 0.0

    def update_goal_bearing(
        self,
        meas_bearing_rad: float,
        goal_x_mm: float,
        goal_y_mm: float,
        heading_deg: float,
        certainty: float,
    ) -> None:
        if not self.initialized:
            return
        xr = float(self.x[0, 0])
        yr = float(self.x[1, 0])
        dx_f = goal_x_mm - xr
        dy_f = goal_y_mm - yr
        r2 = dx_f * dx_f + dy_f * dy_f
        if r2 < 1.0:
            return

        h_rad = math.radians(heading_deg)
        c = math.cos(h_rad)
        s = math.sin(h_rad)
        bx = c * dx_f + s * dy_f
        by = -s * dx_f + c * dy_f
        predicted = math.atan2(bx, by)
        innovation = meas_bearing_rad - predicted
        while innovation > math.pi:
            innovation -= 2.0 * math.pi
        while innovation < -math.pi:
            innovation += 2.0 * math.pi

        h = np.array([[-dy_f / r2, dx_f / r2, 0.0, 0.0]], dtype=np.float64)
        cert = min(1.0, max(0.0, certainty))
        base = _param(self.params, "goal_bearing_base_var_rad2", 0.0035)
        penalty = _param(self.params, "goal_bearing_penalty_rad2", 0.5)
        variance = base + penalty * (1.0 - cert) * (1.0 - cert)
        s_mat = h @ self.p @ h.T + variance
        if float(s_mat[0, 0]) < 1e-9:
            return
        k = self.p @ h.T / float(s_mat[0, 0])
        self.x = self.x + k * innovation
        self.p = (np.eye(4, dtype=np.float64) - k @ h) @ self.p

    def state(self, heading_deg: float) -> PoseState:
        max_stale = _param(self.params, "pose_max_stale_s", 0.18)
        valid = self.initialized and self.age_since_measurement_s <= max_stale
        if not self.initialized:
            return PoseState()
        vx_body, vy_body = field_vel_to_body(float(self.x[2, 0]), float(self.x[3, 0]), heading_deg)
        return PoseState(
            valid=valid,
            x_mm=float(self.x[0, 0]),
            y_mm=float(self.x[1, 0]),
            vx_mm_s=float(self.x[2, 0]),
            vy_mm_s=float(self.x[3, 0]),
            vx_body_m_s=vx_body,
            vy_body_m_s=vy_body,
        )

    def reset(self) -> None:
        self.x.fill(0.0)
        self.p = np.eye(4, dtype=np.float64) * 1e3
        self.initialized = False
        self.age_since_measurement_s = 0.0

    def _cv_predict(self, dt_s: float, accel_var: float) -> None:
        f = np.eye(4, dtype=np.float64)
        f[0, 2] = dt_s
        f[1, 3] = dt_s
        dt2 = dt_s * dt_s
        dt3 = dt2 * dt_s
        q = np.zeros((4, 4), dtype=np.float64)
        q[0, 0] = accel_var * dt3 / 3.0
        q[0, 2] = q[2, 0] = accel_var * dt2 / 2.0
        q[2, 2] = accel_var * dt_s
        q[1, 1] = accel_var * dt3 / 3.0
        q[1, 3] = q[3, 1] = accel_var * dt2 / 2.0
        q[3, 3] = accel_var * dt_s
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + q

import copy
import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import serial


CRC_TABLE = (
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8
)


@dataclass(frozen=True)
class LidarPoint:
    distance_mm: int
    intensity: int
    angle_cd: int


# ---------------------------------------------------------------------------
# Internal localizer types
# ---------------------------------------------------------------------------

class CandidateSource(Enum):
    POINTS = 0
    LINE = 1
    HOUGH = 2


@dataclass
class Ray:
    angle_cd: int
    distance_mm: float
    dx: float
    dy: float
    rel_x_mm: float
    rel_y_mm: float


@dataclass
class HoughAngle:
    angle_deg: float
    cos_value: float
    sin_value: float


@dataclass
class AxisCandidate:
    median: float = 0.0
    count: int = 0
    spread: float = 0.0
    line_extent_mm: float = 0.0
    line_offset_mm: float = 0.0
    wall_position_mm: float = 0.0
    line_angle_deg: float = 0.0
    hough_votes: int = 0
    source: CandidateSource = CandidateSource.POINTS
    has_wall_position: bool = False


@dataclass
class PoseScore:
    inliers: int = 0
    x_inliers: int = 0
    y_inliers: int = 0
    median_residual_mm: float = float('inf')
    mean_residual_mm: float = float('inf')


@dataclass
class PoseCandidate:
    x_mm: float = 0.0
    y_mm: float = 0.0
    x_cluster: Optional[AxisCandidate] = None
    y_cluster: Optional[AxisCandidate] = None
    score: Optional[PoseScore] = None


# ---------------------------------------------------------------------------
# LidarLocalizer — mirrors LidarLocalizer.cpp exactly
# ---------------------------------------------------------------------------

class LidarLocalizer:
    def __init__(self, field_width_mm: float, field_height_mm: float,
                 lidar_yaw_offset_deg: float = 0.0):
        self.field_width_mm = field_width_mm
        self.field_height_mm = field_height_mm
        self.lidar_yaw_offset_deg = lidar_yaw_offset_deg

        # Parameters — kept in sync with LidarLocalizer.hpp
        self._min_dist = 80.0
        self._max_dist = 6000.0
        self._min_intensity = 20
        self._min_axis_samples = 12
        self._cluster_gap_mm = 140.0
        self._cluster_match_radius_mm = 260.0
        self._min_line_samples = 10
        self._line_cluster_gap_mm = 90.0
        self._min_line_extent_mm = 260.0
        self._hough_angle_step_deg = 2.0
        self._hough_tilt_deg = 14.0
        self._hough_bin_mm = 24.0
        self._hough_line_tolerance_mm = 85.0
        self._hough_min_votes = 9
        self._hough_peak_suppress_bins = 5
        self._hough_peak_suppress_angles = 2
        self._hough_max_lines_per_axis = 4
        self._min_wall_component = 0.35
        self._max_candidate_clusters = 4
        self._neighbor_gap_mm = 260.0
        self._wall_inlier_mm = 90.0
        self._wall_axis_margin_mm = 180.0
        self._min_pose_inliers = 28
        self._min_pose_axis_inliers = 8

        self._hough_angles_x = self._build_hough_angles(0.0)
        self._hough_angles_y = self._build_hough_angles(90.0)
        self._prev_pose: Optional[Tuple[float, float]] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(self, points: List[LidarPoint], heading_deg: float) -> dict:
        rays = self._field_rays(points, heading_deg)

        x_estimates: List[float] = []
        y_estimates: List[float] = []
        for ray in rays:
            if abs(ray.dx) >= self._min_wall_component:
                x_est = self._wall_axis_estimate(ray.distance_mm, ray.dx, self.field_width_mm)
                if -200.0 < x_est < self.field_width_mm + 200.0:
                    x_estimates.append(x_est)
            if abs(ray.dy) >= self._min_wall_component:
                y_est = self._wall_axis_estimate(ray.distance_mm, ray.dy, self.field_height_mm)
                if -200.0 < y_est < self.field_height_mm + 200.0:
                    y_estimates.append(y_est)

        prev_x = self._prev_pose[0] if self._prev_pose else None
        prev_y = self._prev_pose[1] if self._prev_pose else None

        x_hough = self._candidate_hough_axis_clusters(rays, True, prev_x, self.field_width_mm)
        y_hough = self._candidate_hough_axis_clusters(rays, False, prev_y, self.field_height_mm)
        x_line = self._candidate_line_axis_clusters(rays, True, prev_x, self.field_width_mm)
        y_line = self._candidate_line_axis_clusters(rays, False, prev_y, self.field_height_mm)
        x_point = self._candidate_axis_clusters(x_estimates, prev_x, self.field_width_mm)
        y_point = self._candidate_axis_clusters(y_estimates, prev_y, self.field_height_mm)

        x_candidates = self._merge_axis_candidates(x_hough + x_line, x_point, prev_x, self.field_width_mm)
        y_candidates = self._merge_axis_candidates(y_hough + y_line, y_point, prev_y, self.field_height_mm)

        base = {"valid": False, "x_mm": None, "y_mm": None,
                "x_samples": len(x_estimates), "y_samples": len(y_estimates)}

        pose = self._choose_pose_from_candidates(x_candidates, y_candidates, rays, prev_x, prev_y)
        if pose is None:
            return base

        self._prev_pose = (pose.x_mm, pose.y_mm)
        return {"valid": True, "x_mm": pose.x_mm, "y_mm": pose.y_mm,
                "x_samples": len(x_estimates), "y_samples": len(y_estimates)}

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _build_hough_angles(self, center_angle_deg: float) -> List[HoughAngle]:
        steps = int((self._hough_tilt_deg * 2.0) / self._hough_angle_step_deg) + 1
        start = center_angle_deg - self._hough_tilt_deg
        angles = []
        for i in range(steps):
            deg = start + i * self._hough_angle_step_deg
            rad = math.radians(deg)
            angles.append(HoughAngle(deg, math.cos(rad), math.sin(rad)))
        return angles

    def _field_rays(self, points: List[LidarPoint], heading_deg: float) -> List[Ray]:
        rays = []
        for p in points:
            d = float(p.distance_mm)
            if d < self._min_dist or d > self._max_dist or p.intensity < self._min_intensity:
                continue
            field_angle_deg = heading_deg + self.lidar_yaw_offset_deg + p.angle_cd * 0.01
            rad = math.radians(field_angle_deg)
            dx = math.cos(rad)
            dy = math.sin(rad)
            rays.append(Ray(p.angle_cd, d, dx, dy, d * dx, d * dy))
        return self._filter_isolated_rays(rays)

    def _filter_isolated_rays(self, rays: List[Ray]) -> List[Ray]:
        if len(rays) < 3:
            return rays
        rays = sorted(rays, key=lambda r: r.angle_cd)
        n = len(rays)
        kept = []
        for i in range(n):
            prev_r = rays[(i - 1) % n]
            next_r = rays[(i + 1) % n]
            if (self._ray_gap(rays[i], prev_r) <= self._neighbor_gap_mm or
                    self._ray_gap(rays[i], next_r) <= self._neighbor_gap_mm):
                kept.append(rays[i])
        return kept

    @staticmethod
    def _ray_gap(a: Ray, b: Ray) -> float:
        return math.hypot(a.rel_x_mm - b.rel_x_mm, a.rel_y_mm - b.rel_y_mm)

    @staticmethod
    def _wall_axis_estimate(distance_mm: float, direction: float, axis_limit_mm: float) -> float:
        return (axis_limit_mm - distance_mm * direction) if direction > 0.0 else (-distance_mm * direction)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    # ------------------------------------------------------------------
    # Statistics helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _median(values: List[float]) -> float:
        if not values:
            return float('nan')
        ordered = sorted(values)
        mid = len(ordered) // 2
        if len(ordered) % 2 == 0:
            return 0.5 * (ordered[mid - 1] + ordered[mid])
        return ordered[mid]

    @staticmethod
    def _percentile(values: List[float], fraction: float) -> float:
        if not values:
            return float('nan')
        ordered = sorted(values)
        if len(ordered) == 1:
            return ordered[0]
        pos = (len(ordered) - 1) * fraction
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            return ordered[lo]
        ratio = pos - lo
        return ordered[lo] * (1.0 - ratio) + ordered[hi] * ratio

    # ------------------------------------------------------------------
    # Point-cluster pipeline
    # ------------------------------------------------------------------

    def _cluster_estimates(self, estimates: List[float]) -> List[AxisCandidate]:
        if not estimates:
            return []
        ordered = sorted(estimates)
        clusters = []
        start = 0
        for i in range(1, len(ordered) + 1):
            if i < len(ordered) and (ordered[i] - ordered[i - 1]) <= self._cluster_gap_mm:
                continue
            vals = ordered[start:i]
            q1 = vals[len(vals) // 4]
            q3 = vals[(3 * len(vals)) // 4]
            clusters.append(AxisCandidate(
                median=self._median(vals),
                count=len(vals),
                spread=q3 - q1,
            ))
            start = i
        return clusters

    def _candidate_axis_clusters(self, estimates: List[float],
                                  prev_value: Optional[float],
                                  axis_limit_mm: float) -> List[AxisCandidate]:
        if len(estimates) < self._min_axis_samples:
            return []
        clusters = [c for c in self._cluster_estimates(estimates)
                    if c.count >= self._min_axis_samples]
        if not clusters:
            return []
        center = axis_limit_mm * 0.5

        def key(c: AxisCandidate):
            cont = abs(c.median - prev_value) if prev_value is not None else abs(c.median - center)
            return (-c.count, c.spread, cont)

        clusters.sort(key=key)
        return clusters[:self._max_candidate_clusters]

    # ------------------------------------------------------------------
    # Line-feature pipeline
    # ------------------------------------------------------------------

    def _line_feature_from_rays(self, rays: List[Ray], x_axis: bool) -> Optional[AxisCandidate]:
        if len(rays) < self._min_line_samples:
            return None
        coords = sorted(r.rel_x_mm if x_axis else r.rel_y_mm for r in rays)
        perps = sorted(r.rel_y_mm if x_axis else r.rel_x_mm for r in rays)
        extent = self._percentile(perps, 0.90) - self._percentile(perps, 0.10)
        if extent < self._min_line_extent_mm:
            return None
        q1 = coords[len(coords) // 4]
        q3 = coords[(3 * len(coords)) // 4]
        return AxisCandidate(
            median=self._median(coords),
            count=len(rays),
            spread=q3 - q1,
            line_extent_mm=extent,
        )

    def _candidate_line_axis_clusters(self, rays: List[Ray], x_axis: bool,
                                       prev_value: Optional[float],
                                       axis_limit_mm: float) -> List[AxisCandidate]:
        if len(rays) < self._min_line_samples:
            return []

        ordered = sorted(rays, key=lambda r: r.rel_x_mm if x_axis else r.rel_y_mm)
        features: List[AxisCandidate] = []
        start = 0
        for i in range(1, len(ordered) + 1):
            if i < len(ordered):
                cur = ordered[i].rel_x_mm if x_axis else ordered[i].rel_y_mm
                prv = ordered[i - 1].rel_x_mm if x_axis else ordered[i - 1].rel_y_mm
                if (cur - prv) <= self._line_cluster_gap_mm:
                    continue
            feat = self._line_feature_from_rays(ordered[start:i], x_axis)
            if feat is not None:
                features.append(feat)
            start = i

        candidates: List[AxisCandidate] = []
        for feat in features:
            # Mirror C++: `auto feature : features` copies feature, then feature.median is
            # mutated in-place across the two wallPosition iterations.
            feature = copy.copy(feat)
            for wall_position in [0.0, axis_limit_mm]:
                pose_value = wall_position - feature.median
                if pose_value < 0.0 or pose_value > axis_limit_mm:
                    continue
                feature.median = pose_value
                feature.line_offset_mm = wall_position - pose_value
                feature.wall_position_mm = wall_position
                feature.has_wall_position = True
                feature.source = CandidateSource.LINE
                candidates.append(copy.copy(feature))

        if not candidates:
            return []

        center = axis_limit_mm * 0.5

        def key(c: AxisCandidate):
            cont = abs(c.median - prev_value) if prev_value is not None else abs(c.median - center)
            return (-c.count, -c.line_extent_mm, c.spread, cont)

        candidates.sort(key=key)
        return candidates[:self._max_candidate_clusters]

    # ------------------------------------------------------------------
    # Hough pipeline
    # ------------------------------------------------------------------

    def _fit_hough_line(self, rays: List[Ray], x_axis: bool, angle: HoughAngle,
                         peak_rho_mm: float, vote_count: int) -> Optional[AxisCandidate]:
        inlier_rhos: List[float] = []
        inlier_perps: List[float] = []
        for ray in rays:
            rho = ray.rel_x_mm * angle.cos_value + ray.rel_y_mm * angle.sin_value
            if abs(rho - peak_rho_mm) > self._hough_line_tolerance_mm:
                continue
            inlier_rhos.append(rho)
            inlier_perps.append(ray.rel_y_mm if x_axis else ray.rel_x_mm)

        if len(inlier_rhos) < self._min_line_samples:
            return None

        med_rho = self._median(inlier_rhos)
        filtered_rhos: List[float] = []
        filtered_perps: List[float] = []
        for rho, perp in zip(inlier_rhos, inlier_perps):
            if abs(rho - med_rho) <= self._hough_line_tolerance_mm:
                filtered_rhos.append(rho)
                filtered_perps.append(perp)

        if len(filtered_rhos) < self._min_line_samples:
            return None

        extent = self._percentile(filtered_perps, 0.90) - self._percentile(filtered_perps, 0.10)
        if extent < self._min_line_extent_mm:
            return None

        axis_component = angle.cos_value if x_axis else angle.sin_value
        if abs(axis_component) < 0.65:
            return None

        ordered_rhos = sorted(filtered_rhos)
        q1 = ordered_rhos[len(ordered_rhos) // 4]
        q3 = ordered_rhos[(3 * len(ordered_rhos)) // 4]

        return AxisCandidate(
            line_offset_mm=self._median(filtered_rhos) / axis_component,
            line_angle_deg=angle.angle_deg,
            count=len(filtered_rhos),
            hough_votes=vote_count,
            spread=q3 - q1,
            line_extent_mm=extent,
            source=CandidateSource.HOUGH,
        )

    def _hough_axis_lines(self, rays: List[Ray], x_axis: bool) -> List[AxisCandidate]:
        if len(rays) < self._min_line_samples:
            return []

        angles = self._hough_angles_x if x_axis else self._hough_angles_y

        # (angle_idx, bin_idx, votes)
        peaks = []
        for angle_idx, angle in enumerate(angles):
            vote_bins: dict = {}
            for ray in rays:
                rho = ray.rel_x_mm * angle.cos_value + ray.rel_y_mm * angle.sin_value
                bin_idx = int(round(rho / self._hough_bin_mm))
                vote_bins[bin_idx] = vote_bins.get(bin_idx, 0) + 1
            for bin_idx, votes in vote_bins.items():
                if votes >= self._hough_min_votes:
                    peaks.append((angle_idx, bin_idx, votes))

        if not peaks:
            return []

        # C++: sort by tie(votes, angleIndex) descending
        peaks.sort(key=lambda p: (p[2], p[0]), reverse=True)

        accepted: List[AxisCandidate] = []
        kept: List[Tuple[int, int]] = []  # (angle_idx, bin_idx)
        for angle_idx, bin_idx, votes in peaks:
            suppressed = any(
                abs(bin_idx - ka) <= self._hough_peak_suppress_bins and
                abs(angle_idx - ki) <= self._hough_peak_suppress_angles
                for ki, ka in kept
            )
            if suppressed:
                continue
            peak_rho_mm = bin_idx * self._hough_bin_mm
            line = self._fit_hough_line(rays, x_axis, angles[angle_idx], peak_rho_mm, votes)
            if line is not None:
                accepted.append(line)
                kept.append((angle_idx, bin_idx))
                if len(accepted) >= self._hough_max_lines_per_axis:
                    break

        return accepted

    def _candidate_hough_axis_clusters(self, rays: List[Ray], x_axis: bool,
                                        prev_value: Optional[float],
                                        axis_limit_mm: float) -> List[AxisCandidate]:
        lines = self._hough_axis_lines(rays, x_axis)
        if not lines:
            return []

        candidates: List[AxisCandidate] = []
        for line in lines:
            for wall_position in [0.0, axis_limit_mm]:
                pose_value = wall_position - line.line_offset_mm
                if pose_value < 0.0 or pose_value > axis_limit_mm:
                    continue
                candidates.append(AxisCandidate(
                    median=pose_value,
                    count=line.count,
                    spread=line.spread,
                    line_extent_mm=line.line_extent_mm,
                    line_offset_mm=line.line_offset_mm,
                    wall_position_mm=wall_position,
                    line_angle_deg=line.line_angle_deg,
                    hough_votes=line.hough_votes,
                    source=CandidateSource.HOUGH,
                    has_wall_position=True,
                ))

        if not candidates:
            return []

        center = axis_limit_mm * 0.5

        def key(c: AxisCandidate):
            cont = abs(c.median - prev_value) if prev_value is not None else abs(c.median - center)
            return (-c.hough_votes, -c.count, -c.line_extent_mm, c.spread, cont)

        candidates.sort(key=key)
        return candidates[:self._max_candidate_clusters]

    # ------------------------------------------------------------------
    # Merge + scoring
    # ------------------------------------------------------------------

    @staticmethod
    def _source_bonus(source: CandidateSource) -> int:
        if source == CandidateSource.HOUGH:
            return 2
        if source == CandidateSource.LINE:
            return 1
        return 0

    def _merge_axis_candidates(self, line_clusters: List[AxisCandidate],
                                point_clusters: List[AxisCandidate],
                                prev_value: Optional[float],
                                axis_limit_mm: float) -> List[AxisCandidate]:
        candidates = list(line_clusters)
        for pc in point_clusters:
            matched = any(
                abs(pc.median - lc.median) <= self._cluster_match_radius_mm * 0.5
                for lc in line_clusters
            )
            if matched:
                continue
            c = copy.copy(pc)
            c.source = CandidateSource.POINTS
            c.line_extent_mm = 0.0
            c.line_offset_mm = 0.0
            c.has_wall_position = False
            candidates.append(c)

        if not candidates:
            return []

        center = axis_limit_mm * 0.5

        def key(c: AxisCandidate):
            cont = abs(c.median - prev_value) if prev_value is not None else abs(c.median - center)
            bonus = self._source_bonus(c.source)
            return (-bonus, -c.line_extent_mm, -c.count, c.spread, cont)

        candidates.sort(key=key)
        return candidates[:self._max_candidate_clusters]

    def _score_pose_against_walls(self, robot_x_mm: float, robot_y_mm: float,
                                   rays: List[Ray]) -> PoseScore:
        residuals: List[float] = []
        x_residuals: List[float] = []
        y_residuals: List[float] = []

        for ray in rays:
            hit_x = robot_x_mm + ray.rel_x_mm
            hit_y = robot_y_mm + ray.rel_y_mm

            if -self._wall_axis_margin_mm <= hit_y <= self.field_height_mm + self._wall_axis_margin_mm:
                x_res = min(abs(hit_x), abs(self.field_width_mm - hit_x))
                if x_res <= self._wall_inlier_mm:
                    x_residuals.append(x_res)
                    residuals.append(x_res)

            if -self._wall_axis_margin_mm <= hit_x <= self.field_width_mm + self._wall_axis_margin_mm:
                y_res = min(abs(hit_y), abs(self.field_height_mm - hit_y))
                if y_res <= self._wall_inlier_mm:
                    y_residuals.append(y_res)
                    residuals.append(y_res)

        score = PoseScore(
            inliers=len(residuals),
            x_inliers=len(x_residuals),
            y_inliers=len(y_residuals),
        )
        if residuals:
            score.median_residual_mm = self._median(residuals)
            score.mean_residual_mm = sum(residuals) / len(residuals)
        return score

    def _choose_pose_from_candidates(self,
                                      x_clusters: List[AxisCandidate],
                                      y_clusters: List[AxisCandidate],
                                      rays: List[Ray],
                                      prev_x: Optional[float],
                                      prev_y: Optional[float]) -> Optional[PoseCandidate]:
        if not x_clusters or not y_clusters or not rays:
            return None

        best_pose: Optional[PoseCandidate] = None
        best_key = None

        for xc in x_clusters:
            for yc in y_clusters:
                x_mm = self._clamp(xc.median, 0.0, self.field_width_mm)
                y_mm = self._clamp(yc.median, 0.0, self.field_height_mm)
                score = self._score_pose_against_walls(x_mm, y_mm, rays)

                if score.inliers < self._min_pose_inliers:
                    continue
                if score.x_inliers < self._min_pose_axis_inliers or score.y_inliers < self._min_pose_axis_inliers:
                    continue

                if prev_x is not None and prev_y is not None:
                    cont = math.hypot(x_mm - prev_x, y_mm - prev_y)
                else:
                    cont = math.hypot(x_mm - self.field_width_mm * 0.5, y_mm - self.field_height_mm * 0.5)

                key = (
                    score.inliers,
                    min(score.x_inliers, score.y_inliers),
                    -cont,
                    xc.line_extent_mm + yc.line_extent_mm,
                    xc.count + yc.count,
                    -score.median_residual_mm,
                    -score.mean_residual_mm,
                )

                if best_pose is None or key > best_key:
                    best_key = key
                    best_pose = PoseCandidate(x_mm, y_mm, xc, yc, score)

        return best_pose


# ---------------------------------------------------------------------------
# LD19 frame reader — unchanged
# ---------------------------------------------------------------------------

def ld19_crc8(payload):
    crc = 0
    for byte_value in payload:
        crc = CRC_TABLE[(crc ^ byte_value) & 0xFF]
    return crc


class LD19Reader:
    FRAME_LEN = 47  # 0x54 + verlen + 45 trailing bytes

    def __init__(self, port, baud, timeout):
        self._serial = None
        self._buffer = bytearray()
        try:
            self._serial = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            print(f"[LIDAR] Connected on {port} @ {baud}")
        except Exception as exc:
            print(f"[LIDAR] Failed to open serial: {exc}")

    @property
    def is_connected(self):
        return self._serial is not None

    def close(self):
        if self._serial is not None:
            self._serial.close()
            self._serial = None
            print("[LIDAR] Closed")

    def poll_points(self):
        if self._serial is None:
            return []

        points = []
        try:
            waiting = self._serial.in_waiting
            if waiting > 0:
                self._buffer.extend(self._serial.read(waiting))
        except Exception:
            return []

        while True:
            if len(self._buffer) < self.FRAME_LEN:
                break
            try:
                start = self._buffer.index(0x54)
            except ValueError:
                self._buffer.clear()
                break

            if start > 0:
                del self._buffer[:start]
            if len(self._buffer) < self.FRAME_LEN:
                break
            if self._buffer[1] != 0x2C:
                del self._buffer[0]
                continue

            frame = bytes(self._buffer[:self.FRAME_LEN])
            del self._buffer[:self.FRAME_LEN]
            frame_points = self._parse_frame(frame)
            if frame_points:
                points.extend(frame_points)
        return points

    def _parse_frame(self, frame):
        if len(frame) != self.FRAME_LEN:
            return []
        if frame[0] != 0x54 or frame[1] != 0x2C:
            return []
        if ld19_crc8(frame[:-1]) != frame[-1]:
            return []

        start_angle = frame[4] | (frame[5] << 8)
        end_angle = frame[42] | (frame[43] << 8)
        step = self._angle_step(start_angle, end_angle, 11)

        points = []
        base = 6
        for i in range(12):
            idx = base + i * 3
            distance = frame[idx] | (frame[idx + 1] << 8)
            intensity = frame[idx + 2]
            angle_cd = (start_angle + step * i) % 36000
            points.append(LidarPoint(distance_mm=distance, intensity=intensity, angle_cd=angle_cd))
        return points

    @staticmethod
    def _angle_step(start_angle, end_angle, len_minus_one):
        if start_angle <= end_angle:
            return (end_angle - start_angle) // len_minus_one
        return (36000 + end_angle - start_angle) // len_minus_one

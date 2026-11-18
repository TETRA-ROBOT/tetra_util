"""라이다 스캔을 점유 격자 지도에 맞춰 보는 계산. ROS 없이 테스트한다 (test/test_scan_matcher.py).

점수는 '지도 벽에서 hit_tolerance 안에 떨어진 스캔 점의 비율'이다. AMCL 의 likelihood_field 가중치와 달리
0~1 로 정규화돼 있어 현장마다 같은 문턱값을 쓸 수 있다 (M 실기 2026-11-19: 맞는 자세 87%, 0.25 m·12° 틀린 자세 23%).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SearchWindow:
    """center 를 기준으로 ±radius (m), ±yaw_range (rad) 를 xy_step·yaw_step 간격으로 훑는다."""
    radius: float
    yaw_range: float
    xy_step: float
    yaw_step: float


@dataclass(frozen=True)
class SearchResult:
    pose: Pose2D
    ratio: float
    # best 에서 ambiguity_distance·ambiguity_yaw 보다 떨어진 자세 중 가장 높은 점수. 높으면 다른 후보와 헷갈린다
    runner_up_ratio: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_to_points(ranges: np.ndarray, angle_min: float, angle_increment: float,
                   min_range: float, max_range: float, max_points: int) -> np.ndarray:
    """LaserScan 거리값을 센서 기준 (x, y) 점으로 바꾸고, 유효한 점을 고르게 max_points 개까지 남긴다."""
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= min_range) & (ranges <= max_range)
    ranges, angles = ranges[valid], angles[valid]
    if len(ranges) > max_points:
        keep = np.linspace(0, len(ranges) - 1, max_points).astype(int)
        ranges, angles = ranges[keep], angles[keep]
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))


def base_pose_from_sensor(sensor_in_map: Pose2D, sensor_in_base: Pose2D) -> Pose2D:
    """지도에서 찾은 센서 자세와 센서 장착 위치로 base_footprint 자세를 구한다."""
    yaw = normalize_angle(sensor_in_map.yaw - sensor_in_base.yaw)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    x = sensor_in_map.x - (cos_yaw * sensor_in_base.x - sin_yaw * sensor_in_base.y)
    y = sensor_in_map.y - (sin_yaw * sensor_in_base.x + cos_yaw * sensor_in_base.y)
    return Pose2D(x, y, yaw)


class MapMatcher:
    def __init__(self, occupied: np.ndarray, resolution: float, origin_x: float, origin_y: float,
                 hit_tolerance: float) -> None:
        """occupied: (높이, 너비) bool 배열. 행이 y, 열이 x (nav_msgs/OccupancyGrid 순서)."""
        self._resolution = resolution
        self._origin = np.array([origin_x, origin_y])
        self._height, self._width = occupied.shape
        wall_distance = distance_transform_edt(~occupied) * resolution
        self._hit = wall_distance <= hit_tolerance

    def ratio(self, points: np.ndarray, pose: Pose2D) -> float:
        zero_shift = np.zeros(1, dtype=int)
        return float(self._scores(points, pose, zero_shift, np.zeros(1))[0, 0, 0])

    def search(self, points: np.ndarray, center: Pose2D, window: SearchWindow,
               ambiguity_distance: float, ambiguity_yaw: float) -> SearchResult:
        """창을 거칠게 훑은 뒤 가장 좋은 자세 주변을 지도 해상도·0.5° 간격으로 다시 훑는다."""
        step_cells = max(1, round(window.xy_step / self._resolution))
        steps = int(round(window.radius / (step_cells * self._resolution)))
        shifts = np.arange(-steps, steps + 1) * step_cells
        yaw_offsets = _yaw_offsets(window.yaw_range, window.yaw_step)
        scores = self._scores(points, center, shifts, yaw_offsets)

        k, row, col = np.unravel_index(int(np.argmax(scores)), scores.shape)
        coarse = Pose2D(center.x + shifts[col] * self._resolution,
                        center.y + shifts[row] * self._resolution,
                        normalize_angle(center.yaw + yaw_offsets[k]))
        runner_up = _runner_up(scores, shifts * self._resolution, yaw_offsets, (k, row, col),
                               ambiguity_distance, ambiguity_yaw)

        fine_shifts = np.arange(-step_cells, step_cells + 1)
        fine_yaws = _yaw_offsets(window.yaw_step, math.radians(0.5))
        fine_scores = self._scores(points, coarse, fine_shifts, fine_yaws)
        k, row, col = np.unravel_index(int(np.argmax(fine_scores)), fine_scores.shape)
        best = Pose2D(coarse.x + fine_shifts[col] * self._resolution,
                      coarse.y + fine_shifts[row] * self._resolution,
                      normalize_angle(coarse.yaw + fine_yaws[k]))
        return SearchResult(best, float(fine_scores[k, row, col]), runner_up)

    def _scores(self, points: np.ndarray, center: Pose2D, shifts: np.ndarray,
                yaw_offsets: np.ndarray) -> np.ndarray:
        """(yaw, y 이동, x 이동) 별 점수. 점을 한 번 격자 칸으로 바꾼 뒤 정수 칸만큼 옮겨서 이동마다 다시 계산하지 않는다."""
        scores = np.empty((len(yaw_offsets), len(shifts), len(shifts)))
        for k, yaw_offset in enumerate(yaw_offsets):
            yaw = center.yaw + yaw_offset
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            world_x = center.x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
            world_y = center.y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
            cols = np.floor((world_x - self._origin[0]) / self._resolution).astype(int)[None, :] + shifts[:, None]
            rows = np.floor((world_y - self._origin[1]) / self._resolution).astype(int)[None, :] + shifts[:, None]
            col_ok = (cols >= 0) & (cols < self._width)
            row_ok = (rows >= 0) & (rows < self._height)
            hits = self._hit[np.clip(rows, 0, self._height - 1)[:, None, :],
                             np.clip(cols, 0, self._width - 1)[None, :, :]]
            hits &= row_ok[:, None, :] & col_ok[None, :, :]
            scores[k] = hits.mean(axis=2)
        return scores


def _yaw_offsets(yaw_range: float, yaw_step: float) -> np.ndarray:
    # ±180° 를 훑을 때 -180° 와 +180° 를 두 번 세지 않는다
    if yaw_range >= math.pi - 1e-9:
        return np.arange(-math.pi, math.pi, yaw_step)
    steps = int(round(yaw_range / yaw_step))
    return np.arange(-steps, steps + 1) * yaw_step


def _runner_up(scores: np.ndarray, offsets_m: np.ndarray, yaw_offsets: np.ndarray, best_index: tuple,
               ambiguity_distance: float, ambiguity_yaw: float) -> float:
    k, row, col = best_index
    dy = (offsets_m - offsets_m[row])[None, :, None]
    dx = (offsets_m - offsets_m[col])[None, None, :]
    dyaw = np.abs(np.arctan2(np.sin(yaw_offsets - yaw_offsets[k]), np.cos(yaw_offsets - yaw_offsets[k])))[:, None, None]
    far = (np.hypot(dx, dy) > ambiguity_distance) | (dyaw > ambiguity_yaw)
    return float(scores[far].max()) if far.any() else 0.0

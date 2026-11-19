"""출발 전과 멈춘 뒤의 스캔을 맞춰 로봇이 실제로 움직인 양을 잰다 (오도메트리와 무관한 기준).

점 대 점 ICP 를 오도메트리 이동량에서 시작한다. 오도메트리 오차는 몇 % 라 시작점이 정답 가까이에 있다.
"""
from dataclasses import dataclass
import math
from typing import Optional, Sequence

import numpy as np
from scipy.spatial import cKDTree

MAX_ITERATIONS = 60
# 처음에는 넓게 짝을 짓고 점점 좁힌다 (m)
START_PAIR_DISTANCE = 0.5
FINAL_PAIR_DISTANCE = 0.05
# 회전 배율이 2% 틀리면 두 바퀴에 14° 가 어긋난다. ICP 는 그만큼 떨어진 데서 시작하면 엉뚱한 데 붙으므로 각도를 먼저 훑는다
YAW_SEARCH = math.radians(20.0)
YAW_SEARCH_STEP = math.radians(0.5)
SEARCH_HIT_DISTANCE = 0.1


@dataclass(frozen=True)
class Motion:
    """출발 자세 기준 도착 자세 (x 앞, y 왼쪽, yaw 반시계)."""
    x: float
    y: float
    yaw: float

    @property
    def distance(self) -> float:
        return math.hypot(self.x, self.y)


@dataclass(frozen=True)
class MatchResult:
    motion: Motion
    rmse: float
    inlier_ratio: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_to_base_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                        range_min: float, range_max: float, sensor_in_base: Motion) -> np.ndarray:
    """LaserScan 거리값을 로봇(base) 좌표 점들로 바꾼다. 라이다가 중심에서 떨어져 있으면 제자리 회전에도 움직이므로 base 로 옮긴다."""
    values = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(values.size)
    valid = np.isfinite(values) & (values > max(range_min, 0.05)) & (values < range_max)
    sensor_x = values[valid] * np.cos(angles[valid])
    sensor_y = values[valid] * np.sin(angles[valid])
    cos_yaw, sin_yaw = math.cos(sensor_in_base.yaw), math.sin(sensor_in_base.yaw)
    return np.column_stack((
        sensor_in_base.x + cos_yaw * sensor_x - sin_yaw * sensor_y,
        sensor_in_base.y + sin_yaw * sensor_x + cos_yaw * sensor_y,
    ))


def _apply(points: np.ndarray, motion: Motion) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(motion.yaw), math.sin(motion.yaw)
    return np.column_stack((
        motion.x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1],
        motion.y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1],
    ))


def _best_rigid(source: np.ndarray, target: np.ndarray) -> Motion:
    """source 를 target 에 겹치는 회전·이동 (최소제곱, 2D Kabsch)."""
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    cross = (source - source_mean).T @ (target - target_mean)
    yaw = math.atan2(cross[0, 1] - cross[1, 0], cross[0, 0] + cross[1, 1])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    x = target_mean[0] - (cos_yaw * source_mean[0] - sin_yaw * source_mean[1])
    y = target_mean[1] - (sin_yaw * source_mean[0] + cos_yaw * source_mean[1])
    return Motion(x, y, yaw)


def _search_yaw(tree: cKDTree, end_points: np.ndarray, initial: Motion) -> Motion:
    """오도메트리 각도 ±YAW_SEARCH 안에서 도착 스캔이 출발 스캔과 가장 많이 겹치는 각도를 고른다 (이동은 오도메트리 값)."""
    best, best_hits = initial, -1
    for offset in np.arange(-YAW_SEARCH, YAW_SEARCH + 1e-9, YAW_SEARCH_STEP):
        candidate = Motion(initial.x, initial.y, normalize_angle(initial.yaw + offset))
        distances, _ = tree.query(_apply(end_points, candidate), distance_upper_bound=SEARCH_HIT_DISTANCE)
        hits = int(np.isfinite(distances).sum())
        if hits > best_hits:
            best, best_hits = candidate, hits
    return best


def match(start_points: np.ndarray, end_points: np.ndarray, initial: Motion) -> Optional[MatchResult]:
    """도착 스캔(end, 도착 자세 기준)을 출발 스캔(start, 출발 자세 기준)에 맞춰 도착 자세를 구한다.

    @return 점이 너무 적으면 None
    """
    if len(start_points) < 30 or len(end_points) < 30:
        return None
    tree = cKDTree(start_points)
    motion = _search_yaw(tree, end_points, initial)
    pair_distance = START_PAIR_DISTANCE
    for _ in range(MAX_ITERATIONS):
        moved = _apply(end_points, motion)
        distances, indices = tree.query(moved, distance_upper_bound=pair_distance)
        paired = np.isfinite(distances)
        if paired.sum() < 20:
            return None
        step = _best_rigid(moved[paired], start_points[indices[paired]])
        cos_step, sin_step = math.cos(step.yaw), math.sin(step.yaw)
        motion = Motion(
            step.x + cos_step * motion.x - sin_step * motion.y,
            step.y + sin_step * motion.x + cos_step * motion.y,
            normalize_angle(motion.yaw + step.yaw),
        )
        pair_distance = max(FINAL_PAIR_DISTANCE, pair_distance * 0.7)
        if abs(step.yaw) < 1e-6 and math.hypot(step.x, step.y) < 1e-5 and pair_distance == FINAL_PAIR_DISTANCE:
            break
    distances, _ = tree.query(_apply(end_points, motion), distance_upper_bound=FINAL_PAIR_DISTANCE)
    inliers = distances[np.isfinite(distances)]
    rmse = float(np.sqrt(np.mean(inliers ** 2))) if inliers.size else float('inf')
    return MatchResult(motion, rmse, float(inliers.size) / len(end_points))

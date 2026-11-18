import math

import numpy as np

from tetra_relocalization.scan_matcher import (
    MapMatcher, Pose2D, SearchWindow, base_pose_from_sensor, scan_to_points)

RESOLUTION = 0.05
LOCAL = SearchWindow(radius=1.0, yaw_range=math.radians(30), xy_step=0.1, yaw_step=math.radians(2))
WIDE = SearchWindow(radius=3.0, yaw_range=math.pi, xy_step=0.1, yaw_step=math.radians(3))


def room_map() -> np.ndarray:
    """10 m x 8 m 방. 대칭이 깨지도록 기둥과 벽 돌출을 둔다."""
    grid = np.zeros((160, 200), dtype=bool)
    grid[0, :] = grid[-1, :] = True
    grid[:, 0] = grid[:, -1] = True
    grid[40:46, 60:66] = True       # 기둥
    grid[100:160, 140] = True       # 칸막이
    grid[20, 150:200] = True        # 벽 돌출
    return grid


def corridor_map() -> np.ndarray:
    """특징 없는 긴 복도: 길이 방향으로는 어디에 있어도 똑같이 보인다."""
    grid = np.zeros((40, 400), dtype=bool)
    grid[10, :] = grid[30, :] = True
    return grid


def visible_points(grid: np.ndarray, sensor: Pose2D, max_range: float = 8.0, count: int = 180) -> np.ndarray:
    rows, cols = np.nonzero(grid)
    world = np.column_stack(((cols + 0.5) * RESOLUTION, (rows + 0.5) * RESOLUTION))
    rel = world - [sensor.x, sensor.y]
    keep = np.hypot(rel[:, 0], rel[:, 1]) < max_range
    rel = rel[keep]
    cos_yaw, sin_yaw = math.cos(sensor.yaw), math.sin(sensor.yaw)
    local = np.column_stack((cos_yaw * rel[:, 0] + sin_yaw * rel[:, 1], -sin_yaw * rel[:, 0] + cos_yaw * rel[:, 1]))
    pick = np.random.default_rng(0).choice(len(local), size=min(count, len(local)), replace=False)
    return local[pick]


def test_ratio_is_high_at_true_pose_and_low_when_rotated() -> None:
    grid = room_map()
    matcher = MapMatcher(grid, RESOLUTION, 0.0, 0.0, hit_tolerance=0.1)
    truth = Pose2D(4.0, 3.0, math.radians(20))
    points = visible_points(grid, truth)
    assert matcher.ratio(points, truth) > 0.95
    assert matcher.ratio(points, Pose2D(truth.x + 0.25, truth.y, truth.yaw + math.radians(12))) < 0.5


def test_local_search_recovers_pose_offset_like_the_m_robot() -> None:
    grid = room_map()
    matcher = MapMatcher(grid, RESOLUTION, 0.0, 0.0, hit_tolerance=0.1)
    truth = Pose2D(4.0, 3.0, math.radians(20))
    points = visible_points(grid, truth)
    result = matcher.search(points, Pose2D(4.2, 3.15, math.radians(32)), LOCAL, 0.5, math.radians(15))
    assert math.hypot(result.pose.x - truth.x, result.pose.y - truth.y) < 0.06
    assert abs(result.pose.yaw - truth.yaw) < math.radians(1.5)
    assert result.ratio > 0.9
    assert result.ratio - result.runner_up_ratio > 0.1


def test_wide_search_finds_a_kidnapped_robot() -> None:
    grid = room_map()
    matcher = MapMatcher(grid, RESOLUTION, 0.0, 0.0, hit_tolerance=0.1)
    truth = Pose2D(6.0, 5.0, math.radians(-100))
    points = visible_points(grid, truth)
    result = matcher.search(points, Pose2D(4.5, 3.5, math.radians(60)), WIDE, 0.5, math.radians(15))
    assert math.hypot(result.pose.x - truth.x, result.pose.y - truth.y) < 0.06
    assert abs(math.remainder(result.pose.yaw - truth.yaw, 2 * math.pi)) < math.radians(1.5)


def test_featureless_corridor_is_reported_as_ambiguous() -> None:
    grid = corridor_map()
    matcher = MapMatcher(grid, RESOLUTION, 0.0, 0.0, hit_tolerance=0.1)
    truth = Pose2D(10.0, 1.0, 0.0)
    points = visible_points(grid, truth, max_range=4.0)
    result = matcher.search(points, Pose2D(10.3, 1.05, math.radians(5)), LOCAL, 0.5, math.radians(15))
    assert result.ratio - result.runner_up_ratio < 0.1


def test_base_pose_from_sensor_undoes_the_mount_offset() -> None:
    mount = Pose2D(0.137, 0.0, 0.0)
    base = base_pose_from_sensor(Pose2D(1.0, 1.0, math.pi / 2), mount)
    assert math.isclose(base.x, 1.0, abs_tol=1e-9)
    assert math.isclose(base.y, 1.0 - 0.137, abs_tol=1e-9)
    assert math.isclose(base.yaw, math.pi / 2)


def test_scan_to_points_drops_invalid_ranges_and_subsamples() -> None:
    ranges = np.array([1.0, np.inf, 0.01, 2.0, 30.0, 1.0])
    points = scan_to_points(ranges, 0.0, math.pi / 2, 0.1, 15.0, max_points=2)
    assert points.shape == (2, 2)
    assert np.allclose(points[0], [1.0, 0.0])

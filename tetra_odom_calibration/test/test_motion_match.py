import math

import numpy as np

from tetra_odom_calibration.evaluation import ROTATION, STRAIGHT, evaluate, propose, unwrap_turn
from tetra_odom_calibration.motion_match import Motion, match, normalize_angle, scan_to_base_points
from tetra_odom_calibration.motion_plan import (MotionLimits, front_clearance, nearest_obstacle, rotation_speed,
                                                straight_speed)

RNG = np.random.default_rng(5)


def room_points() -> np.ndarray:
    """6 m x 4 m 방 벽과 기둥 두 개 (출발 자세 기준, 로봇은 방 가운데 근처)."""
    walls = []
    for x in np.arange(-2.0, 4.0, 0.03):
        walls += [(x, -2.0), (x, 2.0)]
    for y in np.arange(-2.0, 2.0, 0.03):
        walls += [(-2.0, y), (4.0, y)]
    for angle in np.arange(0.0, 2 * math.pi, 0.1):
        walls += [(2.5 + 0.2 * math.cos(angle), 1.0 + 0.2 * math.sin(angle)),
                  (1.0 + 0.15 * math.cos(angle), -1.2 + 0.15 * math.sin(angle))]
    return np.array(walls)


def seen_from(points: np.ndarray, pose: Motion, noise: float = 0.01) -> np.ndarray:
    """출발 기준 점들을 pose 에 선 로봇 기준으로 옮긴다 (+ 거리 잡음)."""
    shifted = points - np.array([pose.x, pose.y])
    cos_yaw, sin_yaw = math.cos(-pose.yaw), math.sin(-pose.yaw)
    local = np.column_stack((cos_yaw * shifted[:, 0] - sin_yaw * shifted[:, 1],
                             sin_yaw * shifted[:, 0] + cos_yaw * shifted[:, 1]))
    return local + RNG.normal(0.0, noise, local.shape)


def test_straight_drive_is_measured_from_scans() -> None:
    room = room_points()
    truth = Motion(1.5, 0.02, math.radians(0.5))
    # 오도메트리는 3% 짧게 잰 상태
    odom = Motion(1.455, 0.0, 0.0)
    result = match(seen_from(room, Motion(0, 0, 0)), seen_from(room, truth), odom)
    assert result is not None
    assert abs(result.motion.distance - truth.distance) < 0.005
    trial = evaluate(STRAIGHT, odom, 0.0, result)
    assert trial.accepted
    assert abs(trial.ratio - truth.distance / odom.distance) < 0.004


def test_two_turns_are_unwrapped() -> None:
    room = room_points()
    true_turn = 4 * math.pi * 1.02
    odom_turned = 4 * math.pi
    result = match(seen_from(room, Motion(0, 0, 0)), seen_from(room, Motion(0.0, 0.0, normalize_angle(true_turn))),
                   Motion(0.0, 0.0, normalize_angle(odom_turned)))
    trial = evaluate(ROTATION, Motion(0.0, 0.0, 0.0), odom_turned, result)
    assert trial.accepted
    assert abs(trial.measured - true_turn) < math.radians(0.2)
    assert abs(trial.ratio - 1.02) < 0.001


def test_unwrap_handles_negative_turns() -> None:
    assert abs(unwrap_turn(-4 * math.pi, math.radians(-5)) - (-4 * math.pi - math.radians(5))) < 1e-9


def test_changed_scene_is_rejected() -> None:
    room = room_points()
    other = room_points() * 1.3
    result = match(seen_from(room, Motion(0, 0, 0)), seen_from(other, Motion(1.0, 0.0, 0.0)), Motion(1.0, 0.0, 0.0))
    trial = evaluate(STRAIGHT, Motion(1.0, 0.0, 0.0), 0.0, result)
    assert not trial.accepted


def test_proposal_uses_median_of_accepted_trials() -> None:
    room = room_points()
    trials = []
    for true_distance in (1.010, 1.012, 1.011):
        result = match(seen_from(room, Motion(0, 0, 0)), seen_from(room, Motion(true_distance, 0, 0)),
                       Motion(1.0, 0, 0))
        trials.append(evaluate(STRAIGHT, Motion(1.0, 0.0, 0.0), 0.0, result))
    proposal = propose(1.0, trials, STRAIGHT)
    assert proposal is not None and proposal.trials == 3
    assert abs(proposal.scale - 1.011) < 0.003
    # 지금 배율이 이미 1.01 이면 그 위에 곱한다
    assert abs(propose(1.01, trials, STRAIGHT).scale - 1.01 * 1.011) < 0.003
    assert propose(1.0, trials, ROTATION) is None


def test_offset_lidar_points_are_moved_to_base() -> None:
    points = scan_to_base_points([1.0, float('inf'), 2.0], 0.0, math.pi / 2, 0.05, 10.0, Motion(0.3, 0.0, math.pi))
    assert np.allclose(points, [[-0.7, 0.0], [2.3, 0.0]], atol=1e-9)


def test_speed_profiles_slow_down_and_stop() -> None:
    limits = MotionLimits()
    assert straight_speed(0.0, 1.0, limits) == limits.linear_speed
    assert limits.min_linear_speed <= straight_speed(0.95, 1.0, limits) < limits.linear_speed
    assert straight_speed(1.0, 1.0, limits) == 0.0
    assert rotation_speed(0.0, 4 * math.pi, limits) == limits.angular_speed
    assert rotation_speed(4 * math.pi, 4 * math.pi, limits) == 0.0


def test_clearance_checks() -> None:
    points = np.array([[2.0, 0.1], [0.5, 1.0], [-0.4, 0.0]])
    assert front_clearance(points, 0.35) == 2.0
    assert abs(nearest_obstacle(points) - 0.4) < 1e-9
    assert math.isinf(front_clearance(np.empty((0, 2)), 0.35))

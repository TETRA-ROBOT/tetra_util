"""시험 주행 속도와 안전 거리. 느리게 움직여야 바퀴가 미끄러지지 않아 순수한 바퀴 오차만 잰다."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class MotionLimits:
    linear_speed: float = 0.15     # m/s
    min_linear_speed: float = 0.04
    angular_speed: float = 0.4     # rad/s
    min_angular_speed: float = 0.1
    slow_distance: float = 0.2     # 목표까지 이만큼 남으면 줄인다 (m)
    slow_angle: float = 0.4        # (rad)


def straight_speed(traveled: float, target: float, limits: MotionLimits) -> float:
    """@return 앞으로 보낼 속도. 목표에 닿았으면 0"""
    remaining = target - traveled
    if remaining <= 0.0:
        return 0.0
    scale = min(1.0, remaining / limits.slow_distance)
    return max(limits.min_linear_speed, limits.linear_speed * scale)


def rotation_speed(turned: float, target: float, limits: MotionLimits) -> float:
    """@param turned 지금까지 돈 크기 (부호 없음). @return 각속도 크기. 목표에 닿았으면 0"""
    remaining = target - turned
    if remaining <= 0.0:
        return 0.0
    scale = min(1.0, remaining / limits.slow_angle)
    return max(limits.min_angular_speed, limits.angular_speed * scale)


def front_clearance(points: np.ndarray, half_width: float) -> float:
    """로봇 앞 폭 2 * half_width 통로에서 가장 가까운 장애물까지의 앞쪽 거리 (base 기준). 없으면 inf"""
    if points.size == 0:
        return math.inf
    ahead = points[(points[:, 0] > 0.0) & (np.abs(points[:, 1]) < half_width)]
    return float(ahead[:, 0].min()) if ahead.size else math.inf


def nearest_obstacle(points: np.ndarray) -> float:
    """제자리 회전할 때 닿을 수 있는 가장 가까운 점까지 거리 (base 중심 기준)"""
    if points.size == 0:
        return math.inf
    return float(np.hypot(points[:, 0], points[:, 1]).min())

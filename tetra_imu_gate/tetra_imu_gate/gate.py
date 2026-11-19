"""바퀴가 서 있는 동안 IMU 를 EKF 로 넘기지 않을지 정한다 (ROS 와 무관한 순수 로직).

바퀴 회전 속도가 0 이면 로봇이 돌고 있지 않은 것이 확실하다. 그런데 EKF 는 그때도 IMU 회전 속도를 섞어
자이로 영점 오차가 방향으로 쌓인다. 2026-11-20 M 실기 정지 60 초: 자이로 평균 -50.7°/h, EKF 방향 -6.3°/h
(바퀴 드라이버가 정지 중 회전 분산을 1e-9 로 줘도 EKF process noise 가 매 주기 분산을 다시 키워 IMU 가 약 12% 섞였다).
몇 시간 서 있으면 수십 도가 틀어져 AMCL 이 위치를 잃었다. 정지 중에는 IMU 를 막아 EKF 가 바퀴(회전 0)만 보게 한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class GateConfig:
    # 바퀴 드라이버가 정지로 보는 문턱값과 같다 (tetra_wheel_driver.cpp FillOdometryCovariance)
    standstill_velocity: float = 1e-4
    # 멈춘 직후 관성으로 몸체가 조금 더 도는 동안은 IMU 를 그대로 넘긴다 (s)
    settle_time: float = 0.5
    # 바퀴 오도메트리가 이보다 오래 끊기면 IMU 로 이어 가야 하므로 막지 않는다 (s)
    odom_timeout: float = 0.5


class StandstillGate:
    """바퀴 오도메트리로 정지 여부를 추적하고, IMU 메시지를 넘길지 답한다. 시간은 초 단위."""

    def __init__(self, config: GateConfig) -> None:
        self._config = config
        self._last_odom_time: Optional[float] = None
        self._standstill_since: Optional[float] = None

    def on_odometry(self, now: float, linear_x: float, angular_z: float) -> None:
        self._last_odom_time = now
        limit = self._config.standstill_velocity
        if abs(linear_x) < limit and abs(angular_z) < limit:
            if self._standstill_since is None:
                self._standstill_since = now
        else:
            self._standstill_since = None

    def should_forward_imu(self, now: float) -> bool:
        if self._last_odom_time is None or now - self._last_odom_time > self._config.odom_timeout:
            return True
        if self._standstill_since is None:
            return True
        return now - self._standstill_since < self._config.settle_time

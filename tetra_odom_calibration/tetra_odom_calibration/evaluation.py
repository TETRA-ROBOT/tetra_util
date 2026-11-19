"""한 번의 직진·회전 시험 결과를 배율로 바꾸고, 여러 번 한 결과를 모아 적용할 값을 정한다."""
from dataclasses import asdict, dataclass
import math
import statistics
from typing import Dict, List, Optional

from .motion_match import MatchResult, Motion, normalize_angle

STRAIGHT = 'straight'
ROTATION = 'rotation'
# 드라이버(odom_scale.hpp)가 받는 범위
MIN_SCALE = 0.8
MAX_SCALE = 1.2
# 스캔 맞춤을 믿을 조건: 도착 스캔 점의 이만큼이 출발 스캔과 5 cm 안에서 겹치고, 그 오차가 이 정도
MIN_INLIER_RATIO = 0.6
MAX_RMSE = 0.03
# 한 번에 이보다 크게 다르면 바퀴가 미끄러졌거나 스캔을 잘못 맞춘 것이다
MAX_RATIO_ERROR = 0.1


@dataclass
class Trial:
    kind: str
    odom: float       # 오도메트리가 잰 거리(m) 또는 누적 회전(rad)
    measured: float   # 스캔으로 잰 값
    ratio: float      # measured / odom
    rmse: float
    inlier_ratio: float
    accepted: bool
    reason: str = ''

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


def unwrap_turn(odom_turned: float, scan_yaw: float) -> float:
    """스캔은 한 바퀴 안의 각도만 안다. 오도메트리 누적 회전에 둘의 차이(±180° 안)를 더해 실제 누적 회전을 구한다."""
    return odom_turned + normalize_angle(scan_yaw - normalize_angle(odom_turned))


def evaluate(kind: str, odom_motion: Motion, odom_turned: float, result: Optional[MatchResult]) -> Trial:
    """@param odom_motion 오도메트리로 본 출발 자세 기준 도착 자세. @param odom_turned 누적 회전(rad, 부호 포함)."""
    odom_value = odom_motion.distance if kind == STRAIGHT else odom_turned
    if result is None:
        return Trial(kind, odom_value, 0.0, 0.0, 0.0, 0.0, False, 'scan_too_sparse')
    measured = result.motion.distance if kind == STRAIGHT else unwrap_turn(odom_turned, result.motion.yaw)
    ratio = measured / odom_value if abs(odom_value) > 1e-6 else 0.0
    reason = ''
    if result.inlier_ratio < MIN_INLIER_RATIO or result.rmse > MAX_RMSE:
        reason = 'poor_scan_match'
    elif abs(ratio - 1.0) > MAX_RATIO_ERROR:
        reason = 'ratio_out_of_range'
    return Trial(kind, odom_value, measured, ratio, result.rmse, result.inlier_ratio, not reason, reason)


@dataclass
class Proposal:
    scale: float
    trials: int
    spread: float  # 받아들인 시험들의 비율 차이 (최대 - 최소)


def propose(current_scale: float, trials: List[Trial], kind: str) -> Optional[Proposal]:
    """받아들인 시험 비율의 중앙값을 지금 배율에 곱한다. 오도메트리는 지금 배율이 이미 들어간 값이다."""
    ratios = [trial.ratio for trial in trials if trial.kind == kind and trial.accepted]
    if not ratios:
        return None
    scale = min(MAX_SCALE, max(MIN_SCALE, current_scale * statistics.median(ratios)))
    return Proposal(round(scale, 5), len(ratios), max(ratios) - min(ratios))


def angle_deg(value: float) -> float:
    return math.degrees(value)

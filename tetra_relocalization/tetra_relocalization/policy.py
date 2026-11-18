"""위치를 잃었는지, 찾은 자세로 바로잡아도 되는지 정한다. ROS 없이 테스트한다 (test/test_policy.py).

AMCL 의 무작위 파티클 주입(recovery_alpha)은 사람이 지나가 점수가 잠깐 떨어져도 지도 곳곳에 파티클을 뿌려 위치를 튀게 했다
(EDU 2026-11-10, M 2026-11-19). 그래서 주입은 끄고, 여기서 '확실히 더 잘 맞고 헷갈리는 후보가 없는' 자세일 때만 바로잡는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .scan_matcher import Pose2D, SearchResult, normalize_angle


@dataclass(frozen=True)
class PolicyConfig:
    lost_ratio: float = 0.45          # 일치율이 이보다 낮으면 위치를 잃은 것으로 의심한다
    lost_duration: float = 3.0        # (s) 그 상태가 이만큼 이어져야 찾는다. 사람이 잠깐 가리는 것은 넘긴다
    accept_ratio: float = 0.70        # 찾은 자세가 이 이상 맞아야 바로잡는다
    min_gain: float = 0.25            # 지금 자세보다 이만큼은 더 맞아야 한다 (지도와 달라진 현장에서 헛보정 방지)
    min_margin: float = 0.10          # 다른 후보보다 이만큼은 더 맞아야 한다 (복도처럼 헷갈리는 곳에서는 바로잡지 않는다)
    min_correction_distance: float = 0.05  # (m) 이보다 작게 움직이는 보정은 AMCL 에 맡긴다
    min_correction_yaw: float = math.radians(1.0)
    retry_interval: float = 5.0       # (s) 찾기에 실패한 뒤 다시 찾기까지
    hold_after_initial_pose: float = 10.0  # (s) 사람이나 이 노드가 위치를 지정한 뒤 AMCL 이 수렴할 시간


class Decision(Enum):
    CORRECT = 'corrected'
    NO_MATCH = 'no_match'             # 충분히 맞는 자세가 없다 (지도와 현장이 다르거나 탐색 범위 밖)
    NO_GAIN = 'no_gain'               # 지금 자세와 별 차이가 없다
    AMBIGUOUS = 'ambiguous'           # 비슷하게 맞는 다른 자세가 있다
    ALREADY_ALIGNED = 'already_aligned'


def decide(config: PolicyConfig, current: Pose2D, current_ratio: float, result: SearchResult) -> Decision:
    if result.ratio < config.accept_ratio:
        return Decision.NO_MATCH
    if result.ratio - current_ratio < config.min_gain:
        return Decision.NO_GAIN
    if result.ratio - result.runner_up_ratio < config.min_margin:
        return Decision.AMBIGUOUS
    moved = math.hypot(result.pose.x - current.x, result.pose.y - current.y)
    turned = abs(normalize_angle(result.pose.yaw - current.yaw))
    if moved < config.min_correction_distance and turned < config.min_correction_yaw:
        return Decision.ALREADY_ALIGNED
    return Decision.CORRECT


class LostDetector:
    """일치율이 lost_ratio 밑으로 lost_duration 넘게 이어지면 찾기를 요청한다. hold() 동안은 요청하지 않는다."""

    def __init__(self, config: PolicyConfig) -> None:
        self._config = config
        self._low_since: float | None = None
        self._hold_until = -math.inf

    def update(self, now: float, ratio: float) -> bool:
        if now < self._hold_until:
            return False
        if ratio >= self._config.lost_ratio:
            self._low_since = None
            return False
        if self._low_since is None:
            self._low_since = now
        return now - self._low_since >= self._config.lost_duration

    def hold(self, now: float, seconds: float) -> None:
        self._hold_until = max(self._hold_until, now + seconds)
        self._low_since = None

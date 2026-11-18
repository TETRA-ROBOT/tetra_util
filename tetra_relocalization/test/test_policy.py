import math

from tetra_relocalization.policy import Decision, LostDetector, PolicyConfig, decide
from tetra_relocalization.scan_matcher import Pose2D, SearchResult

CONFIG = PolicyConfig()
CURRENT = Pose2D(2.03, -1.54, math.radians(21.6))


def test_m_robot_measurement_is_corrected() -> None:
    # 2026-11-19 M 실기: AMCL 자세 23%, 0.25 m·12° 옆 자세 87%
    found = SearchResult(Pose2D(1.83, -1.39, math.radians(9.5)), 0.87, 0.55)
    assert decide(CONFIG, CURRENT, 0.23, found) is Decision.CORRECT


def test_poor_best_match_is_not_applied() -> None:
    found = SearchResult(Pose2D(1.83, -1.39, 0.0), 0.60, 0.30)
    assert decide(CONFIG, CURRENT, 0.20, found) is Decision.NO_MATCH


def test_changed_site_where_nothing_fits_much_better_is_left_alone() -> None:
    found = SearchResult(Pose2D(1.83, -1.39, 0.0), 0.75, 0.40)
    assert decide(CONFIG, CURRENT, 0.60, found) is Decision.NO_GAIN


def test_two_similar_candidates_are_ambiguous() -> None:
    found = SearchResult(Pose2D(1.83, -1.39, 0.0), 0.85, 0.80)
    assert decide(CONFIG, CURRENT, 0.30, found) is Decision.AMBIGUOUS


def test_tiny_correction_is_left_to_amcl() -> None:
    found = SearchResult(Pose2D(CURRENT.x + 0.02, CURRENT.y, CURRENT.yaw), 0.90, 0.40)
    assert decide(CONFIG, CURRENT, 0.40, found) is Decision.ALREADY_ALIGNED


def test_short_occlusion_does_not_trigger_search() -> None:
    detector = LostDetector(CONFIG)
    assert not detector.update(0.0, 0.2)
    assert not detector.update(2.0, 0.2)
    assert not detector.update(2.5, 0.8)   # 사람이 지나갔다
    assert not detector.update(5.0, 0.2)
    assert detector.update(8.0, 0.2)


def test_hold_suppresses_search_and_restarts_the_timer() -> None:
    detector = LostDetector(CONFIG)
    detector.update(0.0, 0.2)
    detector.hold(3.0, 10.0)
    assert not detector.update(12.0, 0.2)
    assert not detector.update(13.0, 0.2)
    assert not detector.update(15.0, 0.2)
    assert detector.update(16.0, 0.2)

"""합성 체커보드로 인식·표본 판단·계산을 검사한다 (카메라 없이)."""
import math

import cv2
import numpy as np
import pytest

from tetra_camera_calibration import board as board_lib
from tetra_camera_calibration.session import CalibrationSession

BOARD = board_lib.BoardSpec(cols=8, rows=6, square_size=0.025)
WIDTH, HEIGHT = 640, 480
TRUE_K = np.array([[846.7, 0.0, 316.1], [0.0, 848.3, 272.6], [0.0, 0.0, 1.0]])
TRUE_D = np.array([0.168, -0.296, 0.0063, -0.0108, 0.0])


def render_board(homography: np.ndarray) -> np.ndarray:
    """칸 40 px 짜리 보드를 그려 homography 로 옮긴 RGB 영상."""
    square = 40
    texture = np.full(((BOARD.rows + 3) * square, (BOARD.cols + 3) * square), 255, np.uint8)
    for r in range(BOARD.rows + 1):
        for c in range(BOARD.cols + 1):
            if (r + c) % 2 == 0:
                y, x = (r + 1) * square, (c + 1) * square
                texture[y:y + square, x:x + square] = 0
    warped = cv2.warpPerspective(texture, homography, (WIDTH, HEIGHT), borderValue=255)
    return cv2.cvtColor(warped, cv2.COLOR_GRAY2RGB)


def shifted(dx: float, dy: float, scale: float = 1.0) -> np.ndarray:
    return np.array([[scale, 0, dx], [0, scale, dy], [0, 0, 1]], dtype=np.float64)


def synthetic_views():
    """여러 자세의 보드를 왜곡 있는 카메라로 투영한 모서리 좌표."""
    views = []
    for rx in (-0.4, 0.0, 0.4):
        for ry in (-0.4, 0.0, 0.4):
            for tz in (0.5, 0.8):
                rvec = np.array([rx, ry, 0.1 * rx])
                tvec = np.array([-0.09, -0.06, tz])
                points, _ = cv2.projectPoints(BOARD.object_points(), rvec, tvec, TRUE_K, TRUE_D)
                views.append(points.astype(np.float32))
    return views


def test_detects_board_corners():
    corners = board_lib.detect_board(cv2.cvtColor(render_board(shifted(100, 80)), cv2.COLOR_RGB2GRAY), BOARD)
    assert corners is not None
    assert corners.shape == (BOARD.cols * BOARD.rows, 1, 2)


def test_no_board_returns_none():
    blank = np.full((HEIGHT, WIDTH), 200, np.uint8)
    assert board_lib.detect_board(blank, BOARD) is None


def test_params_follow_board_position():
    left = board_lib.board_params(board_lib.detect_board(
        cv2.cvtColor(render_board(shifted(10, 80)), cv2.COLOR_RGB2GRAY), BOARD), BOARD, WIDTH, HEIGHT)
    right = board_lib.board_params(board_lib.detect_board(
        cv2.cvtColor(render_board(shifted(260, 80)), cv2.COLOR_RGB2GRAY), BOARD), BOARD, WIDTH, HEIGHT)
    assert right[0] > left[0] + 0.3
    assert abs(right[1] - left[1]) < 0.05
    assert left[3] < 0.05  # 정면이면 기울기 0


def test_new_sample_and_coverage():
    samples = [[0.1, 0.1, 0.3, 0.0]]
    assert not board_lib.is_new_sample([0.12, 0.1, 0.3, 0.0], samples)
    assert board_lib.is_new_sample([0.6, 0.1, 0.3, 0.0], samples)
    covered = board_lib.coverage(samples + [[0.8, 0.8, 0.6, 0.5]])
    assert covered == pytest.approx([1.0, 1.0, 0.75, 1.0])


def test_calibrate_recovers_intrinsics():
    result = board_lib.calibrate(synthetic_views(), BOARD, WIDTH, HEIGHT)
    assert result.rms < 0.05
    assert result.camera_matrix[0] == pytest.approx(TRUE_K[0, 0], rel=0.01)
    assert result.camera_matrix[2] == pytest.approx(TRUE_K[0, 2], abs=2.0)
    assert result.distortion[0] == pytest.approx(TRUE_D[0], abs=0.02)
    assert len(result.projection) == 12 and result.projection[3] == 0.0


def test_calibrate_needs_enough_samples():
    with pytest.raises(ValueError):
        board_lib.calibrate(synthetic_views()[:3], BOARD, WIDTH, HEIGHT)


def test_session_auto_captures_still_new_boards_once():
    session = CalibrationSession(BOARD)
    frame = render_board(shifted(60, 60))
    session.process_frame(frame)  # 첫 프레임은 비교할 이전 프레임이 없어 찍지 않는다
    session.process_frame(frame)
    session.process_frame(frame)  # 같은 자리는 다시 찍지 않는다
    assert session.status()['samples'] == 1
    moved = render_board(shifted(300, 200, 0.8))
    session.process_frame(moved)
    session.process_frame(moved)
    assert session.status()['samples'] == 2


def test_manual_capture_requires_visible_still_board():
    session = CalibrationSession(BOARD, auto_capture=False)
    assert not session.capture()['success']
    frame = render_board(shifted(60, 60))
    session.process_frame(frame)
    assert not session.capture()['success']  # 이전 프레임이 없으면 멈춘 것인지 모른다
    session.process_frame(frame)
    assert session.capture()['success']
    assert session.status()['samples'] == 1


def test_board_params_skew_for_rotated_board():
    angle = math.radians(25)
    homography = np.array([[math.cos(angle), -0.4, 150], [0.0, 1.0, 60], [0.0, 0.0006, 1.0]])
    corners = board_lib.detect_board(cv2.cvtColor(render_board(homography), cv2.COLOR_RGB2GRAY), BOARD)
    assert corners is not None
    assert board_lib.board_params(corners, BOARD, WIDTH, HEIGHT)[3] > 0.1

"""체커보드 인식, 표본 다양성(커버리지) 판단, 내부 파라미터 계산. ROS 와 무관한 순수 OpenCV 코드.

표본 판단은 ROS camera_calibration(cameracalibrator.py)과 같은 기준을 쓴다:
보드 위치 x, y, 크기, 기울기 네 값이 기존 표본들과 충분히 달라야 새 표본으로 받는다.
"""
from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 표본 사이 네 값(x, y, 크기, 기울기) 차이 합이 이보다 커야 새 표본으로 받는다 (ROS camera_calibration 과 같음)
MIN_SAMPLE_DISTANCE = 0.2
# 각 값이 이 폭만큼 퍼지면 그 항목 커버리지 100% (ROS: x 0.7, y 0.7, size 0.4, skew 0.5)
COVERAGE_RANGES = (0.7, 0.7, 0.4, 0.5)
# 프레임 사이 모서리 평균 이동이 이보다 크면 흔들리는 중으로 보고 찍지 않는다 (어두우면 노출이 길어 번진다)
MAX_STILL_MOTION_PX = 2.0
MIN_SAMPLES_TO_CALIBRATE = 8
RECOMMENDED_SAMPLES = 15


@dataclass
class BoardSpec:
    """cols, rows 는 안쪽 모서리 개수 (칸 수 - 1), square_size 는 한 칸 길이(m)."""

    cols: int
    rows: int
    square_size: float

    def object_points(self) -> np.ndarray:
        grid = np.zeros((self.cols * self.rows, 3), np.float32)
        grid[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_size
        return grid


@dataclass
class Detection:
    corners: np.ndarray  # (N, 1, 2) float32, 영상 좌표
    params: Tuple[float, float, float, float]  # x, y, size, skew (0~1)


@dataclass
class CalibrationResult:
    width: int
    height: int
    camera_matrix: List[float]  # k 3x3 행 우선
    distortion: List[float]  # plumb_bob 5개
    projection: List[float]  # p 3x4 행 우선
    rms: float  # 재투영 오차 (px)
    per_sample_errors: List[float] = field(default_factory=list)


def detect_board(gray: np.ndarray, board: BoardSpec) -> Optional[np.ndarray]:
    """보드를 찾으면 서브픽셀로 다듬은 모서리를, 못 찾으면 None 을 돌려준다."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, (board.cols, board.rows), flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    # 다듬는 창은 칸 크기보다 작아야 옆 모서리로 끌려가지 않는다
    spacing = _min_corner_spacing(corners, board)
    radius = int(max(2, min(11, math.floor(spacing * 0.5))))
    return cv2.cornerSubPix(gray, corners, (radius, radius), (-1, -1), criteria)


def _min_corner_spacing(corners: np.ndarray, board: BoardSpec) -> float:
    points = corners.reshape(board.rows, board.cols, 2)
    horizontal = np.linalg.norm(np.diff(points, axis=1), axis=2).min()
    vertical = np.linalg.norm(np.diff(points, axis=0), axis=2).min()
    return float(min(horizontal, vertical))


def board_params(corners: np.ndarray, board: BoardSpec, width: int, height: int) -> Tuple[float, float, float, float]:
    """보드의 영상 내 위치·크기·기울기를 0~1 로 나타낸다 (ROS camera_calibration _get_parameters 와 같은 정의)."""
    points = corners.reshape(-1, 2)
    up_left = points[0]
    up_right = points[board.cols - 1]
    down_right = points[-1]
    down_left = points[-board.cols]
    area = _quad_area(up_left, up_right, down_right, down_left)
    border = math.sqrt(area)
    mean_x, mean_y = points.mean(axis=0)
    p_x = _clamp01((mean_x - border / 2.0) / max(1.0, width - border))
    p_y = _clamp01((mean_y - border / 2.0) / max(1.0, height - border))
    p_size = math.sqrt(area / float(width * height))
    skew = _clamp01(2.0 * abs(math.pi / 2.0 - _angle(up_left, up_right, down_right)))
    return (p_x, p_y, p_size, skew)


def _quad_area(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> float:
    def cross(u: np.ndarray, v: np.ndarray) -> float:
        return float(u[0] * v[1] - u[1] * v[0])
    return abs(cross(b - a, c - a) + cross(c - a, d - a)) / 2.0


def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    ab = a - b
    cb = c - b
    cosine = float(np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb)))
    return math.acos(max(-1.0, min(1.0, cosine)))


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def is_still(corners: np.ndarray, previous: Optional[np.ndarray]) -> bool:
    if previous is None or previous.shape != corners.shape:
        return False
    motion = np.linalg.norm(corners.reshape(-1, 2) - previous.reshape(-1, 2), axis=1).mean()
    return float(motion) < MAX_STILL_MOTION_PX


def is_new_sample(params: Sequence[float], samples: Sequence[Sequence[float]]) -> bool:
    return all(sum(abs(a - b) for a, b in zip(params, s)) > MIN_SAMPLE_DISTANCE for s in samples)


def coverage(samples: Sequence[Sequence[float]]) -> List[float]:
    """x, y, 크기, 기울기 각각 얼마나 고르게 찍었는지 0~1."""
    if not samples:
        return [0.0, 0.0, 0.0, 0.0]
    values = np.array(samples)
    spans = values.max(axis=0) - values.min(axis=0)
    return [float(min(1.0, span / target)) for span, target in zip(spans, COVERAGE_RANGES)]


def is_ready(samples: Sequence[Sequence[float]]) -> bool:
    if len(samples) >= 40:
        return True
    return len(samples) >= RECOMMENDED_SAMPLES and min(coverage(samples)) >= 0.7


def calibrate(corner_sets: Sequence[np.ndarray], board: BoardSpec, width: int, height: int) -> CalibrationResult:
    """plumb_bob(k1, k2, p1, p2, k3=0) 으로 계산한다. k3 는 ROS camera_calibration 기본값처럼 고정한다."""
    if len(corner_sets) < MIN_SAMPLES_TO_CALIBRATE:
        raise ValueError(f'표본이 {MIN_SAMPLES_TO_CALIBRATE}장 이상 필요합니다 (지금 {len(corner_sets)}장)')
    object_points = [board.object_points() for _ in corner_sets]
    image_points = [c.astype(np.float32) for c in corner_sets]
    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, (width, height), None, None, flags=cv2.CALIB_FIX_K3)
    d = np.asarray(d, dtype=np.float64).reshape(-1)[:5]
    # 보정한 영상의 내부 파라미터 p: 검은 테두리 없이 유효 영역만 남기는 alpha=0 (ROS camera_calibration 기본)
    new_k, _ = cv2.getOptimalNewCameraMatrix(k, d, (width, height), 0.0, (width, height))
    projection = np.zeros((3, 4))
    projection[:, :3] = new_k
    errors = []
    for objp, imgp, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        reprojected, _ = cv2.projectPoints(objp, rvec, tvec, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum((reprojected - imgp) ** 2, axis=2)))))
    return CalibrationResult(
        width=width,
        height=height,
        camera_matrix=[float(v) for v in k.reshape(-1)],
        distortion=[float(v) for v in d],
        projection=[float(v) for v in projection.reshape(-1)],
        rms=float(rms),
        per_sample_errors=errors,
    )

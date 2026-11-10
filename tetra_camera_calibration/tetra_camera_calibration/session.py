"""한 번의 캘리브레이션 작업 상태: 프레임마다 보드를 찾고, 자동/수동으로 표본을 모으고, 계산한다. ROS 와 무관."""
import threading
from typing import Dict, List, Optional

import cv2
import numpy as np

from . import board as board_lib

CAPTURE_FLASH_FRAMES = 3


class CalibrationSession:

    def __init__(self, board: board_lib.BoardSpec, auto_capture: bool = True) -> None:
        self.board = board
        self.auto_capture = auto_capture
        self._lock = threading.Lock()
        self._samples: List[List[float]] = []
        self._corner_sets: List[np.ndarray] = []
        self._previous_corners: Optional[np.ndarray] = None
        self._current: Optional[board_lib.Detection] = None
        self._current_still = False
        self._image_size: Optional[tuple] = None
        self._flash = 0
        self.result: Optional[board_lib.CalibrationResult] = None
        self.last_message = ''

    def process_frame(self, rgb: np.ndarray) -> np.ndarray:
        """보드를 찾아 (자동이면) 표본으로 넣고, 미리보기용 BGR 영상을 돌려준다."""
        height, width = rgb.shape[:2]
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners = board_lib.detect_board(gray, self.board)
        with self._lock:
            if self._image_size is not None and self._image_size != (width, height):
                # 해상도가 바뀌면 이전 표본과 섞어 계산할 수 없다
                self._reset_locked('영상 해상도가 바뀌어 표본을 비웠습니다')
            self._image_size = (width, height)
            detection = None
            still = False
            if corners is not None:
                params = board_lib.board_params(corners, self.board, width, height)
                detection = board_lib.Detection(corners=corners, params=params)
                still = board_lib.is_still(corners, self._previous_corners)
                if self.auto_capture and still and board_lib.is_new_sample(params, self._samples):
                    self._add_locked(detection)
            self._previous_corners = corners
            self._current = detection
            self._current_still = still
            preview = self._draw_locked(rgb, detection)
        return preview

    def capture(self) -> Dict[str, object]:
        """지금 보이는 보드를 표본으로 넣는다 (비슷한 표본이 있어도 넣는다)."""
        with self._lock:
            if self._current is None:
                return {'success': False, 'message': '보드가 보이지 않습니다'}
            if not self._current_still:
                return {'success': False, 'message': '보드가 움직이고 있습니다. 잠시 멈춰 주세요'}
            self._add_locked(self._current)
            return {'success': True, 'message': f'표본 {len(self._samples)}장'}

    def calibrate(self) -> board_lib.CalibrationResult:
        with self._lock:
            corner_sets = list(self._corner_sets)
            size = self._image_size
        if size is None:
            raise ValueError('아직 영상을 받지 못했습니다')
        result = board_lib.calibrate(corner_sets, self.board, size[0], size[1])
        with self._lock:
            self.result = result
        return result

    def reset(self) -> None:
        with self._lock:
            self._reset_locked('표본을 비웠습니다')

    def status(self) -> Dict[str, object]:
        with self._lock:
            result = self.result
            return {
                'board': {'cols': self.board.cols, 'rows': self.board.rows, 'squareSize': self.board.square_size},
                'autoCapture': self.auto_capture,
                'boardFound': self._current is not None,
                'samples': len(self._samples),
                'minSamples': board_lib.MIN_SAMPLES_TO_CALIBRATE,
                'recommendedSamples': board_lib.RECOMMENDED_SAMPLES,
                'coverage': dict(zip(('x', 'y', 'size', 'skew'), board_lib.coverage(self._samples))),
                'ready': board_lib.is_ready(self._samples),
                'imageSize': list(self._image_size) if self._image_size else None,
                'result': None if result is None else {
                    'rms': result.rms,
                    'fx': result.camera_matrix[0], 'fy': result.camera_matrix[4],
                    'cx': result.camera_matrix[2], 'cy': result.camera_matrix[5],
                    'distortion': result.distortion,
                    'worstSampleError': max(result.per_sample_errors) if result.per_sample_errors else None,
                },
                'message': self.last_message,
            }

    def _add_locked(self, detection: board_lib.Detection) -> None:
        self._samples.append(list(detection.params))
        self._corner_sets.append(detection.corners.copy())
        # 표본이 바뀌면 예전 계산 결과는 맞지 않는다
        self.result = None
        self._flash = CAPTURE_FLASH_FRAMES
        self.last_message = f'표본 {len(self._samples)}장'

    def _reset_locked(self, message: str) -> None:
        self._samples.clear()
        self._corner_sets.clear()
        self._previous_corners = None
        self.result = None
        self.last_message = message

    def _draw_locked(self, rgb: np.ndarray, detection: Optional[board_lib.Detection]) -> np.ndarray:
        preview = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        # 이미 찍은 자리를 흐리게 보여 줘 빈 곳으로 보드를 옮기게 한다
        for corners in self._corner_sets:
            hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.int32))
            cv2.polylines(preview, [hull], True, (90, 160, 90), 1)
        if detection is not None:
            cv2.drawChessboardCorners(preview, (self.board.cols, self.board.rows), detection.corners, True)
        if self._flash > 0:
            # 표본을 찍은 순간 테두리를 초록으로 깜빡인다
            cv2.rectangle(preview, (0, 0), (preview.shape[1] - 1, preview.shape[0] - 1), (0, 220, 0), 8)
            self._flash -= 1
        return preview

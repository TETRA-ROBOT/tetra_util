"""CameraInfo 메시지와 보정 결과·보정 파일 사이 변환, 저장 위치 검사."""
import os
from typing import Dict, Optional
from urllib.parse import urlparse

import yaml
from sensor_msgs.msg import CameraInfo

from .board import CalibrationResult

IDENTITY_3X3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# 로봇별 저장소. 실행 스크립트(start_real_robot.sh)와 관제 API 가 환경 변수 PERSISTENT_DIR 로 정한다.
# 변수가 없을 때만 이 기본값을 쓴다 (후면 카메라 런치 tetra_bringup/launch/usb_camera.launch.py 와 같은 규칙)
DEFAULT_PERSISTENT_DIR = '/home/tetra/tetra_integration/persistent'


def default_persistent_dir() -> str:
    """persistent_dir 파라미터를 주지 않았을 때 쓸 로봇별 저장소 경로."""
    return os.environ.get('PERSISTENT_DIR') or DEFAULT_PERSISTENT_DIR


def result_to_camera_info(result: CalibrationResult, frame_id: str) -> CameraInfo:
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = result.width
    info.height = result.height
    info.distortion_model = 'plumb_bob'
    info.d = list(result.distortion)
    info.k = list(result.camera_matrix)
    info.r = list(IDENTITY_3X3)
    info.p = list(result.projection)
    return info


def load_camera_info_yaml(path: str, frame_id: str) -> CameraInfo:
    """camera_calibration_parsers 형식 yaml (usb_cam config/camera_info.yaml) 을 읽는다."""
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = int(data['image_width'])
    info.height = int(data['image_height'])
    info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
    info.d = [float(v) for v in data['distortion_coefficients']['data']]
    info.k = [float(v) for v in data['camera_matrix']['data']]
    info.r = [float(v) for v in data.get('rectification_matrix', {}).get('data', IDENTITY_3X3)]
    info.p = [float(v) for v in data['projection_matrix']['data']]
    return info


def camera_info_summary(info: Optional[CameraInfo]) -> Optional[Dict[str, object]]:
    """화면에 보일 값만 추린다 (fx, fy, cx, cy, 왜곡 계수)."""
    if info is None:
        return None
    k = list(info.k)
    return {
        'width': int(info.width),
        'height': int(info.height),
        'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5],
        'distortion': [float(v) for v in info.d],
        'calibrated': any(abs(v) > 1e-9 for v in k),
    }


def calibration_file_from_url(url: str) -> Optional[str]:
    """camera_info_url 이 file:// 이면 그 경로, 아니면 None (package:// 는 소스의 공용 파일이다)."""
    if not url:
        return None
    parsed = urlparse(url)
    if parsed.scheme != 'file':
        return None
    return os.path.realpath(parsed.path)


def is_robot_specific_file(path: Optional[str], persistent_dir: str) -> bool:
    """로봇별 저장소(PERSISTENT_DIR) 안의 파일이어야 저장한다. 공용 기본값을 덮어 다른 로봇까지 바뀌면 안 된다."""
    if not path:
        return False
    root = os.path.realpath(persistent_dir)
    return os.path.commonpath([root, path]) == root

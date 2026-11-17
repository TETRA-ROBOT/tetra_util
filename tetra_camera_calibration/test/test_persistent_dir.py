"""로봇별 저장소 경로: 환경 변수 PERSISTENT_DIR 이 먼저고, 없을 때만 기본 경로를 쓴다."""
from tetra_camera_calibration import camera_info_io


def test_environment_variable_wins(monkeypatch):
    monkeypatch.setenv('PERSISTENT_DIR', '/data/robot-7/persistent')
    assert camera_info_io.default_persistent_dir() == '/data/robot-7/persistent'


def test_default_when_the_variable_is_missing(monkeypatch):
    monkeypatch.delenv('PERSISTENT_DIR', raising=False)
    assert camera_info_io.default_persistent_dir() == camera_info_io.DEFAULT_PERSISTENT_DIR


def test_empty_variable_counts_as_missing(monkeypatch):
    # 빈 값을 그대로 쓰면 저장 위치가 상대 경로 'camera_info/...' 가 된다
    monkeypatch.setenv('PERSISTENT_DIR', '')
    assert camera_info_io.default_persistent_dir() == camera_info_io.DEFAULT_PERSISTENT_DIR


def test_calibration_file_must_be_inside_the_persistent_dir(tmp_path):
    inside = tmp_path / 'camera_info' / 'rear_camera.yaml'
    assert camera_info_io.is_robot_specific_file(str(inside), str(tmp_path))
    assert not camera_info_io.is_robot_specific_file('/opt/ros/share/usb_cam/config/camera_info.yaml', str(tmp_path))

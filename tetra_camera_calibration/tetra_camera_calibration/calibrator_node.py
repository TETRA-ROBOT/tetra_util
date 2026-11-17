"""카메라 캘리브레이션 노드. 웹 관제 관리자 화면(카메라 캘리브레이션)이 필요할 때만 띄운다.

영상을 받아 체커보드 표본을 모으고 내부 파라미터를 계산한 뒤, 카메라 드라이버의 set_camera_info 로 적용한다.
드라이버(camera_info_manager)는 적용과 동시에 camera_info_url 파일에 저장하고 새 camera_info 를 바로 발행한다.
"""
import os
import shutil
import threading
import time
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.srv import SetCameraInfo

from . import board as board_lib
from . import camera_info_io
from .http_api import start_server
from .session import CalibrationSession

FUTURE_TIMEOUT_S = 5.0


class CalibratorNode(Node):

    def __init__(self) -> None:
        super().__init__('camera_calibrator')
        self.camera_node = self.declare_parameter('camera_node', '/camera').value
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        info_topic = self.declare_parameter('camera_info_topic', '/camera/camera_info').value
        set_info_service = self.declare_parameter('set_camera_info_service', '/camera/set_camera_info').value
        # 관제 API 는 이 파라미터를 직접 넘긴다. 손으로 띄울 때는 환경 변수 PERSISTENT_DIR, 그것도 없으면 기본 경로
        self.persistent_dir = self.declare_parameter('persistent_dir', camera_info_io.default_persistent_dir()).value
        self.default_info_path = self.declare_parameter('default_camera_info', '').value or default_camera_info_path()
        port = int(self.declare_parameter('http_port', 8091).value)
        self.period = 1.0 / float(self.declare_parameter('process_rate', 5.0).value)
        self.idle_timeout = float(self.declare_parameter('idle_timeout', 600.0).value)
        spec = board_lib.BoardSpec(
            cols=int(self.declare_parameter('board_cols', 8).value),
            rows=int(self.declare_parameter('board_rows', 6).value),
            square_size=float(self.declare_parameter('square_size', 0.025).value),
        )
        self.session = CalibrationSession(spec)
        self.stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_image: Optional[Image] = None
        self._last_image_time = 0.0
        self._image_error = ''
        self._jpeg_cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._jpeg_seq = 0
        self._camera_info: Optional[CameraInfo] = None
        self._last_touch = time.monotonic()

        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.set_info_client = self.create_client(SetCameraInfo, set_info_service)
        self.param_client = AsyncParameterClient(self, self.camera_node)
        self.create_timer(5.0, self._check_idle)
        threading.Thread(target=self._worker, name='calibration-worker', daemon=True).start()
        self.server = start_server(port, self, self.stop_event)
        self.get_logger().info(
            f'camera calibration on 127.0.0.1:{port}, board {spec.cols}x{spec.rows} {spec.square_size * 1000:.1f} mm, '
            f'image {image_topic}')

    # ---- ROS 콜백 ----
    def _on_image(self, msg: Image) -> None:
        with self._frame_lock:
            self._latest_image = msg
            self._last_image_time = time.monotonic()

    def _on_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _check_idle(self) -> None:
        # 화면을 닫고 정지를 못 보냈어도 혼자 남아 CPU 를 쓰지 않게 한다
        if time.monotonic() - self._last_touch > self.idle_timeout:
            self.get_logger().info('no requests for a while, exiting')
            self.stop_event.set()
            rclpy.try_shutdown()

    # ---- 영상 처리 스레드 ----
    def _worker(self) -> None:
        while not self.stop_event.is_set():
            started = time.monotonic()
            with self._frame_lock:
                msg, self._latest_image = self._latest_image, None
            if msg is not None:
                self._process(msg)
            time.sleep(max(0.0, self.period - (time.monotonic() - started)))

    def _process(self, msg: Image) -> None:
        rgb = image_to_rgb(msg)
        if rgb is None:
            self._image_error = f'지원하지 않는 영상 형식: {msg.encoding}'
            return
        self._image_error = ''
        preview = self.session.process_frame(rgb)
        ok, encoded = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            return
        with self._jpeg_cond:
            self._jpeg = encoded.tobytes()
            self._jpeg_seq += 1
            self._jpeg_cond.notify_all()

    # ---- http_api.CalibratorCommands ----
    def touch(self) -> None:
        self._last_touch = time.monotonic()

    def latest_jpeg(self, after: int, timeout: float) -> Optional[tuple]:
        with self._jpeg_cond:
            self._jpeg_cond.wait_for(lambda: self._jpeg_seq > after or self.stop_event.is_set(), timeout)
            if self._jpeg is None or self._jpeg_seq <= after:
                return None
            return self._jpeg_seq, self._jpeg

    def status(self) -> Dict[str, object]:
        payload = self.session.status()
        age = time.monotonic() - self._last_image_time if self._last_image_time else None
        payload['imageAgeSec'] = None if age is None else round(age, 2)
        payload['imageError'] = self._image_error
        payload['currentCameraInfo'] = camera_info_io.camera_info_summary(self._camera_info)
        return payload

    def command(self, name: str, body: Dict[str, object]) -> Dict[str, object]:
        try:
            if name == 'capture':
                return self.session.capture()
            if name == 'reset':
                self.session.reset()
                return {'success': True, 'message': '표본을 비웠습니다'}
            if name == 'auto':
                enabled = body.get('enabled')
                if not isinstance(enabled, bool):
                    return {'success': False, 'message': 'enabled 는 true/false 여야 합니다'}
                self.session.auto_capture = enabled
                return {'success': True, 'message': ''}
            if name == 'calibrate':
                result = self.session.calibrate()
                return {'success': True, 'message': f'재투영 오차 {result.rms:.3f} px', 'status': self.status()}
            if name == 'apply':
                return self._apply_result()
            if name == 'restore-default':
                return self._restore_default()
        except ValueError as error:
            return {'success': False, 'message': str(error)}
        return {'success': False, 'message': f'unknown command {name}'}

    # ---- 적용 ----
    def _apply_result(self) -> Dict[str, object]:
        result = self.session.result
        if result is None:
            return {'success': False, 'message': '먼저 계산하세요'}
        frame_id = self._camera_info.header.frame_id if self._camera_info else 'camera'
        return self._apply(camera_info_io.result_to_camera_info(result, frame_id))

    def _restore_default(self) -> Dict[str, object]:
        path = self.default_info_path
        if not path or not os.path.isfile(path):
            return {'success': False, 'message': f'기본 보정 파일이 없습니다: {path}'}
        frame_id = self._camera_info.header.frame_id if self._camera_info else 'camera'
        return self._apply(camera_info_io.load_camera_info_yaml(path, frame_id))

    def _apply(self, info: CameraInfo) -> Dict[str, object]:
        url = self._camera_info_url()
        if url is None:
            return {'success': False, 'message': f'카메라 드라이버({self.camera_node})의 camera_info_url 을 읽지 못했습니다'}
        target = camera_info_io.calibration_file_from_url(url)
        if not camera_info_io.is_robot_specific_file(target, self.persistent_dir):
            return {'success': False, 'message': (
                f'카메라가 로봇별 보정 파일이 아닌 {url} 을 쓰고 있어 저장하지 않았습니다 '
                '(공용 기본값을 덮으면 다른 로봇도 바뀝니다). 로봇(브링업)을 다시 시작한 뒤 적용하세요')}
        if os.path.isfile(target):
            shutil.copy2(target, target + '.bak')
        if not self.set_info_client.wait_for_service(timeout_sec=2.0):
            return {'success': False, 'message': '카메라 드라이버의 set_camera_info 서비스가 없습니다'}
        request = SetCameraInfo.Request()
        request.camera_info = info
        response = wait_future(self.set_info_client.call_async(request))
        if response is None:
            return {'success': False, 'message': '카메라 드라이버가 응답하지 않습니다'}
        if not response.success:
            return {'success': False, 'message': f'저장 실패: {response.status_message}'}
        self.get_logger().info(f'applied camera calibration to {target}')
        return {'success': True, 'message': f'적용했습니다 ({target}). 이전 값은 .bak 으로 남겼습니다', 'file': target}

    def _camera_info_url(self) -> Optional[str]:
        if not self.param_client.wait_for_services(timeout_sec=2.0):
            return None
        response = wait_future(self.param_client.get_parameters(['camera_info_url']))
        if response is None or not response.values:
            return None
        return response.values[0].string_value


def default_camera_info_path() -> str:
    """소스에 들어 있는 공용 기본 보정값 (usb_cam config/camera_info.yaml). 되돌리기에만 쓴다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('usb_cam'), 'config', 'camera_info.yaml')
    except (ImportError, LookupError):
        return ''


def wait_future(future: rclpy.task.Future) -> Optional[object]:
    """HTTP 스레드에서 기다린다. 결과는 main 스레드의 spin 이 채운다."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(FUTURE_TIMEOUT_S):
        return None
    return future.result()


def image_to_rgb(msg: Image) -> Optional[np.ndarray]:
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        return None
    data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
    image = data.reshape(msg.height, msg.width, channels)
    if msg.encoding == 'bgr8':
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if msg.encoding == 'mono8':
        return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    return image


def main() -> None:
    rclpy.init()
    node = CalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_event.set()
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

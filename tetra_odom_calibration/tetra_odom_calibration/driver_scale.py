"""바퀴 드라이버(tetra_wheel_driver)의 오도메트리 보정 배율 파라미터를 읽고 바꾼다.

드라이버는 바뀐 값을 바로 쓰고 ~/.config/tetra/wheel_odom_scale.txt 에 남긴다 (odom_scale.hpp).
"""
import threading
from typing import Dict, Union

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

SCALE_NAMES = ('odom_linear_scale', 'odom_angular_scale')
SET_TIMEOUT_S = 5.0


class WheelScaleClient:

    def __init__(self, node: Node, driver_name: str) -> None:
        self.client = AsyncParameterClient(node, driver_name)
        # available=False: 드라이버가 없거나 보정 배율이 없는 예전 빌드
        self.scale: Dict[str, Union[float, bool]] = {'linear': 1.0, 'angular': 1.0, 'available': False}

    def refresh(self) -> None:
        if not self.client.services_are_ready():
            self.scale['available'] = False
            return
        self.client.get_parameters(list(SCALE_NAMES)).add_done_callback(self._on_values)

    def _on_values(self, future: rclpy.task.Future) -> None:
        values = future.result().values if future.result() else []
        doubles = [value.double_value for value in values if value.type == Parameter.Type.DOUBLE.value]
        if len(doubles) == 2:
            self.scale = {'linear': doubles[0], 'angular': doubles[1], 'available': True}
        else:
            self.scale['available'] = False

    def apply(self, linear: float, angular: float) -> Dict[str, object]:
        """HTTP 스레드에서 부른다 (응답은 ROS 실행기 스레드가 받아 준다)."""
        if not self.scale['available']:
            return {'success': False, 'message': 'driver_unsupported'}
        params = [Parameter(SCALE_NAMES[0], Parameter.Type.DOUBLE, float(linear)),
                  Parameter(SCALE_NAMES[1], Parameter.Type.DOUBLE, float(angular))]
        done = threading.Event()
        future = self.client.set_parameters(params)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(SET_TIMEOUT_S) or future.result() is None:
            return {'success': False, 'message': 'driver_timeout'}
        failed = [result.reason for result in future.result().results if not result.successful]
        if failed:
            return {'success': False, 'message': 'invalid_scale'}
        self.scale = {'linear': float(linear), 'angular': float(angular), 'available': True}
        return {'success': True, 'message': 'applied'}

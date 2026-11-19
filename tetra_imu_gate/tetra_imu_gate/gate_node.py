"""imu/data 를 imu/data_gated 로 넘기되, 바퀴(odom)가 서 있는 동안은 넘기지 않는다.

EKF(tetra_bringup/params/ekf_m.yaml)는 imu/data_gated 를 받는다. 막는 동안 EKF 는 바퀴 회전 속도(0)만 보아
방향이 흐르지 않는다. 바퀴 오도메트리가 끊기면 막지 않아 예전처럼 IMU 로 이어 간다. 근거는 gate.py.
막는 상태가 바뀔 때마다 ~/imu_blocked (Bool, transient_local) 로 알린다.
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from .gate import GateConfig, StandstillGate


class ImuStandstillGateNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_standstill_gate')
        defaults = GateConfig()
        config = GateConfig(
            standstill_velocity=float(self.declare_parameter('standstill_velocity', defaults.standstill_velocity).value),
            settle_time=float(self.declare_parameter('settle_time', defaults.settle_time).value),
            odom_timeout=float(self.declare_parameter('odom_timeout', defaults.odom_timeout).value),
        )
        self._gate = StandstillGate(config)
        self._blocked = False
        self._imu_publisher = self.create_publisher(Imu, 'imu/data_gated', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._blocked_publisher = self.create_publisher(Bool, '~/imu_blocked', latched)
        self._blocked_publisher.publish(Bool(data=False))
        self.create_subscription(Odometry, 'odom', self._on_odometry, 30)
        self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odometry(self, odometry: Odometry) -> None:
        twist = odometry.twist.twist
        self._gate.on_odometry(self._now(), twist.linear.x, twist.angular.z)

    def _on_imu(self, imu_message: Imu) -> None:
        forward = self._gate.should_forward_imu(self._now())
        if forward:
            self._imu_publisher.publish(imu_message)
        if self._blocked == forward:
            self._blocked = not forward
            self._blocked_publisher.publish(Bool(data=self._blocked))
            self.get_logger().debug(f'imu blocked: {self._blocked}')


def main() -> None:
    rclpy.init()
    node = ImuStandstillGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

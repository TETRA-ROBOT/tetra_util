"""AMCL 이 위치를 잃으면 스캔을 지도에 맞춰 찾은 자세로 /initialpose 를 보내 바로잡는다.

매 check_period 마다 지금 자세에서 스캔이 지도와 맞는 비율을 재 ~/scan_match_ratio 로 낸다.
비율이 낮게 이어지면 근처(local_*)를, 거기서 못 찾으면 넓게(wide_*) 찾고, policy.decide 가 허락할 때만 바로잡는다.
결과는 ~/event 에 JSON 으로 낸다. apply 를 false 로 두면 찾기만 하고 바로잡지 않는다 (현장 시험용).
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .policy import Decision, LostDetector, PolicyConfig, decide
from .scan_matcher import (MapMatcher, Pose2D, SearchResult, SearchWindow, base_pose_from_sensor,
                           scan_to_points)

NOMOTION_UPDATES = 3


def _pose_from_transform(transform: TransformStamped) -> Pose2D:
    t, q = transform.transform.translation, transform.transform.rotation
    return Pose2D(t.x, t.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def _pose_json(pose: Pose2D) -> dict:
    return {'x': round(pose.x, 3), 'y': round(pose.y, 3), 'yaw_deg': round(math.degrees(pose.yaw), 1)}


class Relocalizer(Node):
    def __init__(self) -> None:
        super().__init__('auto_relocalizer')
        self._declare_parameters()
        self._config = PolicyConfig(**{name: self._param(name) for name in PolicyConfig.__dataclass_fields__})
        self._detector = LostDetector(self._config)
        self._matcher: MapMatcher | None = None
        self._scan: LaserScan | None = None
        self._odom: Odometry | None = None
        self._pending_nomotion = 0
        self._amcl_active = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self._param('map_topic'), self._on_map, latched)
        self.create_subscription(LaserScan, self._param('scan_topic'), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, self._param('odom_topic'), self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._on_initial_pose, 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self._ratio_pub = self.create_publisher(Float32, '~/scan_match_ratio', 10)
        self._event_pub = self.create_publisher(String, '~/event', 10)
        self._nomotion_client = self.create_client(Empty, 'request_nomotion_update')
        # 맵핑으로 바꿀 때 관제는 Nav2 를 끄지 않고 lifecycle 로 멈춘다. 그동안 /map 은 SLAM 지도라 여기서 손대지 않는다
        self._amcl_state_client = self.create_client(GetState, f"{self._param('amcl_node')}/get_state")
        self.create_timer(2.0, self._poll_amcl_state)
        self.create_timer(self._param('check_period'), self._check)
        self.create_timer(0.3, self._request_nomotion_update)

    def _declare_parameters(self) -> None:
        defaults = {
            'apply': True, 'amcl_node': 'amcl', 'map_topic': 'map', 'scan_topic': 'scan', 'odom_topic': 'odom',
            'global_frame': 'map', 'base_frame': 'base_footprint', 'check_period': 1.0,
            'max_points': 180, 'max_range': 15.0, 'hit_tolerance': 0.10, 'scan_max_age': 1.0,
            # 빨리 돌 때는 스캔이 휘어 일치율이 낮게 나온다
            'max_linear_speed': 0.8, 'max_angular_speed': 0.4,
            'local_radius': 1.0, 'local_yaw_range_deg': 30.0, 'local_xy_step': 0.1, 'local_yaw_step_deg': 2.0,
            # 0 이면 넓게 찾지 않는다
            'wide_radius': 3.0, 'wide_yaw_range_deg': 180.0, 'wide_xy_step': 0.1, 'wide_yaw_step_deg': 3.0,
            'ambiguity_distance': 0.5, 'ambiguity_yaw_deg': 15.0,
            'corrected_xy_stddev': 0.1, 'corrected_yaw_stddev_deg': 3.0,
        }
        defaults.update({name: field.default for name, field in PolicyConfig.__dataclass_fields__.items()})
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name: str) -> bool | int | float | str:
        return self.get_parameter(name).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_map(self, grid: OccupancyGrid) -> None:
        info = grid.info
        occupied = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width) > 50
        self._matcher = MapMatcher(occupied, info.resolution, info.origin.position.x, info.origin.position.y,
                                   self._param('hit_tolerance'))
        self._detector.hold(self._now(), self._config.hold_after_initial_pose)
        self.get_logger().info(f'지도를 받았습니다 ({info.width}x{info.height}, {info.resolution:.3f} m)')

    def _on_scan(self, scan: LaserScan) -> None:
        self._scan = scan

    def _on_odom(self, odom: Odometry) -> None:
        self._odom = odom

    def _on_initial_pose(self, _: PoseWithCovarianceStamped) -> None:
        # 사람이 웹에서 지정했든 이 노드가 보냈든 AMCL 이 수렴할 시간을 준다
        self._detector.hold(self._now(), self._config.hold_after_initial_pose)

    def _poll_amcl_state(self) -> None:
        if not self._amcl_state_client.service_is_ready():
            self._set_amcl_active(False)
            return
        future = self._amcl_state_client.call_async(GetState.Request())
        future.add_done_callback(
            lambda done: self._set_amcl_active(done.result() is not None
                                               and done.result().current_state.id == State.PRIMARY_STATE_ACTIVE))

    def _set_amcl_active(self, active: bool) -> None:
        if active and not self._amcl_active:
            # 다시 켜질 때는 보통 새 지도·새 초기 위치가 함께 온다
            self._detector.hold(self._now(), self._config.hold_after_initial_pose)
        self._amcl_active = active

    def _moving_too_fast(self) -> bool:
        if self._odom is None:
            return False
        twist = self._odom.twist.twist
        return abs(twist.linear.x) > self._param('max_linear_speed') or abs(twist.angular.z) > self._param('max_angular_speed')

    def _lookup(self, target: str, source: str, stamp: Time) -> Pose2D | None:
        try:
            return _pose_from_transform(self._tf_buffer.lookup_transform(target, source, stamp, Duration(seconds=0.2)))
        except TransformException:
            return None

    def _check(self) -> None:
        scan = self._scan
        if not self._amcl_active or self._matcher is None or scan is None or self._moving_too_fast():
            return
        stamp = Time.from_msg(scan.header.stamp)
        if (self.get_clock().now() - stamp).nanoseconds * 1e-9 > self._param('scan_max_age'):
            return
        sensor = self._lookup(self._param('global_frame'), scan.header.frame_id, stamp)
        mount = self._lookup(self._param('base_frame'), scan.header.frame_id, Time())
        if sensor is None or mount is None:
            return
        points = scan_to_points(np.asarray(scan.ranges, dtype=float), scan.angle_min, scan.angle_increment,
                                max(scan.range_min, 0.05), min(scan.range_max, self._param('max_range')),
                                self._param('max_points'))
        if len(points) < 30:
            return
        ratio = self._matcher.ratio(points, sensor)
        self._ratio_pub.publish(Float32(data=float(ratio)))
        if self._detector.update(self._now(), ratio):
            self._relocalize(points, sensor, mount, ratio, scan)

    def _relocalize(self, points: np.ndarray, sensor: Pose2D, mount: Pose2D, ratio: float, scan: LaserScan) -> None:
        decision, result = Decision.NO_MATCH, None
        for window in self._windows():
            result = self._matcher.search(points, sensor, window, self._param('ambiguity_distance'),
                                          math.radians(self._param('ambiguity_yaw_deg')))
            decision = decide(self._config, sensor, ratio, result)
            if decision is not Decision.NO_MATCH:
                break
        current = base_pose_from_sensor(sensor, mount)
        found = base_pose_from_sensor(result.pose, mount)
        applied = decision is Decision.CORRECT and self._param('apply')
        self._report(decision, applied, current, found, ratio, result)
        if applied:
            self._publish_initial_pose(found, scan)
        else:
            self._detector.hold(self._now(), self._config.retry_interval)

    def _windows(self) -> list[SearchWindow]:
        windows = [SearchWindow(self._param('local_radius'), math.radians(self._param('local_yaw_range_deg')),
                                self._param('local_xy_step'), math.radians(self._param('local_yaw_step_deg')))]
        if self._param('wide_radius') > 0.0:
            windows.append(SearchWindow(self._param('wide_radius'), math.radians(self._param('wide_yaw_range_deg')),
                                        self._param('wide_xy_step'), math.radians(self._param('wide_yaw_step_deg'))))
        return windows

    def _report(self, decision: Decision, applied: bool, current: Pose2D, found: Pose2D, ratio: float,
                result: SearchResult) -> None:
        event = {'decision': decision.value, 'applied': applied, 'from': _pose_json(current), 'to': _pose_json(found),
                 'ratio_before': round(ratio, 3), 'ratio_after': round(result.ratio, 3),
                 'runner_up_ratio': round(result.runner_up_ratio, 3)}
        self._event_pub.publish(String(data=json.dumps(event)))
        log = self.get_logger().warn if decision is Decision.CORRECT else self.get_logger().info
        log(f'위치 재추정: {json.dumps(event, ensure_ascii=False)}')

    def _publish_initial_pose(self, pose: Pose2D, scan: LaserScan) -> None:
        message = PoseWithCovarianceStamped()
        # 스캔 시각의 자세다. AMCL 이 그 뒤 움직인 만큼을 odom 으로 옮겨 적용한다
        message.header.stamp = scan.header.stamp
        message.header.frame_id = self._param('global_frame')
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        xy_variance = self._param('corrected_xy_stddev') ** 2
        message.pose.covariance[0] = message.pose.covariance[7] = xy_variance
        message.pose.covariance[35] = math.radians(self._param('corrected_yaw_stddev_deg')) ** 2
        self._initial_pose_pub.publish(message)
        self._detector.hold(self._now(), self._config.hold_after_initial_pose)
        self._pending_nomotion = NOMOTION_UPDATES

    def _request_nomotion_update(self) -> None:
        # 서 있는 로봇은 AMCL 이 갱신하지 않으므로 새 파티클을 스캔에 몇 번 맞춰 둔다 (웹 위치 추정과 같은 방식)
        if self._pending_nomotion <= 0 or not self._nomotion_client.service_is_ready():
            return
        self._pending_nomotion -= 1
        self._nomotion_client.call_async(Empty.Request())


def main() -> None:
    rclpy.init()
    node = Relocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""바퀴 오도메트리 보정 노드. 웹 관제 관리자 화면(바퀴 오도메트리 보정)이 필요할 때만 띄운다.

로봇을 천천히 직진·제자리 회전시키고, 출발 전과 멈춘 뒤의 스캔을 맞춰 실제 이동량을 잰다 (motion_match).
오도메트리와의 비율로 바퀴 드라이버의 odom_linear_scale / odom_angular_scale 을 제안하고, 적용하면 드라이버가 파일에 남긴다.
"""
import math
import threading
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

from . import evaluation
from .driver_scale import WheelScaleClient
from .evaluation import ROTATION, STRAIGHT, Trial
from .http_api import start_server
from .motion_match import Motion, match, normalize_angle, scan_to_base_points
from .motion_plan import MotionLimits, front_clearance, nearest_obstacle, rotation_speed, straight_speed

TICK_PERIOD = 0.05


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCalibratorNode(Node):

    def __init__(self) -> None:
        super().__init__('odom_calibrator')
        declare = self.declare_parameter
        self.base_frame = declare('base_frame', 'base_footprint').value
        self.corridor_half_width = float(declare('corridor_half_width', 0.35).value)
        self.stop_distance = float(declare('stop_distance', 0.45).value)
        self.rotation_clearance = float(declare('rotation_clearance', 0.45).value)
        self.settle_time = float(declare('settle_time', 1.5).value)
        self.scans_per_pose = int(declare('scans_per_pose', 3).value)
        self.deadman_timeout = float(declare('deadman_timeout', 5.0).value)
        self.idle_timeout = float(declare('idle_timeout', 600.0).value)
        self.limits = MotionLimits()
        self._lock = threading.Lock()
        self._sensor_in_base: Optional[Motion] = None
        self._scan_points: Optional[np.ndarray] = None
        self._odom: Optional[Odometry] = None
        self._last_touch = time.monotonic()
        self._phase = 'idle'
        self._kind = ''
        self._target = 0.0
        self._direction = 1.0
        self._captured: List[np.ndarray] = []
        self._start_scans: List[np.ndarray] = []
        self._start_pose = (0.0, 0.0, 0.0)
        self._turned = 0.0
        self._last_yaw = 0.0
        self._phase_since = 0.0
        self._deadline = 0.0
        self._message = ''
        self.trials: List[Trial] = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, declare('scan_topic', 'scan').value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, declare('odom_topic', 'odom').value, self._on_odom, 20)
        self.cmd_pub = self.create_publisher(Twist, declare('cmd_vel_topic', 'cmd_vel_manual').value, 10)
        self.driver = WheelScaleClient(self, declare('wheel_driver', '/tetra_wheel_driver').value)
        self.create_timer(TICK_PERIOD, self._tick)
        self.create_timer(5.0, self.driver.refresh)
        port = int(declare('http_port', 8092).value)
        self.server = start_server(port, self)
        self.get_logger().info(f'odometry calibration on 127.0.0.1:{port}')

    # ---- ROS 콜백 ----
    def _on_scan(self, msg: LaserScan) -> None:
        if self._sensor_in_base is None:
            self._sensor_in_base = self._lookup_sensor(msg.header.frame_id)
            if self._sensor_in_base is None:
                return
        points = scan_to_base_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max, self._sensor_in_base)
        with self._lock:
            self._scan_points = points
            if self._phase in ('capture_start', 'capture_end'):
                self._captured.append(points)

    def _lookup_sensor(self, frame: str) -> Optional[Motion]:
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return Motion(tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            yaw = yaw_of(msg)
            if self._odom is not None and self._phase != 'idle':
                self._turned += normalize_angle(yaw - self._last_yaw)
            self._last_yaw = yaw
            self._odom = msg

    # ---- 시험 진행 (20 Hz) ----
    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            phase = self._phase
            if phase == 'idle':
                if now - self._last_touch > self.idle_timeout:
                    self.get_logger().info('관리자 화면 요청이 없어 종료합니다')
                    raise SystemExit
                return
            if now - self._last_touch > self.deadman_timeout:
                self._abort('screen_lost')
            elif phase == 'capture_start' and len(self._captured) >= self.scans_per_pose:
                self._begin_motion(now)
            elif phase == 'moving':
                self._drive(now)
            elif phase == 'settling':
                self._send(0.0, 0.0)
                twist = self._odom.twist.twist if self._odom else None
                still = twist is not None and abs(twist.linear.x) < 1e-3 and abs(twist.angular.z) < 1e-3
                if now - self._phase_since >= self.settle_time and still:
                    self._set_phase('capture_end', now)
            elif phase == 'capture_end' and len(self._captured) >= self.scans_per_pose:
                self._finish()
            elif phase.startswith('capture') and now - self._phase_since > 5.0:
                self._abort('no_scan')

    def _set_phase(self, phase: str, now: float) -> None:
        self._phase = phase
        self._phase_since = now
        self._captured = []

    def _begin_motion(self, now: float) -> None:
        self._start_scans = self._captured
        points = self._start_scans[-1]
        if self._kind == STRAIGHT and front_clearance(points, self.corridor_half_width) < \
                self._target + self.stop_distance:
            self._abort('blocked_front')
            return
        if self._kind == ROTATION and nearest_obstacle(points) < self.rotation_clearance:
            self._abort('blocked_around')
            return
        odom = self._odom
        self._start_pose = (odom.pose.pose.position.x, odom.pose.pose.position.y, yaw_of(odom))
        self._turned = 0.0
        speed = self.limits.linear_speed if self._kind == STRAIGHT else self.limits.angular_speed
        self._deadline = now + self._target / speed * 2.5 + 10.0
        self._set_phase('moving', now)

    def _drive(self, now: float) -> None:
        points = self._scan_points if self._scan_points is not None else np.empty((0, 2))
        if now > self._deadline:
            self._abort('timeout')
            return
        if self._kind == STRAIGHT:
            if front_clearance(points, self.corridor_half_width) < self.stop_distance:
                self._abort('blocked_front')
                return
            speed = straight_speed(self._odom_motion().distance, self._target, self.limits)
            self._send(speed, 0.0)
        else:
            if nearest_obstacle(points) < self.rotation_clearance - 0.1:
                self._abort('blocked_around')
                return
            speed = rotation_speed(abs(self._turned), self._target, self.limits)
            self._send(0.0, speed * self._direction)
        if speed == 0.0:
            self._set_phase('settling', now)

    def _odom_motion(self) -> Motion:
        x0, y0, yaw0 = self._start_pose
        odom = self._odom
        dx, dy = odom.pose.pose.position.x - x0, odom.pose.pose.position.y - y0
        return Motion(math.cos(yaw0) * dx + math.sin(yaw0) * dy, -math.sin(yaw0) * dx + math.cos(yaw0) * dy,
                      normalize_angle(yaw_of(odom) - yaw0))

    def _finish(self) -> None:
        odom_motion = self._odom_motion()
        initial = Motion(odom_motion.x, odom_motion.y, normalize_angle(self._turned))
        results = [match(self._start_scans[0], end, initial) for end in self._captured]
        results = [result for result in results if result is not None]
        # 스캔 몇 장 중 가운데 값을 쓴다 (한 장의 잡음에 흔들리지 않게)
        best = sorted(results, key=lambda r: r.motion.distance if self._kind == STRAIGHT
                      else evaluation.unwrap_turn(self._turned, r.motion.yaw))
        chosen = best[len(best) // 2] if best else None
        trial = evaluation.evaluate(self._kind, odom_motion, self._turned, chosen)
        self.trials.append(trial)
        self._message = 'done' if trial.accepted else trial.reason
        self._phase = 'idle'
        self.get_logger().info(f'{self._kind}: odom {trial.odom:.4f}, scan {trial.measured:.4f}, '
                               f'ratio {trial.ratio:.4f}, rmse {trial.rmse:.4f}, inliers {trial.inlier_ratio:.2f}')

    def _abort(self, reason: str) -> None:
        self._send(0.0, 0.0)
        self._phase = 'idle'
        self._message = reason
        self.get_logger().warn(f'시험을 멈췄습니다: {reason}')

    def _send(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    # ---- HTTP (관리자 화면) ----
    def touch(self) -> None:
        self._last_touch = time.monotonic()

    def status(self) -> Dict[str, object]:
        with self._lock:
            progress = 0.0
            if self._phase == 'moving' and self._target > 0.0:
                done = self._odom_motion().distance if self._kind == STRAIGHT else abs(self._turned)
                progress = min(1.0, done / self._target)
            linear = evaluation.propose(self.driver.scale['linear'], self.trials, STRAIGHT)
            angular = evaluation.propose(self.driver.scale['angular'], self.trials, ROTATION)
            return {
                'phase': self._phase, 'kind': self._kind, 'progress': progress, 'message': self._message,
                'scale': dict(self.driver.scale), 'trials': [trial.to_dict() for trial in self.trials],
                'proposal': {'linear': linear.__dict__ if linear else None,
                             'angular': angular.__dict__ if angular else None},
                'ready': {'scan': self._scan_points is not None, 'odom': self._odom is not None},
            }

    def command(self, name: str, body: Dict[str, object]) -> Dict[str, object]:
        if name == 'stop':
            with self._lock:
                if self._phase != 'idle':
                    self._abort('stopped')
            return {'success': True}
        if name in (STRAIGHT, ROTATION):
            return self._start(name, body)
        if name == 'clear':
            self.trials = []
            return {'success': True}
        if name in ('apply', 'restore-default'):
            try:
                linear, angular = (1.0, 1.0) if name == 'restore-default' else (float(body['linear']), float(body['angular']))
            except (KeyError, TypeError, ValueError):
                return {'success': False, 'message': 'invalid_scale'}
            result = self.driver.apply(linear, angular)
            # 지난 시험은 예전 배율로 잰 값이다. 남겨 두면 새 배율에 같은 비율을 또 곱해 보정이 두 번 들어간다
            if result.get('success'):
                self.trials = []
            return result
        return {'success': False, 'message': 'unknown_command'}

    def _start(self, kind: str, body: Dict[str, object]) -> Dict[str, object]:
        with self._lock:
            if self._phase != 'idle':
                return {'success': False, 'message': 'busy'}
            if self._odom is None or self._scan_points is None:
                return {'success': False, 'message': 'no_scan' if self._scan_points is None else 'no_odom'}
            try:
                if kind == STRAIGHT:
                    self._target = min(3.0, max(0.5, float(body.get('distance', 1.0))))
                else:
                    self._target = min(3.0, max(1.0, float(body.get('turns', 2.0)))) * 2.0 * math.pi
                    self._direction = -1.0 if body.get('direction') == 'cw' else 1.0
            except (TypeError, ValueError):
                return {'success': False, 'message': 'invalid_target'}
            self._kind = kind
            self._message = ''
            self._set_phase('capture_start', time.monotonic())
        return {'success': True}


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OdomCalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node._send(0.0, 0.0)
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

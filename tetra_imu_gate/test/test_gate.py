from tetra_imu_gate.gate import GateConfig, StandstillGate

CONFIG = GateConfig(standstill_velocity=1e-4, settle_time=0.5, odom_timeout=0.5)


def _gate_standing_since(start: float, until: float) -> StandstillGate:
    gate = StandstillGate(CONFIG)
    time = start
    while time <= until:
        gate.on_odometry(time, 0.0, 0.0)
        time += 1 / 30
    return gate


def test_forwards_before_any_odometry() -> None:
    assert StandstillGate(CONFIG).should_forward_imu(10.0)


def test_forwards_while_moving() -> None:
    gate = StandstillGate(CONFIG)
    gate.on_odometry(1.0, 0.2, 0.0)
    assert gate.should_forward_imu(1.01)


def test_forwards_while_turning_in_place() -> None:
    gate = StandstillGate(CONFIG)
    gate.on_odometry(1.0, 0.0, -0.3)
    assert gate.should_forward_imu(1.01)


def test_forwards_during_settle_time_after_stopping() -> None:
    gate = _gate_standing_since(1.0, 1.3)
    assert gate.should_forward_imu(1.31)


def test_blocks_once_standing_still_long_enough() -> None:
    gate = _gate_standing_since(1.0, 2.0)
    assert not gate.should_forward_imu(2.01)


def test_motion_reopens_immediately() -> None:
    gate = _gate_standing_since(1.0, 2.0)
    gate.on_odometry(2.03, 0.0, 0.05)
    assert gate.should_forward_imu(2.04)


def test_tiny_encoder_noise_still_counts_as_standing() -> None:
    gate = StandstillGate(CONFIG)
    gate.on_odometry(1.0, 5e-5, -5e-5)
    gate.on_odometry(2.0, -5e-5, 5e-5)
    assert not gate.should_forward_imu(2.01)


def test_forwards_when_wheel_odometry_stops_arriving() -> None:
    # 바퀴 드라이버가 끊기면 EKF 가 IMU 로 이어 가야 한다
    gate = _gate_standing_since(1.0, 2.0)
    assert gate.should_forward_imu(2.6)

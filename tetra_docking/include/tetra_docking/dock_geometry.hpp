#pragma once

#include <cmath>

namespace tetra_docking
{

struct PlanarOffset
{
  double x{0.0};
  double y{0.0};
};

/**
 * 도크 축 기준 오프셋(앞 forward, 왼쪽 left)을 yaw 만큼 돌려 고정 프레임의 변위로 바꾼다
 * @param yaw 도크 축 방향 (rad)
 * @param forward 축 방향 오프셋 (m). external_detection_translation_x
 * @param left 축 왼쪽 오프셋 (m). external_detection_translation_y
 */
inline PlanarOffset rotateDockOffset(double yaw, double forward, double left)
{
  return {std::cos(yaw) * forward - std::sin(yaw) * left, std::sin(yaw) * forward + std::cos(yaw) * left};
}

/**
 * 태그 위치에서 도크 자세의 위치를 구한다: 태그에서 도크 축 방향으로 (forward, left) 만큼 떨어진 점.
 * SimpleChargingDock::getRefinedPose 의 이동 보정과 같은 식이다
 * @param tag_x, tag_y 고정 프레임의 태그 위치 (m)
 * @param yaw 도크 축 방향 (rad)
 */
inline PlanarOffset dockPositionFromTag(double tag_x, double tag_y, double yaw, double forward, double left)
{
  return {
    tag_x + std::cos(yaw) * forward - std::sin(yaw) * left,
    tag_y + std::sin(yaw) * forward + std::cos(yaw) * left};
}

/**
 * dockPositionFromTag 의 반대: 도크 자세의 위치에서 태그 위치를 되찾는다
 */
inline PlanarOffset tagPositionFromDock(double dock_x, double dock_y, double yaw, double forward, double left)
{
  const PlanarOffset offset = rotateDockOffset(yaw, forward, left);
  return {dock_x - offset.x, dock_y - offset.y};
}

/**
 * 도크 축 깊이로 도착을 판정한다 (docked_by_depth). ROS 없이 계산만 해 단위 시험을 한다
 * 접점을 요구하는 충전 도킹은 contact_overtravel 만큼 더 깊어질 때까지 도착으로 보지 않는다: 도착 판정 뒤 도킹 서버는
 * waitForCharge 동안 속도를 내지 않아, 접점은 휠 드라이버 cmd_vel_timeout(0.5 s) 동안 마지막 명령으로 밀려 들어갈 때만 붙었다
 * @param depth 도크 축(충전기 쪽 +)으로 본 로봇 깊이, 0 이면 도크 자세 (m)
 * @param lateral 도크 축에서 로봇의 좌우 거리 (m)
 * @param threshold docking_threshold (m)
 * @param overtravel 도크 자세를 지나 더 들어갈 거리 (m, 0 이면 예전 판정)
 * @param window 판정을 쓰는 범위 (m). 지난 도킹의 도크 자세로 다른 곳에서 도착이 되지 않게 한다
 */
inline bool dockDepthReached(double depth, double lateral, double threshold, double overtravel, double window)
{
  return depth >= overtravel - threshold && depth <= window && std::abs(lateral) <= window;
}

/**
 * 깊이 도착 판정에 실제로 쓸 contact_overtravel. 접근 중(매 제어 주기 getRefinedPose)일 때만 더 깊게 본다.
 * isDocked() 는 도킹·언도킹 목표를 시작할 때도 불린다 ("이미 도킹됨" / "도크에 없음" 확인). 충전기에 서 있는 로봇은 접점이
 * 붙은 자리(MH 실측 도크 자세 + 7 mm)에 멈춰 있어, 그때도 더 깊게 보면 접점 신호가 잠깐 끊긴 사이(하루 1,300 회 넘게, 약 2 초)에
 * 도킹 안 됨이 되어 충전기 안에서 staging 으로 나가거나 언도킹 후진을 건너뛴다
 * @param charge_requires_contact 접점이 있어야 충전으로 보는가
 * @param overtravel contact_overtravel (m)
 * @param seconds_since_refine 마지막 getRefinedPose 뒤 지난 시간 (s, 음수면 아직 없음)
 * @param approach_window 이보다 짧으면 접근 중으로 본다 (s)
 * @returns 깊이 판정에 더할 거리 (m)
 */
inline double activeContactOvertravel(
  bool charge_requires_contact, double overtravel, double seconds_since_refine, double approach_window)
{
  const bool approaching = seconds_since_refine >= 0.0 && seconds_since_refine < approach_window;
  return charge_requires_contact && approaching ? overtravel : 0.0;
}

}  // namespace tetra_docking

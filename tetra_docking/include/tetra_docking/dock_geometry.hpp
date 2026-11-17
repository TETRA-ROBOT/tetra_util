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

}  // namespace tetra_docking

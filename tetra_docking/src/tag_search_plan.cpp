// 태그 탐색의 계산과 상태 (ROS 없이 테스트한다: test/test_tag_search_plan.cpp). 로봇에 명령을 내는 쪽은 tag_search.cpp
#include "tetra_docking/tag_search_plan.hpp"

#include <cmath>

#include "angles/angles.h"

namespace tetra_docking
{

SearchStepCommand planSearchStep(double yaw, double origin_yaw, int step, double search_angle, double search_speed)
{
  const double targets[kSearchStepCount] = {search_angle, -search_angle, 0.0};
  const double error = angles::normalize_angle(origin_yaw + targets[step] - yaw);
  SearchStepCommand command;
  if (std::abs(error) < kSearchAngleTolerance) {
    command.next_step = step + 1;
    return command;
  }
  command.angular_velocity = std::copysign(search_speed, error);
  command.next_step = step;
  return command;
}

std::optional<double> centeringVelocity(double tag_x, double tag_forward, double search_speed)
{
  if (tag_forward <= 0.0) {
    return std::nullopt;
  }
  const double bearing = tag_x / tag_forward;
  if (std::abs(bearing) <= kCenteredBearing) {
    return std::nullopt;
  }
  // 시계 방향(음수)으로 돌면 카메라가 앞을 보든 뒤를 보든 시야가 영상 오른쪽으로 옮겨간다
  return -std::copysign(search_speed, bearing);
}

void TagSearchState::reset()
{
  seen_ = false;
  active_ = false;
  step_ = 0;
}

void TagSearchState::markSeen()
{
  seen_ = true;
  active_ = false;
}

void TagSearchState::beginSweep(double origin_yaw)
{
  active_ = true;
  origin_yaw_ = origin_yaw;
}

bool TagSearchState::needsConfirmation(bool search_enabled, bool yaw_lock_enabled) const
{
  if (!search_enabled && !yaw_lock_enabled) {
    return false;
  }
  return !seen_;
}

}  // namespace tetra_docking

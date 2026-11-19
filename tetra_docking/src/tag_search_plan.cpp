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

LostTagAction decideLostTag(
  double remaining, double since_detection, double blind_distance, double blind_timeout,
  double reacquire_distance, double reacquire_time)
{
  if (since_detection > blind_timeout) {
    return LostTagAction::kFail;
  }
  if (blind_distance > 0.0 && remaining <= blind_distance) {
    return LostTagAction::kBlind;
  }
  // 비스듬히 접근하다 태그가 화면 옆으로 빠진 경우: 마지막 자세(odom)는 놓치기 직전의 실제 도크라 그쪽으로 몰면
  // 제어기가 방향을 틀어 태그가 다시 들어온다. 오래 못 찾으면 안 보이는 채로 멀리 가지 않게 멈춘다
  if (reacquire_distance > 0.0 && remaining <= reacquire_distance && since_detection <= reacquire_time) {
    return LostTagAction::kReacquire;
  }
  return LostTagAction::kFail;
}

bool canRetryInPlace(const ApproachLineOffset & offset, double staging_distance, double heading_limit)
{
  // 2 cm 는 staging 도착 허용치 안에서 조금 가까이 선 경우
  return offset.front >= staging_distance - 0.02 &&
         offset.front <= staging_distance + kRetryInPlaceBeyondStaging &&
         std::abs(offset.lateral) <= kRetryInPlaceLateral &&
         std::abs(offset.heading) <= heading_limit;
}

}  // namespace tetra_docking

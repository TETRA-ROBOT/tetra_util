// 태그 탐색: staging 에 도착했는데 태그가 시야 밖이면 제자리에서 천천히 좌우로 돌며 찾는다.
// 멀리서 도킹을 시작하면 태그가 안 보여 staging 을 지도(map) 도크 자세로 잡는데, 좁은 도크 자리에서는 AMCL 방향 오차가
// 10 도를 넘어 (실기 M 2026-11-10: Nav2 도착 직후 -12 도로 보정) 화각 +-20 도인 후면 카메라에서 태그가 빠졌다.
// 태그를 한 번이라도 본 뒤(접근 중)에는 돌지 않는다. 도킹이 실패로 끝나면 도킹 서버가 0 속도를 낸다.
#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <cmath>

namespace tetra_docking
{

namespace
{
double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
// 목표 각도에 이만큼 가까우면 다음 방향으로 넘어간다
constexpr double kSearchAngleTolerance = 0.02;
// 태그가 영상 중앙에서 이만큼(카메라 x/z, 약 4.5도) 안에 와야 확인한다. 시야 끝에서 접근을 시작하면 바로 다시 놓친다
constexpr double kCenteredBearing = 0.08;
}  // namespace

void TetraChargingDock::configureTagSearch(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  // 0 이면 끈다 (EDU, MH, SM). 탐색은 시작 방향 기준 +각도 -> -각도 -> 시작 방향 순서로 돈다
  nav2_util::declare_parameter_if_not_declared(node, name + ".tag_search_angle", rclcpp::ParameterValue(0.0));
  // 어두우면 노출이 길어 빨리 돌면 영상이 번져 태그를 못 읽는다 (M 0.05 rad/s 에서 매 프레임 검출)
  nav2_util::declare_parameter_if_not_declared(node, name + ".tag_search_speed", rclcpp::ParameterValue(0.05));
  node->get_parameter(name + ".tag_search_angle", tag_search_angle_);
  node->get_parameter(name + ".tag_search_speed", tag_search_speed_);
  if (tag_search_angle_ > 0.0) {
    // 도킹 서버와 같은 노드라 cmd_vel 리매핑(cmd_vel_nav2)을 그대로 따른다. lifecycle 활성화와 무관하게 쓰려고 일반 publisher 로 만든다
    search_cmd_pub_ = rclcpp::create_publisher<geometry_msgs::msg::Twist>(
      node->get_node_topics_interface(), "cmd_vel", rclcpp::QoS(1));
  }
}

void TetraChargingDock::resetTagSearch()
{
  tag_seen_this_docking_ = false;
  search_active_ = false;
  search_step_ = 0;
}

// 태그를 본 적 없는 초기 인식 중에만 부른다. true 면 이번 주기에 회전 명령을 냈다
bool TetraChargingDock::searchForTag()
{
  if (!search_cmd_pub_ || tag_seen_this_docking_ || search_step_ >= 3) {
    return false;
  }
  double yaw = 0.0;
  try {
    const auto robot = tf2_buffer_->lookupTransform("odom", base_frame_id_, tf2::TimePointZero);
    yaw = tf2::getYaw(robot.transform.rotation);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Tag search skipped: %s", ex.what());
    return false;
  }
  if (!search_active_) {
    search_active_ = true;
    search_origin_yaw_ = yaw;
    RCLCPP_INFO(node_->get_logger(), "Dock tag not in view, searching +-%.0f deg", tag_search_angle_ * 180.0 / M_PI);
  }
  const double targets[3] = {tag_search_angle_, -tag_search_angle_, 0.0};
  const double error = wrapAngle(search_origin_yaw_ + targets[search_step_] - yaw);
  geometry_msgs::msg::Twist cmd;
  if (std::abs(error) < kSearchAngleTolerance) {
    ++search_step_;
    search_cmd_pub_->publish(cmd);
    if (search_step_ >= 3) {
      RCLCPP_WARN(node_->get_logger(), "Dock tag not found while searching");
    }
    return true;
  }
  cmd.angular.z = std::copysign(tag_search_speed_, error);
  search_cmd_pub_->publish(cmd);
  return true;
}

// 태그가 보이는 주기마다 부른다. 지나가며 한 프레임 스친 것은 믿지 않고, 멈춘 채로 표본이 모일 때(staging 추정과 같은 기준)만
// 확인으로 본다 (실기 M 2026-11-10: Nav2 로 도는 중 스친 검출로 탐색이 꺼져 태그를 못 찾았다). 확인하면 탐색을 멈추고,
// staging 때 yaw 를 못 구했으면 지금 구한 값으로 고정한다
bool TetraChargingDock::confirmTagSeen(const std::string & frame)
{
  // 탐색도 yaw 고정도 끈 모델(EDU, MH, SM)은 예전처럼 확인 없이 바로 접근한다. 이 확인은 멈춘 채 표본 3 개와 정면 각도를
  // 요구해, 꺼진 모델에서도 돌면 비스듬히 도착하거나 검출이 흔들릴 때 initial_perception_timeout 으로 도킹이 실패했다
  if (!search_cmd_pub_ && !lock_dock_yaw_to_staging_) {
    return true;
  }
  if (tag_seen_this_docking_) {
    return true;
  }
  if (search_active_ && search_cmd_pub_) {
    search_cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
  TagDockEstimate estimate;
  if (!estimateDockFromTag(frame, estimate)) {
    return false;
  }
  tag_seen_this_docking_ = true;
  if (search_active_) {
    RCLCPP_INFO(node_->get_logger(), "Dock tag found while searching");
  }
  search_active_ = false;
  if (lock_dock_yaw_to_staging_ && !locked_yaw_valid_) {
    rememberStagingYaw(estimate.dock, frame);
    RCLCPP_INFO(node_->get_logger(), "Dock yaw locked from a still view (%.3f rad in %s)", estimate.dock.yaw, frame.c_str());
  }
  return true;
}

// 탐색 기능을 켠 경우, 아직 확인 전이면 태그를 영상 중앙으로 돌려 맞춘다. true 면 이번 주기에 회전 명령을 냈다
bool TetraChargingDock::centerOnTag()
{
  if (!search_cmd_pub_ || tag_seen_this_docking_ || detected_dock_pose_.header.frame_id != camera_frame_) {
    return false;
  }
  // 카메라 광학 프레임: z 정면, x 영상 오른쪽
  const double forward = detected_dock_pose_.pose.position.z;
  if (forward <= 0.0) {
    return false;
  }
  const double bearing = detected_dock_pose_.pose.position.x / forward;
  if (std::abs(bearing) <= kCenteredBearing) {
    return false;
  }
  // 시계 방향(음수)으로 돌면 카메라가 앞을 보든 뒤를 보든 시야가 영상 오른쪽으로 옮겨간다
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = -std::copysign(tag_search_speed_, bearing);
  search_active_ = true;
  search_cmd_pub_->publish(cmd);
  return true;
}

}  // namespace tetra_docking

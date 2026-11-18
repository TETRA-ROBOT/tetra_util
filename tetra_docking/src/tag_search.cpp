// 태그 탐색: staging 에 도착했는데 태그가 시야 밖이면 제자리에서 천천히 좌우로 돌며 찾는다.
// 멀리서 도킹을 시작하면 태그가 안 보여 staging 을 지도(map) 도크 자세로 잡는데, 좁은 도크 자리에서는 AMCL 방향 오차가
// 10 도를 넘어 (실기 M 2026-11-10: Nav2 도착 직후 -12 도로 보정) 화각 +-20 도인 후면 카메라에서 태그가 빠졌다.
// 태그를 한 번이라도 본 뒤(접근 중)에는 돌지 않는다. 도킹이 실패로 끝나면 도킹 서버가 0 속도를 낸다.
#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"

#include <cmath>

namespace tetra_docking
{

namespace
{
// 탐색 중 돈 각도는 odom 에서 잰다. map 은 AMCL 이 보정할 때마다 방향이 튀어 기준 방향에서 얼마나 돌았는지 잴 수 없다
const char kSearchYawFrame[] = "odom";
}  // namespace

void TetraChargingDock::configureTagSearch(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  // 0 이면 끈다 (2026-11-18 부터 M·MH·EDU·SM 모두 켬, navigation_*.yaml). 탐색은 시작 방향 기준 +각도 -> -각도 -> 시작 방향 순서로 돈다
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
  tag_search_.reset();
}

// 태그를 본 적 없는 초기 인식 중에만 부른다. true 면 이번 주기에 회전 명령을 냈다
bool TetraChargingDock::searchForTag()
{
  if (!search_cmd_pub_ || tag_search_.seen() || tag_search_.sweepFinished()) {
    return false;
  }
  double yaw = 0.0;
  try {
    const auto robot = tf2_buffer_->lookupTransform(kSearchYawFrame, base_frame_id_, tf2::TimePointZero);
    yaw = tf2::getYaw(robot.transform.rotation);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Tag search skipped: %s", ex.what());
    return false;
  }
  if (!tag_search_.active()) {
    tag_search_.beginSweep(yaw);
    RCLCPP_INFO(node_->get_logger(), "Dock tag not in view, searching +-%.0f deg", tag_search_angle_ * 180.0 / M_PI);
  }
  const SearchStepCommand command = planSearchStep(
    yaw, tag_search_.originYaw(), tag_search_.step(), tag_search_angle_, tag_search_speed_);
  tag_search_.advanceTo(command.next_step);
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = command.angular_velocity;
  search_cmd_pub_->publish(cmd);
  if (command.angular_velocity == 0.0 && tag_search_.sweepFinished()) {
    RCLCPP_WARN(node_->get_logger(), "Dock tag not found while searching");
  }
  return true;
}

// 태그가 보이는 주기마다 부른다. 지나가며 한 프레임 스친 것은 믿지 않고, 멈춘 채로 표본이 모일 때(staging 추정과 같은 기준)만
// 확인으로 본다 (실기 M 2026-11-10: Nav2 로 도는 중 스친 검출로 탐색이 꺼져 태그를 못 찾았다). 확인하면 탐색을 멈추고,
// staging 때 yaw 를 못 구했으면 지금 구한 값으로 고정한다
bool TetraChargingDock::confirmTagSeen(const std::string & frame)
{
  // 탐색도 yaw 고정도 끈 설정은 예전처럼 확인 없이 바로 접근한다 (TagSearchState::needsConfirmation)
  if (!tag_search_.needsConfirmation(search_cmd_pub_ != nullptr, lock_dock_yaw_to_staging_)) {
    return true;
  }
  if (tag_search_.active() && search_cmd_pub_) {
    search_cmd_pub_->publish(geometry_msgs::msg::Twist());
  }
  TagDockEstimate estimate;
  if (!estimateDockFromTag(frame, estimate)) {
    return false;
  }
  if (tag_search_.active()) {
    RCLCPP_INFO(node_->get_logger(), "Dock tag found while searching");
  }
  tag_search_.markSeen();
  if (lock_dock_yaw_to_staging_ && !locked_yaw_valid_) {
    rememberStagingYaw(estimate.dock, frame);
    RCLCPP_INFO(node_->get_logger(), "Dock yaw locked from a still view (%.3f rad in %s)", estimate.dock.yaw, frame.c_str());
  }
  return true;
}

// 탐색 기능을 켠 경우, 아직 확인 전이면 태그를 영상 중앙으로 돌려 맞춘다. true 면 이번 주기에 회전 명령을 냈다
bool TetraChargingDock::centerOnTag()
{
  if (!search_cmd_pub_ || tag_search_.seen() || detected_dock_pose_.header.frame_id != camera_frame_) {
    return false;
  }
  const auto angular_velocity = centeringVelocity(
    detected_dock_pose_.pose.position.x, detected_dock_pose_.pose.position.z, tag_search_speed_);
  if (!angular_velocity) {
    return false;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = *angular_velocity;
  tag_search_.markActive();
  search_cmd_pub_->publish(cmd);
  return true;
}

}  // namespace tetra_docking

// 마지막 구간 heading 우선: 도크 바로 앞에서는 횡 오차보다 도크 축과 나란히 들어가는 것을 우선한다.
// 도킹 서버는 제어 목표를 도크 자세보다 0.25 m 안쪽에 두므로, 접점에 닿는 순간에도 목표까지 약 0.3 m 가 남는다.
// graceful 제어 법칙은 남은 거리에 견준 횡 오차(phi)를 k_phi 배 heading 으로 바꾸므로, 검출 흔들림 1 cm 가 접촉 때
// heading 5~9도가 된다. 실기 EDU (2026-10-29 16:20) 축 위 1 cm 안으로 들어온 뒤에도 끝에서 -4도 -> +5.3도로 꺾여
// 범퍼 모서리가 먼저 닿았다. 그래서 도크 앞 heading_priority_far 부터 목표를 로봇의 현재 횡 위치로 점점 옮겨
// heading_priority_near 안에서는 도크 yaw 만 맞추며 곧게 들어가게 한다 (남은 횡 오차는 그대로 둔다).
#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"

#include <algorithm>
#include <cmath>

namespace tetra_docking
{

void TetraChargingDock::configureApproachShaping(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  // 둘 다 0 이면 끈다 (M 동작 유지). 거리는 base_link 에서 도크 자세까지 도크 축 방향 거리다
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".heading_priority_near", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".heading_priority_far", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".heading_priority_near", heading_priority_near_);
  node->get_parameter(name + ".heading_priority_far", heading_priority_far_);
  // 라이다 모드(V 자 홈)는 접점 약 5 cm 앞까지 홈이 보인다. 태그 값(0.30 m 부터 횡 오차 고정)을 그대로 쓰면
  // 그 지점의 횡 오차가 끝까지 남아, 실기 EDU (2026-11-19 녹화 edu_vdock_1119_dock) 0.30 m 앞 -3 cm 가 접점에서 1.3 cm,
  // 구간 안 방향 흔들림으로 0 -> 1.6 cm 로 남아 접점 가장자리에 붙었다. 그래서 라이다는 더 가까이까지 횡 오차를 줄인다
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lidar_heading_priority_near", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lidar_heading_priority_far", rclcpp::ParameterValue(-1.0));
  node->get_parameter(name + ".lidar_heading_priority_near", lidar_heading_priority_near_);
  node->get_parameter(name + ".lidar_heading_priority_far", lidar_heading_priority_far_);
}

void TetraChargingDock::shapeFinalApproach(geometry_msgs::msg::PoseStamped & pose)
{
  const bool lidar_values = lidar_mode_ && lidar_heading_priority_far_ >= 0.0 && lidar_heading_priority_near_ >= 0.0;
  const double near = lidar_values ? lidar_heading_priority_near_ : heading_priority_near_;
  const double far = lidar_values ? lidar_heading_priority_far_ : heading_priority_far_;
  if (far <= near) {
    return;
  }
  geometry_msgs::msg::TransformStamped robot;
  try {
    robot = tf2_buffer_->lookupTransform(pose.header.frame_id, base_frame_id_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Approach shaping skipped: %s", ex.what());
    return;
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double dx = robot.transform.translation.x - pose.pose.position.x;
  const double dy = robot.transform.translation.y - pose.pose.position.y;
  // 도크 프레임: 로봇이 도크 앞이면 remaining > 0, lateral 은 도크 축에서 왼쪽 +
  const double remaining = -(std::cos(yaw) * dx + std::sin(yaw) * dy);
  const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double weight = std::clamp((far - remaining) / (far - near), 0.0, 1.0);
  if (weight <= 0.0) {
    return;
  }
  pose.pose.position.x += -std::sin(yaw) * lateral * weight;
  pose.pose.position.y += std::cos(yaw) * lateral * weight;
  RCLCPP_DEBUG(
    node_->get_logger(), "Approach shaping: %.3f m before the dock, lateral %.3f m, weight %.2f",
    remaining, lateral, weight);
}

}  // namespace tetra_docking

// 도크 yaw 고정: 접근 중에는 태그 위치만 매 프레임 쓰고, 도크 축 방향(yaw)은 staging 때 멈춘 채 평균한 값으로 둔다.
// 작은 태그(M 후면 usb_cam, 1 m 에서 약 70 px)는 정지 중 yaw 가 0.1 도 안으로 맞지만 움직이는 동안 프레임마다 15 도씩 튄다.
// 도크 자세는 태그에서 그 yaw 방향으로 external_detection_translation_x 만큼 떨어진 점이라 (M 0.92 m),
// yaw 가 15 도 틀리면 목표가 옆으로 24 cm 움직여 제어기가 쫓아가며 돌다가 태그를 시야 밖으로 놓쳤다 (실기 M 2026-11-10).
#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"

#include <cmath>

namespace tetra_docking
{

void TetraChargingDock::configureYawLock(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  // 꺼 두면 예전처럼 프레임마다 태그 yaw 를 쓴다 (EDU, MH, SM)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lock_dock_yaw_to_staging", rclcpp::ParameterValue(false));
  // 이보다 오래된 staging 추정은 다른 도킹의 값으로 보고 쓰지 않는다 (dock_approach_timeout x 재시도보다 길게)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".locked_yaw_timeout", rclcpp::ParameterValue(150.0));
  node->get_parameter(name + ".lock_dock_yaw_to_staging", lock_dock_yaw_to_staging_);
  node->get_parameter(name + ".locked_yaw_timeout", locked_yaw_timeout_);
}

void TetraChargingDock::rememberStagingYaw(const DockSample & dock, const std::string & frame)
{
  if (!lock_dock_yaw_to_staging_) {
    return;
  }
  // staging 표본에는 도킹 영점이 들어가 있다. getRefinedPose 는 영점을 나중에 다시 더하므로 빼 둔다
  locked_yaw_ = dock.yaw - dock_offset_yaw_;
  locked_yaw_frame_ = frame;
  locked_yaw_time_ = node_->now();
  locked_yaw_valid_ = true;
}

// SimpleChargingDock::getRefinedPose 결과(태그 위치 + 태그 yaw 방향 translation)를 고정 yaw 로 다시 만든다
void TetraChargingDock::lockDockYaw(geometry_msgs::msg::PoseStamped & pose)
{
  if (!lock_dock_yaw_to_staging_ || !locked_yaw_valid_) {
    return;
  }
  if (node_->now() - locked_yaw_time_ > rclcpp::Duration::from_seconds(locked_yaw_timeout_)) {
    locked_yaw_valid_ = false;
    return;
  }
  double locked = locked_yaw_;
  if (pose.header.frame_id != locked_yaw_frame_) {
    try {
      const auto frames = tf2_buffer_->lookupTransform(
        pose.header.frame_id, locked_yaw_frame_, tf2::TimePointZero);
      locked += tf2::getYaw(frames.transform.rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Dock yaw lock skipped: %s", ex.what());
      return;
    }
  }
  const double measured = tf2::getYaw(pose.pose.orientation);
  const double tx = external_detection_translation_x_;
  const double ty = external_detection_translation_y_;
  const double tag_x = pose.pose.position.x - (std::cos(measured) * tx - std::sin(measured) * ty);
  const double tag_y = pose.pose.position.y - (std::sin(measured) * tx + std::cos(measured) * ty);
  pose.pose.position.x = tag_x + std::cos(locked) * tx - std::sin(locked) * ty;
  pose.pose.position.y = tag_y + std::sin(locked) * tx + std::cos(locked) * ty;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, locked);
  pose.pose.orientation = tf2::toMsg(orientation);
}

}  // namespace tetra_docking

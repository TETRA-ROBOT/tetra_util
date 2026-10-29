#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace tetra_docking
{

TetraChargingDock::TetraChargingDock()
: SimpleChargingDock(),
  marker_family_("tag36h11:"),
  camera_frame_("camera"),
  docking_status_topic_("docking_status"),
  contact_status_val_(2),
  latest_detected_tag_id_(-1),
  tag_detected_(false),
  current_docking_status_(0),
  tag_tf_as_detection_(false)
{
}

void TetraChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  SimpleChargingDock::configure(parent, name, tf);
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock parent node in TetraChargingDock::configure"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".marker_family", rclcpp::ParameterValue("tag36h11:"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".camera_frame", rclcpp::ParameterValue("camera"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".docking_status_topic", rclcpp::ParameterValue("docking_status"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".contact_status_val", rclcpp::ParameterValue(2));
  // 태그 TF 를 detected_dock_pose 로 넘겨 SimpleChargingDock 의 회전·이동 보정과 검출 시간 초과를 쓴다 (EDU).
  // 꺼 두면 예전처럼 태그 프레임 자세를 그대로 도크 자세로 쓴다 (M 동작 유지)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".tag_tf_as_detection", rclcpp::ParameterValue(false));

  node->get_parameter(name + ".marker_family", marker_family_);
  node->get_parameter(name + ".camera_frame", camera_frame_);
  node->get_parameter(name + ".docking_status_topic", docking_status_topic_);
  node->get_parameter(name + ".contact_status_val", contact_status_val_);
  node->get_parameter(name + ".tag_tf_as_detection", tag_tf_as_detection_);

  docking_status_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    docking_status_topic_, 1,
    std::bind(&TetraChargingDock::dockingStatusCallback, this, std::placeholders::_1));

  apriltag_sub_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detections", 10,
    std::bind(&TetraChargingDock::apriltagCallback, this, std::placeholders::_1));

  toggle_on_client_ = node->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured TetraChargingDock plugin: family=%s, contact_val=%d, tag_tf_as_detection=%d",
    marker_family_.c_str(), contact_status_val_, tag_tf_as_detection_);
}

void TetraChargingDock::dockingStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  current_docking_status_ = msg->data;
}

void TetraChargingDock::apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  if (!msg->detections.empty()) {
    latest_detected_tag_id_ = msg->detections[0].id;
    tag_detected_ = true;
  }
}

std::string TetraChargingDock::tagFrameFor(const std::string & id) const
{
  if (id.empty() && latest_detected_tag_id_ >= 0) {
    return marker_family_ + std::to_string(latest_detected_tag_id_);
  }
  return marker_family_ + id;
}

// apriltag 노드는 태그 자세를 카메라 광학 프레임 기준 TF 로만 낸다. 태그 프레임은 z 가 카메라 쪽을 향해
// 그대로 쓰면 도크 yaw 가 90도 틀어지므로, 검출 시각을 붙인 PoseStamped 로 바꿔 SimpleChargingDock 보정을 거친다.
// TF 버퍼의 마지막 값은 태그가 사라져도 남아 있어, 검출 시각을 그대로 두어야 external_detection_timeout 이 걸린다
void TetraChargingDock::updateDetectionFromTagTf(const std::string & id)
{
  const std::string tag_frame = tagFrameFor(id);
  try {
    if (!tf2_buffer_->canTransform(camera_frame_, tag_frame, tf2::TimePointZero)) {
      return;
    }
    auto tf = tf2_buffer_->lookupTransform(camera_frame_, tag_frame, tf2::TimePointZero);
    detected_dock_pose_.header = tf.header;
    detected_dock_pose_.pose.position.x = tf.transform.translation.x;
    detected_dock_pose_.pose.position.y = tf.transform.translation.y;
    detected_dock_pose_.pose.position.z = tf.transform.translation.z;
    detected_dock_pose_.pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed for tag %s: %s", tag_frame.c_str(), ex.what());
  }
}

bool TetraChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id)
{
  if (tag_tf_as_detection_) {
    updateDetectionFromTagTf(id);
    return SimpleChargingDock::getRefinedPose(pose, id);
  }

  std::string target_frame = tagFrameFor(id);

  if (tf2_buffer_->canTransform(pose.header.frame_id, target_frame, tf2::TimePointZero)) {
    try {
      auto tf = tf2_buffer_->lookupTransform(pose.header.frame_id, target_frame, tf2::TimePointZero);
      pose.header.stamp = tf.header.stamp;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "TF lookup failed for tag %s: %s", target_frame.c_str(), ex.what());
    }
  }

  // Fallback to base implementation (external detection / database pose)
  return SimpleChargingDock::getRefinedPose(pose, id);
}

// 예전에는 접점이 붙을 때 EKF 를 고정 map 좌표(home_x, home_y)로 리셋했다. 도크 위치는 지도마다 달라
// 고정 좌표와 다르면 위치가 순간이동하므로 기능을 없앴다 (2026-10-28)
bool TetraChargingDock::isDocked()
{
  if (current_docking_status_ >= contact_status_val_) {
    return true;
  }
  return SimpleChargingDock::isDocked();
}

bool TetraChargingDock::isCharging()
{
  if (current_docking_status_ >= contact_status_val_) {
    return true;
  }
  return SimpleChargingDock::isCharging();
}

bool TetraChargingDock::disableCharging()
{
  return true;
}

bool TetraChargingDock::hasStoppedCharging()
{
  return current_docking_status_ < contact_status_val_;
}

}  // namespace tetra_docking

PLUGINLIB_EXPORT_CLASS(tetra_docking::TetraChargingDock, opennav_docking_core::ChargingDock)

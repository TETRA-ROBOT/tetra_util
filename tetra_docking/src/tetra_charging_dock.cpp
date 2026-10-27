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
  current_docking_status_(0)
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

  node->get_parameter(name + ".marker_family", marker_family_);
  node->get_parameter(name + ".camera_frame", camera_frame_);
  node->get_parameter(name + ".docking_status_topic", docking_status_topic_);
  node->get_parameter(name + ".contact_status_val", contact_status_val_);

  docking_status_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    docking_status_topic_, 1,
    std::bind(&TetraChargingDock::dockingStatusCallback, this, std::placeholders::_1));

  apriltag_sub_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detections", 10,
    std::bind(&TetraChargingDock::apriltagCallback, this, std::placeholders::_1));

  toggle_on_client_ = node->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured TetraChargingDock plugin: family=%s, contact_val=%d",
    marker_family_.c_str(), contact_status_val_);
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

bool TetraChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id)
{
  std::string target_frame = marker_family_ + id;
  if (id.empty() && latest_detected_tag_id_ >= 0) {
    target_frame = marker_family_ + std::to_string(latest_detected_tag_id_);
  }

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

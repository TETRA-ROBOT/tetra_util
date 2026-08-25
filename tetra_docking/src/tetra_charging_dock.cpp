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
  reset_ekf_pose_(true),
  home_x_(-0.7),
  home_y_(0.0),
  latest_detected_tag_id_(-1),
  tag_detected_(false),
  current_docking_status_(0),
  pose_reset_done_(false)
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
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reset_ekf_pose", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".home_x", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".home_y", rclcpp::ParameterValue(0.0));

  node->get_parameter(name + ".marker_family", marker_family_);
  node->get_parameter(name + ".camera_frame", camera_frame_);
  node->get_parameter(name + ".docking_status_topic", docking_status_topic_);
  node->get_parameter(name + ".contact_status_val", contact_status_val_);
  node->get_parameter(name + ".reset_ekf_pose", reset_ekf_pose_);
  node->get_parameter(name + ".home_x", home_x_);
  node->get_parameter(name + ".home_y", home_y_);

  docking_status_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    docking_status_topic_, 1,
    std::bind(&TetraChargingDock::dockingStatusCallback, this, std::placeholders::_1));

  apriltag_sub_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detections", 10,
    std::bind(&TetraChargingDock::apriltagCallback, this, std::placeholders::_1));

  set_pose_client_ = node->create_client<robot_localization::srv::SetPose>("set_pose");
  toggle_on_client_ = node->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured TetraChargingDock plugin: family=%s, contact_val=%d, reset_ekf=%s, home=(%.2f, %.2f)",
    marker_family_.c_str(), contact_status_val_, reset_ekf_pose_ ? "true" : "false", home_x_, home_y_);
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

bool TetraChargingDock::isDocked()
{
  if (current_docking_status_ >= contact_status_val_) {
    if (reset_ekf_pose_ && !pose_reset_done_) {
      resetEkfPose(home_x_, home_y_, 0.0);
      pose_reset_done_ = true;
    }
    return true;
  }

  bool docked = SimpleChargingDock::isDocked();
  if (docked && reset_ekf_pose_ && !pose_reset_done_) {
    resetEkfPose(home_x_, home_y_, 0.0);
    pose_reset_done_ = true;
  }
  return docked;
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
  pose_reset_done_ = false;
  return true;
}

bool TetraChargingDock::hasStoppedCharging()
{
  return current_docking_status_ < contact_status_val_;
}

void TetraChargingDock::resetEkfPose(double x, double y, double yaw)
{
  if (!set_pose_client_->service_is_ready()) {
    return;
  }
  auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
  request->pose.header.frame_id = "map";
  request->pose.header.stamp = node_->now();
  request->pose.pose.pose.position.x = x;
  request->pose.pose.pose.position.y = y;
  request->pose.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  request->pose.pose.pose.orientation = tf2::toMsg(q);

  set_pose_client_->async_send_request(request);
  RCLCPP_INFO(node_->get_logger(), "Sent SetPose to robot_localization: (%.2f, %.2f, %.2f)", x, y, yaw);
}

}  // namespace tetra_docking

PLUGINLIB_EXPORT_CLASS(tetra_docking::TetraChargingDock, opennav_docking_core::ChargingDock)

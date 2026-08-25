#include "tetra_docking/tetra_lift_dock.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace tetra_docking
{

TetraLiftDock::TetraLiftDock()
: SimpleNonChargingDock(),
  marker_family_("tag36h11:"),
  camera_frame_("camera"),
  lift_action_mode_(1),
  lift_executed_(false),
  current_conveyor_sensor_(0),
  latest_detected_tag_id_(-1),
  tag_detected_(false)
{
}

void TetraLiftDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  SimpleNonChargingDock::configure(parent, name, tf);
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock parent node in TetraLiftDock::configure"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".marker_family", rclcpp::ParameterValue("tag36h11:"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".camera_frame", rclcpp::ParameterValue("camera"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lift_action_mode", rclcpp::ParameterValue(1)); // 1: LOAD (UP), 0: UNLOAD (DOWN)

  node->get_parameter(name + ".marker_family", marker_family_);
  node->get_parameter(name + ".camera_frame", camera_frame_);
  node->get_parameter(name + ".lift_action_mode", lift_action_mode_);

  conveyor_sensor_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "conveyor_sensor", 1,
    std::bind(&TetraLiftDock::conveyorSensorCallback, this, std::placeholders::_1));

  apriltag_sub_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detections", 10,
    std::bind(&TetraLiftDock::apriltagCallback, this, std::placeholders::_1));

  toggle_on_client_ = node->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured TetraLiftDock plugin: family=%s, lift_mode=%s",
    marker_family_.c_str(), lift_action_mode_ == 1 ? "LOAD (UP)" : "UNLOAD (DOWN)");
}

void TetraLiftDock::conveyorSensorCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  current_conveyor_sensor_ = msg->data;
}

void TetraLiftDock::apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
  if (!msg->detections.empty()) {
    latest_detected_tag_id_ = msg->detections[0].id;
    tag_detected_ = true;
  }
}

bool TetraLiftDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id)
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

  return SimpleNonChargingDock::getRefinedPose(pose, id);
}

bool TetraLiftDock::isDocked()
{
  bool docked = SimpleNonChargingDock::isDocked();
  if (docked && !lift_executed_) {
    executeLiftAction();
    lift_executed_ = true;
  }
  return docked;
}

void TetraLiftDock::executeLiftAction()
{
  if (toggle_on_client_->service_is_ready()) {
    auto req = std::make_shared<interfaces::srv::ToggleOn::Request>();
    req->id = lift_action_mode_;
    toggle_on_client_->async_send_request(req);
    RCLCPP_INFO(
      node_->get_logger(),
      "Executed lift action: mode=%d (%s)",
      lift_action_mode_, lift_action_mode_ == 1 ? "UP" : "DOWN");
  }
}

}  // namespace tetra_docking

PLUGINLIB_EXPORT_CLASS(tetra_docking::TetraLiftDock, opennav_docking_core::ChargingDock)

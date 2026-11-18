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
    node, name + ".lift_action_mode", rclcpp::ParameterValue(1)); // 보드 CN 코드: 1 싣기(올림), 0 내리기(내림), H 는 2 최대 높이

  node->get_parameter(name + ".marker_family", marker_family_);
  node->get_parameter(name + ".camera_frame", camera_frame_);
  node->get_parameter(name + ".lift_action_mode", lift_action_mode_);

  conveyor_sensor_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "conveyor_sensor", 1,
    std::bind(&TetraLiftDock::conveyorSensorCallback, this, std::placeholders::_1));

  apriltag_sub_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detections", 10,
    std::bind(&TetraLiftDock::apriltagCallback, this, std::placeholders::_1));

  lift_client_ = node->create_client<interfaces::srv::ConveyorAutoMovement>("conveyor_auto_move_cmd");

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured TetraLiftDock plugin: family=%s, lift CN code=%d",
    marker_family_.c_str(), lift_action_mode_);
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
  // 접근 중(아직 도킹 전)에 플래그를 내린다. 예전에는 한 번 올린 뒤 내리는 곳이 없어
  // docking_server 가 살아 있는 동안 두 번째 도킹부터 리프트가 움직이지 않았다
  if (!docked) {
    lift_executed_ = false;
  } else if (!lift_executed_) {
    lift_executed_ = executeLiftAction();
  }
  return docked;
}

bool TetraLiftDock::executeLiftAction()
{
  // 서비스가 아직 없으면 실행한 것으로 치지 않아 다음 isDocked 호출에서 다시 시도한다
  if (!lift_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "Lift service not ready; will retry");
    return false;
  }
  auto req = std::make_shared<interfaces::srv::ConveyorAutoMovement::Request>();
  req->start = lift_action_mode_;
  lift_client_->async_send_request(req);
  RCLCPP_INFO(node_->get_logger(), "Executed lift action: CN%d", lift_action_mode_);
  return true;
}

}  // namespace tetra_docking

PLUGINLIB_EXPORT_CLASS(tetra_docking::TetraLiftDock, opennav_docking_core::ChargingDock)

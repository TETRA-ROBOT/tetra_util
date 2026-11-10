#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>

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
  tag_tf_as_detection_(false),
  charge_requires_contact_(false),
  blind_approach_distance_(0.0),
  blind_approach_timeout_(8.0),
  undock_tag_distance_(0.0),
  undock_tag_lateral_(0.2),
  last_detection_time_(0, 0, RCL_ROS_TIME),
  tag_staging_(false),
  tag_staging_distance_(0.6),
  tag_staging_distance_min_(0.4),
  tag_staging_distance_max_(1.0),
  tag_staging_sample_time_(0.6),
  tag_staging_min_samples_(3),
  tag_staging_max_age_(1.5),
  tag_staging_max_tag_distance_(2.5),
  tag_staging_max_oblique_(0.87),
  tag_staging_skip_lateral_(0.05),
  tag_staging_skip_yaw_(0.1),
  tag_size_(0.1),
  tag_min_pixels_(50.0),
  tag_image_margin_(10.0)
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

  // 기본 SimpleChargingDock 은 배터리 상태를 안 쓰면 "자세 도착"을 충전 중으로 본다. 그러면 스테이션 전원이 꺼져
  // 접점 신호가 없어도 도킹 성공으로 끝나므로, 켜 두면 접점(docking_status)만 충전으로 본다 (EDU).
  // 자세 도착은 isDocked 로 남겨 로봇은 도크 앞에서 멈추고, wait_charge_timeout 뒤 FAILED_TO_CHARGE 로 끝난다
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".charge_requires_contact", rclcpp::ParameterValue(false));
  // 카메라가 도크 바로 앞에서는 태그를 시야 밖으로 놓친다 (실기 EDU, 접점 약 10~15 cm 전).
  // 마지막 검출 도크 자세까지 이 거리 안이면 그 자세(odom)로 마저 들어간다. 0 이면 끈다 (M 동작 유지).
  // 이보다 멀리서 놓치면 예전처럼 검출 실패로 멈춘다 (안 보이는 채로 멀리 주행하지 않도록)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".blind_approach_distance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".blind_approach_timeout", rclcpp::ParameterValue(8.0));
  // 언도킹 전용 인스턴스: 접점이 없어도 카메라 정면 이 거리 안에 태그가 보이면 도크에 붙은 것으로 보고
  // UndockRobot 이 staging_x_offset 만큼 후진하게 한다. 0 이면 끈다 (도킹용 인스턴스는 0 이어야 한다)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".undock_tag_distance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".undock_tag_lateral", rclcpp::ParameterValue(0.2));

  node->get_parameter(name + ".marker_family", marker_family_);
  node->get_parameter(name + ".camera_frame", camera_frame_);
  node->get_parameter(name + ".docking_status_topic", docking_status_topic_);
  node->get_parameter(name + ".contact_status_val", contact_status_val_);
  node->get_parameter(name + ".tag_tf_as_detection", tag_tf_as_detection_);
  node->get_parameter(name + ".charge_requires_contact", charge_requires_contact_);
  node->get_parameter(name + ".blind_approach_distance", blind_approach_distance_);
  node->get_parameter(name + ".blind_approach_timeout", blind_approach_timeout_);
  node->get_parameter(name + ".undock_tag_distance", undock_tag_distance_);
  node->get_parameter(name + ".undock_tag_lateral", undock_tag_lateral_);

  docking_status_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    docking_status_topic_, 1,
    std::bind(&TetraChargingDock::dockingStatusCallback, this, std::placeholders::_1));

  apriltag_sub_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detections", 10,
    std::bind(&TetraChargingDock::apriltagCallback, this, std::placeholders::_1));

  toggle_on_client_ = node->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");
  configureTagStaging(node, name);
  configureBumperGuard(node, name);
  configureApproachShaping(node, name);
  configureDockOffset(node, name);
  configureYawLock(node, name);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured TetraChargingDock plugin %s: family=%s, contact_val=%d, tag_tf_as_detection=%d, "
    "charge_requires_contact=%d, blind_approach_distance=%.2f, undock_tag_distance=%.2f, "
    "tag_staging=%d (%.2f m), bumper_topic=%s",
    name.c_str(), marker_family_.c_str(), contact_status_val_, tag_tf_as_detection_,
    charge_requires_contact_, blind_approach_distance_, undock_tag_distance_, tag_staging_,
    tag_staging_distance_, bumper_topic_.c_str());
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
  stopIfBumperPressed();
  if (tag_tf_as_detection_) {
    updateDetectionFromTagTf(id);
    // 기본 구현은 시간 초과면 경고만 찍고 false 를 돌려준다. 그 전에 최신 검출인지 직접 보고 근접 구간만 마저 간다
    const bool fresh = detected_dock_pose_.header.frame_id.size() > 0 &&
      node_->now() - rclcpp::Time(detected_dock_pose_.header.stamp) <=
      rclcpp::Duration::from_seconds(external_detection_timeout_);
    if (!fresh && getBlindApproachPose(pose)) {
      shapeFinalApproach(pose);
      return true;
    }
    if (!SimpleChargingDock::getRefinedPose(pose, id)) {
      return false;
    }
    lockDockYaw(pose);
    // 도크 자세 도착 판정(isDocked)도 dock_pose_ 를 보므로 영점을 옮긴 자세를 같이 남긴다
    applyDockOffset(pose);
    dock_pose_ = pose;
    // 근접 구간 이어받기에는 실제 도크 자세를 남기고, 제어 목표만 마지막 구간에 맞게 옮긴다
    last_detected_dock_pose_ = pose;
    last_detection_time_ = node_->now();
    shapeFinalApproach(pose);
    return true;
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

// 태그를 놓친 뒤 마지막 검출 도크 자세로 마저 들어갈지 판단한다. 놓친 지 오래됐거나(이전 도킹의 값),
// 남은 거리가 blind_approach_distance 보다 길면 거부해 검출 실패로 멈추게 한다
bool TetraChargingDock::getBlindApproachPose(geometry_msgs::msg::PoseStamped & pose)
{
  if (blind_approach_distance_ <= 0.0 || last_detected_dock_pose_.header.frame_id.empty()) {
    return false;
  }
  if (node_->now() - last_detection_time_ > rclcpp::Duration::from_seconds(blind_approach_timeout_)) {
    return false;
  }
  try {
    auto robot = tf2_buffer_->lookupTransform(
      last_detected_dock_pose_.header.frame_id, base_frame_id_, tf2::TimePointZero);
    const double dx = last_detected_dock_pose_.pose.position.x - robot.transform.translation.x;
    const double dy = last_detected_dock_pose_.pose.position.y - robot.transform.translation.y;
    const double remaining = std::hypot(dx, dy);
    if (remaining > blind_approach_distance_) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Dock tag lost %.2f m before the dock pose (blind limit %.2f m)", remaining,
        blind_approach_distance_);
      return false;
    }
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Dock tag out of view, finishing the last %.2f m on odometry", remaining);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Blind approach pose lookup failed: %s", ex.what());
    return false;
  }
  pose = last_detected_dock_pose_;
  return true;
}

// 카메라 광학 프레임은 z 가 정면, x 가 오른쪽이다. 태그 TF 는 사라져도 버퍼에 남으므로 시각으로 최신 여부를 본다
bool TetraChargingDock::isTagCloseAhead()
{
  const std::string tag_frame = tagFrameFor("");
  try {
    if (!tf2_buffer_->canTransform(camera_frame_, tag_frame, tf2::TimePointZero)) {
      return false;
    }
    auto tf = tf2_buffer_->lookupTransform(camera_frame_, tag_frame, tf2::TimePointZero);
    if (node_->now() - rclcpp::Time(tf.header.stamp) > rclcpp::Duration::from_seconds(0.5)) {
      return false;
    }
    const double forward = tf.transform.translation.z;
    const double lateral = tf.transform.translation.x;
    return forward > 0.0 && forward <= undock_tag_distance_ && std::abs(lateral) <= undock_tag_lateral_;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF lookup failed for tag %s: %s", tag_frame.c_str(), ex.what());
    return false;
  }
}

// 예전에는 접점이 붙을 때 EKF 를 고정 map 좌표(home_x, home_y)로 리셋했다. 도크 위치는 지도마다 달라
// 고정 좌표와 다르면 위치가 순간이동하므로 기능을 없앴다 (2026-10-28)
bool TetraChargingDock::isDocked()
{
  if (current_docking_status_ >= contact_status_val_) {
    return true;
  }
  // 언도킹 전용 인스턴스는 도킹 자세를 모르므로 (매 언도킹마다 로봇 위치가 도크 자세) 태그 거리로만 본다
  if (undock_tag_distance_ > 0.0) {
    return isTagCloseAhead();
  }
  if (docked_by_depth_) {
    return isDockDepthReached();
  }
  return SimpleChargingDock::isDocked();
}

bool TetraChargingDock::isCharging()
{
  if (current_docking_status_ >= contact_status_val_) {
    return true;
  }
  if (charge_requires_contact_) {
    return false;
  }
  return SimpleChargingDock::isCharging();
}

bool TetraChargingDock::disableCharging()
{
  return true;
}

bool TetraChargingDock::hasStoppedCharging()
{
  stopUndockIfBumperPressed();
  return current_docking_status_ < contact_status_val_;
}

}  // namespace tetra_docking

PLUGINLIB_EXPORT_CLASS(tetra_docking::TetraChargingDock, opennav_docking_core::ChargingDock)

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
  // 접근 중 비스듬히 들어가다 태그를 놓치면 blind 거리보다 멀어도 마지막 검출 자세 쪽으로 몰아 다시 찾는다.
  // 0 이면 끈다 (예전처럼 바로 검출 실패). 실기 MH 2026-11-18: 도크 0.53 m 앞에서 놓쳐 904 로 실패
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lost_tag_reacquire_distance", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lost_tag_reacquire_time", rclcpp::ParameterValue(3.0));
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
  node->get_parameter(name + ".lost_tag_reacquire_distance", lost_tag_reacquire_distance_);
  node->get_parameter(name + ".lost_tag_reacquire_time", lost_tag_reacquire_time_);
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
  configureTagSearch(node, name);

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

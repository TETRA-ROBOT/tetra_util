// 라이다 도킹 마커(스테이션의 V 자 홈)로 도킹하기 (detection_source: lidar, EDU)
//
// 검출은 line_docking_detector 가 2D 스캔에서 홈을 찾아 PoseStamped(라이다 프레임)로 낸다 (lidar_detection_topic).
// 그 자세는 AprilTag 와 같은 축 규칙·같은 자리(도킹을 마친 base_link 앞 0.34 m)로 맞춰 두어,
// SimpleChargingDock 의 검출→도크 보정(external_detection_*)과 이 플러그인의 영점·마지막 구간 보정을 태그와 같이 쓴다.
// 태그 전용 단계(태그 staging·탐색·멈춘 채 확인)는 건너뛴다.
//
// 라이다는 홈 꼭짓점이 0.5 m 보다 가까우면 날개 끝이 최소 거리에 잘려 홈을 못 찾는다 (실측 2026-11-18, 도킹 완료 때 0.42 m).
// 그래서 마지막 약 0.1 m 는 blind approach(odom) 로 마저 들어가고, 언도킹 판단은 마지막으로 본 마커를 odom 에 기억해 둔 값으로 한다.
//
// 검출 방식은 관제가 도킹 영점처럼 목표 직전에 파라미터로 넣는다. Nav2 를 다시 띄우지 않고 다음 도킹부터 바뀐다.
// Nav2 기본 검출 토픽(detected_dock_pose)은 쓰지 않는다: SimpleChargingDock 이 그 토픽을 받으면 태그 모드의 검출을 덮어쓴다.
#include "tetra_docking/tetra_charging_dock.hpp"

#include "nav2_util/node_utils.hpp"
#include "tetra_docking/lidar_marker.hpp"

namespace tetra_docking
{

void TetraChargingDock::configureLidarDetection(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  lidar_param_name_ = name + ".detection_source";
  nav2_util::declare_parameter_if_not_declared(node, lidar_param_name_, rclcpp::ParameterValue(std::string("tag")));
  // 비우면 라이다 모드를 쓸 수 없다 (그 모델은 마커 검출기를 띄우지 않는다)
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_detection_topic", rclcpp::ParameterValue(std::string("")));
  // 언도킹 전용 인스턴스: 마커(도킹 완료 때 base_link 앞 0.34 m 자리)가 정면 이 거리 안이면 도크 앞으로 본다
  nav2_util::declare_parameter_if_not_declared(node, name + ".undock_lidar_distance", rclcpp::ParameterValue(0.0));
  // odom 에 기억한 마커를 언도킹 판단에 쓸 수 있는 시간. 오래되면 odom 이 흘러 믿지 않는다
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_marker_memory", rclcpp::ParameterValue(600.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_marker_frame", rclcpp::ParameterValue(std::string("odom")));

  node->get_parameter(name + ".lidar_detection_topic", lidar_detection_topic_);
  node->get_parameter(name + ".undock_lidar_distance", undock_lidar_distance_);
  node->get_parameter(name + ".lidar_marker_memory", lidar_marker_memory_);
  node->get_parameter(name + ".lidar_marker_frame", lidar_marker_frame_);

  const auto source = parseDetectionSource(node->get_parameter(lidar_param_name_).as_string());
  lidar_mode_ = source == DetectionSource::kLidar && !lidar_detection_topic_.empty();

  if (!lidar_detection_topic_.empty()) {
    lidar_detection_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      lidar_detection_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { lidarDetectionCallback(msg); });
  }
  lidar_param_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) { return onDetectionSourceParameter(params); });

  RCLCPP_INFO(
    node->get_logger(), "%s: detection_source=%s, lidar_detection_topic='%s', undock_lidar_distance=%.2f",
    name.c_str(), lidar_mode_ ? "lidar" : "tag", lidar_detection_topic_.c_str(), undock_lidar_distance_);
}

rcl_interfaces::msg::SetParametersResult TetraChargingDock::onDetectionSourceParameter(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : params) {
    if (param.get_name() != lidar_param_name_) {
      continue;
    }
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      result.successful = false;
      result.reason = "detection_source must be a string (tag | lidar)";
      return result;
    }
    const auto source = parseDetectionSource(param.as_string());
    if (!source) {
      result.successful = false;
      result.reason = "detection_source must be tag or lidar";
      return result;
    }
    if (*source == DetectionSource::kLidar && lidar_detection_topic_.empty()) {
      result.successful = false;
      result.reason = "lidar detection is not configured for this dock (lidar_detection_topic is empty)";
      return result;
    }
    const bool lidar = *source == DetectionSource::kLidar;
    if (lidar != lidar_mode_) {
      RCLCPP_INFO(node_->get_logger(), "%s -> %s", lidar_param_name_.c_str(), lidar ? "lidar" : "tag");
    }
    lidar_mode_ = lidar;
  }
  return result;
}

// 검출을 받을 때마다 고정 프레임(odom)의 마커 위치도 남긴다. 도크 바로 앞에서는 홈이 안 보이므로
// 언도킹 판단은 이 값으로 한다 (접근하며 계속 보다가 마지막 약 0.1 m 만 못 보는 것이라 odom 으로 충분히 맞다)
void TetraChargingDock::lidarDetectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    latest_lidar_detection_ = *msg;
  }
  try {
    const auto fixed_from_sensor = tf2_buffer_->lookupTransform(
      lidar_marker_frame_, msg->header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped in_fixed;
    tf2::doTransform(*msg, in_fixed, fixed_from_sensor);
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    lidar_marker_in_fixed_ = in_fixed;
    lidar_marker_time_ = node_->now();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Lidar marker to %s failed: %s",
      lidar_marker_frame_.c_str(), ex.what());
  }
}

// 라이다 검출로 도크 자세를 구하는 한 주기. 최신 검출이면 정밀 접근, 놓쳤으면 도크 바로 앞에서만 마지막 자세로 마저 간다
bool TetraChargingDock::refineFromLidarDetection(geometry_msgs::msg::PoseStamped & pose, const std::string & id)
{
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    if (!latest_lidar_detection_.header.frame_id.empty()) {
      detected_dock_pose_ = latest_lidar_detection_;
    }
  }
  const bool fresh = !detected_dock_pose_.header.frame_id.empty() &&
    node_->now() - rclcpp::Time(detected_dock_pose_.header.stamp) <=
    rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (!fresh) {
    if (getBlindApproachPose(pose)) {
      shapeFinalApproach(pose);
      return true;
    }
    return false;
  }
  return approachDetectedDock(pose, id);
}

bool TetraChargingDock::isLidarMarkerCloseAhead()
{
  geometry_msgs::msg::PoseStamped marker;
  rclcpp::Time seen_at;
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    marker = lidar_marker_in_fixed_;
    seen_at = lidar_marker_time_;
  }
  if (marker.header.frame_id.empty() ||
    node_->now() - seen_at > rclcpp::Duration::from_seconds(lidar_marker_memory_))
  {
    return false;
  }
  try {
    const auto robot = tf2_buffer_->lookupTransform(marker.header.frame_id, base_frame_id_, tf2::TimePointZero);
    const MarkerOffset offset = markerRelativeToRobot(
      robot.transform.translation.x, robot.transform.translation.y, tf2::getYaw(robot.transform.rotation),
      marker.pose.position.x, marker.pose.position.y);
    return isMarkerCloseAhead(offset, undock_lidar_distance_, undock_tag_lateral_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Robot pose lookup for the lidar marker failed: %s", ex.what());
    return false;
  }
}

}  // namespace tetra_docking

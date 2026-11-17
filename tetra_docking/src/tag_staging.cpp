// 태그 정렬 staging: 도킹을 시작할 때 태그가 보이면 그 검출로 도크 자세를 다시 구해 태그 정면 축 위의
// staging 자세를 돌려준다. 도킹 서버는 이 자세로 Nav2(NavigateToPose)를 보내고, 거기서 마지막 직선 접근을 한다.
// 비스듬한 자리에서 바로 접근하면 제어 법칙이 도는 동안 태그가 시야 밖으로 나가 검출 실패(904)로 멈췄다
// (SM 은 RGB 센서가 중심에서 옆으로 치우쳐 더 일찍 놓쳤다).
#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"
#include "angles/angles.h"
#include "tetra_docking/dock_geometry.hpp"

#include <cmath>
#include <algorithm>
#include <thread>

namespace tetra_docking
{

namespace
{
// 표본끼리 이만큼 넘게 다르면 태그 자세가 흔들리는 것으로 보고 쓰지 않는다
constexpr double kMaxSampleSpreadXy = 0.08;
constexpr double kMaxSampleSpreadYaw = 0.12;
// 언도킹은 로봇 자신의 자세를 도크 자세로 넘긴다 (docking_server undockRobot)
constexpr double kUndockPoseTolerance = 0.02;
}  // namespace

void TetraChargingDock::configureTagStaging(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  // 선언과 읽기를 한 번에 한다 (기본값은 생성자 값과 같다)
  auto param = [&](const std::string & key, const rclcpp::ParameterValue & value, auto & target) {
      nav2_util::declare_parameter_if_not_declared(node, name + "." + key, value);
      node->get_parameter(name + "." + key, target);
    };
  // 꺼 두면 staging 은 목표 도크 자세 + staging_x_offset 그대로다 (M, 언도킹 전용 인스턴스)
  param("tag_staging", rclcpp::ParameterValue(false), tag_staging_);
  // 도크 자세(접점 위치의 base_link)에서 staging 까지 거리. 카메라 정보가 있으면 태그가 영상에 다 들어오도록 조정한다
  param("tag_staging_distance", rclcpp::ParameterValue(0.6), tag_staging_distance_);
  param("tag_staging_distance_min", rclcpp::ParameterValue(0.4), tag_staging_distance_min_);
  param("tag_staging_distance_max", rclcpp::ParameterValue(1.0), tag_staging_distance_max_);
  // 검출 자세는 프레임마다 수 cm, 수 도씩 흔들린다. 이 시간 동안 모아 평균한다
  param("tag_staging_sample_time", rclcpp::ParameterValue(0.6), tag_staging_sample_time_);
  param("tag_staging_min_samples", rclcpp::ParameterValue(3), tag_staging_min_samples_);
  param("tag_staging_max_age", rclcpp::ParameterValue(1.5), tag_staging_max_age_);
  // 멀거나 비스듬한 태그는 yaw 추정이 크게 틀려 staging 이 축에서 벗어난다
  param("tag_staging_max_tag_distance", rclcpp::ParameterValue(2.5), tag_staging_max_tag_distance_);
  param("tag_staging_max_oblique", rclcpp::ParameterValue(0.87), tag_staging_max_oblique_);
  // 이미 도크 축 위에서 도크를 보고 있으면 (staging 보다 가까워도) Nav2 로 물러나지 않고 바로 접근한다
  param("tag_staging_skip_lateral", rclcpp::ParameterValue(0.05), tag_staging_skip_lateral_);
  param("tag_staging_skip_yaw", rclcpp::ParameterValue(0.1), tag_staging_skip_yaw_);
  param("tag_size", rclcpp::ParameterValue(0.1), tag_size_);
  param("tag_min_pixels", rclcpp::ParameterValue(50.0), tag_min_pixels_);
  param("tag_image_margin", rclcpp::ParameterValue(10.0), tag_image_margin_);
  param("camera_info_topic", rclcpp::ParameterValue(std::string("")), camera_info_topic_);
  // 비우면 도크 프레임(map)에서 staging 을 낸다. map 은 AMCL 이 로봇이 움직일 때마다 보정해, 태그를 본 뒤 첫 주행에서
  // map->odom 이 바뀌면 staging 이 실제 태그 축에서 벗어난다 (실기 M 2026-11-10: Nav2 는 도착인데 태그가 14도 옆).
  // odom 으로 내면 Nav2 가 재계획마다 최신 TF 로 목표를 옮겨 실제 태그 기준으로 선다
  param("tag_staging_frame", rclcpp::ParameterValue(std::string("")), tag_staging_frame_);

  if (tag_staging_ && !camera_info_topic_.empty()) {
    camera_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TetraChargingDock::cameraInfoCallback, this, std::placeholders::_1));
  }
}

void TetraChargingDock::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(intrinsics_mutex_);
  intrinsics_.fx = msg->k[0];
  intrinsics_.cx = msg->k[2];
  intrinsics_.fy = msg->k[4];
  intrinsics_.cy = msg->k[5];
  intrinsics_.width = static_cast<int>(msg->width);
  intrinsics_.height = static_cast<int>(msg->height);
}

geometry_msgs::msg::PoseStamped TetraChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  if (isUndockRequest(pose, frame)) {
    return SimpleChargingDock::getStagingPose(pose, frame);
  }
  // 새 도킹: 지난 도킹의 탐색 상태·고정 yaw 를 쓰지 않는다
  resetTagSearch();
  locked_yaw_valid_ = false;
  if (!tag_staging_) {
    return SimpleChargingDock::getStagingPose(pose, frame);
  }
  const std::string & staging_frame = tag_staging_frame_.empty() ? frame : tag_staging_frame_;
  TagDockEstimate estimate;
  if (!estimateDockFromTag(staging_frame, estimate)) {
    RCLCPP_WARN(node_->get_logger(), "Tag staging unavailable, staging from the goal dock pose");
    return SimpleChargingDock::getStagingPose(pose, frame);
  }

  const double distance = stagingDistanceFor(estimate);
  const DockSample & dock = estimate.dock;
  rememberStagingYaw(dock, staging_frame);
  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = staging_frame;
  staging.header.stamp = node_->now();
  staging.pose.position.x = dock.x - std::cos(dock.yaw) * distance;
  staging.pose.position.y = dock.y - std::sin(dock.yaw) * distance;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, dock.yaw + staging_yaw_offset_);
  staging.pose.orientation = tf2::toMsg(orientation);

  geometry_msgs::msg::PoseStamped robot_pose;
  if (robotOnApproachLine(dock, staging_frame, distance, robot_pose)) {
    // 도킹 서버는 로봇이 staging 자세 허용 범위 안이면 Nav2 주행을 건너뛴다
    staging = robot_pose;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "Tag staging: dock (%.2f, %.2f, %.2f rad) in %s from %zu samples, staging %.2f m before the dock "
    "at (%.2f, %.2f)", dock.x, dock.y, dock.yaw, staging_frame.c_str(), estimate.samples, distance,
    staging.pose.position.x, staging.pose.position.y);
  staging_pose_pub_->publish(staging);
  return staging;
}

bool TetraChargingDock::isUndockRequest(const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  try {
    auto robot = tf2_buffer_->lookupTransform(frame, base_frame_id_, tf2::TimePointZero);
    return std::hypot(
      pose.position.x - robot.transform.translation.x,
      pose.position.y - robot.transform.translation.y) < kUndockPoseTolerance;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

// 카메라 주기(15 Hz)로 새 태그 TF 가 오는 것을 sample_time 동안 모은다. 액션 스레드에서 불리므로 잠깐 기다려도 된다
bool TetraChargingDock::estimateDockFromTag(const std::string & frame, TagDockEstimate & estimate)
{
  const std::string tag_frame = tagFrameFor("");
  std::vector<DockSample> samples;
  double height_sum = 0.0;
  rclcpp::Time last_stamp(0, 0, RCL_ROS_TIME);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(tag_staging_sample_time_);
  while (rclcpp::ok()) {
    collectTagSample(frame, tag_frame, last_stamp, samples, height_sum);
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (samples.size() < static_cast<size_t>(std::max(1, tag_staging_min_samples_))) {
    RCLCPP_WARN(
      node_->get_logger(), "Tag staging: only %zu usable detections of %s", samples.size(),
      tag_frame.c_str());
    return false;
  }
  if (!averageDockSamples(samples, kMaxSampleSpreadXy, kMaxSampleSpreadYaw, estimate.dock)) {
    RCLCPP_WARN(node_->get_logger(), "Tag staging: detections of %s are unstable", tag_frame.c_str());
    return false;
  }
  estimate.tag_height = height_sum / static_cast<double>(samples.size());
  estimate.samples = samples.size();
  return true;
}

bool TetraChargingDock::collectTagSample(
  const std::string & frame, const std::string & tag_frame, rclcpp::Time & last_stamp,
  std::vector<DockSample> & samples, double & height_sum)
{
  try {
    if (!tf2_buffer_->canTransform(camera_frame_, tag_frame, tf2::TimePointZero)) {
      return false;
    }
    auto camera_to_tag = tf2_buffer_->lookupTransform(camera_frame_, tag_frame, tf2::TimePointZero);
    const rclcpp::Time stamp(camera_to_tag.header.stamp, RCL_ROS_TIME);
    // TF 버퍼의 마지막 값은 태그가 사라져도 남는다. 같은 검출을 두 번 세거나 오래된 검출을 쓰지 않는다
    if (stamp == last_stamp) {
      return false;
    }
    last_stamp = stamp;
    if (node_->now() - stamp > rclcpp::Duration::from_seconds(tag_staging_max_age_) ||
      !isTagUsableForStaging(camera_to_tag.transform))
    {
      return false;
    }
    const auto timeout = tf2::durationFromSec(0.2);
    auto fixed_to_tag = tf2_buffer_->lookupTransform(
      frame, tag_frame, tf2_ros::fromRclcpp(stamp), timeout);
    auto base_to_tag = tf2_buffer_->lookupTransform(
      base_frame_id_, tag_frame, tf2_ros::fromRclcpp(stamp), timeout);
    samples.push_back(dockSampleFromTag(fixed_to_tag.transform));
    height_sum += base_to_tag.transform.translation.z;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Tag staging sample skipped: %s", ex.what());
    return false;
  }
}

// 카메라 광학 프레임 기준: 태그 z 축은 카메라 쪽을 향하므로, 시선과 반대 방향일수록 정면이다
bool TetraChargingDock::isTagUsableForStaging(const geometry_msgs::msg::Transform & camera_to_tag) const
{
  const tf2::Vector3 position(
    camera_to_tag.translation.x, camera_to_tag.translation.y, camera_to_tag.translation.z);
  const double distance = position.length();
  if (distance <= 0.0 || distance > tag_staging_max_tag_distance_) {
    return false;
  }
  tf2::Quaternion rotation;
  tf2::fromMsg(camera_to_tag.rotation, rotation);
  const tf2::Vector3 normal = tf2::quatRotate(rotation, tf2::Vector3(0.0, 0.0, 1.0));
  const double facing = -normal.dot(position / distance);
  return std::acos(std::clamp(facing, -1.0, 1.0)) <= tag_staging_max_oblique_;
}

// SimpleChargingDock::getRefinedPose 와 같은 회전·이동 보정을 필터 없이 한 표본에 적용한다
DockSample TetraChargingDock::dockSampleFromTag(const geometry_msgs::msg::Transform & fixed_to_tag) const
{
  tf2::Quaternion tag_rotation;
  tf2::fromMsg(fixed_to_tag.rotation, tag_rotation);
  const double yaw = tf2::getYaw(tag_rotation * external_detection_rotation_);
  DockSample sample;
  sample.yaw = yaw;
  const PlanarOffset dock = dockPositionFromTag(
    fixed_to_tag.translation.x, fixed_to_tag.translation.y, yaw,
    external_detection_translation_x_, external_detection_translation_y_);
  sample.x = dock.x;
  sample.y = dock.y;
  // staging 축도 접근 중 도크 자세와 같은 영점을 써야 마지막 구간에서 옆으로 꺾지 않는다
  return applyDockOffset(sample);
}

double TetraChargingDock::stagingDistanceFor(const TagDockEstimate & estimate)
{
  StagingViewInputs inputs;
  {
    std::lock_guard<std::mutex> lock(intrinsics_mutex_);
    inputs.intrinsics = intrinsics_;
  }
  if (!inputs.intrinsics.valid()) {
    return tag_staging_distance_;
  }
  try {
    auto optical_from_base = tf2_buffer_->lookupTransform(camera_frame_, base_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(optical_from_base.transform, inputs.optical_from_base);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Tag staging: camera TF unavailable (%s)", ex.what());
    return tag_staging_distance_;
  }
  inputs.tag_ahead_of_dock = -(external_detection_translation_x_ + dock_offset_x_);
  inputs.tag_lateral = -(external_detection_translation_y_ + dock_offset_y_);
  inputs.tag_height = estimate.tag_height;
  inputs.backward = std::cos(staging_yaw_offset_) < 0.0;
  inputs.tag_size = tag_size_;
  inputs.margin_px = tag_image_margin_;
  inputs.min_pixels = tag_min_pixels_;
  const double distance = chooseStagingDistance(
    inputs, tag_staging_distance_, tag_staging_distance_min_, tag_staging_distance_max_);

  // 직선 접근 중 태그가 영상에 온전히 남는 가장 가까운 거리. 이보다 가까우면 blind_approach 가 이어받아야 한다
  inputs.margin_px = 0.0;
  double visible_until = distance;
  while (visible_until > 0.0 && projectTagAtStaging(inputs, visible_until - 0.01).in_view) {
    visible_until -= 0.01;
  }
  const auto view = projectTagAtStaging(inputs, distance);
  RCLCPP_INFO(
    node_->get_logger(),
    "Tag staging view: tag %.0f px wide at %.2f m, fully visible down to %.2f m before the dock "
    "(blind approach %.2f m)", view.pixel_width, distance, visible_until, blind_approach_distance_);
  if (visible_until > blind_approach_distance_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Tag leaves the image %.2f m before the dock but blind_approach_distance is %.2f m",
      visible_until, blind_approach_distance_);
  }
  return distance;
}

// 도크 축 기준 로봇 위치: along 은 도크 앞(로봇 쪽)이 음수, lateral 은 축에서 옆으로 벗어난 거리
bool TetraChargingDock::robotOnApproachLine(
  const DockSample & dock, const std::string & frame, double staging_distance,
  geometry_msgs::msg::PoseStamped & robot_pose)
{
  try {
    auto robot = tf2_buffer_->lookupTransform(frame, base_frame_id_, tf2::TimePointZero);
    const double dx = robot.transform.translation.x - dock.x;
    const double dy = robot.transform.translation.y - dock.y;
    const double front = -(std::cos(dock.yaw) * dx + std::sin(dock.yaw) * dy);
    const double lateral = -std::sin(dock.yaw) * dx + std::cos(dock.yaw) * dy;
    // 후진 도킹(M)은 staging_yaw_offset 이 pi 라 로봇이 도크 yaw 와 반대로 선다. staging 자세 heading 과 비교한다
    const double heading = angles::normalize_angle(
      tf2::getYaw(robot.transform.rotation) - (dock.yaw + staging_yaw_offset_));
    if (front <= 0.0 || front > staging_distance + 0.05 ||
      std::abs(lateral) > tag_staging_skip_lateral_ || std::abs(heading) > tag_staging_skip_yaw_)
    {
      return false;
    }
    robot_pose.header.frame_id = frame;
    robot_pose.header.stamp = node_->now();
    robot_pose.pose.position.x = robot.transform.translation.x;
    robot_pose.pose.position.y = robot.transform.translation.y;
    robot_pose.pose.orientation = robot.transform.rotation;
    RCLCPP_INFO(
      node_->get_logger(), "Tag staging: already on the dock axis (%.2f m ahead, %.3f m off, %.3f rad), "
      "approaching from here", front, lateral, heading);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Tag staging: robot pose unavailable (%s)", ex.what());
    return false;
  }
}

}  // namespace tetra_docking

// 태그 검출을 잠깐 모아 도크 자세를 추정한다. 태그 정렬 staging(tag_staging.cpp)과 접근 전 멈춰서 확인(tag_search.cpp)이 쓴다.
// tag_staging.cpp 에서 옮겨 왔다
#include "tetra_docking/tetra_charging_dock.hpp"
#include "tetra_docking/dock_geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace tetra_docking
{

namespace
{
// 표본끼리 이만큼 넘게 다르면 태그 자세가 흔들리는 것으로 보고 쓰지 않는다
constexpr double kMaxSampleSpreadXy = 0.08;
constexpr double kMaxSampleSpreadYaw = 0.12;
}  // namespace

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

}  // namespace tetra_docking

// 도킹 접근 중 매 주기 도크 자세를 구한다 (getRefinedPose): 태그 TF 를 검출로 넘기기, 가운데 맞추기·확인·탐색·정밀 접근의
// 순서, 태그를 놓쳤을 때 마지막 자세로 마저 들어가기. tetra_charging_dock.cpp 에서 옮겨 왔다
#include "tetra_docking/tetra_charging_dock.hpp"

#include <cmath>

namespace tetra_docking
{

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
  last_refine_time_ = node_->now();
  stopIfBumperPressed();
  // 관제 옵션 dock_detection 이 lidar 이면 태그 대신 스테이션의 V 자 홈 검출을 쓴다 (lidar_detection.cpp)
  if (lidar_mode_) {
    return refineFromLidarDetection(pose, id);
  }
  if (tag_tf_as_detection_) {
    return refineFromTagDetection(pose, id);
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

// 태그 검출로 도크 자세를 구하는 한 주기 (tag_tf_as_detection). 단계는 위에서 아래로:
//   1) 태그가 보이지만 영상 가장자리면 가운데로 돌린다            -> 이번 주기는 자세 없음
//   2) 멈춘 채로 확인한다. 아직이면 좌우로 돌며 찾는다             -> 이번 주기는 자세 없음
//   3) 확인했으면 검출 자세로 정밀 접근한다
// 태그가 안 보일 때는 도크 바로 앞이면 마지막 자세로 마저 들어가고, 아직 본 적이 없으면 돌며 찾는다.
// 1)·2) 는 태그 탐색이나 yaw 고정을 켠 설정만 거친다 (2026-11-18 부터 M·MH·EDU·SM 탐색 켬). 둘 다 끄면 바로 3) 으로 간다
bool TetraChargingDock::refineFromTagDetection(geometry_msgs::msg::PoseStamped & pose, const std::string & id)
{
  updateDetectionFromTagTf(id);
  // 기본 구현은 시간 초과면 경고만 찍고 false 를 돌려준다. 그 전에 최신 검출인지 직접 보고 근접 구간만 마저 간다
  const bool fresh = detected_dock_pose_.header.frame_id.size() > 0 &&
    node_->now() - rclcpp::Time(detected_dock_pose_.header.stamp) <=
    rclcpp::Duration::from_seconds(external_detection_timeout_);
  if (fresh) {
    // 보이긴 하지만 아직 멈춘 채로 확인하지 못했으면 (지나가며 스친 검출) 계속 찾는다
    if (centerOnTag()) {
      return false;
    }
    if (!confirmTagSeen(pose.header.frame_id)) {
      searchForTag();
      return false;
    }
  } else {
    if (getBlindApproachPose(pose)) {
      shapeFinalApproach(pose);
      return true;
    }
    // 아직 태그를 못 본 초기 인식이면 제자리에서 돌며 찾는다 (찾을 때까지 검출 실패로 돌려준다)
    if (searchForTag()) {
      return false;
    }
  }
  return approachDetectedDock(pose, id);
}

// 검출 자세에 회전·이동 보정(SimpleChargingDock), yaw 고정, 도킹 영점, 마지막 구간 보정을 차례로 적용한다
bool TetraChargingDock::approachDetectedDock(geometry_msgs::msg::PoseStamped & pose, const std::string & id)
{
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

// 태그를 놓친 뒤 마지막 검출 도크 자세(odom)로 계속 갈지 판단한다 (decideLostTag).
// 가까우면 마저 들어가고(blind), 조금 멀면 그쪽으로 몰아 태그를 다시 찾고(reacquire), 아니면 거부해 검출 실패로 멈춘다
bool TetraChargingDock::getBlindApproachPose(geometry_msgs::msg::PoseStamped & pose)
{
  if (last_detected_dock_pose_.header.frame_id.empty() ||
    (blind_approach_distance_ <= 0.0 && lost_tag_reacquire_distance_ <= 0.0))
  {
    return false;
  }
  double remaining = 0.0;
  try {
    auto robot = tf2_buffer_->lookupTransform(
      last_detected_dock_pose_.header.frame_id, base_frame_id_, tf2::TimePointZero);
    remaining = std::hypot(
      last_detected_dock_pose_.pose.position.x - robot.transform.translation.x,
      last_detected_dock_pose_.pose.position.y - robot.transform.translation.y);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Blind approach pose lookup failed: %s", ex.what());
    return false;
  }
  const double since_detection = (node_->now() - last_detection_time_).seconds();
  switch (decideLostTag(
      remaining, since_detection, blind_approach_distance_, blind_approach_timeout_,
      lost_tag_reacquire_distance_, lost_tag_reacquire_time_))
  {
    case LostTagAction::kBlind:
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Dock tag out of view, finishing the last %.2f m on odometry", remaining);
      break;
    case LostTagAction::kReacquire:
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Dock tag lost %.2f m before the dock, turning toward its last position to find it again (%.1f s)",
        remaining, since_detection);
      break;
    case LostTagAction::kFail:
    default:
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Dock tag lost %.2f m before the dock pose (blind limit %.2f m, reacquire %.2f m within %.1f s)",
        remaining, blind_approach_distance_, lost_tag_reacquire_distance_, lost_tag_reacquire_time_);
      return false;
  }
  pose = last_detected_dock_pose_;
  return true;
}

}  // namespace tetra_docking

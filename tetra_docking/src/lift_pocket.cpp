// 리프트 포트(ㄷ 자 받침대)에 라이다로 들어가기 (TetraLiftDock, detection_source: lidar)
//
// 예전 리프트 도킹은 포트에 붙인 AprilTag 를 보고 들어갔다. 포트는 다리 두 개 사이로 로봇이 들어가 짐을 뜨는 ㄷ 자라
// 태그 없이도 2D 스캔에서 두 다리의 안쪽 면이 보인다. 매 주기:
//   1. 새 스캔이 있으면 두 다리를 직선으로 맞춰 포트 입구 자세를 구하고(pocket_detection.cpp) odom 에 기억·평활한다.
//      사전 자세는 처음에는 지도에 등록한 도크 자세(입구 + insert_depth), 그 뒤로는 지난 검출이다.
//   2. 도크 자세 = 입구에서 축을 따라 insert_depth 안쪽. 도킹 서버의 graceful 제어기가 그 자세로 몬다.
//   3. 안전: 로봇 외곽 꼭짓점이 입구를 넘을 때 다리 안쪽 면에서 entry_margin 안으로 들어오면(틀어진 채 진입) 멈추고,
//      옆·앞 스캔 점이 외곽에서 min_side/front_clearance 안이면 멈춘다 (FailedToControl → 도킹 실패, 관제 알림).
//      도킹 서버의 costmap 충돌 검사는 dock_collision_threshold 안을 보지 않아 다리 사이에서는 이 검사만 남는다.
//   4. 도착은 포트 축 깊이로 판정하고(좌우 잔여 오차로 판정 폭을 못 맞춰 멈추지 않는 것 방지), 도착하면 바로 0 속도를 낸다.
//      도킹 서버는 도착해도 0 을 내지 않아 휠 드라이버 cmd_vel_timeout 동안 더 구른다 (contact_stop.cpp 와 같은 이유).
// 다리가 잠깐 안 보여도 입구를 넘은 뒤에는 기억한 자세(odom)로 마저 들어간다. 입구 앞에서 pocket_lost_timeout 넘게 안 보이면 실패다.
#include "tetra_docking/tetra_lift_dock.hpp"

#include <cmath>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
#include "tetra_docking/lidar_marker.hpp"

namespace tetra_docking
{

namespace
{
// getRefinedPose 사이가 이보다 길면 새 도킹 목표로 보고 기억한 포트를 버린다 (도킹 서버는 50 Hz 로 부른다)
constexpr double kNewSessionGap = 2.0;
// 로봇 앞 끝이 입구에서 이 거리 안이면 "들어가는 중"으로 보고 옆 여유를 검사한다. 멀리서는 다리 아닌 주변 물체를 보지 않는다
constexpr double kClearanceCheckDistance = 0.10;

template<typename T>
T declareAndGet(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name, const T & default_value)
{
  nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(default_value));
  return node->get_parameter(name).get_value<T>();
}

double frontOfRobotInPocket(const PocketPose & robot_in_pocket, const RectFootprint & footprint)
{
  return robot_in_pocket.x + footprint.front * std::cos(robot_in_pocket.yaw);
}
}  // namespace

void TetraLiftDock::configurePocket(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  const auto source = parseDetectionSource(declareAndGet<std::string>(node, name + ".detection_source", "tag"));
  scan_topic_ = declareAndGet<std::string>(node, name + ".pocket_scan_topic", "");
  pocket_frame_ = declareAndGet<std::string>(node, name + ".pocket_frame", "odom");
  pocket_params_.inner_width = declareAndGet<double>(node, name + ".pocket_inner_width", 1.0);
  pocket_params_.leg_length = declareAndGet<double>(node, name + ".pocket_leg_length", 1.0);
  pocket_params_.band_half_width = declareAndGet<double>(node, name + ".pocket_band_half_width", 0.10);
  pocket_params_.min_leg_points = declareAndGet<int>(node, name + ".pocket_min_leg_points", 8);
  pocket_params_.min_leg_span = declareAndGet<double>(node, name + ".pocket_min_leg_span", 0.25);
  pocket_params_.max_width_error = declareAndGet<double>(node, name + ".pocket_max_width_error", 0.06);
  pocket_params_.max_yaw_change = declareAndGet<double>(node, name + ".pocket_max_yaw_change", 0.35);
  insert_depth_ = declareAndGet<double>(node, name + ".pocket_insert_depth", 0.5);
  footprint_.front = declareAndGet<double>(node, name + ".footprint_front", 0.434);
  footprint_.back = declareAndGet<double>(node, name + ".footprint_back", -0.504);
  footprint_.half_width = declareAndGet<double>(node, name + ".footprint_half_width", 0.434);
  min_side_clearance_ = declareAndGet<double>(node, name + ".pocket_min_side_clearance", 0.03);
  min_front_clearance_ = declareAndGet<double>(node, name + ".pocket_min_front_clearance", 0.05);
  entry_margin_ = declareAndGet<double>(node, name + ".pocket_entry_margin", 0.03);
  entry_lookahead_ = declareAndGet<double>(node, name + ".pocket_entry_lookahead", 0.03);
  pocket_lost_timeout_ = declareAndGet<double>(node, name + ".pocket_lost_timeout", 2.0);
  pocket_filter_coef_ = declareAndGet<double>(node, name + ".pocket_filter_coef", 0.3);
  clearance_side_reach_ = declareAndGet<double>(node, name + ".pocket_clearance_side_reach", 0.25);

  pocket_mode_ = source == DetectionSource::kLidar && !scan_topic_.empty();
  if (source == DetectionSource::kLidar && scan_topic_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "%s: detection_source is lidar but pocket_scan_topic is empty, using tags", name.c_str());
  }
  if (!pocket_mode_) {
    return;
  }
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { scanCallback(msg); });
  // 도킹 서버와 같은 노드라 cmd_vel 리매핑(cmd_vel_nav2)을 따른다 (contact_stop.cpp 와 같음)
  stop_pub_ = rclcpp::create_publisher<geometry_msgs::msg::Twist>(node->get_node_topics_interface(), "cmd_vel", rclcpp::QoS(1));
  RCLCPP_INFO(
    node->get_logger(), "%s: lidar pocket docking on %s, inner width %.3f m, leg %.3f m, insert depth %.3f m",
    name.c_str(), scan_topic_.c_str(), pocket_params_.inner_width, pocket_params_.leg_length, insert_depth_);
}

void TetraLiftDock::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
}

bool TetraLiftDock::lookupPose2d(const std::string & target, const std::string & source, PocketPose & out)
{
  try {
    const auto tf = tf2_buffer_->lookupTransform(target, source, tf2::TimePointZero);
    out = {tf.transform.translation.x, tf.transform.translation.y, tf2::getYaw(tf.transform.rotation)};
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Pocket TF %s <- %s: %s", target.c_str(), source.c_str(), ex.what());
    return false;
  }
}

bool TetraLiftDock::refineFromPocket(geometry_msgs::msg::PoseStamped & pose)
{
  const rclcpp::Time now = node_->now();
  if (last_pocket_refine_.nanoseconds() == 0 || (now - last_pocket_refine_).seconds() > kNewSessionGap) {
    pocket_valid_ = false;
  }
  last_pocket_refine_ = now;

  PocketPose robot_in_fixed;
  if (!lookupPose2d(pocket_frame_, base_frame_id_, robot_in_fixed)) {
    return false;
  }
  PocketPose prior = pocket_in_fixed_;
  if (!pocket_valid_) {
    // 처음에는 지도에 등록한 도크 자세(로봇이 짐을 뜨는 자리)에서 insert_depth 만큼 앞을 입구로 본다
    geometry_msgs::msg::PoseStamped registered = pose;
    registered.header.stamp = rclcpp::Time(0);
    try {
      registered = tf2_buffer_->transform(registered, pocket_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Pocket: registered dock pose to %s failed: %s", pocket_frame_.c_str(), ex.what());
      return false;
    }
    const PocketPose dock{registered.pose.position.x, registered.pose.position.y, tf2::getYaw(registered.pose.orientation)};
    prior = composePose(dock, PocketPose{-insert_depth_, 0.0, 0.0});
  }
  updatePocketFromScan(robot_in_fixed, prior);
  if (!pocket_valid_) {
    return false;
  }

  const PocketPose robot_in_pocket = composePose(invertPose(pocket_in_fixed_), robot_in_fixed);
  const bool entered = frontOfRobotInPocket(robot_in_pocket, footprint_) > 0.0;
  if (!entered && (now - pocket_seen_).seconds() > pocket_lost_timeout_) {
    RCLCPP_WARN(node_->get_logger(), "Pocket legs lost for %.1f s before entering", (now - pocket_seen_).seconds());
    return false;
  }
  if (!entryPoseClear(robot_in_pocket, footprint_, pocket_params_.inner_width, entry_margin_, entry_lookahead_)) {
    stop_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_ERROR(
      node_->get_logger(), "Pocket entry blocked: robot is %.3f m off the pocket axis, %.1f deg turned",
      robot_in_pocket.y, angles::to_degrees(robot_in_pocket.yaw));
    throw opennav_docking_core::FailedToControl("Robot is not aligned with the lift port; stopping before the legs");
  }

  const PocketPose target = composePose(pocket_in_fixed_, PocketPose{insert_depth_, 0.0, 0.0});
  geometry_msgs::msg::PoseStamped target_msg;
  target_msg.header.frame_id = pocket_frame_;
  target_msg.header.stamp = rclcpp::Time(0);
  target_msg.pose.position.x = target.x;
  target_msg.pose.position.y = target.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, target.yaw);
  target_msg.pose.orientation = tf2::toMsg(q);
  const std::string out_frame = pose.header.frame_id.empty() ? pocket_frame_ : pose.header.frame_id;
  try {
    pose = tf2_buffer_->transform(target_msg, out_frame, tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Pocket: dock pose to %s failed: %s", out_frame.c_str(), ex.what());
    return false;
  }
  pose.header.stamp = now;
  dock_pose_ = pose;
  return true;
}

bool TetraLiftDock::updatePocketFromScan(const PocketPose & robot_in_fixed, const PocketPose & prior_in_fixed)
{
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }
  if (!scan || rclcpp::Time(scan->header.stamp) == used_scan_stamp_) {
    return false;
  }
  used_scan_stamp_ = rclcpp::Time(scan->header.stamp);
  PocketPose scan_in_base;
  if (!lookupPose2d(base_frame_id_, scan->header.frame_id, scan_in_base)) {
    return false;
  }
  std::vector<Point2> points;
  points.reserve(scan->ranges.size());
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double r = scan->ranges[i];
    if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) {
      continue;
    }
    const double a = scan->angle_min + i * scan->angle_increment;
    const auto p = composePose(scan_in_base, PocketPose{r * std::cos(a), r * std::sin(a), 0.0});
    points.push_back({p.x, p.y});
  }

  const PocketPose base_in_fixed_inv = invertPose(robot_in_fixed);
  const auto detection = detectPocket(points, composePose(base_in_fixed_inv, prior_in_fixed), pocket_params_);
  if (detection.found) {
    const PocketPose measured = composePose(robot_in_fixed, detection.entrance);
    if (!pocket_valid_) {
      pocket_in_fixed_ = measured;
    } else {
      const double k = pocket_filter_coef_;
      pocket_in_fixed_.x += k * (measured.x - pocket_in_fixed_.x);
      pocket_in_fixed_.y += k * (measured.y - pocket_in_fixed_.y);
      pocket_in_fixed_.yaw = angles::normalize_angle(
        pocket_in_fixed_.yaw + k * angles::normalize_angle(measured.yaw - pocket_in_fixed_.yaw));
    }
    pocket_valid_ = true;
    pocket_seen_ = node_->now();
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "Pocket not found: %s (left %d, right %d points)",
      detection.reason.c_str(), detection.left_points, detection.right_points);
  }
  if (pocket_valid_) {
    checkPocketClearance(points, composePose(invertPose(pocket_in_fixed_), robot_in_fixed));
  }
  return detection.found;
}

void TetraLiftDock::checkPocketClearance(const std::vector<Point2> & points_in_base, const PocketPose & robot_in_pocket)
{
  // 입구에서 멀리 있을 때는 다리가 아닌 주변 물체(벽·사람)를 보지 않는다. 그쪽은 도킹 서버의 costmap 충돌 검사가 맡는다
  if (frontOfRobotInPocket(robot_in_pocket, footprint_) < -kClearanceCheckDistance) {
    return;
  }
  const auto clearance = footprintClearance(points_in_base, footprint_, clearance_side_reach_, min_front_clearance_ + 0.10);
  if (clearance.left >= min_side_clearance_ && clearance.right >= min_side_clearance_ && clearance.front >= min_front_clearance_) {
    return;
  }
  stop_pub_->publish(geometry_msgs::msg::Twist());
  RCLCPP_ERROR(
    node_->get_logger(), "Pocket clearance too small: left %.3f, right %.3f, front %.3f m",
    clearance.left, clearance.right, clearance.front);
  throw opennav_docking_core::FailedToControl("Lift port leg or obstacle too close to the robot; stopped");
}

bool TetraLiftDock::isDockedInPocket()
{
  // 지난 도킹의 포트를 기억한 채라면 쓰지 않는다. 다른 곳에서 그 포트 축 앞쪽에 서 있기만 해도 도착으로 볼 수 있다
  if (!pocket_valid_ || (node_->now() - last_pocket_refine_).seconds() > kNewSessionGap) {
    return false;
  }
  PocketPose robot_in_fixed;
  if (!lookupPose2d(pocket_frame_, base_frame_id_, robot_in_fixed)) {
    return false;
  }
  const PocketPose robot_in_pocket = composePose(invertPose(pocket_in_fixed_), robot_in_fixed);
  // 깊이만 본다. 좌우·각도는 진입 검사가 이미 다리 안쪽으로 묶어 둔다
  const bool docked = robot_in_pocket.x >= insert_depth_ - docking_threshold_;
  if (docked && (node_->now() - last_pocket_refine_).seconds() <= 1.0) {
    stop_pub_->publish(geometry_msgs::msg::Twist());
  }
  return docked;
}

}  // namespace tetra_docking

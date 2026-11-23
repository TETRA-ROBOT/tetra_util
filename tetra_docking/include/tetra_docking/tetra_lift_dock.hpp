#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking/simple_non_charging_dock.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "interfaces/srv/conveyor_auto_movement.hpp"
#include "tetra_docking/pocket_detection.hpp"

#include <mutex>

namespace tetra_docking
{

class TetraLiftDock : public opennav_docking::SimpleNonChargingDock
{
public:
  TetraLiftDock();
  virtual ~TetraLiftDock() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;

  bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id) override;
  bool isDocked() override;

protected:
  void conveyorSensorCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  bool executeLiftAction();

  // 라이다 포트(ㄷ 자) 모드 (src/lift_pocket.cpp). detection_source: lidar 면 태그 대신 스캔으로 포트 다리를 찾아 들어간다
  void configurePocket(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool refineFromPocket(geometry_msgs::msg::PoseStamped & pose);
  bool updatePocketFromScan(const PocketPose & robot_in_fixed, const PocketPose & prior_in_fixed);
  void checkPocketClearance(const std::vector<Point2> & points_in_base, const PocketPose & robot_in_pocket);
  bool lookupPose2d(const std::string & target, const std::string & source, PocketPose & out);
  bool isDockedInPocket();

  bool pocket_mode_{false};
  std::string scan_topic_;
  std::string pocket_frame_;
  PocketParams pocket_params_;
  RectFootprint footprint_;
  double insert_depth_{0.5};
  double min_side_clearance_{0.03};
  double min_front_clearance_{0.05};
  double entry_margin_{0.03};
  double entry_lookahead_{0.03};
  double pocket_lost_timeout_{2.0};
  double pocket_filter_coef_{0.3};
  double clearance_side_reach_{0.25};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
  std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time used_scan_stamp_{0, 0, RCL_ROS_TIME};
  // 포트 입구 자세 (pocket_frame_, 보통 odom). 다리가 잠깐 가려져도 이 값으로 들어간다
  bool pocket_valid_{false};
  PocketPose pocket_in_fixed_;
  rclcpp::Time pocket_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pocket_refine_{0, 0, RCL_ROS_TIME};

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;

  // 리프트는 전원 보드 CN 명령(conveyor_auto_move_cmd)으로 움직인다.
  // 예전에는 turn_on_cmd(LT)를 불렀는데, LT 는 LED 채널 명령이라 리프트는 움직이지 않고 LED 색만 바뀌었다
  rclcpp::Client<interfaces::srv::ConveyorAutoMovement>::SharedPtr lift_client_;

  // Parameters
  std::string marker_family_;
  std::string camera_frame_;
  int lift_action_mode_; // 보드 CN 코드. 1 = 싣기(올림), 0 = 내리기(내림). H 는 2 = 최대 높이
  bool lift_executed_;
  int current_conveyor_sensor_;
  int latest_detected_tag_id_;
  geometry_msgs::msg::PoseStamped latest_tag_pose_;
  bool tag_detected_;
};

}  // namespace tetra_docking

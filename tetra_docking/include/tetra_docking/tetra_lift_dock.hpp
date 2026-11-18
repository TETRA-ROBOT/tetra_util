#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking/simple_non_charging_dock.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "interfaces/srv/conveyor_auto_movement.hpp"

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

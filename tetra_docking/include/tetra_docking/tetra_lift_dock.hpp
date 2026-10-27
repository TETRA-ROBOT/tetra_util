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
#include "interfaces/srv/toggle_on.hpp"

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

  // Service Clients
  rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client_;

  // Parameters
  std::string marker_family_;
  std::string camera_frame_;
  int lift_action_mode_; // 1 for LOAD (UP), 0 for UNLOAD (DOWN)
  bool lift_executed_;
  int current_conveyor_sensor_;
  int latest_detected_tag_id_;
  geometry_msgs::msg::PoseStamped latest_tag_pose_;
  bool tag_detected_;
};

}  // namespace tetra_docking

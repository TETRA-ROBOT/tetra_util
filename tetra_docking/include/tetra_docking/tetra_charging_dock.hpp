#pragma once

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking/simple_charging_dock.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "robot_localization/srv/set_pose.hpp"
#include "interfaces/srv/toggle_on.hpp"

namespace tetra_docking
{

class TetraChargingDock : public opennav_docking::SimpleChargingDock
{
public:
  TetraChargingDock();
  virtual ~TetraChargingDock() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;

  bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id) override;
  bool isDocked() override;
  bool isCharging() override;
  bool disableCharging() override;
  bool hasStoppedCharging() override;

protected:
  void dockingStatusCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void apriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
  void resetEkfPose(double x, double y, double yaw);

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr docking_status_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;

  // Service Clients
  rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
  rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client_;

  // Parameters
  std::string marker_family_;
  std::string camera_frame_;
  std::string docking_status_topic_;
  int contact_status_val_;
  bool reset_ekf_pose_;
  double home_x_;
  double home_y_;
  int latest_detected_tag_id_;
  geometry_msgs::msg::PoseStamped latest_tag_pose_;
  bool tag_detected_;
  int current_docking_status_;
  bool pose_reset_done_;
};

}  // namespace tetra_docking

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
  std::string tagFrameFor(const std::string & id) const;
  void updateDetectionFromTagTf(const std::string & id);
  bool getBlindApproachPose(geometry_msgs::msg::PoseStamped & pose);
  bool isTagCloseAhead();

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr docking_status_sub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;

  // Service Clients
  rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client_;

  // Parameters
  std::string marker_family_;
  std::string camera_frame_;
  std::string docking_status_topic_;
  int contact_status_val_;
  int latest_detected_tag_id_;
  geometry_msgs::msg::PoseStamped latest_tag_pose_;
  bool tag_detected_;
  int current_docking_status_;
  bool tag_tf_as_detection_;
  bool charge_requires_contact_;
  double blind_approach_distance_;
  double blind_approach_timeout_;
  double undock_tag_distance_;
  double undock_tag_lateral_;
  geometry_msgs::msg::PoseStamped last_detected_dock_pose_;
  rclcpp::Time last_detection_time_;
};

}  // namespace tetra_docking

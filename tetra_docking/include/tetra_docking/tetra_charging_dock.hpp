#pragma once

#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#include "opennav_docking/simple_charging_dock.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "tetra_docking/tag_view.hpp"

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
  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) override;
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

  // 태그 정렬 staging (tag_staging.cpp)
  struct TagDockEstimate
  {
    DockSample dock;
    double tag_height{0.0};
    size_t samples{0};
  };
  void configureTagStaging(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  bool isUndockRequest(const geometry_msgs::msg::Pose & pose, const std::string & frame);
  bool estimateDockFromTag(const std::string & frame, TagDockEstimate & estimate);
  bool collectTagSample(
    const std::string & frame, const std::string & tag_frame, rclcpp::Time & last_stamp,
    std::vector<DockSample> & samples, double & height_sum);
  bool isTagUsableForStaging(const geometry_msgs::msg::Transform & camera_to_tag) const;
  DockSample dockSampleFromTag(const geometry_msgs::msg::Transform & fixed_to_tag) const;
  double stagingDistanceFor(const TagDockEstimate & estimate);
  bool robotOnApproachLine(
    const DockSample & dock, const std::string & frame, double staging_distance,
    geometry_msgs::msg::PoseStamped & robot_pose);

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

  // 태그 정렬 staging 파라미터
  bool tag_staging_;
  double tag_staging_distance_;
  double tag_staging_distance_min_;
  double tag_staging_distance_max_;
  double tag_staging_sample_time_;
  int tag_staging_min_samples_;
  double tag_staging_max_age_;
  double tag_staging_max_tag_distance_;
  double tag_staging_max_oblique_;
  double tag_staging_skip_lateral_;
  double tag_staging_skip_yaw_;
  double tag_size_;
  double tag_min_pixels_;
  double tag_image_margin_;
  std::string camera_info_topic_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::mutex intrinsics_mutex_;
  CameraIntrinsics intrinsics_;
};

}  // namespace tetra_docking

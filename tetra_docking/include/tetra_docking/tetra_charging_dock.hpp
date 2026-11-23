#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <atomic>
#include <cstdint>
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
#include "tetra_docking/tag_search_plan.hpp"

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
  // 접근 중 도크 자세 (refined_pose.cpp)
  bool refineFromTagDetection(geometry_msgs::msg::PoseStamped & pose, const std::string & id);
  bool approachDetectedDock(geometry_msgs::msg::PoseStamped & pose, const std::string & id);
  bool getBlindApproachPose(geometry_msgs::msg::PoseStamped & pose);
  bool isTagCloseAhead();

  // 라이다 도킹 마커(스테이션의 V 자 홈) 모드 (src/lidar_detection.cpp, detection_source: lidar)
  void configureLidarDetection(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  rcl_interfaces::msg::SetParametersResult onDetectionSourceParameter(const std::vector<rclcpp::Parameter> & params);
  void lidarDetectionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool refineFromLidarDetection(geometry_msgs::msg::PoseStamped & pose, const std::string & id);
  bool isLidarMarkerCloseAhead();
  std::atomic<bool> lidar_mode_{false};
  std::string lidar_param_name_;
  std::string lidar_detection_topic_;
  std::string lidar_marker_frame_{"odom"};
  double undock_lidar_distance_{0.0};
  double lidar_marker_memory_{600.0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_detection_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr lidar_param_handle_;
  std::mutex lidar_mutex_;
  geometry_msgs::msg::PoseStamped latest_lidar_detection_;
  geometry_msgs::msg::PoseStamped lidar_marker_in_fixed_;
  rclcpp::Time lidar_marker_time_{0, 0, RCL_ROS_TIME};

  // 태그 정렬 staging (tag_staging.cpp), 태그 검출로 도크 자세 추정 (tag_dock_estimate.cpp)
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
  bool isRetryingApproach();
  // 마지막 getRefinedPose 시각. 도킹 서버는 접근이 실패하면 곧바로 getStagingPose 로 staging 에 되돌아가므로
  // 이게 방금이면 그 호출은 재시도다
  rclcpp::Time last_refine_time_{0, 0, RCL_ROS_TIME};
  // 같은 시각을 접점 콜백 스레드에서도 읽으므로 원자 값으로 따로 둔다 (contact_stop.cpp)
  std::atomic<int64_t> last_refine_ns_{0};

  // 접점 정지 (contact_stop.cpp)
  void configureContactStop(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  bool isApproachCoasting() const;
  void stopOnContact();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr contact_stop_pub_;
  std::atomic<bool> contact_stop_logged_{false};

  // 범퍼 정지 (bumper_guard.cpp)
  void configureBumperGuard(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  void bumperCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void stopIfBumperPressed();
  void stopUndockIfBumperPressed();

  // 도킹 영점 (dock_offset.cpp)
  void configureDockOffset(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  rcl_interfaces::msg::SetParametersResult onDockOffsetParameters(const std::vector<rclcpp::Parameter> & params);
  DockSample applyDockOffset(const DockSample & dock) const;
  void applyDockOffset(geometry_msgs::msg::PoseStamped & pose) const;
  bool isDockDepthReached();

  // 태그 탐색 (tag_search.cpp)
  void configureTagSearch(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  void resetTagSearch();
  bool searchForTag();
  bool confirmTagSeen(const std::string & frame);
  bool centerOnTag();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr search_cmd_pub_;
  double tag_search_angle_{0.0};
  double tag_search_speed_{0.05};
  TagSearchState tag_search_;

  // 도크 yaw 고정 (staging_yaw_lock.cpp)
  void configureYawLock(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  void rememberStagingYaw(const DockSample & dock, const std::string & frame);
  void lockDockYaw(geometry_msgs::msg::PoseStamped & pose);
  bool lock_dock_yaw_to_staging_{false};
  double locked_yaw_timeout_{150.0};
  bool locked_yaw_valid_{false};
  double locked_yaw_{0.0};
  std::string locked_yaw_frame_;
  rclcpp::Time locked_yaw_time_{0, 0, RCL_ROS_TIME};

  // 마지막 구간 heading 우선 (approach_shaping.cpp)
  void configureApproachShaping(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name);
  void shapeFinalApproach(geometry_msgs::msg::PoseStamped & pose);

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
  // 구독 콜백과 도킹 액션 스레드가 같이 읽고 쓴다
  std::atomic<int> current_docking_status_;
  bool tag_tf_as_detection_;
  bool charge_requires_contact_;
  double blind_approach_distance_;
  double blind_approach_timeout_;
  // 접근 중 태그를 놓치면 이 거리 안에서는 마지막 검출 자세 쪽으로 몰아 다시 찾는다 (0 이면 끔)
  double lost_tag_reacquire_distance_{0.0};
  // 놓친 뒤 이 시간 안에 다시 보이지 않으면 검출 실패로 멈춘다 (s)
  double lost_tag_reacquire_time_{3.0};
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
  std::string tag_staging_frame_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::mutex intrinsics_mutex_;
  CameraIntrinsics intrinsics_;

  // 범퍼 정지 파라미터
  std::string bumper_topic_;
  int bumper_mask_{1};
  double bumper_grace_{0.0};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bumper_sub_;
  // 눌리기 시작한 ROS 시각(ns), 안 눌렸으면 0
  std::atomic<int64_t> bumper_pressed_since_ns_{0};
  std::atomic<int64_t> bumper_last_msg_ns_{0};
  // 언도킹(후진) 중 이 비트가 눌리면 멈춘다. 0 이면 보지 않는다 (MH 는 뒤 범퍼 0x08)
  int undock_bumper_mask_{0};
  std::atomic<int> bumper_last_data_{0};

  // 도킹 영점 파라미터 (관제가 도킹 직전에 바꾼다)
  std::string offset_param_prefix_;
  std::atomic<double> dock_offset_x_{0.0};
  std::atomic<double> dock_offset_y_{0.0};
  std::atomic<double> dock_offset_yaw_{0.0};
  // 도착 판정을 도크 축 깊이만으로 한다 (좌우 잔여 오차가 docking_threshold 보다 커도 멈추게)
  bool docked_by_depth_{false};
  // charge_requires_contact 일 때 깊이 도착 판정을 도크 자세보다 이만큼 깊게 둔다 (m, 0 이면 끔)
  double contact_overtravel_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr offset_param_handle_;

  // 마지막 구간 heading 우선 파라미터
  double heading_priority_near_{0.0};
  double heading_priority_far_{0.0};
  // 라이다 모드 전용 값. 음수면 위 태그 값을 그대로 쓴다
  double lidar_heading_priority_near_{-1.0};
  double lidar_heading_priority_far_{-1.0};
};

}  // namespace tetra_docking

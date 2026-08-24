#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// ROS 2 Messages
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"

// TF2
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Nav2 Action and Service
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

// Custom Services
#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "interfaces/srv/docking_control.hpp"
#include "interfaces/srv/docking_stop.hpp"
#include "interfaces/srv/imu_reset.hpp"
#include "interfaces/srv/save_map.hpp"
#include "interfaces/srv/delete_map.hpp"
#include "interfaces/srv/run_mapping.hpp"
#include "interfaces/srv/run3_d_mapping.hpp"
#include "interfaces/srv/run_navigation.hpp"
#include "interfaces/srv/get_information.hpp"
#include "interfaces/srv/set_location.hpp"
#include "interfaces/srv/goto_location.hpp"
#include "interfaces/srv/goto_location2.hpp"
#include "interfaces/srv/goto_cancel.hpp"
#include "interfaces/srv/set_maxspeed.hpp"

// AprilTag
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

// robot_localization
#include "robot_localization/srv/set_pose.hpp"

// Local types
#include "tetra_service/tetra_types.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class TETRA_SERVICE : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TETRA_SERVICE();
    virtual ~TETRA_SERVICE() = default;

private:
    // ── Parameter & Node Setup ──
    void initializeParameters();
    void initializeRosEntities(
        const std::string& cmd_vel_topic,
        const std::string& scan_topic,
        const std::string& scan2_topic,
        const std::string& scan_merge_topic,
        bool use_rtabmap_initialpose);

    // ── Subscribers Callbacks ──
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void ApriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void SickCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void CygbotCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void LaserScanMergeCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void BumperCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void EMGCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void BatteryCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void DockingStatusCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void ConveyorSensorCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void ConveyorMovementCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void AMCL_PaticleCloud_Callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg);
    void cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void Initialpose_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void map_Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // ── Timer Callbacks ──
    void DockingThread_function();
    void TF_CALC_Timer();
    void Footprint_CALC_Timer();
    void PoseResetMonitor_Timer();

    // ── Docking & Lift Station Tracking Methods ──
    bool charging_station_tracking(int marker_id);
    bool charging_station_yaw_tracking();
    bool charging_station_tracking2(int marker_id);
    bool approach_station_move();
    bool depart_station_move();

    bool lift_station_tracking(int marker_id);
    bool lift_station_yaw_tracking();
    bool lift_station_tracking2(int marker_id);
    bool un_lift_station_tracking(int marker_id);
    bool un_lift_station_yaw_tracking();
    bool un_lift_station_tracking2(int marker_id);
    bool lift_check();
    bool un_lift_check();
    bool depart_lift_move(int mode);

    void docking_exit();
    void publish_docking_progress(int progress);
    double rotation_movement();

    // ── Service Servers Callbacks ──
    void Docking_Command(
        const std::shared_ptr<interfaces::srv::DockingControl::Request> request,
        std::shared_ptr<interfaces::srv::DockingControl::Response> response);
    void Docking_Stop_Command(
        const std::shared_ptr<interfaces::srv::DockingStop::Request> request,
        std::shared_ptr<interfaces::srv::DockingStop::Response> response);
    void Save_Map_Command(
        const std::shared_ptr<interfaces::srv::SaveMap::Request> request,
        std::shared_ptr<interfaces::srv::SaveMap::Response> response);
    void Delete_Map_Command(
        const std::shared_ptr<interfaces::srv::DeleteMap::Request> request,
        std::shared_ptr<interfaces::srv::DeleteMap::Response> response);
    void Mapping_Mode_Command(
        const std::shared_ptr<interfaces::srv::RunMapping::Request> request,
        std::shared_ptr<interfaces::srv::RunMapping::Response> response);
    void Mapping_3D_Mode_Command(
        const std::shared_ptr<interfaces::srv::Run3DMapping::Request> request,
        std::shared_ptr<interfaces::srv::Run3DMapping::Response> response);
    void Navigation_Mode_Command(
        const std::shared_ptr<interfaces::srv::RunNavigation::Request> request,
        std::shared_ptr<interfaces::srv::RunNavigation::Response> response);
    void Get_Information_Command(
        const std::shared_ptr<interfaces::srv::GetInformation::Request> request,
        std::shared_ptr<interfaces::srv::GetInformation::Response> response);
    void Set_Location_Command(
        const std::shared_ptr<interfaces::srv::SetLocation::Request> request,
        std::shared_ptr<interfaces::srv::SetLocation::Response> response);
    void Goto_Location_Command(
        const std::shared_ptr<interfaces::srv::GotoLocation::Request> request,
        std::shared_ptr<interfaces::srv::GotoLocation::Response> response);
    void Goto_Location2_Command(
        const std::shared_ptr<interfaces::srv::GotoLocation2::Request> request,
        std::shared_ptr<interfaces::srv::GotoLocation2::Response> response);
    void Goto_Cancel_Command(
        const std::shared_ptr<interfaces::srv::GotoCancel::Request> request,
        std::shared_ptr<interfaces::srv::GotoCancel::Response> response);
    void Set_Maxspeed_Command(
        const std::shared_ptr<interfaces::srv::SetMaxspeed::Request> request,
        std::shared_ptr<interfaces::srv::SetMaxspeed::Response> response);

    // ── Service & Action Clients Helper Methods ──
    void LedControl_Call(int id, int led_brightness);
    void LedToggleControl_Call(int de_index, int light_accel, int led_high_brightness, int light_decel, int led_low_brightness);
    void ToggleOn_Call(int id);
    void ImuReset_Call();
    bool Clear_Costmap();
    void set_robot_localization_pose(double x, double y, double yaw);

    // ── Client Response Callbacks ──
    void LedControl_callback(rclcpp::Client<interfaces::srv::LedControl>::SharedFuture future);
    void LedToggleControl_callback(rclcpp::Client<interfaces::srv::LedToggleControl>::SharedFuture future);
    void ToggleOn_callback(rclcpp::Client<interfaces::srv::ToggleOn>::SharedFuture future);
    void ImuReset_callback(rclcpp::Client<interfaces::srv::ImuReset>::SharedFuture future);
    void request_nomotion_update_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future);
    void clear_entire_global_costmap_callback(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future);
    void clear_entire_local_costmap_callback(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future);

    // ── Nav2 Action Callbacks ──
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigateToPose::WrappedResult& result);

    // ── Process Control Helpers ──
    void run_process_async(const std::string& command);

private:
    // ── TF & Timers ──
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr TF_timer_;
    rclcpp::TimerBase::SharedPtr Footprint_timer_;
    rclcpp::TimerBase::SharedPtr pose_reset_monitor_timer_;

    // ── Publishers ──
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pose_reset_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr movebase_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr footprint_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr docking_progress_publisher;

    // ── Subscribers ──
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sick_scan_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr cygbot_scan_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_merge_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bumper_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emg_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tetra_battery_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr docking_status_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_sensor_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr conveyor_movement_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particlecloud_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;

    // ── Service Servers ──
    rclcpp::Service<interfaces::srv::DockingControl>::SharedPtr docking_cmd_srv;
    rclcpp::Service<interfaces::srv::DockingStop>::SharedPtr docking_stop_cmd_srv;
    rclcpp::Service<interfaces::srv::SaveMap>::SharedPtr save_map_cmd_srv;
    rclcpp::Service<interfaces::srv::DeleteMap>::SharedPtr delete_map_cmd_srv;
    rclcpp::Service<interfaces::srv::RunMapping>::SharedPtr run_mapping_cmd_srv;
    rclcpp::Service<interfaces::srv::Run3DMapping>::SharedPtr run_3d_mapping_cmd_srv;
    rclcpp::Service<interfaces::srv::RunNavigation>::SharedPtr run_navigation_cmd_srv;
    rclcpp::Service<interfaces::srv::GetInformation>::SharedPtr get_information_cmd_srv;
    rclcpp::Service<interfaces::srv::SetLocation>::SharedPtr set_location_cmd_srv;
    rclcpp::Service<interfaces::srv::GotoLocation>::SharedPtr goto_location_cmd_srv;
    rclcpp::Service<interfaces::srv::GotoLocation2>::SharedPtr goto_location2_cmd_srv;
    rclcpp::Service<interfaces::srv::GotoCancel>::SharedPtr goto_cancel_cmd_srv;
    rclcpp::Service<interfaces::srv::SetMaxspeed>::SharedPtr set_maxspeed_cmd_srv;

    // ── Service & Action Clients ──
    rclcpp::Client<interfaces::srv::LedControl>::SharedPtr led_control_client;
    rclcpp::Client<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_client;
    rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client;
    rclcpp::Client<interfaces::srv::ImuReset>::SharedPtr imu_reset_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr request_nomotion_update_client;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_global_costmap_client;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_local_costmap_client;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> set_speed_parameter_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_action_client;

    // ── State Data Structs ──
    ODOMETRY _pOdometry;
    HOME_POSE _pHomePose;
    GOAL_POSE _pGoal_pose;
    AR_TAG_POSE _pAR_tag_pose;
    LAND_MARK_POSE _pLandMarkPose;
    TF_POSE _pTF_pose;
    ROBOT _pRobot;
    FALG_VALUE _pFlag_Value;
    DYNAMIC_PARAM _pDynamic_param;
    DOCKING_POSE _pDocking_pose;

    // ── State Variables ──
    int m_dHome_ID = 0;
    bool m_bActive_map = false;
    int m_iDocking_CommandMode = 0;
    int m_iNoMarker_cnt = 0;
    int m_iRotation_cnt = 0;
    int m_iRotation_Mode = 0;
    int m_iTrun_cnt = 0;
    int m_iBack_cnt = 0;
    int m_iPose_reset_cnt = 0;
    int m_iYawAlign_cnt = 0;
    int m_iStepBack_cnt = 0;
    int m_iApproach_cnt = 0;
    int m_iDepart_cnt = 0;
    int ex_ilaunchMode = 0;
    int ex_iRetry_count = 0;
    int ex_iRetry_Max_count = 9999;
    double m_dTF_calc_poseX = 0.0;
    double m_dTF_calc_poseY = 0.0;
    double m_dTF_calc_theta = 0.0;
};

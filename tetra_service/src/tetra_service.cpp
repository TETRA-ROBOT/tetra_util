#include "rclcpp/rclcpp.hpp"
// nav2 msg
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
// std
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
// geometry msg
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/point.hpp"
// sensor msg
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
// tf2
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// Custom Service
#include "interfaces/srv/led_control.hpp"
#include "interfaces/srv/led_toggle_control.hpp"
#include "interfaces/srv/toggle_on.hpp"
#include "interfaces/srv/docking_control.hpp"      // docking
#include "interfaces/srv/docking_stop.hpp"         // docking_stop
#include "interfaces/srv/imu_reset.hpp"            // imu reset
#include "interfaces/srv/save_map.hpp"             // Map save service
#include "interfaces/srv/delete_map.hpp"           // Map delete service
#include "interfaces/srv/run_mapping.hpp"          // cartographer / slam run service
#include "interfaces/srv/run3_d_mapping.hpp"       // rtabmap run service
#include "interfaces/srv/run_navigation.hpp"       // NAV2 run service
#include "interfaces/srv/get_information.hpp"      // Get Information service
#include "interfaces/srv/set_location.hpp"         // Save Location to string Location name
#include "interfaces/srv/goto_location.hpp"        // Goto string Location name
#include "interfaces/srv/goto_location2.hpp"       // Goto location Pose
#include "interfaces/srv/goto_cancel.hpp"          // Goto Command Cancel
#include "interfaces/srv/set_maxspeed.hpp"         // Set Maxspeed

// Action
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// apriltag_ros
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
// robot_localization Client service
#include "robot_localization/srv/set_pose.hpp"

#include <chrono>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <atomic>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;
#define HIGH_BATTERY 95
#define LOW_BATTERY 15
#define MAX_RETRY_CNT 10
#define BUF_LEN 4096

int m_dHome_ID = 0;

FILE *fp;
int status;
char Textbuffer[BUF_LEN];

bool m_bActive_map = false;
int  m_iDocking_CommandMode = 0;
int  m_iNoMarker_cnt = 0;
int  m_iRotation_cnt = 0;
int  m_iRotation_Mode = 0; // Docking Rotation Mode Select
int  m_iTrun_cnt = 0;
int  m_iBack_cnt = 0;
int  m_iPose_reset_cnt = 0;
// ros2 launch mode check
int  ex_ilaunchMode = 0;
int  ex_iRetry_count = 0;
int  ex_iRetry_Max_count = 9999;

// CALC TF distance data
double m_dTF_calc_poseX = 0.0;
double m_dTF_calc_poseY = 0.0;
double m_dTF_calc_theta = 0.0;

typedef struct ODOMETRY
{
  double dOdom_position_x = 0.0;
  double dOdom_position_y = 0.0;
  double dOdom_position_z = 0.0;
  double dOdom_quaternion_x = 0.0;
  double dOdom_quaternion_y = 0.0;
  double dOdom_quaternion_z = 0.0;
  double dOdom_quaternion_w = 1.0;

  double dOdom_euler_roll = 0.0;
  double dOdom_euler_pitch = 0.0;
  double dOdom_euler_yaw = 0.0;
  // calc rad to degree
  double dOdom_theta_deg = 0.0;
} ODOMETRY;
ODOMETRY _pOdometry;

typedef struct HOME_POSE
{
    string HOME_strLOCATION = "HOME";
    double HOME_dPOSITION_X = -0.7;
    double HOME_dPOSITION_Y = 0.0;
    double HOME_dPOSITION_Z = 0.0;
    double HOME_dQUATERNION_X = 0.0;
    double HOME_dQUATERNION_Y = 0.0;
    double HOME_dQUATERNION_Z = 0.0;
    double HOME_dQUATERNION_W = 1.0;
} HOME_POSE;
HOME_POSE _pHomePose;

// goal_pose
typedef struct GOAL_POSE
{
    double goal_positionX = 0.0;
    double goal_positionY = 0.0;
    double goal_positionZ = 0.0;
    double goal_quarterX = 0.0;
    double goal_quarterY = 0.0;
    double goal_quarterZ = 0.0;
    double goal_quarterW = 1.0;
} GOAL_POSE;
GOAL_POSE _pGoal_pose;

// April_TAG Pose
typedef struct AR_TAG_POSE
{
    string m_strFamily = "tag36h11:";
    int m_iSelect_April_tag_id = 0;
    int m_iApril_tag_id = -1;
    double m_dApril_tag_pose_x = 0.0;
    double m_dApril_tag_pose_y = 0.0;
    double m_dApril_tag_pose_z = 0.0;
    double m_dApril_tag_orientation_x = 0.0;
    double m_dApril_tag_orientation_y = 0.0;
    double m_dApril_tag_orientation_z = 0.0;
    double m_dApril_tag_orientation_w = 1.0;
    double m_dApril_tag_roll = 0.0;
    double m_dApril_tag_pitch = 0.0;
    double m_dApril_tag_yaw = 0.0;
    // Transform AR Tag Axis -> Robot Axis
    double m_transform_old_pose_x = 0.0;
    double m_transform_old_pose_y = 0.0;
    double m_transform_pose_x = 0.0;
    double m_transform_pose_y = 0.0;
    double m_dPositioning_Angle = 0.0;
    // Calc Odom to apriltag_TF
    double m_target_yaw = 0.0;
    double m_target_theta = 0.0;
} AR_TAG_POSE;
AR_TAG_POSE _pAR_tag_pose;

typedef struct LAND_MARK_POSE
{
    float init_position_x = 0.0;
    float init_position_y = 0.0;
    float init_position_z = 0.0;
    float init_orientation_z = 0.0;
    float init_orientation_w = 1.0;
} LAND_MARK_POSE;
LAND_MARK_POSE _pLandMarkPose;

typedef struct TF_POSE
{
    double poseTFx = 0.0;
    double poseTFy = 0.0;
    double poseTFz = 0.0;
    double poseTFqx = 0.0;
    double poseTFqy = 0.0;
    double poseTFqz = 0.0;
    double poseTFqw = 1.0;

    double pose_euler_roll = 0.0;
    double pose_euler_pitch = 0.0;
    double pose_euler_yaw = 0.0;

    double theta_deg = 0.0;
} TF_POSE;
TF_POSE _pTF_pose;

typedef struct ROBOT
{
    int  HOME_id = 0;
    int  m_iDocking_id = 0;
    bool m_bFalg_DockingExit = false;
    int  m_iCallback_Battery = 0;
    int  m_iCallback_ErrorCode = 0;
    int  m_iCallback_EMG = 0;
    int  m_iCallback_Bumper = 0;
    int  m_iCallback_Charging_status = 0;
    // Bumper Collision Behavior
    int  m_iBumperCollisionBehavor_cnt = 0;
    // move result
    int  m_iMovebase_Result = 0; // Success: 3 | Aborted: 4 | Cancel: 1 | Unknown: 0
    // Conveyor Info (Option)
    int  m_iConveyor_Sensor_info = 0;
    int  m_iConveyor_id = 0;
    int  m_iConveyor_movement = 0; // 0: normal, 1: Loading, 2: Unloading
} ROBOT;
ROBOT _pRobot;

typedef struct FALG_VALUE
{
    bool m_bflag_NextStep = false;
    bool m_bflag_ComebackHome = false;
    bool m_bfalg_DockingExit = false;
    bool m_bflag_Conveyor_docking = false;
    bool m_bFlag_Disable_bumper = false;
    int  m_Onetime_reset_cnt = 0;
    // Tracking obstacle Check
    bool m_bFlag_Obstacle_Right = false;
    bool m_bFlag_Obstacle_Center = false;
    bool m_bFlag_Obstacle_Left = false;
    bool m_bFlag_Obstacle_Left_b = false;
    bool m_bFlag_Obstacle_Right_b = false;
    // PCL obstacle Check
    bool m_bFlag_Obstacle_PCL1 = false;
    bool m_bFlag_Obstacle_PCL2 = false;
    // Cygbot Check
    bool m_bFlag_Obstacle_cygbot = false;
    // Merge scan check
    bool m_bFlag_MergeScan_Front = false;
    bool m_bFlag_MergeScan_Rear = false;
    bool m_bFlag_MergeScan_Left = false;
    bool m_bFlag_MergeScan_Right = false;
    // Error Flag
    bool m_bumperhit_flag = false;
    bool m_emgpush_flag = false;
    bool BUMPER_BT = false;
    // Dynamic reconfigure flag
    bool m_bTebMarker_reconfigure_flag = false;
    bool m_bflag_patrol = false;
    bool m_bflag_patrol2 = false;
    bool m_bflag_goto_cancel = false;
    bool m_bflagGo = false;
    bool m_bflagGo2 = false;
    bool m_bCorneringFlag = true;
    // no motion service call flag
    bool m_bFlag_nomotion = true;
    bool m_bflag_auto_patrol = false;
    bool m_bFlag_Initialpose = false;
    // Setgoal_pub flag
    bool m_bFlag_pub = false;
} FALG_VALUE;
FALG_VALUE _pFlag_Value;

// dynamic parameter
typedef struct DYNAMIC_PARAM
{
    double MAX_Linear_velocity = 0.8;
    double m_linear_vel = 0.0;
    double m_angular_vel = 0.0;
    bool m_bFlag_onetime = true;
} DYNAMIC_PARAM;
DYNAMIC_PARAM _pDynamic_param;

typedef struct DOCKING_POSE
{
    double camera_TF_distance = -0.3955;
    double target_yaw = 0.0;
    double target_distance = 0.0;
    double before_TF_Pose = 0.0;
    bool Onemore_time = false;
} DOCKING_POSE;
DOCKING_POSE _pDocking_pose;

std::atomic<bool> stop_requested(false);
void signal_handler(int signal) 
{
    stop_requested = true;
    printf("Signal received: %d. Preparing to shutdown...\n", signal);
}

class TETRA_SERVICE: public rclcpp::Node
{
public:
    TETRA_SERVICE() : Node("tetra_service")
    {
        // Initialize the TF2 buffer and listener
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // Parameters
        std::string robot_model = "default";
        this->declare_parameter<std::string>("robot_model", "default");
        this->get_parameter("robot_model", robot_model);

        // Determine default settings based on robot_model
        double default_home_x = -0.7;
        double default_home_y = 0.0;
        double default_max_vel = 0.8;
        std::string default_cmd_vel = "cmd_vel_nav";
        std::string default_scan = "scan";
        std::string default_scan2 = "scan2";
        std::string default_scan_merge = "";
        bool default_rtabmap_initialpose = false;

        if (robot_model == "m") {
            default_home_x = 0.7;
            default_max_vel = 1.0;
            default_cmd_vel = "cmd_vel_nav";
            default_scan = "scan";
        } else if (robot_model == "h") {
            default_home_x = -0.7;
            default_max_vel = 0.8;
            default_cmd_vel = "cmd_vel_nav";
            default_scan = "scan";
        } else if (robot_model == "edu") {
            default_home_x = -0.7;
            default_max_vel = 0.8;
            default_cmd_vel = "cmd_vel_manual";
            default_scan = "scan";
        } else if (robot_model == "mh") {
            default_home_x = -0.7;
            default_max_vel = 0.8;
            default_cmd_vel = "cmd_vel_manual";
            default_scan_merge = "scan_merge";
        } else if (robot_model == "s") {
            default_home_x = -0.7;
            default_max_vel = 1.0;
            default_cmd_vel = "cmd_vel_nav";
            default_scan = "scan";
        } else if (robot_model == "sm") {
            default_home_x = -0.7;
            default_max_vel = 0.8;
            default_cmd_vel = "cmd_vel_manual";
            default_scan_merge = "scan_merge";
        }

        this->declare_parameter<int>("m_dHome_ID", 0);
        this->declare_parameter<double>("home_position_x", default_home_x);
        this->declare_parameter<double>("home_position_y", default_home_y);
        this->declare_parameter<double>("max_linear_velocity", default_max_vel);
        this->declare_parameter<std::string>("cmd_vel_topic", default_cmd_vel);
        this->declare_parameter<std::string>("scan_topic", default_scan);
        this->declare_parameter<std::string>("scan2_topic", default_scan2);
        this->declare_parameter<std::string>("scan_merge_topic", default_scan_merge);
        this->declare_parameter<bool>("use_rtabmap_initialpose", default_rtabmap_initialpose);
        this->declare_parameter<double>("camera_tf_distance", -0.3955);

        // Fetch configured values
        m_dHome_ID = this->get_parameter("m_dHome_ID").as_int();
        _pHomePose.HOME_dPOSITION_X = this->get_parameter("home_position_x").as_double();
        _pHomePose.HOME_dPOSITION_Y = this->get_parameter("home_position_y").as_double();
        _pDynamic_param.MAX_Linear_velocity = this->get_parameter("max_linear_velocity").as_double();
        std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        std::string scan_topic = this->get_parameter("scan_topic").as_string();
        std::string scan2_topic = this->get_parameter("scan2_topic").as_string();
        std::string scan_merge_topic = this->get_parameter("scan_merge_topic").as_string();
        bool use_rtabmap_initialpose = this->get_parameter("use_rtabmap_initialpose").as_bool();
        _pDocking_pose.camera_TF_distance = this->get_parameter("camera_tf_distance").as_double();

        RCLCPP_INFO(this->get_logger(), "Initialized tetra_service (model: %s, Home_ID: %d, Home_X: %.2f, MaxVel: %.2f, cmd_vel: %s)",
                    robot_model.c_str(), m_dHome_ID, _pHomePose.HOME_dPOSITION_X, _pDynamic_param.MAX_Linear_velocity, cmd_vel_topic.c_str());

        // Publishers
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
        pose_reset_publisher = this->create_publisher<std_msgs::msg::Int32>("pose_reset", 10);
        if (use_rtabmap_initialpose) {
            initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", rclcpp::QoS(1).transient_local().reliable());
        } else {
            initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", 10);
        }
        movebase_publisher = this->create_publisher<std_msgs::msg::Int32>("movebase", 10);
        footprint_publisher = this->create_publisher<geometry_msgs::msg::Pose>("footprint", 10);

        // Subscribers
        apriltag_subscriber = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "detections", 10, std::bind(&TETRA_SERVICE::ApriltagCallback, this, _1));

        if (!scan_topic.empty()) {
            sick_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic, rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::SickCallback, this, _1));
        }

        if (!scan2_topic.empty()) {
            cygbot_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan2_topic, rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::CygbotCallback, this, _1));
        }

        if (!scan_merge_topic.empty()) {
            scan_merge_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_merge_topic, rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::LaserScanMergeCallback, this, _1));
        }

        bumper_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "bumper_data", 1, std::bind(&TETRA_SERVICE::BumperCallback, this, _1));

        emg_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "emg_state", 1, std::bind(&TETRA_SERVICE::EMGCallback, this, _1));

        tetra_battery_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "tetra_battery", 1, std::bind(&TETRA_SERVICE::BatteryCallback, this, _1));

        docking_status_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "docking_status", 1, std::bind(&TETRA_SERVICE::DockingStatusCallback, this, _1));

        conveyor_sensor_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "conveyor_sensor", 1, std::bind(&TETRA_SERVICE::ConveyorSensorCallback, this, _1));

        conveyor_movement_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "conveyor_movement", 1, std::bind(&TETRA_SERVICE::ConveyorMovementCallback, this, _1));

        joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::joyCallback, this, _1));

        particlecloud_subscriber = this->create_subscription<nav2_msgs::msg::ParticleCloud>(
            "particle_cloud", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::AMCL_PaticleCloud_Callback, this, _1));

        cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::cmd_vel_Callback, this, _1));

        initialpose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::Initialpose_Callback, this, _1));

        map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::SensorDataQoS(), std::bind(&TETRA_SERVICE::map_Callback, this, _1));

        // Services
        docking_cmd_srv = create_service<interfaces::srv::DockingControl>(
            "docking_cmd", std::bind(&TETRA_SERVICE::Docking_Command, this, std::placeholders::_1, std::placeholders::_2));

        docking_stop_cmd_srv = create_service<interfaces::srv::DockingStop>(
            "docking_stop_cmd", std::bind(&TETRA_SERVICE::Docking_Stop_Command, this, std::placeholders::_1, std::placeholders::_2));

        save_map_cmd_srv = create_service<interfaces::srv::SaveMap>(
            "save_map_cmd", std::bind(&TETRA_SERVICE::Save_Map_Command, this, std::placeholders::_1, std::placeholders::_2));

        delete_map_cmd_srv = create_service<interfaces::srv::DeleteMap>(
            "delete_map_cmd", std::bind(&TETRA_SERVICE::Delete_Map_Command, this, std::placeholders::_1, std::placeholders::_2));

        run_mapping_cmd_srv = create_service<interfaces::srv::RunMapping>(
            "run_mapping_cmd", std::bind(&TETRA_SERVICE::Mapping_Mode_Command, this, std::placeholders::_1, std::placeholders::_2));

        run_3d_mapping_cmd_srv = create_service<interfaces::srv::Run3DMapping>(
            "run_3d_mapping_cmd", std::bind(&TETRA_SERVICE::Mapping_3D_Mode_Command, this, std::placeholders::_1, std::placeholders::_2));

        run_navigation_cmd_srv = create_service<interfaces::srv::RunNavigation>(
            "run_navigation_cmd", std::bind(&TETRA_SERVICE::Navigation_Mode_Command, this, std::placeholders::_1, std::placeholders::_2));

        get_information_cmd_srv = create_service<interfaces::srv::GetInformation>(
            "get_information_cmd", std::bind(&TETRA_SERVICE::Get_Information_Command, this, std::placeholders::_1, std::placeholders::_2));

        set_location_cmd_srv = create_service<interfaces::srv::SetLocation>(
            "set_location_cmd", std::bind(&TETRA_SERVICE::Set_Location_Command, this, std::placeholders::_1, std::placeholders::_2));

        goto_location_cmd_srv = create_service<interfaces::srv::GotoLocation>(
            "goto_location_cmd", std::bind(&TETRA_SERVICE::Goto_Location_Command, this, std::placeholders::_1, std::placeholders::_2));

        goto_location2_cmd_srv = create_service<interfaces::srv::GotoLocation2>(
            "goto_location2_cmd", std::bind(&TETRA_SERVICE::Goto_Location2_Command, this, std::placeholders::_1, std::placeholders::_2));

        goto_cancel_cmd_srv = create_service<interfaces::srv::GotoCancel>(
            "goto_cancel_cmd", std::bind(&TETRA_SERVICE::Goto_Cancel_Command, this, std::placeholders::_1, std::placeholders::_2));

        set_maxspeed_cmd_srv = create_service<interfaces::srv::SetMaxspeed>(
            "set_maxspeed_cmd", std::bind(&TETRA_SERVICE::Set_Maxspeed_Command, this, std::placeholders::_1, std::placeholders::_2));

        // Clients
        led_control_client = this->create_client<interfaces::srv::LedControl>("led_cmd");
        led_toggle_control_client = this->create_client<interfaces::srv::LedToggleControl>("led_toggle_cmd");
        toggle_on_client = this->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");
        imu_reset_client = this->create_client<interfaces::srv::ImuReset>("all_data_reset");
        request_nomotion_update_client = this->create_client<std_srvs::srv::Empty>("request_nomotion_update");
        clear_entire_global_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
        clear_entire_local_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
        set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");
        set_speed_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "controller_server");

        // Action Client
        nav_to_pose_action_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Timers
        timer_ = this->create_wall_timer(10ms, std::bind(&TETRA_SERVICE::DockingThread_function, this));
        TF_timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::TF_CALC_Timer, this));
        Footprint_timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::Footprint_CALC_Timer, this));
        pose_reset_monitor_timer_ = this->create_wall_timer(100ms, std::bind(&TETRA_SERVICE::PoseResetMonitor_Timer, this));
    }

    rclcpp::Time current_time;
    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::Int32 pose_reset_data;
    geometry_msgs::msg::PoseWithCovarianceStamped initPose;
    std_msgs::msg::Int32 movebase;
    geometry_msgs::msg::Pose footprint;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr TF_timer_;
    rclcpp::TimerBase::SharedPtr Footprint_timer_;
    rclcpp::TimerBase::SharedPtr pose_reset_monitor_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pose_reset_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr movebase_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr footprint_publisher;

    // Subscriptions
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
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;

    // Action
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_action_client;

    void PoseResetMonitor_Timer() 
    {
        if (m_bActive_map) 
        {
            if (m_iPose_reset_cnt > 500) 
            {
                RCLCPP_INFO(this->get_logger(), "RESET Robot POSE Call!!!");
                this->Clear_Costmap();
                this->Reset_Robot_Pose();
                m_iPose_reset_cnt = 0;
            } 
            else 
            {
                if (_pRobot.m_iCallback_Charging_status == 2 || 
                    _pRobot.m_iCallback_Charging_status == 3 || 
                    _pRobot.m_iCallback_Charging_status == 6 || 
                    _pRobot.m_iCallback_Charging_status == 7) 
                {
                    m_iPose_reset_cnt++;
                } 
                else 
                {
                    m_iPose_reset_cnt = 0;
                }
            }
        }
    }

    // Subscribe Callback Functions
    void cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        _pDynamic_param.m_linear_vel  = msg->linear.x;
        _pDynamic_param.m_angular_vel = msg->angular.z;
    }

    void AMCL_PaticleCloud_Callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
    {
        auto m_request3 = std::make_shared<std_srvs::srv::Empty::Request>();
        int m_iParticleCloud_size = msg->particles.size();
        if(m_iParticleCloud_size > 501 && _pDynamic_param.m_linear_vel == 0.0 && _pDynamic_param.m_angular_vel == 0.0 && _pFlag_Value.m_bFlag_Initialpose)
        {
            if(_pFlag_Value.m_bFlag_nomotion)
            {
                while(!request_nomotion_update_client->wait_for_service(1s))
                {
                    if(!rclcpp::ok())
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                }
                request_nomotion_update_client->async_send_request(m_request3);
            }   
        }
        else
        {
            _pFlag_Value.m_bFlag_Initialpose = false;
            _pFlag_Value.m_bFlag_nomotion = false;
        }
    }

    void Initialpose_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
    {
        _pFlag_Value.m_bFlag_Initialpose = true;
        _pFlag_Value.m_bFlag_nomotion = true;
    }

    void map_Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if(msg->data.size() > 0)
        {
            m_bActive_map = true;
        }
        else
        {
            m_bActive_map = false;
        }
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        if(joy->buttons.size() > 7)
        {
            if(joy->buttons[6]) // LT button
            {	
                _pRobot.m_iDocking_id = 0;
                m_iDocking_CommandMode = 1;
                _pFlag_Value.m_bfalg_DockingExit = false;
            }
            if(joy->buttons[7]) // RT button
            {	
                m_iDocking_CommandMode = 0;
                _pFlag_Value.m_bfalg_DockingExit = true;
            }
        }
    }

    void ApriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) 
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        int size = msg->detections.size();
        if(size != 0)
        {
            for (const auto &detection : msg->detections) 
            {
                ar_tag_pose_.m_iApril_tag_id = detection.id;
            }
            string strTF_frame_name = ar_tag_pose_.m_strFamily + to_string(ar_tag_pose_.m_iApril_tag_id);
            if (!tf2_buffer_->canTransform("camera", strTF_frame_name, tf2::TimePointZero)) {
                return;
            }
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                transformStamped = tf2_buffer_->lookupTransform("camera", strTF_frame_name, tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                return;
            }

            ar_tag_pose_.m_dApril_tag_pose_x = transformStamped.transform.translation.x;
            ar_tag_pose_.m_dApril_tag_pose_y = transformStamped.transform.translation.y;
            ar_tag_pose_.m_dApril_tag_pose_z = transformStamped.transform.translation.z;
            ar_tag_pose_.m_dApril_tag_orientation_x = transformStamped.transform.rotation.x;
            ar_tag_pose_.m_dApril_tag_orientation_y = transformStamped.transform.rotation.y;
            ar_tag_pose_.m_dApril_tag_orientation_z = transformStamped.transform.rotation.z;
            ar_tag_pose_.m_dApril_tag_orientation_w = transformStamped.transform.rotation.w;
            
            tf2::Quaternion q;
            q.setW(transformStamped.transform.rotation.w);
            q.setX(transformStamped.transform.rotation.x);
            q.setY(transformStamped.transform.rotation.y);
            q.setZ(transformStamped.transform.rotation.z);
            tf2::Matrix3x3 m(q);
            m.getRotation(q);
            m.getRPY(ar_tag_pose_.m_dApril_tag_roll, ar_tag_pose_.m_dApril_tag_pitch, ar_tag_pose_.m_dApril_tag_yaw);
            ar_tag_pose_.m_dPositioning_Angle = ar_tag_pose_.m_dApril_tag_pitch;
            ar_tag_pose_.m_transform_pose_x = ar_tag_pose_.m_dApril_tag_pose_z;
            ar_tag_pose_.m_transform_pose_y = ar_tag_pose_.m_dApril_tag_pose_x - 0.0175;
        }
        else
        {
            ar_tag_pose_.m_iApril_tag_id = -1;
            ar_tag_pose_.m_transform_pose_x = 0.0;
            ar_tag_pose_.m_transform_pose_y = 0.0;
            ar_tag_pose_.m_dApril_tag_pitch = 0.0;
            ar_tag_pose_.m_dApril_tag_yaw = 0.0;
            ar_tag_pose_.m_dPositioning_Angle = 0.0;
        }
    }

    void SickCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int size = msg->ranges.size();
        if (size <= 0) return;

        int C_minIndex = size * 1 / 3;
        int C_maxIndex = size * 2 / 3;
        int C_closestIndex = -1;
        double C_minVal = 0.3;

        for (int i = C_minIndex; i < C_maxIndex && i < size; i++)
        {
            if ((msg->ranges[i] <= C_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                C_minVal = msg->ranges[i];
                C_closestIndex = i;
            }
        }
        if(C_closestIndex >= 0 && msg->ranges[C_closestIndex] > 0.1)
        {
            _pFlag_Value.m_bFlag_Obstacle_Center = true;
        }
        else
        {
            _pFlag_Value.m_bFlag_Obstacle_Center = false;
        }
    }

    void CygbotCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int size = msg->ranges.size();
        if (size <= 0) return;

        int C_minIndex = 0;
        int C_maxIndex = size;
        int C_closestIndex = -1;
        double C_minVal = 0.3;

        for (int i = C_minIndex; i < C_maxIndex; i++)
        {
            if ((msg->ranges[i] <= C_minVal) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                C_minVal = msg->ranges[i];
                C_closestIndex = i;
            }
        }
        if(C_closestIndex >= 0 && msg->ranges[C_closestIndex] > 0.1)
        {
            _pFlag_Value.m_bFlag_Obstacle_cygbot = true;
        }
        else
        {
            _pFlag_Value.m_bFlag_Obstacle_cygbot = false;
        }
    }

    void LaserScanMergeCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        int size = msg->ranges.size();
        if (size < 1000) return;

        // Rear Position
        int Rear_closestIndex = -1;
        float m_fRear_min_val = 0.434 + 0.3;

        for (int i = 0; i < 136 && i < size; i++)
        {
            if ((msg->ranges[i] <= m_fRear_min_val) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                m_fRear_min_val = msg->ranges[i];
                Rear_closestIndex = i;
            }
        }
        for (int i = 952; i < 1084 && i < size; i++)
        {
            if ((msg->ranges[i] <= m_fRear_min_val) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                m_fRear_min_val = msg->ranges[i];
                Rear_closestIndex = i;
            }
        }
        _pFlag_Value.m_bFlag_MergeScan_Rear = (Rear_closestIndex > 0);

        // Right Position
        int Right_closestIndex = -1;
        float m_fRight_min_val = 0.434 + 0.3;

        for (int i = 136; i < 408 && i < size; i++)
        {
            if ((msg->ranges[i] <= m_fRight_min_val) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                m_fRight_min_val = msg->ranges[i];
                Right_closestIndex = i;
            }
        }
        _pFlag_Value.m_bFlag_MergeScan_Right = (Right_closestIndex > 0);

        // Front Position
        int Front_closestIndex = -1;
        float m_fFront_min_val = 0.434 + 0.3;

        for (int i = 408; i < 680 && i < size; i++)
        {
            if ((msg->ranges[i] <= m_fFront_min_val) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                m_fFront_min_val = msg->ranges[i];
                Front_closestIndex = i;
            }
        }
        _pFlag_Value.m_bFlag_MergeScan_Front = (Front_closestIndex > 0);

        // Left Position
        int Left_closestIndex = -1;
        float m_fLeft_min_val = 0.434 + 0.3;

        for (int i = 680; i < 952 && i < size; i++)
        {
            if ((msg->ranges[i] <= m_fLeft_min_val) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
            {
                m_fLeft_min_val = msg->ranges[i];
                Left_closestIndex = i;
            }
        }
        _pFlag_Value.m_bFlag_MergeScan_Left = (Left_closestIndex > 0);
    }

    void BumperCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        _pRobot.m_iCallback_Bumper = msg->data;
        if(_pRobot.m_iCallback_Bumper != 0)
        {
            if(!_pFlag_Value.m_bFlag_Disable_bumper)
            {
                LedToggleControl_Call(1, 10, 100, 10, 1);
                ToggleOn_Call(18);
            }
            _pFlag_Value.m_bumperhit_flag = true;
        }
        else
        {			
            if(_pFlag_Value.m_bumperhit_flag)
            {
                LedToggleControl_Call(1, 3, 100, 3, 1);
                ToggleOn_Call(63); // White led
                _pFlag_Value.m_bumperhit_flag = false;
            }
        }
    }

    void EMGCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        _pRobot.m_iCallback_EMG = msg->data;
        if(_pRobot.m_iCallback_EMG != 0)
        {
            LedToggleControl_Call(1, 10, 100, 10, 1);
            ToggleOn_Call(18);
            _pFlag_Value.m_emgpush_flag = false;
        }
        else
        {
            if(!_pFlag_Value.m_emgpush_flag)
            {
                LedToggleControl_Call(1, 3, 100, 3, 1);
                ToggleOn_Call(63);
                _pFlag_Value.m_emgpush_flag = true;
            }
        }
    }

    void BatteryCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        _pRobot.m_iCallback_Battery = msg->data;
    }

    void DockingStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        _pRobot.m_iCallback_Charging_status = msg->data;
    }

    void ConveyorSensorCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        _pRobot.m_iConveyor_Sensor_info = msg->data;
    }

    void ConveyorMovementCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        _pRobot.m_iConveyor_movement = msg->data;
    }

    // Client Callbacks
    void LedControl_callback(rclcpp::Client<interfaces::srv::LedControl>::SharedFuture /*future*/) {}
    void LedToggleControl_callback(rclcpp::Client<interfaces::srv::LedToggleControl>::SharedFuture /*future*/) {}
    void ToggleOn_callback(rclcpp::Client<interfaces::srv::ToggleOn>::SharedFuture /*future*/) {}
    void ImuReset_callback(rclcpp::Client<interfaces::srv::ImuReset>::SharedFuture /*future*/) {}

    bool Depart_Station2Move()
    {
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if(ar_tag_pose_.m_transform_pose_x < 0.8 && ar_tag_pose_.m_iApril_tag_id != -1)
        {
            cmd.linear.x = -0.05; 
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            bResult = false;
        }
        else
        {
            cmd.linear.x = 0.0; 
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);

            Set_goal(_pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY, _pGoal_pose.goal_positionZ,
                     _pGoal_pose.goal_quarterX, _pGoal_pose.goal_quarterY, _pGoal_pose.goal_quarterZ, _pGoal_pose.goal_quarterW);

            m_iDocking_CommandMode = 0;
            bResult = true;
        }
        return bResult;
    }

    double Rotation_Movement()
    {
        double iResult = 0.1;

        if(_pFlag_Value.m_bFlag_Obstacle_Right)
        {
            m_iRotation_Mode = 1;
            printf(" CCW Rotation--- \n");
        }
        else if(_pFlag_Value.m_bFlag_Obstacle_Left)
        {
            m_iRotation_Mode = 2;
            printf(" CW Rotation+++ \n");
        }

        switch(m_iRotation_Mode)
        {
            case 0: iResult = 0.1; break;
            case 1: iResult = -0.1; break;
            case 2: iResult = 0.1; break;
        }

        return iResult;
    }

    void Docking_EXIT()
    {
        cmd.linear.x = 0.0; 
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        m_iDocking_CommandMode = 0;
        printf("[EXIT]: Docking Exit !! \n");
    }

    void Reset_EKF_SetPose()
    {
        auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
        request->pose.pose.pose.position.x = 0.0;
        request->pose.pose.pose.position.y = 0.0;
        request->pose.pose.pose.position.z = 0.0;
        request->pose.pose.pose.orientation.x = 0.0;
        request->pose.pose.pose.orientation.y = 0.0;
        request->pose.pose.pose.orientation.z = 0.0;
        request->pose.pose.pose.orientation.w = 1.0;
        request->pose.pose.covariance[0] = 0.25;
        request->pose.pose.covariance[6 * 1 + 1] = 0.25;
        request->pose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

        set_pose_client_->async_send_request(
            request, 
            [this](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture /*future*/) {
                RCLCPP_DEBUG(this->get_logger(), "robot_localization SetPose.");
            }
        );

        rclcpp::sleep_for(std::chrono::milliseconds(50));
        initPose.header.stamp = this->now();
        initPose.header.frame_id = "map";
        initPose.pose.pose.position.x = 0.0;
        initPose.pose.pose.position.y = 0.0;
        initPose.pose.pose.position.z = 0.0;
        initPose.pose.pose.orientation.x = 0.0;
        initPose.pose.pose.orientation.y = 0.0;
        initPose.pose.pose.orientation.z = 0.0;
        initPose.pose.pose.orientation.w = 1.0;
        initPose.pose.covariance[0] = 0.25;
        initPose.pose.covariance[6 * 1 + 1] = 0.25;
        initPose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

        initial_pose_publisher->publish(initPose);
    }

    bool Marker_Reset_Robot_Pose()
    {
        _pFlag_Value.m_bFlag_nomotion = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        bool bResult = false;
        if (ar_tag_pose_.m_iApril_tag_id == -1) 
        {
            return false;
        }
        else 
        {
            bResult = true;
        }

        initPose.header.stamp = this->now();
        initPose.header.frame_id = "map";
        initPose.pose.pose.position.x = _pLandMarkPose.init_position_x;
        initPose.pose.pose.position.y = _pLandMarkPose.init_position_y;
        initPose.pose.pose.position.z = _pLandMarkPose.init_position_z;
        initPose.pose.pose.orientation.x = 0.0;
        initPose.pose.pose.orientation.y = 0.0;
        initPose.pose.pose.orientation.z = -1.0 * _pLandMarkPose.init_orientation_z;
        initPose.pose.pose.orientation.w = _pLandMarkPose.init_orientation_w;

        initPose.pose.covariance[0] = 0.25;
        initPose.pose.covariance[6 * 1 + 1] = 0.25;
        initPose.pose.covariance[6 * 5 + 5] = 0.06853892326654787;

        initial_pose_publisher->publish(initPose);
        _pFlag_Value.m_bFlag_nomotion = true;

        return bResult;
    }

    void Reset_Robot_Pose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if(_pRobot.HOME_id == ar_tag_pose_.m_iApril_tag_id)
        {
            _pFlag_Value.m_bFlag_nomotion = false;
            ImuReset_Call();

            pose_reset_data.data = 1;
            pose_reset_publisher->publish(pose_reset_data);

            Reset_EKF_SetPose();
        }

        _pFlag_Value.m_bFlag_Initialpose = true;
        LedToggleControl_Call(1, 3, 100, 3, 1);
        ToggleOn_Call(63);
    }

    void TF_CALC_Timer()
    {
        try 
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf2_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

            _pTF_pose.poseTFx = transformStamped.transform.translation.x;
            _pTF_pose.poseTFy = transformStamped.transform.translation.y;
            _pTF_pose.poseTFz = transformStamped.transform.translation.z;
            _pTF_pose.poseTFqx = transformStamped.transform.rotation.x;
            _pTF_pose.poseTFqy = transformStamped.transform.rotation.y;
            _pTF_pose.poseTFqz = transformStamped.transform.rotation.z;
            _pTF_pose.poseTFqw = transformStamped.transform.rotation.w;
            
            tf2::Quaternion q;
            q.setW(transformStamped.transform.rotation.w);
            q.setX(transformStamped.transform.rotation.x);
            q.setY(transformStamped.transform.rotation.y);
            q.setZ(transformStamped.transform.rotation.z);
            tf2::Matrix3x3 m(q);
            m.getRotation(q);
            m.getRPY(_pTF_pose.pose_euler_roll, _pTF_pose.pose_euler_pitch, _pTF_pose.pose_euler_yaw);
            _pTF_pose.theta_deg = _pTF_pose.pose_euler_yaw * (180.0 / M_PI);
        } 
        catch (tf2::TransformException &ex) 
        {
            // Transform may not be immediately available
        }
    }

    void Footprint_CALC_Timer()
    {
        try 
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf2_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

            footprint.position.x = transformStamped.transform.translation.x;
            footprint.position.y = transformStamped.transform.translation.y;
            footprint.position.z = transformStamped.transform.translation.z;
            footprint.orientation.x = transformStamped.transform.rotation.x;
            footprint.orientation.y = transformStamped.transform.rotation.y;
            footprint.orientation.z = transformStamped.transform.rotation.z;
            footprint.orientation.w = transformStamped.transform.rotation.w;
            footprint_publisher->publish(footprint);
        } 
        catch (tf2::TransformException &ex) 
        {
            // Transform not available yet
        }
    }

    bool SaveLocation(string str_location, 
                      double m_dPoseTFx,  double m_dPoseTFy,  double m_dPoseTFz,
                      double m_dPoseTFqx, double m_dPoseTFqy, double m_dPoseTFqz, double m_dPoseTFqw)
    {
        bool bResult = false;
        string m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";    
        fp = fopen(m_strFilePathName.c_str(), "w");
        if(fp == NULL)
        { 
            printf("file is null \n");
            bResult = false;
        }
        else
        {
            fprintf(fp, "0,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",
                    m_dPoseTFx, m_dPoseTFy, m_dPoseTFz, m_dPoseTFqx, m_dPoseTFqy, m_dPoseTFqz, m_dPoseTFqw);
            fclose(fp);
            bResult = true;
        }
        return bResult;
    }

    bool OpenLocationFile(string str_location)
    {
        bool bResult = false;
        string m_strFilePathName;

        if(str_location == "HOME")
        {
            _pGoal_pose.goal_positionX = _pHomePose.HOME_dPOSITION_X;
            _pGoal_pose.goal_positionY = _pHomePose.HOME_dPOSITION_Y;
            _pGoal_pose.goal_positionZ = _pHomePose.HOME_dPOSITION_Z;
            _pGoal_pose.goal_quarterX = _pHomePose.HOME_dQUATERNION_X;
            _pGoal_pose.goal_quarterY = _pHomePose.HOME_dQUATERNION_Y;
            _pGoal_pose.goal_quarterZ = _pHomePose.HOME_dQUATERNION_Z;
            _pGoal_pose.goal_quarterW = _pHomePose.HOME_dQUATERNION_W;
            bResult = true; 
        }
        else
        {
            m_strFilePathName = "/home/tetra/DATA/" + str_location + ".txt";  
            fp = fopen(m_strFilePathName.c_str(), "r");  

            if(fp != NULL)
            {
                while(!feof(fp))
                {
                    if(fgets(Textbuffer, sizeof(Textbuffer), fp) != NULL)
                    {
                        char* ptr = strtok(Textbuffer, ",");
                        int icnt = 0;

                        while(ptr != NULL)
                        {   
                            ptr = strtok(NULL, ",");
                            switch(icnt)
                            {
                                case 0:
                                    if(ptr != NULL) _pGoal_pose.goal_positionX = atof(ptr);
                                    break;
                                case 1:
                                    if(ptr != NULL) _pGoal_pose.goal_positionY = atof(ptr);
                                    break;
                                case 2:
                                    if(ptr != NULL) _pGoal_pose.goal_positionZ = atof(ptr);
                                    break;
                                case 3:
                                    if(ptr != NULL) _pGoal_pose.goal_quarterX = atof(ptr);
                                    break;
                                case 4:
                                    if(ptr != NULL) _pGoal_pose.goal_quarterY = atof(ptr);
                                    break;
                                case 5:
                                    if(ptr != NULL) _pGoal_pose.goal_quarterZ = atof(ptr);  
                                    break;
                                case 6:
                                    if(ptr != NULL) _pGoal_pose.goal_quarterW = atof(ptr);
                                    break;
                            }
                            icnt++;
                        }
                        bResult = true; 
                    }
                    else
                    {
                        bResult = false; 
                    }
                }                
                fclose(fp);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "File Open Fail Error!: %s", str_location.c_str());
                bResult = false;
            }
        }
        return bResult;
    }

    // Charging Station Docking
    bool ChargingStation_tracking(bool bOn, int marker_id)
    {
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if(bOn)
        {
            float m_fdistance = 0.0;
            if(ar_tag_pose_.m_iApril_tag_id == marker_id)
            {	
                m_fdistance = sqrt(ar_tag_pose_.m_transform_pose_x * ar_tag_pose_.m_transform_pose_x + 
                                   ar_tag_pose_.m_transform_pose_y * ar_tag_pose_.m_transform_pose_y);
                if(m_fdistance > 0.65)
                {
                    cmd.linear.x = 1.0 * (m_fdistance / 1.2) * 0.15; 
                    if(cmd.linear.x > 1.0)
                    {
                        cmd.linear.x = 0.0; 
                        cmd.angular.z = 0.0;
                        printf("[Linear Over speed]: follower is closing \n");
                        return false;
                    }
                        
                    cmd.angular.z = -1.0 * atan2(ar_tag_pose_.m_transform_pose_y, ar_tag_pose_.m_transform_pose_x) / 1.25;

                    if((cmd.angular.z > 1.0) || (cmd.angular.z < -1.0))
                    {
                        cmd.linear.x = 0.0; 
                        cmd.angular.z = 0.0;
                        printf("[Angular Over speed]: follower is closing \n");
                        return false;
                    }

                    cmd_vel_publisher->publish(cmd);
                }
                else
                {
                    printf("ChargingStation_tracking m_fdistance <= 0.65\n");
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    cmd_vel_publisher->publish(cmd);
                    ar_tag_pose_.m_target_yaw = ar_tag_pose_.m_dApril_tag_pitch;
                    ar_tag_pose_.m_target_theta = ar_tag_pose_.m_dPositioning_Angle;
                    if(ar_tag_pose_.m_target_yaw <= 0.0174533 && ar_tag_pose_.m_target_yaw >= -0.0174533)
                    {
                        m_iDocking_CommandMode = 4;
                    }
                    else
                    {
                        m_iDocking_CommandMode = 3;
                    }

                    m_iNoMarker_cnt = 0;
                }
            }
            else
            {
                printf("[ChargingStation_tracking] No Marker, Rotation Movement !! \n");
                cmd.angular.z = Rotation_Movement();
                cmd_vel_publisher->publish(cmd);

                if(m_iNoMarker_cnt > 8000)
                {
                    m_iNoMarker_cnt = 0;
                    cmd.linear.x = 0.0; 
                    cmd.angular.z = 0.0;
                    cmd_vel_publisher->publish(cmd);
                    printf("DockingStation scan Fail !! \n");
                    m_iDocking_CommandMode = 9;
                }
                else
                {
                    m_iNoMarker_cnt++;
                }
            }
        }
        else
        {
            cmd.linear.x = 0.0; 
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            printf("Docking Loop STOP!_not find Marker!! \n");
            m_iDocking_CommandMode = 0;
        }
            
        bResult = true;
        return bResult;
    }

    bool ChargingStation_Yaw_tracking()
    { 
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        float m_fdistance = sqrt(ar_tag_pose_.m_transform_pose_x * ar_tag_pose_.m_transform_pose_x + 
                                 ar_tag_pose_.m_transform_pose_y * ar_tag_pose_.m_transform_pose_y);

        if(ar_tag_pose_.m_target_yaw <= 0.0174533 && ar_tag_pose_.m_target_yaw >= -0.0174533)
        {
            m_iDocking_CommandMode = 4;
            bResult = true;
            return bResult;
        }

        if(ar_tag_pose_.m_target_yaw > 0)
        {
            if(m_iRotation_cnt <= 100)
            {
                if(_pFlag_Value.m_bFlag_MergeScan_Left) 
                {
                    cmd.linear.x = 0.0; 
                    cmd.angular.z = 0.0;
                    cmd_vel_publisher->publish(cmd);
                }
                else
                {
                    cmd.linear.x = -1.0 * m_fdistance * 0.2;
                    cmd.angular.z = -1.0 * ar_tag_pose_.m_target_yaw * 2.0;
                    cmd_vel_publisher->publish(cmd);
                    m_iRotation_cnt++;
                }
            }
            else
            {
                m_iRotation_cnt = 0;
                m_iDocking_CommandMode = 31;
            }
            m_iRotation_Mode = 2;
        }
        else
        {
            if(m_iRotation_cnt <= 100)
            {
                if(_pFlag_Value.m_bFlag_MergeScan_Right)
                {
                    cmd.linear.x = 0.0; 
                    cmd.angular.z = 0.0;
                    cmd_vel_publisher->publish(cmd);
                }
                else
                {
                    cmd.linear.x = -1.0 * m_fdistance * 0.2;
                    cmd.angular.z = -1.0 * ar_tag_pose_.m_target_yaw * 2.0;
                    cmd_vel_publisher->publish(cmd);
                    m_iRotation_cnt++;
                }
            }
            else
            {
                m_iRotation_cnt = 0;
                m_iDocking_CommandMode = 31;
            }
            m_iRotation_Mode = 1;
        }

        bResult = true;
        return bResult;
    }

    bool ChargingStation_back_step()
    {
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        float m_fdistance = sqrt(ar_tag_pose_.m_transform_pose_x * ar_tag_pose_.m_transform_pose_x + 
                                 ar_tag_pose_.m_transform_pose_y * ar_tag_pose_.m_transform_pose_y);
        
        if(m_iBack_cnt < 100)
        {
            cmd.angular.z = 0.0;
            cmd.linear.x = -1.0 * (m_fdistance) * 0.2;
            cmd_vel_publisher->publish(cmd);
            m_iBack_cnt++;
        }
        else
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);

            m_iBack_cnt = 0;
            m_iDocking_CommandMode = 2;
        }

        bResult = true;
        return bResult;
    }
    
    bool ChargingStation_tracking2(int marker_id)
    {
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if(ar_tag_pose_.m_iApril_tag_id == marker_id)
        {
            float m_fdistance = sqrt(ar_tag_pose_.m_transform_pose_x * ar_tag_pose_.m_transform_pose_x + 
                                     ar_tag_pose_.m_transform_pose_y * ar_tag_pose_.m_transform_pose_y);
            if(_pRobot.m_iCallback_Charging_status < 2)
            {
                cmd.linear.x = 1.0 * (m_fdistance / 1.2) * 0.1;
                if(cmd.linear.x > 1.0)
                {
                    cmd.linear.x = 0.0; 
                    cmd.angular.z = 0.0;
                    printf("[Linear Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd.angular.z = -1.0 * atan2(ar_tag_pose_.m_transform_pose_y, ar_tag_pose_.m_transform_pose_x) / 1.25;
                if((cmd.angular.z > 1.0) || (cmd.angular.z < -1.0))
                {
                    cmd.linear.x = 0.0; 
                    cmd.angular.z = 0.0;
                    printf("[Angular Over speed]: follower is closing \n");
                    return false;
                }
                
                cmd_vel_publisher->publish(cmd);
            }
            else
            {
                cmd.linear.x = 0.0; 
                cmd.angular.z = 0.0;
                cmd_vel_publisher->publish(cmd);
                printf("Tracking STOP & Docking Finish !! \n");
                m_iDocking_CommandMode = 8;
                m_iNoMarker_cnt = 0;
            }
        }
        else
        {
            cmd.linear.x = 0.0; 
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            printf("[ChargingStation_tracking2] No Marker 2! \n");
            if(m_iNoMarker_cnt >= 10)
            {
                m_iNoMarker_cnt = 0;
                m_iDocking_CommandMode = 1;
                printf("[ChargingStation_tracking2] No Marker 2_Timeout! \n");
            }
            else
            {
                m_iNoMarker_cnt++;
            }
        }

        bResult = true;
        return bResult;
    }

    void DockingThread_function()
    {
        switch(m_iDocking_CommandMode)
        {
            case 0:
                break;
            case 1:
                printf("Docking Loop 1 Start... \n");
                LedToggleControl_Call(1, 3, 100, 3, 100);
                ToggleOn_Call(63);
                m_iDocking_CommandMode = 2;
                break;
            case 2:
                _pRobot.HOME_id = _pRobot.m_iDocking_id;
                ChargingStation_tracking(true, _pRobot.HOME_id);
                if(_pRobot.m_bFalg_DockingExit)
                {
                    Docking_EXIT();
                    m_iDocking_CommandMode = 0;
                }
                break;
            case 3:
                ChargingStation_Yaw_tracking();
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    m_iDocking_CommandMode = 0;
                }
                break;
            case 4:
                ChargingStation_tracking2(_pRobot.HOME_id);
                if(_pFlag_Value.m_bfalg_DockingExit)
                {
                    Docking_EXIT();
                    m_iDocking_CommandMode = 0;
                }
                break;
            case 8:
                printf("Docking End Loop 8... \n");
                LedToggleControl_Call(1, 3, 100, 3, 100);
                ToggleOn_Call(9);
                printf("TETRA POSE Reset! \n");
                Reset_Robot_Pose();
                LedToggleControl_Call(1, 5, 100, 5, 1);
                ToggleOn_Call(63);
                m_iDocking_CommandMode = 0;
                break;
            case 9:
                LedToggleControl_Call(1, 10, 100, 10, 1);
                ToggleOn_Call(18);
                m_iDocking_CommandMode = 0;
                break;
            case 10:
                Depart_Station2Move();
                break;
            case 31:
                printf("Docking Loop 31 ChargingStation_back_step... \n");
                ChargingStation_back_step();
                break;
            default:
                break;
        }
    }

    // Server Callback Functions
    bool Docking_Command(
        const std::shared_ptr<interfaces::srv::DockingControl::Request> request, 
        const std::shared_ptr<interfaces::srv::DockingControl::Response> response)
    {
        _pRobot.m_iDocking_id = request->id;
        m_iDocking_CommandMode = request->mode;
        response->command_result = true;
        return true;
    }

    bool Docking_Stop_Command(
        const std::shared_ptr<interfaces::srv::DockingStop::Request> /*request*/, 
        const std::shared_ptr<interfaces::srv::DockingStop::Response> response)
    {
        printf("[STOP Call]: Docking Stop !! \n");
        Docking_EXIT();
        response->command_result = true;
        return true;
    }

    bool Save_Map_Command(
        const std::shared_ptr<interfaces::srv::SaveMap::Request> request, 
        const std::shared_ptr<interfaces::srv::SaveMap::Response> response)
    {
        string str_command = "gnome-terminal -- /home/tetra/mapsave.sh " + request->map_name;
        int iResult = std::system(str_command.c_str());
        (void)iResult;
        printf("Save Map Name: %s \n", request->map_name.c_str());	
        response->command_result = true;
        return true;
    }

    bool Delete_Map_Command(
        const std::shared_ptr<interfaces::srv::DeleteMap::Request> request, 
        const std::shared_ptr<interfaces::srv::DeleteMap::Response> response)
    {
        namespace fs = std::filesystem;

        string m_strMapName_yaml = "/home/tetra/ros2_ws/src/tetra_navigation2/maps/" + request->map_name + ".yaml";  
        string m_strMapName_pgm  = "/home/tetra/ros2_ws/src/tetra_navigation2/maps/" + request->map_name + ".pgm";
        string m_strMapName_db   = "/home/tetra/.ros/" + request->map_name + ".db";  

        bool res = false;
        if(fs::exists(m_strMapName_yaml))
        {
            remove(m_strMapName_yaml.c_str());
            remove(m_strMapName_pgm.c_str());
            printf("Delete 2D Map Name: %s \n", request->map_name.c_str());
            res = true;
        }

        if(fs::exists(m_strMapName_db))
        {
            remove(m_strMapName_db.c_str());
            printf("Delete 3D Map Name: %s \n", request->map_name.c_str());
            res = true;
        }

        response->command_result = res;
        return true;
    }

    bool Mapping_Mode_Command(
        const std::shared_ptr<interfaces::srv::RunMapping::Request> /*request*/, 
        const std::shared_ptr<interfaces::srv::RunMapping::Response> response)
    {
        string str_command = "gnome-terminal -- /home/tetra/mapping.sh ";
        int iResult = std::system(str_command.c_str());
        (void)iResult;
        ex_ilaunchMode = 1;
        response->command_result = true;
        return true;
    }

    bool Mapping_3D_Mode_Command(
        const std::shared_ptr<interfaces::srv::Run3DMapping::Request> request, 
        const std::shared_ptr<interfaces::srv::Run3DMapping::Response> response)
    {
        printf("Map_Name _ %s \n", request->map_name.c_str());		
        string str_command = "gnome-terminal -- /home/tetra/mapping.sh " + request->map_name;
        int iResult = std::system(str_command.c_str());
        (void)iResult;
        ex_ilaunchMode = 1;
        response->command_result = true;
        return true;
    }

    bool Navigation_Mode_Command(
        const std::shared_ptr<interfaces::srv::RunNavigation::Request> request, 
        const std::shared_ptr<interfaces::srv::RunNavigation::Response> response)
    {
        printf("Load Map Call _ %s \n", request->map_name.c_str());		
        string str_command = "gnome-terminal -- /home/tetra/navigation.sh " + request->map_name;
        int iResult = std::system(str_command.c_str());
        (void)iResult;
        ex_ilaunchMode = 2;
        response->command_result = true;
        return true;
    }

    bool Get_Information_Command(
        const std::shared_ptr<interfaces::srv::GetInformation::Request> /*request*/, 
        const std::shared_ptr<interfaces::srv::GetInformation::Response> response)
    {
        response->battery = _pRobot.m_iCallback_Battery;
        response->error_code = _pRobot.m_iCallback_ErrorCode;
        response->emg = _pRobot.m_iCallback_EMG;
        response->bumper = _pRobot.m_iCallback_Bumper;
        response->charging = _pRobot.m_iCallback_Charging_status;
        response->running_mode = ex_ilaunchMode;
        response->command_result = true;
        return true;
    }

    bool Set_Location_Command(
        const std::shared_ptr<interfaces::srv::SetLocation::Request> request, 
        const std::shared_ptr<interfaces::srv::SetLocation::Response> response)
    {
        if(request->location == "HOME")
        {
            response->command_result = false;
            printf("[Location Save ERROR]: Cannot save with the name 'HOME'!!\n");
        }
        else
        {
            response->command_result = SaveLocation(request->location,
                                        _pTF_pose.poseTFx, _pTF_pose.poseTFy, _pTF_pose.poseTFz,
                                        _pTF_pose.poseTFqx, _pTF_pose.poseTFqy, _pTF_pose.poseTFqz, _pTF_pose.poseTFqw);
            
            response->pose_x = _pTF_pose.poseTFx;
            response->pose_y = _pTF_pose.poseTFy;
            response->pose_z = _pTF_pose.poseTFz;
            response->pose_qx = _pTF_pose.poseTFqx;
            response->pose_qy = _pTF_pose.poseTFqy;
            response->pose_qz = _pTF_pose.poseTFqz;
            response->pose_qw = _pTF_pose.poseTFqw;
        }
        return true;
    }

    void Set_goal(double position_x, double position_y, double position_z, 
                  double orientation_x, double orientation_y, double orientation_z, double orientation_w) 
    {
        while (!this->nav_to_pose_action_client->wait_for_action_server(1s)) 
        {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(get_logger(), "Waiting for action server...");
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = position_x;
        goal_msg.pose.pose.position.y = position_y;
        goal_msg.pose.pose.position.z = position_z;
        goal_msg.pose.pose.orientation.x = orientation_x;
        goal_msg.pose.pose.orientation.y = orientation_y;
        goal_msg.pose.pose.orientation.z = orientation_z;
        goal_msg.pose.pose.orientation.w = orientation_w;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&TETRA_SERVICE::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&TETRA_SERVICE::resultCallback, this, std::placeholders::_1);

        auto parameters = std::vector<rclcpp::Parameter>();
        parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_angular_vel", 0.785398));
        parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_min_angle", 0.785398));
        set_speed_parameter_client_->set_parameters(parameters);
        _pDynamic_param.m_bFlag_onetime = true;

        nav_to_pose_action_client->async_send_goal(goal_msg, send_goal_options);
    }

    bool Goto_Location_Command(
        const std::shared_ptr<interfaces::srv::GotoLocation::Request> request, 
        const std::shared_ptr<interfaces::srv::GotoLocation::Response> response)
    {
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        Clear_Costmap();
        LedToggleControl_Call(1, 3, 100, 3, 1);
        ToggleOn_Call(63);

        bResult = OpenLocationFile(request->location);
        response->pose_x  = _pGoal_pose.goal_positionX;
        response->pose_y  = _pGoal_pose.goal_positionY;
        response->pose_z  = _pGoal_pose.goal_positionZ;
        response->pose_qx = _pGoal_pose.goal_quarterX;
        response->pose_qy = _pGoal_pose.goal_quarterY;
        response->pose_qz = _pGoal_pose.goal_quarterZ;
        response->pose_qw = _pGoal_pose.goal_quarterW;
        
        if(_pRobot.m_iCallback_Charging_status <= 1 && (ar_tag_pose_.m_iApril_tag_id == -1 || ar_tag_pose_.m_transform_pose_x <= 0.5))
        {
            RCLCPP_INFO(get_logger(), "Goto Normal Loop !!");
            Set_goal(_pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY, _pGoal_pose.goal_positionZ,
                     _pGoal_pose.goal_quarterX, _pGoal_pose.goal_quarterY, _pGoal_pose.goal_quarterZ, _pGoal_pose.goal_quarterW);
            bResult = true;
        }
        else
        {
            m_iDocking_CommandMode = 10;
            bResult = true;
        }

        if(request->location == "HOME")
        {
            _pFlag_Value.m_bflag_ComebackHome = true;
        }

        response->command_result = bResult;
        return true;
    }

    bool Goto_Location2_Command(
        const std::shared_ptr<interfaces::srv::GotoLocation2::Request> request, 
        const std::shared_ptr<interfaces::srv::GotoLocation2::Response> response)
    {
        bool bResult = false;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        Clear_Costmap();
        LedToggleControl_Call(1, 3, 100, 3, 1);
        ToggleOn_Call(63);
        
        _pGoal_pose.goal_positionX = request->pose_x;
        _pGoal_pose.goal_positionY = request->pose_y;
        _pGoal_pose.goal_quarterX = request->pose_qx;
        _pGoal_pose.goal_quarterY = request->pose_qy;
        _pGoal_pose.goal_quarterZ = request->pose_qz;
        _pGoal_pose.goal_quarterW = request->pose_qw;
        
        if(_pRobot.m_iCallback_Charging_status <= 1 && (ar_tag_pose_.m_iApril_tag_id == -1 || ar_tag_pose_.m_transform_pose_x <= 0.5))
        {
            RCLCPP_INFO(get_logger(), "Goto Normal Loop !!");
            Set_goal(request->pose_x, request->pose_y, request->pose_z,
                     request->pose_qx, request->pose_qy, request->pose_qz, request->pose_qw);
            bResult = true;
        }
        else
        {
            m_iDocking_CommandMode = 10;
            bResult = true;
        }
        
        response->command_result = bResult;
        return true;
    }
    
    bool Goto_Cancel_Command(
        const std::shared_ptr<interfaces::srv::GotoCancel::Request> /*request*/, 
        const std::shared_ptr<interfaces::srv::GotoCancel::Response> response)
    {
        if (!this->nav_to_pose_action_client->action_server_is_ready()) 
        {
            RCLCPP_INFO(get_logger(), "Waiting for action server...");
        }

        nav_to_pose_action_client->async_cancel_all_goals();
        response->command_result = true;
        return true;
    }

    bool Set_Maxspeed_Command(
        const std::shared_ptr<interfaces::srv::SetMaxspeed::Request> request, 
        const std::shared_ptr<interfaces::srv::SetMaxspeed::Response> response)
    {
        auto parameters = std::vector<rclcpp::Parameter>();
        parameters.emplace_back(rclcpp::Parameter("FollowPath.desired_linear_vel", request->max_speed));
        set_speed_parameter_client_->set_parameters(parameters);

        _pDynamic_param.MAX_Linear_velocity = request->max_speed;
        RCLCPP_INFO(get_logger(), "Set_MAX_Linear_velocity = %.3f", _pDynamic_param.MAX_Linear_velocity);
        
        response->command_result = true;
        return true;
    }

    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        _pDynamic_param.m_bFlag_onetime = true;
        if(_pDynamic_param.m_bFlag_onetime)
        {
            if(feedback->distance_remaining < 0.1 && feedback->distance_remaining > 0)
            {
                _pDynamic_param.m_bFlag_onetime = false; 
                
                tf2::Quaternion goal_q(
                    _pGoal_pose.goal_quarterX,
                    _pGoal_pose.goal_quarterY,
                    _pGoal_pose.goal_quarterZ,
                    _pGoal_pose.goal_quarterW
                );
                tf2::Matrix3x3 goal_m(goal_q);
                double goal_roll, goal_pitch, goal_yaw;
                goal_m.getRPY(goal_roll, goal_pitch, goal_yaw);
                
                tf2::Quaternion robot_q(
                    footprint.orientation.x,
                    footprint.orientation.y,
                    footprint.orientation.z,
                    footprint.orientation.w
                );
                tf2::Matrix3x3 robot_m(robot_q);
                double robot_roll, robot_pitch, robot_yaw;
                robot_m.getRPY(robot_roll, robot_pitch, robot_yaw);

                double angle_diff = robot_yaw - goal_yaw;
                while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
                angle_diff = std::abs(angle_diff);
                
                auto parameters = std::vector<rclcpp::Parameter>();
                if(angle_diff <= 0.1)
                {
                    parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_angular_vel", 0.1));
                    parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_min_angle", 0.1));
                }
                else if(angle_diff <= 0.785398)
                {
                    parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_angular_vel", angle_diff));
                    parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_min_angle", angle_diff));
                }
                else
                {
                    parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_angular_vel", 0.785398));
                    parameters.emplace_back(rclcpp::Parameter("FollowPath.rotate_to_heading_min_angle", 0.785398));
                }
                set_speed_parameter_client_->set_parameters(parameters);
            }
        }
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        int ret = -1;
        std::lock_guard<std::mutex> lock(pose_mutex_);
        switch (result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:				
                RCLCPP_INFO(get_logger(), "NavigateToPose: Success!!!");
                _pRobot.m_iMovebase_Result = 3;
                movebase.data = 3;
                movebase_publisher->publish(movebase);
                LedToggleControl_Call(1, 3, 100, 3, 1);
                ToggleOn_Call(63);

                ex_iRetry_count = 0;

                if(_pFlag_Value.m_bflag_ComebackHome)
                {
                    ar_tag_pose_.m_iSelect_April_tag_id = m_dHome_ID;
                    m_iDocking_CommandMode = 1;
                    _pFlag_Value.m_bflag_ComebackHome = false;
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "NavigateToPose: Goal was aborted");
                _pRobot.m_iMovebase_Result = 4;
                movebase.data = 4;
                movebase_publisher->publish(movebase);
                LedToggleControl_Call(1, 10, 100, 10, 1);
                ToggleOn_Call(18);
                
                if(ex_iRetry_count < ex_iRetry_Max_count)
                {
                    RCLCPP_ERROR(get_logger(), "Retry Set_goal !");
                    Clear_Costmap();
                    Set_goal(_pGoal_pose.goal_positionX, _pGoal_pose.goal_positionY, _pGoal_pose.goal_positionZ,
                             _pGoal_pose.goal_quarterX, _pGoal_pose.goal_quarterY, _pGoal_pose.goal_quarterZ, _pGoal_pose.goal_quarterW);
                    ex_iRetry_count++;
                }
                else
                {
                    ex_iRetry_count = 0;
                    ret = std::system("~/screenshot_save.sh > /dev/null 2>&1 &");
                    (void)ret;
                }
                break;
            case rclcpp_action::ResultCode::CANCELED:				
                RCLCPP_ERROR(get_logger(), "NavigateToPose: Goal was canceled");
                _pRobot.m_iMovebase_Result = 1;
                movebase.data = 1;
                movebase_publisher->publish(movebase);
                LedToggleControl_Call(1, 10, 100, 10, 1);
                ToggleOn_Call(18);
                break;
            default:				
                RCLCPP_ERROR(get_logger(), "NavigateToPose: Unknown result code");
                _pRobot.m_iMovebase_Result = 0;
                movebase.data = 4;
                movebase_publisher->publish(movebase);
                ret = std::system("~/screenshot_save.sh > /dev/null 2>&1 &");
                (void)ret;
                break;
        }
    }

    // Client Functions
    void LedControl_Call(int id, int led_brightness) 
    {
        auto request = std::make_shared<interfaces::srv::LedControl::Request>();
        request->id = id;
        request->led_brightness = led_brightness;
        led_control_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::LedControl_callback, this, std::placeholders::_1));
    }

    void LedToggleControl_Call(int de_index, int light_accel, int led_high_brightness, int light_decel, int led_low_brightness) 
    {
        auto request = std::make_shared<interfaces::srv::LedToggleControl::Request>();
        request->de_index = de_index;
        request->light_accel = light_accel;
        request->led_high_brightness = led_high_brightness;
        request->light_decel = light_decel;
        request->led_low_brightness = led_low_brightness;
        led_toggle_control_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::LedToggleControl_callback, this, std::placeholders::_1));
    }

    void ToggleOn_Call(int id) 
    {
        auto request = std::make_shared<interfaces::srv::ToggleOn::Request>();
        request->id = id;
        toggle_on_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::ToggleOn_callback, this, std::placeholders::_1));
    }

    void ImuReset_Call() 
    {
        auto request = std::make_shared<interfaces::srv::ImuReset::Request>();
        imu_reset_client->async_send_request(
            request, std::bind(&TETRA_SERVICE::ImuReset_callback, this, std::placeholders::_1));
    }

    bool Clear_Costmap() 
    {
        if (!clear_entire_global_costmap_client->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_ERROR(this->get_logger(), "Clear global_Costmap Service not available.");
            return false;
        }
        if (!clear_entire_local_costmap_client->wait_for_service(std::chrono::seconds(1))) 
        {
            RCLCPP_ERROR(this->get_logger(), "Clear local_Costmap Service not available.");
            return false;
        }

        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        auto request2 = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        clear_entire_global_costmap_client->async_send_request(
            request, 
            [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture /*future*/) {
                RCLCPP_DEBUG(this->get_logger(), "Global costmap cleared.");
            }
        );
        clear_entire_local_costmap_client->async_send_request(
            request2, 
            [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture /*future*/) {
                RCLCPP_DEBUG(this->get_logger(), "Local costmap cleared.");
            }
        );
        return true;
    }

private:
    rclcpp::Client<interfaces::srv::LedControl>::SharedPtr led_control_client;
    rclcpp::Client<interfaces::srv::LedToggleControl>::SharedPtr led_toggle_control_client;
    rclcpp::Client<interfaces::srv::ToggleOn>::SharedPtr toggle_on_client;
    rclcpp::Client<interfaces::srv::ImuReset>::SharedPtr imu_reset_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr request_nomotion_update_client;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_local_costmap_client;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_global_costmap_client;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> set_speed_parameter_client_;

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

    std::mutex pose_mutex_;
    AR_TAG_POSE ar_tag_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto node = std::make_shared<TETRA_SERVICE>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    node->current_time = node->now();

    // LED On
    node->LedToggleControl_Call(1, 3, 100, 3, 1);
    node->ToggleOn_Call(63);

    printf("□□■■■■□□□■■■■■■□□■■■■■□□□■□□□□□■□■□□□□■■■■□□□■■■■■■□\n");
    printf("□■□□□□■□□■□□□□□□□■□□□□■□□■□□□□□■□■□□□■□□□□■□□■□□□□□□\n");
    printf("□■□□□□■□□■□□□□□□□■□□□□■□□■□□□□□■□■□□■□□□□□□□□■□□□□□□\n");
    printf("□■□□□□□□□■□□□□□□□■□□□□■□□□■□□□■□□■□□■□□□□□□□□■□□□□□□\n");
    printf("□□■■□□□□□■■■■■■□□■■■■■□□□□■□□□■□□■□□■□□□□□□□□■■■■■■□\n");
    printf("□□□□■■□□□■□□□□□□□■□□□□■□□□■□□□■□□■□□■□□□□□□□□■□□□□□□\n");
    printf("□□□□□□■□□■□□□□□□□■□□□□■□□□□■□■□□□■□□■□□□□□□□□■□□□□□□\n");
    printf("□■□□□□■□□■□□□□□□□■□□□□■□□□□■□■□□□■□□■□□□□□□□□■□□□□□□\n");
    printf("□■□□□□■□□■□□□□□□□■□□□□■□□□□□■□□□□■□□□■□□□□■□□■□□□□□□\n");
    printf("□□■■■■□□□■■■■■■□□■□□□□■□□□□□■□□□□■□□□□■■■■□□□■■■■■■□\n");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}

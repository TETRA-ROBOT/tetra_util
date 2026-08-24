#include "tetra_service/tetra_service.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <csignal>
#include <algorithm>

using namespace std::placeholders;

namespace fs = std::filesystem;

static std::atomic<bool> g_stop_requested(false);

void signal_handler(int signal)
{
    g_stop_requested = true;
    RCLCPP_INFO(rclcpp::get_logger("tetra_service"), "Signal %d received. Preparing to shutdown...", signal);
}

TETRA_SERVICE::TETRA_SERVICE() : Node("tetra_service")
{
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    initializeParameters();
}

void TETRA_SERVICE::initializeParameters()
{
    std::string robot_model = "default";
    this->declare_parameter<std::string>("robot_model", "default");
    this->get_parameter("robot_model", robot_model);

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

    RCLCPP_INFO(this->get_logger(),
                "Initialized tetra_service | Model: %s | Home_ID: %d | Home_X: %.2f | MaxVel: %.2f | cmd_vel: %s",
                robot_model.c_str(), m_dHome_ID, _pHomePose.HOME_dPOSITION_X,
                _pDynamic_param.MAX_Linear_velocity, cmd_vel_topic.c_str());

    initializeRosEntities(cmd_vel_topic, scan_topic, scan2_topic, scan_merge_topic, use_rtabmap_initialpose);
}

void TETRA_SERVICE::initializeRosEntities(
    const std::string& cmd_vel_topic,
    const std::string& scan_topic,
    const std::string& scan2_topic,
    const std::string& scan_merge_topic,
    bool use_rtabmap_initialpose)
{
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
    docking_progress_publisher = this->create_publisher<std_msgs::msg::Int32>("docking_progress", 10);

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

    // Service Servers
    docking_cmd_srv = create_service<interfaces::srv::DockingControl>(
        "docking_cmd", std::bind(&TETRA_SERVICE::Docking_Command, this, _1, _2));

    docking_stop_cmd_srv = create_service<interfaces::srv::DockingStop>(
        "docking_stop_cmd", std::bind(&TETRA_SERVICE::Docking_Stop_Command, this, _1, _2));

    save_map_cmd_srv = create_service<interfaces::srv::SaveMap>(
        "save_map_cmd", std::bind(&TETRA_SERVICE::Save_Map_Command, this, _1, _2));

    delete_map_cmd_srv = create_service<interfaces::srv::DeleteMap>(
        "delete_map_cmd", std::bind(&TETRA_SERVICE::Delete_Map_Command, this, _1, _2));

    run_mapping_cmd_srv = create_service<interfaces::srv::RunMapping>(
        "run_mapping_cmd", std::bind(&TETRA_SERVICE::Mapping_Mode_Command, this, _1, _2));

    run_3d_mapping_cmd_srv = create_service<interfaces::srv::Run3DMapping>(
        "run_3d_mapping_cmd", std::bind(&TETRA_SERVICE::Mapping_3D_Mode_Command, this, _1, _2));

    run_navigation_cmd_srv = create_service<interfaces::srv::RunNavigation>(
        "run_navigation_cmd", std::bind(&TETRA_SERVICE::Navigation_Mode_Command, this, _1, _2));

    get_information_cmd_srv = create_service<interfaces::srv::GetInformation>(
        "get_information_cmd", std::bind(&TETRA_SERVICE::Get_Information_Command, this, _1, _2));

    set_location_cmd_srv = create_service<interfaces::srv::SetLocation>(
        "set_location_cmd", std::bind(&TETRA_SERVICE::Set_Location_Command, this, _1, _2));

    goto_location_cmd_srv = create_service<interfaces::srv::GotoLocation>(
        "goto_location_cmd", std::bind(&TETRA_SERVICE::Goto_Location_Command, this, _1, _2));

    goto_location2_cmd_srv = create_service<interfaces::srv::GotoLocation2>(
        "goto_location2_cmd", std::bind(&TETRA_SERVICE::Goto_Location2_Command, this, _1, _2));

    goto_cancel_cmd_srv = create_service<interfaces::srv::GotoCancel>(
        "goto_cancel_cmd", std::bind(&TETRA_SERVICE::Goto_Cancel_Command, this, _1, _2));

    set_maxspeed_cmd_srv = create_service<interfaces::srv::SetMaxspeed>(
        "set_maxspeed_cmd", std::bind(&TETRA_SERVICE::Set_Maxspeed_Command, this, _1, _2));

    // Service & Action Clients
    led_control_client = this->create_client<interfaces::srv::LedControl>("led_cmd");
    led_toggle_control_client = this->create_client<interfaces::srv::LedToggleControl>("led_toggle_cmd");
    toggle_on_client = this->create_client<interfaces::srv::ToggleOn>("turn_on_cmd");
    imu_reset_client = this->create_client<interfaces::srv::ImuReset>("all_data_reset");
    request_nomotion_update_client = this->create_client<std_srvs::srv::Empty>("request_nomotion_update");
    clear_entire_global_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
    clear_entire_local_costmap_client = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
    set_pose_client_ = this->create_client<robot_localization::srv::SetPose>("set_pose");
    set_speed_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "controller_server");
    nav_to_pose_action_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Timers
    timer_ = this->create_wall_timer(10ms, std::bind(&TETRA_SERVICE::DockingThread_function, this));
    TF_timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::TF_CALC_Timer, this));
    Footprint_timer_ = this->create_wall_timer(20ms, std::bind(&TETRA_SERVICE::Footprint_CALC_Timer, this));
    pose_reset_monitor_timer_ = this->create_wall_timer(100ms, std::bind(&TETRA_SERVICE::PoseResetMonitor_Timer, this));
}

// ─────────────────────── Subscribers Callbacks ───────────────────────

void TETRA_SERVICE::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if (joy->buttons.size() > 7) {
        if (joy->buttons[6]) { // LT button: Auto Docking
            auto request = std::make_shared<interfaces::srv::DockingControl::Request>();
            auto response = std::make_shared<interfaces::srv::DockingControl::Response>();
            request->id = 0;
            request->mode = 1;
            Docking_Command(request, response);
        }
        if (joy->buttons[7]) { // RT button: Docking Stop
            auto request = std::make_shared<interfaces::srv::DockingStop::Request>();
            auto response = std::make_shared<interfaces::srv::DockingStop::Response>();
            Docking_Stop_Command(request, response);
        }
    }
}

void TETRA_SERVICE::ApriltagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    int iTag_id = -1;
    double dX = 0.0, dY = 0.0, dZ = 0.0;
    double dOri_x = 0.0, dOri_y = 0.0, dOri_z = 0.0, dOri_w = 1.0;

    int iArray_size = msg->detections.size();
    if (iArray_size > 0) {
        for (const auto &detection : msg->detections) {
            _pAR_tag_pose.m_iApril_tag_id = detection.id;
        }
        std::string strTF_frame_name = _pAR_tag_pose.m_strFamily + std::to_string(_pAR_tag_pose.m_iApril_tag_id);
        if (!tf2_buffer_->canTransform("camera", strTF_frame_name, tf2::TimePointZero)) {
            return;
        }
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf2_buffer_->lookupTransform("camera", strTF_frame_name, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            return;
        }

        _pAR_tag_pose.m_dApril_tag_pose_x = transformStamped.transform.translation.x;
        _pAR_tag_pose.m_dApril_tag_pose_y = transformStamped.transform.translation.y;
        _pAR_tag_pose.m_dApril_tag_pose_z = transformStamped.transform.translation.z;
        _pAR_tag_pose.m_dApril_tag_orientation_x = transformStamped.transform.rotation.x;
        _pAR_tag_pose.m_dApril_tag_orientation_y = transformStamped.transform.rotation.y;
        _pAR_tag_pose.m_dApril_tag_orientation_z = transformStamped.transform.rotation.z;
        _pAR_tag_pose.m_dApril_tag_orientation_w = transformStamped.transform.rotation.w;

        tf2::Quaternion q(
            _pAR_tag_pose.m_dApril_tag_orientation_x,
            _pAR_tag_pose.m_dApril_tag_orientation_y,
            _pAR_tag_pose.m_dApril_tag_orientation_z,
            _pAR_tag_pose.m_dApril_tag_orientation_w);
        tf2::Matrix3x3 m(q);
        m.getRPY(_pAR_tag_pose.m_dApril_tag_roll,
                 _pAR_tag_pose.m_dApril_tag_pitch,
                 _pAR_tag_pose.m_dApril_tag_yaw);

        _pAR_tag_pose.m_transform_pose_x = _pAR_tag_pose.m_dApril_tag_pose_z;
        _pAR_tag_pose.m_transform_pose_y = -_pAR_tag_pose.m_dApril_tag_pose_x;
        _pAR_tag_pose.m_dPositioning_Angle = -_pAR_tag_pose.m_dApril_tag_pitch;
        _pAR_tag_pose.m_target_yaw = _pAR_tag_pose.m_dApril_tag_pitch;
    }
}

void TETRA_SERVICE::SickCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int num_readings = scan->ranges.size();
    float fDistance_center = 0.55f;
    float fDistance_side = 0.35f;

    _pFlag_Value.m_bFlag_Obstacle_Center = false;
    _pFlag_Value.m_bFlag_Obstacle_Right = false;
    _pFlag_Value.m_bFlag_Obstacle_Left = false;

    for (int i = 0; i < num_readings; i++) {
        float r = scan->ranges[i];
        if (std::isnan(r) || std::isinf(r)) continue;

        if (i >= (num_readings / 2 - 30) && i <= (num_readings / 2 + 30)) {
            if (r < fDistance_center && r > 0.05f) _pFlag_Value.m_bFlag_Obstacle_Center = true;
        } else if (i < (num_readings / 2 - 30) && i >= 0) {
            if (r < fDistance_side && r > 0.05f) _pFlag_Value.m_bFlag_Obstacle_Right = true;
        } else if (i > (num_readings / 2 + 30) && i < num_readings) {
            if (r < fDistance_side && r > 0.05f) _pFlag_Value.m_bFlag_Obstacle_Left = true;
        }
    }
}

void TETRA_SERVICE::CygbotCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int num_readings = scan->ranges.size();
    float fDistance_center = 0.45f;
    _pFlag_Value.m_bFlag_Obstacle_cygbot = false;

    for (int i = 0; i < num_readings; i++) {
        float r = scan->ranges[i];
        if (std::isnan(r) || std::isinf(r)) continue;
        if (r < fDistance_center && r > 0.05f) {
            _pFlag_Value.m_bFlag_Obstacle_cygbot = true;
            break;
        }
    }
}

void TETRA_SERVICE::LaserScanMergeCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int num_readings = scan->ranges.size();
    float fDistance_front = 0.55f;
    float fDistance_rear = 0.35f;
    _pFlag_Value.m_bFlag_MergeScan_Front = false;
    _pFlag_Value.m_bFlag_MergeScan_Rear = false;
    _pFlag_Value.m_bFlag_MergeScan_Left = false;
    _pFlag_Value.m_bFlag_MergeScan_Right = false;

    for (int i = 0; i < num_readings; i++) {
        float r = scan->ranges[i];
        if (std::isnan(r) || std::isinf(r)) continue;
        if (i >= (num_readings / 2 - 45) && i <= (num_readings / 2 + 45)) {
            if (r < fDistance_front && r > 0.05f) _pFlag_Value.m_bFlag_MergeScan_Front = true;
        }
        if (i < 45 || i > num_readings - 45) {
            if (r < fDistance_rear && r > 0.05f) _pFlag_Value.m_bFlag_MergeScan_Rear = true;
        }
    }
}

void TETRA_SERVICE::BumperCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot.m_iCallback_Bumper = msg->data;
    if (_pRobot.m_iCallback_Bumper != 0) {
        _pFlag_Value.m_bumperhit_flag = true;
        LedControl_Call(2, 255); // Red alert LED
    }
}

void TETRA_SERVICE::EMGCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot.m_iCallback_EMG = msg->data;
    if (_pRobot.m_iCallback_EMG != 0) {
        _pFlag_Value.m_emgpush_flag = true;
    }
}

void TETRA_SERVICE::BatteryCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot.m_iCallback_Battery = msg->data;
}

void TETRA_SERVICE::DockingStatusCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot.m_iCallback_Charging_status = msg->data;
}

void TETRA_SERVICE::ConveyorSensorCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot.m_iConveyor_Sensor_info = msg->data;
}

void TETRA_SERVICE::ConveyorMovementCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    _pRobot.m_iConveyor_movement = msg->data;
    _pRobot.LIFT_MOVEMENT = msg->data;
}

void TETRA_SERVICE::AMCL_PaticleCloud_Callback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
{
    (void)msg;
}

void TETRA_SERVICE::cmd_vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    _pDynamic_param.m_linear_vel = msg->linear.x;
    _pDynamic_param.m_angular_vel = msg->angular.z;
}

void TETRA_SERVICE::Initialpose_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    (void)msg;
    _pFlag_Value.m_bFlag_Initialpose = true;
}

void TETRA_SERVICE::map_Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    (void)msg;
    m_bActive_map = true;
}

// ──────────────── Docking & Lift Tracking Methods ────────────────────

double TETRA_SERVICE::rotation_movement()
{
    if (m_iRotation_Mode == 1) return 0.12;  // Left
    if (m_iRotation_Mode == 2) return -0.12; // Right
    return 0.10;
}

void TETRA_SERVICE::publish_docking_progress(int progress)
{
    std_msgs::msg::Int32 p_msg;
    p_msg.data = progress;
    docking_progress_publisher->publish(p_msg);
}

void TETRA_SERVICE::docking_exit()
{
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_publisher->publish(stop_cmd);
    m_iDocking_CommandMode = DOCK_NONE;
    m_iNoMarker_cnt = 0;
    m_iYawAlign_cnt = 0;
    m_iStepBack_cnt = 0;
    _pFlag_Value.m_bfalg_DockingExit = false;
    _pRobot.m_bFalg_DockingExit = false;
    RCLCPP_INFO(this->get_logger(), "Docking loop exited.");
}

bool TETRA_SERVICE::charging_station_tracking(int marker_id)
{
    geometry_msgs::msg::Twist cmd;

    if (_pAR_tag_pose.m_iApril_tag_id == marker_id) {
        m_iNoMarker_cnt = 0;
        double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);

        if (dist > 0.55 && dist < 2.5) {
            cmd.linear.x = std::clamp(dist * 0.2, 0.05, 0.35);
            cmd.angular.z = std::clamp(-1.0 * std::atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.25, -0.4, 0.4);
            cmd_vel_publisher->publish(cmd);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "[ChargingStation_tracking] Reached 0.55m -> Transition to Yaw Align (Mode 3)");
            m_iDocking_CommandMode = DOCK_CHARGE_YAW_ALIGN;
        }
    } else {
        m_iNoMarker_cnt++;
        if (m_iNoMarker_cnt > 300) { // Timeout 3 sec
            RCLCPP_WARN(this->get_logger(), "[ChargingStation_tracking] Marker scan timeout!");
            m_iDocking_CommandMode = DOCK_CHARGE_FAIL;
        } else {
            cmd.angular.z = rotation_movement();
            cmd_vel_publisher->publish(cmd);
        }
    }
    return true;
}

bool TETRA_SERVICE::charging_station_yaw_tracking()
{
    geometry_msgs::msg::Twist cmd;
    double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);
    double target_yaw = _pAR_tag_pose.m_target_yaw;

    // Angle within +- 1.0 deg (0.0175 rad) -> perfect alignment!
    if (std::abs(target_yaw) <= 0.0175) {
        RCLCPP_INFO(this->get_logger(), "[ChargingStation_Yaw_tracking] Yaw aligned (yaw: %.2f°) -> Mode 4", target_yaw * 180.0 / M_PI);
        m_iDocking_CommandMode = DOCK_CHARGE_TRACKING_2;
        m_iYawAlign_cnt = 0;
        m_iStepBack_cnt = 0;
        return true;
    }

    // Step 1: Rotate in-place to align heading with marker normal
    if (m_iYawAlign_cnt < 25) {
        m_iYawAlign_cnt++;
        cmd.linear.x = 0.0;
        cmd.angular.z = std::clamp(-1.0 * target_yaw * 1.2, -0.3, 0.3);
        cmd_vel_publisher->publish(cmd);
        return true;
    }

    // Step 2: Step-back to secure center approach space
    if (m_iStepBack_cnt < 20) {
        m_iStepBack_cnt++;
        if (_pFlag_Value.m_bFlag_MergeScan_Rear) {
            cmd.linear.x = 0.0;
        } else {
            cmd.linear.x = -1.0 * std::clamp(dist * 0.2, 0.05, 0.15);
        }
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        return true;
    }

    // Reset counters and loop back to tracking_1 to re-approach from center line
    m_iYawAlign_cnt = 0;
    m_iStepBack_cnt = 0;
    m_iRotation_Mode = (target_yaw > 0) ? 2 : 1;
    m_iDocking_CommandMode = DOCK_CHARGE_TRACKING_1;
    RCLCPP_INFO(this->get_logger(), "[ChargingStation_Yaw_tracking] Step-back complete -> Re-approaching in Mode 2");
    return true;
}

bool TETRA_SERVICE::charging_station_tracking2(int marker_id)
{
    geometry_msgs::msg::Twist cmd;

    if (_pRobot.m_iCallback_Charging_status < 2 && _pRobot.m_iCallback_Bumper == 0) {
        double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);

        cmd.linear.x = std::clamp((dist / 2.0) * 0.08, 0.02, 0.08); // creeping max 0.08 m/s
        cmd.angular.z = std::clamp(-1.0 * std::atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.5, -0.2, 0.2);

        if (dist <= 0.22 || _pRobot.m_iCallback_Charging_status == 2) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            m_iDocking_CommandMode = DOCK_CHARGE_APPROACH;
            RCLCPP_INFO(this->get_logger(), "[ChargingStation_tracking2] Close to station -> Final Approach (Mode 5)");
        } else {
            cmd_vel_publisher->publish(cmd);
        }
    } else {
        m_iDocking_CommandMode = DOCK_CHARGE_APPROACH;
    }
    return true;
}

bool TETRA_SERVICE::approach_station_move()
{
    geometry_msgs::msg::Twist cmd;
    if (_pRobot.m_iCallback_Charging_status == 2) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        m_iDocking_CommandMode = DOCK_CHARGE_FINISH;
        RCLCPP_INFO(this->get_logger(), "[Approach_Station2Move] Charging contact confirmed! -> Mode 6");
    } else {
        cmd.linear.x = 0.03; // Gentle creeping push into terminal
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
    }
    return true;
}

bool TETRA_SERVICE::depart_station_move()
{
    geometry_msgs::msg::Twist cmd;
    m_iDepart_cnt++;

    if (m_iDepart_cnt < 20) { // 2.0 sec @ 10Hz
        cmd.linear.x = -0.10;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        m_iDepart_cnt = 0;
        m_iDocking_CommandMode = DOCK_NONE;
        RCLCPP_INFO(this->get_logger(), "Depart station completed.");
    }
    return true;
}

// ──────────────── Lift Station Docking Methods ───────────────────────

bool TETRA_SERVICE::lift_station_tracking(int marker_id)
{
    geometry_msgs::msg::Twist cmd;
    if (_pAR_tag_pose.m_iApril_tag_id == marker_id) {
        m_iNoMarker_cnt = 0;
        double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);

        if (dist > 1.3 && dist < 3.0) {
            cmd.linear.x = std::clamp(dist * 0.18, 0.05, 0.30);
            cmd.angular.z = std::clamp(-1.0 * std::atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.3, -0.35, 0.35);
            cmd_vel_publisher->publish(cmd);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            m_iDocking_CommandMode = DOCK_LIFT_LOAD_YAW_ALIGN;
            RCLCPP_INFO(this->get_logger(), "[LiftStation_tracking] Reached 1.3m -> Lift Yaw Align (Mode 13)");
        }
    } else {
        m_iNoMarker_cnt++;
        if (m_iNoMarker_cnt > 350) {
            m_iDocking_CommandMode = DOCK_LIFT_LOAD_FAIL;
        } else {
            cmd.angular.z = rotation_movement();
            cmd_vel_publisher->publish(cmd);
        }
    }
    return true;
}

bool TETRA_SERVICE::lift_station_yaw_tracking()
{
    geometry_msgs::msg::Twist cmd;
    double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);
    double target_yaw = _pAR_tag_pose.m_target_yaw;

    if (std::abs(target_yaw) <= 0.0175) {
        m_iDocking_CommandMode = DOCK_LIFT_LOAD_TRACKING_2;
        m_iYawAlign_cnt = 0;
        m_iStepBack_cnt = 0;
        RCLCPP_INFO(this->get_logger(), "[LiftStation_Yaw_tracking] Lift Yaw aligned -> Mode 14");
        return true;
    }

    if (m_iYawAlign_cnt < 25) {
        m_iYawAlign_cnt++;
        cmd.linear.x = 0.0;
        cmd.angular.z = std::clamp(-1.0 * target_yaw * 1.2, -0.25, 0.25);
        cmd_vel_publisher->publish(cmd);
        return true;
    }

    if (m_iStepBack_cnt < 20) {
        m_iStepBack_cnt++;
        cmd.linear.x = -1.0 * std::clamp(dist * 0.15, 0.05, 0.12);
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        return true;
    }

    m_iYawAlign_cnt = 0;
    m_iStepBack_cnt = 0;
    m_iDocking_CommandMode = DOCK_LIFT_LOAD_TRACKING_1;
    return true;
}

bool TETRA_SERVICE::lift_station_tracking2(int marker_id)
{
    geometry_msgs::msg::Twist cmd;
    double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);

    cmd.linear.x = std::clamp((dist / 2.0) * 0.06, 0.02, 0.06);
    cmd.angular.z = std::clamp(-1.0 * std::atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.5, -0.15, 0.15);

    if (dist <= 0.30 || _pRobot.m_iCallback_Bumper != 0) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        m_iDocking_CommandMode = DOCK_LIFT_LOAD_EXECUTE;
        RCLCPP_INFO(this->get_logger(), "[LiftStation_tracking2] Tunnel Entry Complete -> Executing Lift UP (Mode 15)");
    } else {
        cmd_vel_publisher->publish(cmd);
    }
    return true;
}

bool TETRA_SERVICE::lift_check()
{
    if (_pRobot.m_iConveyor_Sensor_info > 0) {
        RCLCPP_WARN(this->get_logger(), "Robot already has payload on lift! Cannot load.");
        m_iDocking_CommandMode = DOCK_NONE;
        return false;
    }
    m_iDocking_CommandMode = DOCK_LIFT_LOAD_TRACKING_2;
    return true;
}

bool TETRA_SERVICE::un_lift_station_tracking(int marker_id)
{
    geometry_msgs::msg::Twist cmd;
    if (_pAR_tag_pose.m_iApril_tag_id == marker_id) {
        m_iNoMarker_cnt = 0;
        double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);

        if (dist > 1.3 && dist < 3.0) {
            cmd.linear.x = std::clamp(dist * 0.18, 0.05, 0.30);
            cmd.angular.z = std::clamp(-1.0 * std::atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.3, -0.35, 0.35);
            cmd_vel_publisher->publish(cmd);
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_publisher->publish(cmd);
            m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_YAW_ALIGN;
        }
    } else {
        m_iNoMarker_cnt++;
        if (m_iNoMarker_cnt > 350) {
            m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_FAIL;
        } else {
            cmd.angular.z = rotation_movement();
            cmd_vel_publisher->publish(cmd);
        }
    }
    return true;
}

bool TETRA_SERVICE::un_lift_station_yaw_tracking()
{
    geometry_msgs::msg::Twist cmd;
    double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);
    double target_yaw = _pAR_tag_pose.m_target_yaw;

    if (std::abs(target_yaw) <= 0.0175) {
        m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_TRACKING_2;
        m_iYawAlign_cnt = 0;
        m_iStepBack_cnt = 0;
        return true;
    }

    if (m_iYawAlign_cnt < 25) {
        m_iYawAlign_cnt++;
        cmd.linear.x = 0.0;
        cmd.angular.z = std::clamp(-1.0 * target_yaw * 1.2, -0.25, 0.25);
        cmd_vel_publisher->publish(cmd);
        return true;
    }

    if (m_iStepBack_cnt < 20) {
        m_iStepBack_cnt++;
        cmd.linear.x = -1.0 * std::clamp(dist * 0.15, 0.05, 0.12);
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        return true;
    }

    m_iYawAlign_cnt = 0;
    m_iStepBack_cnt = 0;
    m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_TRACKING_1;
    return true;
}

bool TETRA_SERVICE::un_lift_station_tracking2(int marker_id)
{
    geometry_msgs::msg::Twist cmd;
    double dist = std::hypot(_pAR_tag_pose.m_transform_pose_x, _pAR_tag_pose.m_transform_pose_y);

    cmd.linear.x = std::clamp((dist / 2.0) * 0.06, 0.02, 0.06);
    cmd.angular.z = std::clamp(-1.0 * std::atan2(_pAR_tag_pose.m_transform_pose_y, _pAR_tag_pose.m_transform_pose_x) / 1.5, -0.15, 0.15);

    if (dist <= 0.30 || _pRobot.m_iCallback_Bumper != 0) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_EXECUTE;
        RCLCPP_INFO(this->get_logger(), "[Un_LiftStation_tracking2] Drop-off Entry Complete -> Executing Lift DOWN (Mode 25)");
    } else {
        cmd_vel_publisher->publish(cmd);
    }
    return true;
}

bool TETRA_SERVICE::un_lift_check()
{
    if (_pRobot.m_iConveyor_Sensor_info == 0) {
        RCLCPP_WARN(this->get_logger(), "Robot has no payload to unload!");
        m_iDocking_CommandMode = DOCK_NONE;
        return false;
    }
    m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_TRACKING_2;
    return true;
}

bool TETRA_SERVICE::depart_lift_move(int mode)
{
    geometry_msgs::msg::Twist cmd;
    m_iDepart_cnt++;

    double max_cnt = (mode == 1) ? 50 : 120; // 5 sec (-0.3m/s) or 12 sec (-0.12m/s)
    double speed = (mode == 1) ? -0.25 : -0.12;

    if (m_iDepart_cnt < max_cnt) {
        cmd.linear.x = speed;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd);
        m_iDepart_cnt = 0;
        m_iDocking_CommandMode = DOCK_NONE;
        RCLCPP_INFO(this->get_logger(), "Depart lift station completed.");
    }
    return true;
}

// ─────────────────────── Docking State Machine ───────────────────────

void TETRA_SERVICE::DockingThread_function()
{
    if (m_iDocking_CommandMode == DOCK_NONE) return;

    publish_docking_progress(m_iDocking_CommandMode);

    // Safety checks: Bumper / EMG (except during intentional low-speed contact)
    if (_pFlag_Value.m_emgpush_flag ||
        (_pFlag_Value.m_bumperhit_flag && m_iDocking_CommandMode != DOCK_CHARGE_APPROACH &&
         m_iDocking_CommandMode != DOCK_LIFT_LOAD_TRACKING_2 && m_iDocking_CommandMode != DOCK_LIFT_UNLOAD_TRACKING_2)) {
        docking_exit();
        RCLCPP_WARN(this->get_logger(), "Docking aborted due to safety stop (Bumper/EMG)!");
        return;
    }

    switch (m_iDocking_CommandMode) {
        // ── Charging Station Docking (Modes 1~10) ──
        case DOCK_CHARGE_START: // 1
            LedToggleControl_Call(1, 3, 100, 3, 100);
            ToggleOn_Call(63);
            m_iDocking_CommandMode = DOCK_CHARGE_TRACKING_1;
            break;

        case DOCK_CHARGE_TRACKING_1: // 2
            charging_station_tracking(_pAR_tag_pose.m_iSelect_April_tag_id);
            break;

        case DOCK_CHARGE_YAW_ALIGN: // 3
            charging_station_yaw_tracking();
            break;

        case DOCK_CHARGE_TRACKING_2: // 4
            charging_station_tracking2(_pAR_tag_pose.m_iSelect_April_tag_id);
            break;

        case DOCK_CHARGE_APPROACH: // 5
            approach_station_move();
            break;

        case DOCK_CHARGE_FINISH: // 6
            RCLCPP_INFO(this->get_logger(), "TETRA POSE Reset & Charging Station Docking Complete!");
            set_robot_localization_pose(_pHomePose.HOME_dPOSITION_X, _pHomePose.HOME_dPOSITION_Y, 0.0);
            LedToggleControl_Call(1, 5, 100, 5, 1);
            ToggleOn_Call(63);
            m_iDocking_CommandMode = DOCK_NONE;
            break;

        case DOCK_CHARGE_AUTO_PREVIEW: // 7
            if (std::abs(_pAR_tag_pose.m_dPositioning_Angle) <= 0.0175) {
                m_iDocking_CommandMode = DOCK_CHARGE_TRACKING_1;
            } else {
                m_iDocking_CommandMode = DOCK_CHARGE_TRACKING_1;
            }
            break;

        case DOCK_CHARGE_FAIL: // 9
            RCLCPP_WARN(this->get_logger(), "Charging Station Docking FAILED!");
            LedToggleControl_Call(1, 10, 100, 10, 1);
            ToggleOn_Call(18);
            docking_exit();
            break;

        case DOCK_CHARGE_DEPART: // 10
            depart_station_move();
            break;

        // ── Lift Station Loading Docking (Modes 11~19) ──
        case DOCK_LIFT_LOAD_START: // 11
            LedToggleControl_Call(1, 3, 100, 3, 100);
            ToggleOn_Call(63);
            m_iDocking_CommandMode = DOCK_LIFT_LOAD_TRACKING_1;
            break;

        case DOCK_LIFT_LOAD_TRACKING_1: // 12
            lift_station_tracking(_pAR_tag_pose.m_iSelect_April_tag_id);
            break;

        case DOCK_LIFT_LOAD_YAW_ALIGN: // 13
            lift_station_yaw_tracking();
            break;

        case DOCK_LIFT_LOAD_TRACKING_2: // 14
            lift_station_tracking2(_pAR_tag_pose.m_iSelect_April_tag_id);
            break;

        case DOCK_LIFT_LOAD_EXECUTE: // 15
            // Lift UP action
            ToggleOn_Call(1);
            m_iDocking_CommandMode = DOCK_LIFT_LOAD_DEPART;
            break;

        case DOCK_LIFT_LOAD_DEPART: // 17
            depart_lift_move(0);
            break;

        case DOCK_LIFT_LOAD_FAIL: // 19
            RCLCPP_WARN(this->get_logger(), "Lift Loading Docking FAILED!");
            docking_exit();
            break;

        // ── Lift Station Unloading Docking (Modes 21~29) ──
        case DOCK_LIFT_UNLOAD_START: // 21
            LedToggleControl_Call(1, 3, 100, 3, 100);
            ToggleOn_Call(63);
            m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_TRACKING_1;
            break;

        case DOCK_LIFT_UNLOAD_TRACKING_1: // 22
            un_lift_station_tracking(_pAR_tag_pose.m_iSelect_April_tag_id);
            break;

        case DOCK_LIFT_UNLOAD_YAW_ALIGN: // 23
            un_lift_station_yaw_tracking();
            break;

        case DOCK_LIFT_UNLOAD_TRACKING_2: // 24
            un_lift_station_tracking2(_pAR_tag_pose.m_iSelect_April_tag_id);
            break;

        case DOCK_LIFT_UNLOAD_EXECUTE: // 25
            // Lift DOWN action
            ToggleOn_Call(0);
            m_iDocking_CommandMode = DOCK_LIFT_UNLOAD_DEPART;
            break;

        case DOCK_LIFT_UNLOAD_DEPART: // 27
            depart_lift_move(1);
            break;

        case DOCK_LIFT_UNLOAD_FAIL: // 29
            RCLCPP_WARN(this->get_logger(), "Lift Unloading Docking FAILED!");
            docking_exit();
            break;

        default:
            break;
    }
}

// ─────────────────────── TF & Position Timers ────────────────────────

void TETRA_SERVICE::TF_CALC_Timer()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf2_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        _pTF_pose.poseTFx = transformStamped.transform.translation.x;
        _pTF_pose.poseTFy = transformStamped.transform.translation.y;
        _pTF_pose.poseTFz = transformStamped.transform.translation.z;
        _pTF_pose.poseTFqx = transformStamped.transform.rotation.x;
        _pTF_pose.poseTFqy = transformStamped.transform.rotation.y;
        _pTF_pose.poseTFqz = transformStamped.transform.rotation.z;
        _pTF_pose.poseTFqw = transformStamped.transform.rotation.w;

        tf2::Quaternion q(
            _pTF_pose.poseTFqx, _pTF_pose.poseTFqy, _pTF_pose.poseTFqz, _pTF_pose.poseTFqw);
        tf2::Matrix3x3 m(q);
        m.getRPY(_pTF_pose.pose_euler_roll, _pTF_pose.pose_euler_pitch, _pTF_pose.pose_euler_yaw);
        _pTF_pose.theta_deg = _pTF_pose.pose_euler_yaw * 180.0 / M_PI;
    } catch (tf2::TransformException& ex) {
        // TF not available yet
    }
}

void TETRA_SERVICE::Footprint_CALC_Timer()
{
    geometry_msgs::msg::Pose footprint_msg;
    footprint_msg.position.x = _pTF_pose.poseTFx;
    footprint_msg.position.y = _pTF_pose.poseTFy;
    footprint_msg.position.z = _pTF_pose.poseTFz;
    footprint_msg.orientation.x = _pTF_pose.poseTFqx;
    footprint_msg.orientation.y = _pTF_pose.poseTFqy;
    footprint_msg.orientation.z = _pTF_pose.poseTFqz;
    footprint_msg.orientation.w = _pTF_pose.poseTFqw;
    footprint_publisher->publish(footprint_msg);
}

void TETRA_SERVICE::PoseResetMonitor_Timer()
{
    if (_pFlag_Value.m_bFlag_Initialpose) {
        _pFlag_Value.m_bFlag_Initialpose = false;
        std_msgs::msg::Int32 reset_msg;
        reset_msg.data = 1;
        pose_reset_publisher->publish(reset_msg);
    }
}

// ─────────────────────── Service Servers Callbacks ────────────────────

void TETRA_SERVICE::Docking_Command(
    const std::shared_ptr<interfaces::srv::DockingControl::Request> request,
    std::shared_ptr<interfaces::srv::DockingControl::Response> response)
{
    _pAR_tag_pose.m_iSelect_April_tag_id = request->id;
    m_iDocking_CommandMode = request->mode;
    response->command_result = true;
    RCLCPP_INFO(this->get_logger(), "Docking command received: Tag ID=%d, Mode=%d", request->id, request->mode);
}

void TETRA_SERVICE::Docking_Stop_Command(
    const std::shared_ptr<interfaces::srv::DockingStop::Request> request,
    std::shared_ptr<interfaces::srv::DockingStop::Response> response)
{
    (void)request;
    docking_exit();
    response->command_result = true;
    RCLCPP_INFO(this->get_logger(), "Docking stopped by command.");
}

void TETRA_SERVICE::Save_Map_Command(
    const std::shared_ptr<interfaces::srv::SaveMap::Request> request,
    std::shared_ptr<interfaces::srv::SaveMap::Response> response)
{
    std::string map_name = request->map_name;
    if (map_name.empty()) map_name = "map";

    std::string cmd = "ros2 run nav2_map_server map_saver_cli -f ~/maps/" + map_name + " > /dev/null 2>&1 &";
    run_process_async(cmd);
    response->command_result = true;
    RCLCPP_INFO(this->get_logger(), "Save map requested: %s", map_name.c_str());
}

void TETRA_SERVICE::Delete_Map_Command(
    const std::shared_ptr<interfaces::srv::DeleteMap::Request> request,
    std::shared_ptr<interfaces::srv::DeleteMap::Response> response)
{
    std::string map_file = "/home/tetra/maps/" + request->map_name + ".yaml";
    std::string pgm_file = "/home/tetra/maps/" + request->map_name + ".pgm";

    try {
        if (fs::exists(map_file)) fs::remove(map_file);
        if (fs::exists(pgm_file)) fs::remove(pgm_file);
        response->command_result = true;
    } catch (const std::exception& e) {
        response->command_result = false;
    }
}

void TETRA_SERVICE::Mapping_Mode_Command(
    const std::shared_ptr<interfaces::srv::RunMapping::Request> request,
    std::shared_ptr<interfaces::srv::RunMapping::Response> response)
{
    if (request->flag_mapping) {
        run_process_async("ros2 launch tetra_bringup slam.launch.py &");
    } else {
        run_process_async("killall -9 cartographer_node slam_toolbox &");
    }
    response->command_result = true;
}

void TETRA_SERVICE::Mapping_3D_Mode_Command(
    const std::shared_ptr<interfaces::srv::Run3DMapping::Request> request,
    std::shared_ptr<interfaces::srv::Run3DMapping::Response> response)
{
    if (!request->map_name.empty()) {
        run_process_async("ros2 launch tetra_bringup rtabmap.launch.py &");
    } else {
        run_process_async("killall -9 rtabmap &");
    }
    response->command_result = true;
}

void TETRA_SERVICE::Navigation_Mode_Command(
    const std::shared_ptr<interfaces::srv::RunNavigation::Request> request,
    std::shared_ptr<interfaces::srv::RunNavigation::Response> response)
{
    if (!request->map_name.empty()) {
        std::string cmd = "ros2 launch tetra_bringup navigation.launch.py map:=/home/tetra/maps/" + request->map_name + ".yaml &";
        run_process_async(cmd);
    } else {
        run_process_async("killall -9 nav2_bringup &");
    }
    response->command_result = true;
}

void TETRA_SERVICE::Get_Information_Command(
    const std::shared_ptr<interfaces::srv::GetInformation::Request> request,
    std::shared_ptr<interfaces::srv::GetInformation::Response> response)
{
    (void)request;
    response->battery = _pRobot.m_iCallback_Battery;
    response->error_code = _pRobot.m_iCallback_ErrorCode;
    response->emg = _pRobot.m_iCallback_EMG;
    response->bumper = _pRobot.m_iCallback_Bumper;
    response->charging = _pRobot.m_iCallback_Charging_status;
    response->running_mode = 0;
    response->command_result = true;
}

void TETRA_SERVICE::Set_Location_Command(
    const std::shared_ptr<interfaces::srv::SetLocation::Request> request,
    std::shared_ptr<interfaces::srv::SetLocation::Response> response)
{
    (void)request;
    response->pose_x = _pTF_pose.poseTFx;
    response->pose_y = _pTF_pose.poseTFy;
    response->pose_z = 0.0;
    response->pose_qx = 0.0;
    response->pose_qy = 0.0;
    response->pose_qz = _pTF_pose.poseTFqz;
    response->pose_qw = _pTF_pose.poseTFqw;
    response->command_result = true;
}

void TETRA_SERVICE::Goto_Location_Command(
    const std::shared_ptr<interfaces::srv::GotoLocation::Request> request,
    std::shared_ptr<interfaces::srv::GotoLocation::Response> response)
{
    (void)request;
    response->command_result = true;
}

void TETRA_SERVICE::Goto_Location2_Command(
    const std::shared_ptr<interfaces::srv::GotoLocation2::Request> request,
    std::shared_ptr<interfaces::srv::GotoLocation2::Response> response)
{
    if (!nav_to_pose_action_client->wait_for_action_server(2s)) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available!");
        response->command_result = false;
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = request->pose_x;
    goal_msg.pose.pose.position.y = request->pose_y;
    goal_msg.pose.pose.position.z = request->pose_z;
    goal_msg.pose.pose.orientation.x = request->pose_qx;
    goal_msg.pose.pose.orientation.y = request->pose_qy;
    goal_msg.pose.pose.orientation.z = request->pose_qz;
    goal_msg.pose.pose.orientation.w = request->pose_qw;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TETRA_SERVICE::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TETRA_SERVICE::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TETRA_SERVICE::result_callback, this, _1);

    nav_to_pose_action_client->async_send_goal(goal_msg, send_goal_options);
    response->command_result = true;
}

void TETRA_SERVICE::Goto_Cancel_Command(
    const std::shared_ptr<interfaces::srv::GotoCancel::Request> request,
    std::shared_ptr<interfaces::srv::GotoCancel::Response> response)
{
    (void)request;
    nav_to_pose_action_client->async_cancel_all_goals();
    response->command_result = true;
}

void TETRA_SERVICE::Set_Maxspeed_Command(
    const std::shared_ptr<interfaces::srv::SetMaxspeed::Request> request,
    std::shared_ptr<interfaces::srv::SetMaxspeed::Response> response)
{
    auto parameters = std::vector<rclcpp::Parameter>();
    parameters.emplace_back(rclcpp::Parameter("FollowPath.desired_linear_vel", static_cast<double>(request->max_speed)));
    parameters.emplace_back(rclcpp::Parameter("FollowPath.max_vel_x", static_cast<double>(request->max_speed)));
    set_speed_parameter_client_->set_parameters(parameters);
    _pDynamic_param.MAX_Linear_velocity = request->max_speed;
    response->command_result = true;
}

// ─────────────────────── Client Helper Methods ────────────────────────

void TETRA_SERVICE::LedControl_Call(int id, int led_brightness)
{
    if (!led_control_client->service_is_ready()) return;
    auto request = std::make_shared<interfaces::srv::LedControl::Request>();
    request->id = id;
    request->led_brightness = led_brightness;
    led_control_client->async_send_request(
        request, std::bind(&TETRA_SERVICE::LedControl_callback, this, _1));
}

void TETRA_SERVICE::LedToggleControl_Call(int de_index, int light_accel, int led_high_brightness, int light_decel, int led_low_brightness)
{
    if (!led_toggle_control_client->service_is_ready()) return;
    auto request = std::make_shared<interfaces::srv::LedToggleControl::Request>();
    request->de_index = de_index;
    request->light_accel = light_accel;
    request->led_high_brightness = led_high_brightness;
    request->light_decel = light_decel;
    request->led_low_brightness = led_low_brightness;
    led_toggle_control_client->async_send_request(
        request, std::bind(&TETRA_SERVICE::LedToggleControl_callback, this, _1));
}

void TETRA_SERVICE::ToggleOn_Call(int id)
{
    if (!toggle_on_client->service_is_ready()) return;
    auto request = std::make_shared<interfaces::srv::ToggleOn::Request>();
    request->id = id;
    toggle_on_client->async_send_request(
        request, std::bind(&TETRA_SERVICE::ToggleOn_callback, this, _1));
}

void TETRA_SERVICE::ImuReset_Call()
{
    if (!imu_reset_client->service_is_ready()) return;
    auto request = std::make_shared<interfaces::srv::ImuReset::Request>();
    imu_reset_client->async_send_request(
        request, std::bind(&TETRA_SERVICE::ImuReset_callback, this, _1));
}

bool TETRA_SERVICE::Clear_Costmap()
{
    if (clear_entire_global_costmap_client->service_is_ready()) {
        auto req_global = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        clear_entire_global_costmap_client->async_send_request(
            req_global, std::bind(&TETRA_SERVICE::clear_entire_global_costmap_callback, this, _1));
    }
    if (clear_entire_local_costmap_client->service_is_ready()) {
        auto req_local = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        clear_entire_local_costmap_client->async_send_request(
            req_local, std::bind(&TETRA_SERVICE::clear_entire_local_costmap_callback, this, _1));
    }
    return true;
}

void TETRA_SERVICE::set_robot_localization_pose(double x, double y, double yaw)
{
    if (!set_pose_client_->service_is_ready()) return;
    auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
    request->pose.header.frame_id = "map";
    request->pose.header.stamp = this->now();
    request->pose.pose.pose.position.x = x;
    request->pose.pose.pose.position.y = y;
    request->pose.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    request->pose.pose.pose.orientation.x = q.x();
    request->pose.pose.pose.orientation.y = q.y();
    request->pose.pose.pose.orientation.z = q.z();
    request->pose.pose.pose.orientation.w = q.w();

    set_pose_client_->async_send_request(request);
}

// ── Client Callback Handlers ──
void TETRA_SERVICE::LedControl_callback(rclcpp::Client<interfaces::srv::LedControl>::SharedFuture future)
{
    (void)future;
}

void TETRA_SERVICE::LedToggleControl_callback(rclcpp::Client<interfaces::srv::LedToggleControl>::SharedFuture future)
{
    (void)future;
}

void TETRA_SERVICE::ToggleOn_callback(rclcpp::Client<interfaces::srv::ToggleOn>::SharedFuture future)
{
    (void)future;
}

void TETRA_SERVICE::ImuReset_callback(rclcpp::Client<interfaces::srv::ImuReset>::SharedFuture future)
{
    (void)future;
}

void TETRA_SERVICE::request_nomotion_update_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future)
{
    (void)future;
}

void TETRA_SERVICE::clear_entire_global_costmap_callback(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
{
    (void)future;
}

void TETRA_SERVICE::clear_entire_local_costmap_callback(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture future)
{
    (void)future;
}

// ─────────────────────── Nav2 Action Callbacks ────────────────────────

void TETRA_SERVICE::goal_response_callback(const GoalHandleNavigateToPose::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Navigation goal was rejected by Nav2!");
        _pRobot.m_iMovebase_Result = 4;
    } else {
        RCLCPP_INFO(this->get_logger(), "Navigation goal accepted!");
        _pRobot.m_iMovebase_Result = 1;
    }
    std_msgs::msg::Int32 mb_msg;
    mb_msg.data = _pRobot.m_iMovebase_Result;
    movebase_publisher->publish(mb_msg);
}

void TETRA_SERVICE::feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)feedback;
}

void TETRA_SERVICE::result_callback(const GoalHandleNavigateToPose::WrappedResult& result)
{
    std_msgs::msg::Int32 mb_msg;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            _pRobot.m_iMovebase_Result = 3;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation aborted!");
            _pRobot.m_iMovebase_Result = 4;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation canceled!");
            _pRobot.m_iMovebase_Result = 1;
            break;
        default:
            _pRobot.m_iMovebase_Result = 0;
            break;
    }
    mb_msg.data = _pRobot.m_iMovebase_Result;
    movebase_publisher->publish(mb_msg);
}

void TETRA_SERVICE::run_process_async(const std::string& command)
{
    int ret = std::system(command.c_str());
    (void)ret;
}

// ─────────────────────────────── Main ─────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto node = std::make_shared<TETRA_SERVICE>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

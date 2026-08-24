#pragma once

#include <string>
#include <vector>

// Constants
constexpr int HIGH_BATTERY = 95;
constexpr int LOW_BATTERY = 15;
constexpr int MAX_RETRY_CNT = 10;
constexpr size_t BUF_LEN = 4096;

// Odometry data structure
struct ODOMETRY
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
    double dOdom_theta_deg = 0.0;
};

// Home pose definition
struct HOME_POSE
{
    std::string HOME_strLOCATION = "HOME";
    double HOME_dPOSITION_X = -0.7;
    double HOME_dPOSITION_Y = 0.0;
    double HOME_dPOSITION_Z = 0.0;
    double HOME_dQUATERNION_X = 0.0;
    double HOME_dQUATERNION_Y = 0.0;
    double HOME_dQUATERNION_Z = 0.0;
    double HOME_dQUATERNION_W = 1.0;
};

// Goal pose definition
struct GOAL_POSE
{
    double goal_positionX = 0.0;
    double goal_positionY = 0.0;
    double goal_positionZ = 0.0;
    double goal_quarterX = 0.0;
    double goal_quarterY = 0.0;
    double goal_quarterZ = 0.0;
    double goal_quarterW = 1.0;
};

// AprilTag marker pose data
struct AR_TAG_POSE
{
    std::string m_strFamily = "tag36h11:";
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
};

// Landmark pose
struct LAND_MARK_POSE
{
    float init_position_x = 0.0f;
    float init_position_y = 0.0f;
    float init_position_z = 0.0f;
    float init_orientation_z = 0.0f;
    float init_orientation_w = 1.0f;
};

// TF Pose
struct TF_POSE
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
};

// Robot runtime state
struct ROBOT
{
    int  HOME_id = 0;
    int  m_iDocking_id = 0;
    bool m_bFalg_DockingExit = false;
    int  m_iCallback_Battery = 0;
    int  m_iCallback_ErrorCode = 0;
    int  m_iCallback_EMG = 0;
    int  m_iCallback_Bumper = 0;
    int  m_iCallback_Charging_status = 0;
    int  m_iBumperCollisionBehavor_cnt = 0;
    int  m_iMovebase_Result = 0; // Success: 3 | Aborted: 4 | Cancel: 1 | Unknown: 0
    int  m_iConveyor_Sensor_info = 0;
    int  m_iConveyor_id = 0;
    int  m_iConveyor_movement = 0; // 0: normal, 1: Loading, 2: Unloading
};

// Runtime flags
struct FALG_VALUE
{
    bool m_bflag_NextStep = false;
    bool m_bflag_ComebackHome = false;
    bool m_bfalg_DockingExit = false;
    bool m_bflag_Conveyor_docking = false;
    bool m_bFlag_Disable_bumper = false;
    int  m_Onetime_reset_cnt = 0;
    
    // Obstacle flags
    bool m_bFlag_Obstacle_Right = false;
    bool m_bFlag_Obstacle_Center = false;
    bool m_bFlag_Obstacle_Left = false;
    bool m_bFlag_Obstacle_Left_b = false;
    bool m_bFlag_Obstacle_Right_b = false;
    bool m_bFlag_Obstacle_PCL1 = false;
    bool m_bFlag_Obstacle_PCL2 = false;
    bool m_bFlag_Obstacle_cygbot = false;
    bool m_bFlag_MergeScan_Front = false;
    bool m_bFlag_MergeScan_Rear = false;
    bool m_bFlag_MergeScan_Left = false;
    bool m_bFlag_MergeScan_Right = false;
    
    // Error flags
    bool m_bumperhit_flag = false;
    bool m_emgpush_flag = false;
    bool BUMPER_BT = false;
    
    // Dynamic flags
    bool m_bTebMarker_reconfigure_flag = false;
    bool m_bflag_patrol = false;
    bool m_bflag_patrol2 = false;
    bool m_bflag_goto_cancel = false;
    bool m_bflagGo = false;
    bool m_bflagGo2 = false;
    bool m_bCorneringFlag = true;
    bool m_bFlag_nomotion = true;
    bool m_bflag_auto_patrol = false;
    bool m_bFlag_Initialpose = false;
    bool m_bFlag_pub = false;
};

// Dynamic velocity parameters
struct DYNAMIC_PARAM
{
    double MAX_Linear_velocity = 0.8;
    double m_linear_vel = 0.0;
    double m_angular_vel = 0.0;
    bool m_bFlag_onetime = true;
};

// Docking specific parameters
struct DOCKING_POSE
{
    double camera_TF_distance = -0.3955;
    double target_yaw = 0.0;
    double target_distance = 0.0;
    double before_TF_Pose = 0.0;
    bool Onemore_time = false;
};

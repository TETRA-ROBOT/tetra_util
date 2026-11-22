// 도킹 영점: 태그로 구한 도크 자세(도킹을 마친 base_link 자세)를 로봇마다 미세하게 옮긴다.
// 같은 모델이라도 충전기·범퍼·카메라 장착이 로봇마다 조금씩 달라, 태그 기준으로는 맞게 들어가도 충전 접점이
// 로봇 중앙에서 비껴 붙거나 너무 깊이 밀어 범퍼가 눌린다. 관제 관리자 설정 값을 API 가 도킹 목표 직전에
// <plugin>.dock_offset_x / _y / _yaw 파라미터로 넣으므로, Nav2 를 다시 띄우지 않고 바로 다음 도킹부터 반영된다.
//
// 좌표는 도킹을 마친 로봇 기준이다: x + 더 깊게(충전기 쪽), y + 로봇 왼쪽, yaw + 반시계(로봇 중심 기준 회전).
#include <algorithm>
#include <cmath>

#include "tetra_docking/tetra_charging_dock.hpp"
#include "tetra_docking/dock_geometry.hpp"
#include "nav2_util/node_utils.hpp"

namespace tetra_docking
{

namespace
{
// 기구 오차 보정용이라 이보다 크면 입력 실수로 보고 거부한다 (태그·도크 설정 자체가 틀린 것)
constexpr double kMaxOffsetMeters = 0.10;
constexpr double kMaxOffsetYawRad = 10.0 * M_PI / 180.0;
// 깊이 도착 판정을 쓰는 범위 (도크 자세에서 좌우·지나친 거리)
constexpr double kDepthCheckWindow = 0.10;
// 범퍼·충전기 몸체를 넘어 밀지 않도록 둔 상한
constexpr double kMaxContactOvertravel = 0.05;
// 마지막 getRefinedPose 가 이보다 최근이면 접근 중으로 본다. 도킹 서버 controller_frequency 50 Hz 면 20 ms 마다 불리므로
// 부하로 몇 주기 늦어도 접근 중이고, 목표 시작 때의 확인은 지난 접근(수 초 이상 전)이라 예전 판정을 쓴다
constexpr double kApproachWindow = 0.25;
}  // namespace

void TetraChargingDock::configureDockOffset(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  offset_param_prefix_ = name + ".";
  nav2_util::declare_parameter_if_not_declared(node, name + ".dock_offset_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".dock_offset_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".dock_offset_yaw", rclcpp::ParameterValue(0.0));
  dock_offset_x_ = node->get_parameter(name + ".dock_offset_x").as_double();
  dock_offset_y_ = node->get_parameter(name + ".dock_offset_y").as_double();
  dock_offset_yaw_ = node->get_parameter(name + ".dock_offset_yaw").as_double();
  // 기본 도착 판정은 hypot(앞뒤, 좌우) < docking_threshold 라, 영점으로 도크 자세를 충전기 바로 앞에 두고 판정 폭을 좁히면
  // 좌우 잔여 오차(1~2 cm, 마지막 구간은 곧게 들어간다) 때문에 끝내 만족하지 않고 접점이 올 때까지 충전기를 민다
  // (실기 MH 2026-11-03 13:50: 목표 0.677 m, 판정 폭 5 mm 인데 0.673 m 까지 밀어 범퍼가 눌렸다).
  // 켜면 도크 축 방향 깊이가 도크 자세 - docking_threshold 에 닿을 때 도착으로 본다. 충전 접점은 좌우 여유가 있다
  nav2_util::declare_parameter_if_not_declared(node, name + ".docked_by_depth", rclcpp::ParameterValue(false));
  docked_by_depth_ = node->get_parameter(name + ".docked_by_depth").as_bool();
  // 접점을 요구하는 충전 도킹(charge_requires_contact)에서 깊이 도착 판정을 도크 자세보다 이만큼 깊게 둔다.
  // 도착 판정 뒤 도킹 서버는 waitForCharge 동안 속도를 내지 않아, 접점은 휠 드라이버 cmd_vel_timeout(0.5 s) 동안
  // 마지막 명령(약 0.04 m/s, 약 2 cm)으로 밀려 들어갈 때만 붙었다 (실기 MH 2026-11-18 10:51, 16:39, 18:19: 도착 판정
  // 0.5 초 뒤 휠 정지와 함께 충전). 도킹 속도를 낮추면(0.05 m/s -> 접근 0.02 m/s) 1 cm 만 밀려 접점 앞에 멈춘다.
  // 접점을 못 만나면 앞 범퍼 bumper_grace 뒤 멈추는 것은 그대로다
  nav2_util::declare_parameter_if_not_declared(node, name + ".contact_overtravel", rclcpp::ParameterValue(0.0));
  contact_overtravel_ = std::clamp(
    node->get_parameter(name + ".contact_overtravel").as_double(), 0.0, kMaxContactOvertravel);

  // 도킹 서버와 모든 플러그인이 한 노드를 쓰므로, 이 인스턴스의 영점 파라미터만 보고 나머지는 그대로 통과시킨다
  offset_param_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) { return onDockOffsetParameters(params); });
}

rcl_interfaces::msg::SetParametersResult TetraChargingDock::onDockOffsetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : params) {
    const std::string & full = param.get_name();
    if (full.rfind(offset_param_prefix_ + "dock_offset_", 0) != 0) {
      continue;
    }
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = full + " must be a double";
      return result;
    }
    const double value = param.as_double();
    const bool is_yaw = full == offset_param_prefix_ + "dock_offset_yaw";
    if (!std::isfinite(value) || std::abs(value) > (is_yaw ? kMaxOffsetYawRad : kMaxOffsetMeters)) {
      result.successful = false;
      result.reason = full + " out of range";
      return result;
    }
  }
  // 모두 유효할 때만 반영한다 (일부만 바뀐 영점으로 도킹하지 않게)
  for (const auto & param : params) {
    const std::string & full = param.get_name();
    if (full == offset_param_prefix_ + "dock_offset_x") {
      dock_offset_x_ = param.as_double();
    } else if (full == offset_param_prefix_ + "dock_offset_y") {
      dock_offset_y_ = param.as_double();
    } else if (full == offset_param_prefix_ + "dock_offset_yaw") {
      dock_offset_yaw_ = param.as_double();
    }
  }
  return result;
}

DockSample TetraChargingDock::applyDockOffset(const DockSample & dock) const
{
  const double dx = dock_offset_x_;
  const double dy = dock_offset_y_;
  DockSample shifted = dock;
  shifted.x += std::cos(dock.yaw) * dx - std::sin(dock.yaw) * dy;
  shifted.y += std::sin(dock.yaw) * dx + std::cos(dock.yaw) * dy;
  shifted.yaw = dock.yaw + dock_offset_yaw_;
  return shifted;
}

void TetraChargingDock::applyDockOffset(geometry_msgs::msg::PoseStamped & pose) const
{
  DockSample dock;
  dock.x = pose.pose.position.x;
  dock.y = pose.pose.position.y;
  dock.yaw = tf2::getYaw(pose.pose.orientation);
  const DockSample shifted = applyDockOffset(dock);
  pose.pose.position.x = shifted.x;
  pose.pose.position.y = shifted.y;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, shifted.yaw);
  pose.pose.orientation = tf2::toMsg(orientation);
}

bool TetraChargingDock::isDockDepthReached()
{
  if (dock_pose_.header.frame_id.empty()) {
    return false;
  }
  try {
    const auto robot = tf2_buffer_->lookupTransform(
      dock_pose_.header.frame_id, base_frame_id_, tf2::TimePointZero);
    const double dock_yaw = tf2::getYaw(dock_pose_.pose.orientation);
    const double dx = robot.transform.translation.x - dock_pose_.pose.position.x;
    const double dy = robot.transform.translation.y - dock_pose_.pose.position.y;
    // 도크 축(도크 자세가 바라보는 방향, 충전기 쪽 +)으로 본 로봇 깊이. 0 이면 도크 자세 깊이, 음수면 아직 앞
    const double depth = std::cos(dock_yaw) * dx + std::sin(dock_yaw) * dy;
    const double lateral = -std::sin(dock_yaw) * dx + std::cos(dock_yaw) * dy;
    // 깊이만 보면 반평면이라, 지난 도킹의 dock_pose_ 가 남아 있을 때 다른 곳에서도 "이미 도킹됨"이 된다. 도크 근처로 제한한다
    // contact_overtravel 은 접근 중에만 더한다 (activeContactOvertravel 설명: 충전기 위 로봇의 시작 확인은 예전 판정)
    const double since_refine = last_refine_time_.nanoseconds() == 0 ?
      -1.0 : (node_->now() - last_refine_time_).seconds();
    const double overtravel = activeContactOvertravel(
      charge_requires_contact_, contact_overtravel_, since_refine, kApproachWindow);
    return dockDepthReached(depth, lateral, docking_threshold_, overtravel, kDepthCheckWindow);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Dock depth check failed: %s", ex.what());
    return false;
  }
}

}  // namespace tetra_docking

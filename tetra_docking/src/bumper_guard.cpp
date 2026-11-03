// 범퍼 정지: 도킹 접근 중 충전 접점 없이 앞 범퍼가 눌리면 도킹을 바로 끝낸다.
// 구동 보드는 앞 범퍼만으로는 서보를 끄지 않아(bumper_data 서보 비트만 본다), 실기 EDU (2026-10-29 15:54) 는
// 비스듬히 도크에 닿은 뒤에도 제어 법칙이 계속 밀어 사용자가 취소할 때까지 약 15 초 동안 범퍼를 누르고 있었다.
// 도킹 서버는 플러그인 예외를 받으면 그 제어 주기 안에 0 속도를 내고 목표를 실패(905)로 끝낸다.
#include "tetra_docking/tetra_charging_dock.hpp"
#include "nav2_util/node_utils.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"

namespace tetra_docking
{

namespace
{
constexpr int64_t kBumperStaleNs = 500000000;  // 0.5 s
}  // namespace

void TetraChargingDock::configureBumperGuard(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  // 비워 두면 범퍼를 보지 않는다 (M, 언도킹 전용 인스턴스는 후진이라 앞 범퍼와 무관)
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".bumper_topic", rclcpp::ParameterValue(std::string("")));
  // tetra_wheel_driver bumper_data: 0x01 앞 범퍼, 0x02 서보 꺼짐, 0x08 뒤 범퍼
  nav2_util::declare_parameter_if_not_declared(node, name + ".bumper_mask", rclcpp::ParameterValue(1));
  // 접점과 범퍼가 거의 같이 닿을 때 전원 보드 docking_status 가 늦게 올 수 있어 이만큼 눌린 채여야 멈춘다
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".bumper_grace", rclcpp::ParameterValue(0.0));
  // 언도킹 후진 중 볼 범퍼 비트. 구동 보드는 범퍼만으로 서보를 끄지 않아 뒤에 부딪혀도 staging 까지 계속 민다
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".undock_bumper_mask", rclcpp::ParameterValue(0));
  node->get_parameter(name + ".bumper_topic", bumper_topic_);
  node->get_parameter(name + ".bumper_mask", bumper_mask_);
  node->get_parameter(name + ".bumper_grace", bumper_grace_);
  node->get_parameter(name + ".undock_bumper_mask", undock_bumper_mask_);
  if (bumper_topic_.empty()) {
    return;
  }
  bumper_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    bumper_topic_, 10,
    std::bind(&TetraChargingDock::bumperCallback, this, std::placeholders::_1));
}

void TetraChargingDock::bumperCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  bumper_last_msg_ns_ = node_->now().nanoseconds();
  bumper_last_data_ = msg->data;
  const bool pressed = (msg->data & bumper_mask_) != 0;
  if (!pressed) {
    bumper_pressed_since_ns_ = 0;
    return;
  }
  if (bumper_pressed_since_ns_ == 0) {
    bumper_pressed_since_ns_ = node_->now().nanoseconds();
  }
}

// approachDock 은 매 주기 isDocked(접점) 를 먼저 보고 getRefinedPose 를 부른다. 접점이 있으면 여기까지 오지 않는다
void TetraChargingDock::stopIfBumperPressed()
{
  const int64_t since = bumper_pressed_since_ns_;
  if (bumper_topic_.empty() || since == 0 || current_docking_status_ >= contact_status_val_) {
    return;
  }
  const int64_t now_ns = node_->now().nanoseconds();
  // 구동 드라이버가 멈춰 마지막 "눌림"만 남은 경우는 믿지 않는다 (실시간 값은 제어 주기마다 온다)
  if (now_ns - bumper_last_msg_ns_ > kBumperStaleNs) {
    return;
  }
  const double pressed_for = (now_ns - since) / 1e9;
  if (pressed_for < bumper_grace_) {
    return;
  }
  RCLCPP_ERROR(
    node_->get_logger(),
    "Bumper pressed for %.2f s without charging contact (docking_status %d), stopping docking",
    pressed_for, current_docking_status_.load());
  throw opennav_docking_core::FailedToControl("Bumper pressed before charging contact");
}

// 언도킹 루프는 후진하는 매 주기 hasStoppedCharging 을 부른다. 여기서 던지면 도킹 서버가 그 주기에 0 속도를 내고 언도킹을 실패로 끝낸다
void TetraChargingDock::stopUndockIfBumperPressed()
{
  if (bumper_topic_.empty() || undock_bumper_mask_ == 0) {
    return;
  }
  if (node_->now().nanoseconds() - bumper_last_msg_ns_ > kBumperStaleNs) {
    return;
  }
  const int bumper_data = bumper_last_data_;
  if ((bumper_data & undock_bumper_mask_) == 0) {
    return;
  }
  RCLCPP_ERROR(
    node_->get_logger(), "Bumper pressed while undocking (bumper_data 0x%X), stopping", bumper_data);
  throw opennav_docking_core::FailedToControl("Bumper pressed while undocking");
}

}  // namespace tetra_docking

// 접점 정지: 도킹 접근 중 충전 접점(docking_status >= contact_status_val, 평소 1)이 오면 그 즉시 0 속도를 낸다.
// 도킹 서버(opennav_docking)는 isDocked 가 참이면 제어 루프를 빠져나가 waitForCharge 로 넘어가는데 0 속도를 내지 않는다.
// 휠 드라이버는 cmd_vel_timeout(0.5 s) 동안 마지막 명령(약 0.04 m/s)을 계속 써서, 접점이 붙은 뒤에도 약 2 cm 를 더 밀어
// 스테이션이 밀렸다 (실기 MH 2026-11-23). 깊이 도착 판정 뒤 접점을 향해 구르는 동안(contact_overtravel)도 같다.
#include "tetra_docking/tetra_charging_dock.hpp"

namespace tetra_docking
{

namespace
{
// 마지막 getRefinedPose 가 이보다 최근이면 바퀴가 아직 마지막 접근 명령으로 구르고 있을 수 있다
// (휠 드라이버 cmd_vel_timeout 0.5 s + 여유). 그 밖(대기·언도킹·조그)에서는 0 속도를 내지 않는다
constexpr double kCoastWindow = 1.0;
}  // namespace

void TetraChargingDock::configureContactStop(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  // 도킹 서버와 같은 노드라 cmd_vel 리매핑(cmd_vel_nav2, 휠 드라이버 입력)을 그대로 따른다.
  // lifecycle 활성화와 무관하게 쓰려고 일반 publisher 로 만든다 (tag_search.cpp 와 같음)
  contact_stop_pub_ = rclcpp::create_publisher<geometry_msgs::msg::Twist>(
    node->get_node_topics_interface(), "cmd_vel", rclcpp::QoS(1));
}

bool TetraChargingDock::isApproachCoasting() const
{
  const int64_t last_ns = last_refine_ns_.load();
  if (last_ns == 0) {
    return false;
  }
  return (node_->now().nanoseconds() - last_ns) / 1e9 <= kCoastWindow;
}

// 접점 콜백(전원 보드 30 Hz)과 도킹 서버의 isDocked/isCharging 확인(50 Hz) 양쪽에서 부른다.
// 콜백에서 0 을 낸 직후 도킹 서버가 그 주기의 명령을 한 번 더 낼 수 있어, 다음 확인에서 다시 0 을 낸다
void TetraChargingDock::stopOnContact()
{
  if (!contact_stop_pub_ || current_docking_status_ < contact_status_val_ || !isApproachCoasting()) {
    return;
  }
  contact_stop_pub_->publish(geometry_msgs::msg::Twist());
  if (!contact_stop_logged_.exchange(true)) {
    RCLCPP_INFO(
      node_->get_logger(), "Charging contact (docking_status %d) while approaching, stopping wheels",
      current_docking_status_.load());
  }
}

}  // namespace tetra_docking

#pragma once

// 라이다 도킹 마커(스테이션의 V 자 홈) 모드에 쓰는 순수 계산. ROS 없이 테스트한다 (test/test_lidar_marker.cpp)

#include <cmath>
#include <optional>
#include <string>

namespace tetra_docking
{

/** 도크를 무엇으로 찾는가. 관제가 도킹 목표 직전에 <plugin>.detection_source 로 넣는다 */
enum class DetectionSource { kTag, kLidar };

/**
 * @param value 파라미터 문자열 ("tag" | "lidar")
 * @returns 모르는 값이면 비어 있다 (파라미터 설정을 거부한다)
 */
inline std::optional<DetectionSource> parseDetectionSource(const std::string & value)
{
  if (value == "tag") {
    return DetectionSource::kTag;
  }
  if (value == "lidar") {
    return DetectionSource::kLidar;
  }
  return std::nullopt;
}

/** 로봇 기준 마커 위치: forward 는 로봇 정면(+), lateral 은 로봇 왼쪽(+) */
struct MarkerOffset
{
  double forward{0.0};
  double lateral{0.0};
};

/**
 * 고정 프레임(odom)의 마커 위치를 로봇 기준으로 바꾼다
 * @param robot_x, robot_y, robot_yaw 고정 프레임의 로봇 자세
 * @param marker_x, marker_y 고정 프레임의 마커 위치
 */
inline MarkerOffset markerRelativeToRobot(
  double robot_x, double robot_y, double robot_yaw, double marker_x, double marker_y)
{
  const double dx = marker_x - robot_x;
  const double dy = marker_y - robot_y;
  const double c = std::cos(robot_yaw);
  const double s = std::sin(robot_yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

/**
 * 언도킹 전용 인스턴스의 "도크 앞에 있다" 판정: 마커가 정면으로 max_forward 안, 좌우 max_lateral 안
 */
inline bool isMarkerCloseAhead(const MarkerOffset & marker, double max_forward, double max_lateral)
{
  return marker.forward > 0.0 && marker.forward <= max_forward && std::abs(marker.lateral) <= max_lateral;
}

}  // namespace tetra_docking

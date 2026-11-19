#include <gtest/gtest.h>

#include <cmath>

#include "tetra_docking/lidar_marker.hpp"

using tetra_docking::DetectionSource;
using tetra_docking::isMarkerCloseAhead;
using tetra_docking::markerRelativeToRobot;
using tetra_docking::parseDetectionSource;

TEST(LidarMarker, ParsesOnlyKnownDetectionSources)
{
  EXPECT_EQ(parseDetectionSource("tag"), DetectionSource::kTag);
  EXPECT_EQ(parseDetectionSource("lidar"), DetectionSource::kLidar);
  EXPECT_FALSE(parseDetectionSource("Lidar").has_value());
  EXPECT_FALSE(parseDetectionSource("").has_value());
}

TEST(LidarMarker, MarkerPositionInRobotFrame)
{
  // 로봇이 (1, 2) 에서 +y(90도)를 보고, 마커가 그 정면 0.5 m · 왼쪽 0.1 m 에 있다
  const auto ahead = markerRelativeToRobot(1.0, 2.0, M_PI / 2.0, 0.9, 2.5);
  EXPECT_NEAR(ahead.forward, 0.5, 1e-9);
  EXPECT_NEAR(ahead.lateral, 0.1, 1e-9);
}

TEST(LidarMarker, CloseAheadNeedsMarkerInFrontWithinLimits)
{
  // EDU 값 (navigation_edu.yaml tetra_backoff_dock: undock_lidar_distance 0.6, undock_tag_lateral 0.2)
  // 도킹을 마친 자리: 마커(태그면과 같은 자리)가 정면 0.34 m
  EXPECT_TRUE(isMarkerCloseAhead({0.34, 0.0}, 0.6, 0.2));
  // 언도킹 후진(0.3 m)을 마친 자리는 0.64 m 라 더는 도크 앞이 아니다 (다시 후진하지 않는다)
  EXPECT_FALSE(isMarkerCloseAhead({0.64, 0.0}, 0.6, 0.2));
  // 옆이나 뒤에 있으면 도크 앞이 아니다
  EXPECT_FALSE(isMarkerCloseAhead({0.3, 0.25}, 0.6, 0.2));
  EXPECT_FALSE(isMarkerCloseAhead({-0.3, 0.0}, 0.6, 0.2));
}

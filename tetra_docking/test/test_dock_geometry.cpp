// 태그 → 도크 오프셋 회전(dock_geometry.hpp), 도크 표본 평균과 각도 정규화(tag_view.cpp)
#include <cmath>
#include <vector>

#include "angles/angles.h"
#include "gtest/gtest.h"
#include "tetra_docking/dock_geometry.hpp"
#include "tetra_docking/tag_view.hpp"

using tetra_docking::DockSample;
using tetra_docking::PlanarOffset;
using tetra_docking::averageDockSamples;
using tetra_docking::dockPositionFromTag;
using tetra_docking::rotateDockOffset;
using tetra_docking::tagPositionFromDock;

namespace
{
// M: 도크 자세는 태그에서 도크 축 방향으로 0.92 m 떨어진 점이다 (external_detection_translation_x)
constexpr double kForward = 0.92;
constexpr double kLeft = 0.03;
constexpr double kTiny = 1e-12;
}  // namespace

TEST(RotateDockOffset, AlongTheAxisWhenYawIsZero)
{
  const PlanarOffset offset = rotateDockOffset(0.0, kForward, kLeft);
  EXPECT_NEAR(offset.x, kForward, kTiny);
  EXPECT_NEAR(offset.y, kLeft, kTiny);
}

TEST(RotateDockOffset, QuarterTurnMovesForwardToTheLeft)
{
  const PlanarOffset offset = rotateDockOffset(M_PI / 2.0, kForward, kLeft);
  EXPECT_NEAR(offset.x, -kLeft, kTiny);
  EXPECT_NEAR(offset.y, kForward, kTiny);
}

TEST(RotateDockOffset, KeepsTheLength)
{
  for (const double yaw : {-3.0, -1.2, 0.4, 2.7}) {
    const PlanarOffset offset = rotateDockOffset(yaw, kForward, kLeft);
    EXPECT_NEAR(std::hypot(offset.x, offset.y), std::hypot(kForward, kLeft), kTiny);
  }
}

TEST(DockPositionFromTag, AddsTheRotatedOffsetToTheTag)
{
  const PlanarOffset dock = dockPositionFromTag(2.0, -1.0, M_PI, kForward, kLeft);
  EXPECT_NEAR(dock.x, 2.0 - kForward, kTiny);
  EXPECT_NEAR(dock.y, -1.0 - kLeft, kTiny);
}

TEST(DockPositionFromTag, TagPositionFromDockUndoesIt)
{
  for (const double yaw : {-2.9, -0.3, 0.0, 1.1, 3.1}) {
    const PlanarOffset dock = dockPositionFromTag(1.5, 0.7, yaw, kForward, kLeft);
    const PlanarOffset tag = tagPositionFromDock(dock.x, dock.y, yaw, kForward, kLeft);
    EXPECT_NEAR(tag.x, 1.5, kTiny);
    EXPECT_NEAR(tag.y, 0.7, kTiny);
  }
}

// yaw 고정(staging_yaw_lock.cpp)이 하는 일: 잰 yaw 로 붙인 오프셋을 떼고 고정 yaw 로 다시 붙인다.
// 잰 yaw 가 15 도 튀면 도크 자세가 옆으로 24 cm 움직이는데, 고정하면 태그 기준 제자리로 돌아온다
TEST(DockPositionFromTag, RelockingAJumpedYawPutsTheDockBackOnTheAxis)
{
  const double locked_yaw = 0.0;
  const double jumped_yaw = 15.0 * M_PI / 180.0;
  const PlanarOffset jumped_dock = dockPositionFromTag(0.0, 0.0, jumped_yaw, kForward, 0.0);
  EXPECT_NEAR(jumped_dock.y, 0.238, 0.001);

  const PlanarOffset tag = tagPositionFromDock(jumped_dock.x, jumped_dock.y, jumped_yaw, kForward, 0.0);
  const PlanarOffset relocked = dockPositionFromTag(tag.x, tag.y, locked_yaw, kForward, 0.0);
  EXPECT_NEAR(relocked.x, kForward, kTiny);
  EXPECT_NEAR(relocked.y, 0.0, kTiny);
}

TEST(NormalizeAngle, WrapsIntoPlusMinusPi)
{
  EXPECT_NEAR(angles::normalize_angle(0.3), 0.3, kTiny);
  EXPECT_NEAR(angles::normalize_angle(2.0 * M_PI + 0.3), 0.3, kTiny);
  EXPECT_NEAR(angles::normalize_angle(-2.0 * M_PI - 0.3), -0.3, kTiny);
  EXPECT_NEAR(angles::normalize_angle(M_PI + 0.2), -M_PI + 0.2, kTiny);
  EXPECT_NEAR(angles::normalize_angle(-M_PI - 0.2), M_PI - 0.2, kTiny);
}

// 예전 코드의 atan2(sin, cos) 과 같은 값을 준다 (정확히 +-pi 인 경우만 부호가 다를 수 있다)
TEST(NormalizeAngle, MatchesTheOldAtan2Form)
{
  for (double angle = -9.0; angle <= 9.0; angle += 0.37) {
    EXPECT_NEAR(angles::normalize_angle(angle), std::atan2(std::sin(angle), std::cos(angle)), 1e-9) << angle;
  }
}

TEST(AverageDockSamples, MeansPositionAndYaw)
{
  const std::vector<DockSample> samples = {{1.00, 2.00, 0.10}, {1.02, 2.02, 0.12}, {0.98, 1.98, 0.08}};
  DockSample mean;
  ASSERT_TRUE(averageDockSamples(samples, 0.08, 0.12, mean));
  EXPECT_NEAR(mean.x, 1.0, 1e-9);
  EXPECT_NEAR(mean.y, 2.0, 1e-9);
  EXPECT_NEAR(mean.yaw, 0.10, 1e-6);
}

// 도크가 지도 왼쪽(+-pi)을 보면 표본 yaw 가 3.13, -3.13 처럼 나뉜다. 산술 평균(0)이 아니라 pi 근처여야 한다
TEST(AverageDockSamples, AveragesYawAcrossThePiBoundary)
{
  const std::vector<DockSample> samples = {{0.0, 0.0, 3.13}, {0.0, 0.0, -3.13}};
  DockSample mean;
  ASSERT_TRUE(averageDockSamples(samples, 0.08, 0.12, mean));
  EXPECT_NEAR(std::abs(mean.yaw), M_PI, 1e-6);
}

TEST(AverageDockSamples, RejectsSamplesThatSpreadTooMuch)
{
  DockSample mean;
  EXPECT_FALSE(averageDockSamples({{0.0, 0.0, 0.0}, {0.3, 0.0, 0.0}}, 0.08, 0.12, mean));
  EXPECT_FALSE(averageDockSamples({{0.0, 0.0, 0.0}, {0.0, 0.0, 0.5}}, 0.08, 0.12, mean));
  EXPECT_FALSE(averageDockSamples({}, 0.08, 0.12, mean));
}

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tetra_docking/pocket_detection.hpp"

using tetra_docking::composePose;
using tetra_docking::detectPocket;
using tetra_docking::entryPoseClear;
using tetra_docking::footprintClearance;
using tetra_docking::invertPose;
using tetra_docking::PocketParams;
using tetra_docking::PocketPose;
using tetra_docking::Point2;
using tetra_docking::RectFootprint;

namespace
{

PocketParams params()
{
  PocketParams p;
  p.inner_width = 1.0;
  p.leg_length = 1.0;
  return p;
}

/** 입구 자세 entrance 인 ㄷ 자 포트의 다리 안쪽 면 점 (안쪽 끝 벽 포함) */
std::vector<Point2> pocketScan(const PocketPose & entrance, double width, double length, double step = 0.02)
{
  std::vector<Point2> local;
  for (double u = 0.0; u <= length + 1e-9; u += step) {
    local.push_back({u, width / 2});
    local.push_back({u, -width / 2});
  }
  for (double v = -width / 2; v <= width / 2 + 1e-9; v += step) {
    local.push_back({length, v});
  }
  std::vector<Point2> out;
  for (const auto & p : local) {
    const auto q = composePose(entrance, PocketPose{p.x, p.y, 0.0});
    out.push_back({q.x, q.y});
  }
  return out;
}

}  // namespace

TEST(PocketDetection, FindsEntranceOfStraightPocket)
{
  const PocketPose truth{1.2, 0.03, 0.05};
  const auto scan = pocketScan(truth, 1.0, 1.0);
  const auto result = detectPocket(scan, PocketPose{1.15, -0.05, 0.0}, params());
  ASSERT_TRUE(result.found) << result.reason;
  EXPECT_NEAR(result.entrance.x, truth.x, 0.02);
  EXPECT_NEAR(result.entrance.y, truth.y, 0.01);
  EXPECT_NEAR(result.entrance.yaw, truth.yaw, 0.01);
  EXPECT_NEAR(result.width, 1.0, 0.01);
}

TEST(PocketDetection, RejectsGapOfWrongWidth)
{
  const auto scan = pocketScan(PocketPose{1.0, 0.0, 0.0}, 1.2, 1.0);
  const auto result = detectPocket(scan, PocketPose{1.0, 0.0, 0.0}, params());
  EXPECT_FALSE(result.found);
}

TEST(PocketDetection, NeedsBothLegs)
{
  std::vector<Point2> scan;
  for (double u = 0.0; u <= 1.0; u += 0.02) {
    scan.push_back({1.0 + u, 0.5});
  }
  const auto result = detectPocket(scan, PocketPose{1.0, 0.0, 0.0}, params());
  EXPECT_FALSE(result.found);
  EXPECT_EQ(result.right_points, 0);
}

TEST(PocketDetection, IgnoresFloorSpeckInFrontOfLeg)
{
  const PocketPose truth{1.0, 0.0, 0.0};
  auto scan = pocketScan(truth, 1.0, 1.0);
  scan.push_back({0.9, 0.5});
  const auto result = detectPocket(scan, truth, params());
  ASSERT_TRUE(result.found);
  EXPECT_NEAR(result.entrance.x, 1.0, 0.03);
}

TEST(PocketDetection, WorksFromInsideThePocket)
{
  // 로봇이 이미 반쯤 들어가 입구가 로봇 뒤(base_link x < 0)에 있다
  const PocketPose truth{-0.3, 0.01, -0.02};
  const auto scan = pocketScan(truth, 1.0, 1.0);
  const auto result = detectPocket(scan, truth, params());
  ASSERT_TRUE(result.found) << result.reason;
  EXPECT_NEAR(result.entrance.y, 0.01, 0.01);
}

TEST(PocketClearance, MeasuresGapToLegsBesideTheRobot)
{
  RectFootprint fp{0.4, -0.5, 0.4};
  std::vector<Point2> scan{{0.0, 0.47}, {0.1, -0.45}, {0.6, 0.0}, {2.0, 0.0}};
  const auto c = footprintClearance(scan, fp, 0.3, 0.5);
  EXPECT_NEAR(c.left, 0.07, 1e-9);
  EXPECT_NEAR(c.right, 0.05, 1e-9);
  EXPECT_NEAR(c.front, 0.2, 1e-9);
}

TEST(PocketEntry, RejectsRobotOffsetBeforeEnteringLegs)
{
  RectFootprint fp{0.4, -0.5, 0.4};
  // 입구 0.45 m 앞, 가운데: 아직 입구를 넘지 않아 통과
  EXPECT_TRUE(entryPoseClear(PocketPose{-0.45, 0.0, 0.0}, fp, 1.0, 0.05, 0.03));
  // 앞 끝이 입구를 넘는 자리에서 옆으로 8 cm 틀어짐: 여유 0.1 - 0.08 = 2 cm < 5 cm 라 거부
  EXPECT_FALSE(entryPoseClear(PocketPose{-0.38, 0.08, 0.0}, fp, 1.0, 0.05, 0.03));
  // 가운데지만 비스듬함: 앞 모서리가 다리 쪽으로 나간다
  EXPECT_FALSE(entryPoseClear(PocketPose{-0.3, 0.0, 0.15}, fp, 1.0, 0.05, 0.03));
  // 안쪽 깊이 곧게 들어와 있음
  EXPECT_TRUE(entryPoseClear(PocketPose{0.6, 0.01, 0.01}, fp, 1.0, 0.05, 0.03));
}

TEST(PocketPoseMath, InverseUndoesCompose)
{
  const PocketPose a{1.0, -2.0, 0.7};
  const PocketPose b{0.3, 0.4, -0.2};
  const auto back = composePose(invertPose(a), composePose(a, b));
  EXPECT_NEAR(back.x, b.x, 1e-9);
  EXPECT_NEAR(back.y, b.y, 1e-9);
  EXPECT_NEAR(back.yaw, b.yaw, 1e-9);
}

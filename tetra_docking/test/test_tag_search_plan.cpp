// 태그 탐색 계산과 상태 (src/tag_search_plan.cpp). 실제 회전 명령은 tag_search.cpp 가 이 결과를 그대로 낸다
#include <cmath>

#include "gtest/gtest.h"
#include "tetra_docking/tag_search_plan.hpp"

using tetra_docking::SearchStepCommand;
using tetra_docking::TagSearchState;
using tetra_docking::centeringVelocity;
using tetra_docking::kCenteredBearing;
using tetra_docking::kSearchAngleTolerance;
using tetra_docking::kSearchStepCount;
using tetra_docking::planSearchStep;
using tetra_docking::decideLostTag;
using tetra_docking::LostTagAction;

namespace
{
// M 의 설정 (navigation_m.yaml): 좌우 0.35 rad, 0.05 rad/s
constexpr double kAngle = 0.35;
constexpr double kSpeed = 0.05;
constexpr double kControlPeriod = 0.05;

// 로봇이 낸 속도대로 정확히 돈다고 보고 탐색을 끝까지 돌린다. 지나간 방향(시작 방향 기준)의 최댓값·최솟값을 돌려준다
struct SweepResult
{
  double max_offset{0.0};
  double min_offset{0.0};
  double final_offset{0.0};
  int cycles{0};
};

SweepResult runSweep(double origin_yaw)
{
  SweepResult result;
  double yaw = origin_yaw;
  int step = 0;
  while (step < kSearchStepCount && result.cycles < 100000) {
    const SearchStepCommand command = planSearchStep(yaw, origin_yaw, step, kAngle, kSpeed);
    yaw += command.angular_velocity * kControlPeriod;
    step = command.next_step;
    const double offset = std::remainder(yaw - origin_yaw, 2.0 * M_PI);
    result.max_offset = std::max(result.max_offset, offset);
    result.min_offset = std::min(result.min_offset, offset);
    result.final_offset = offset;
    ++result.cycles;
  }
  return result;
}
}  // namespace

TEST(PlanSearchStep, TurnsLeftFirstTowardsThePositiveAngle)
{
  const SearchStepCommand command = planSearchStep(0.0, 0.0, 0, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(command.angular_velocity, kSpeed);
  EXPECT_EQ(command.next_step, 0);
}

TEST(PlanSearchStep, StopsOneCycleAndMovesOnWhenTheTargetIsReached)
{
  const SearchStepCommand command = planSearchStep(kAngle - kSearchAngleTolerance / 2.0, 0.0, 0, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(command.angular_velocity, 0.0);
  EXPECT_EQ(command.next_step, 1);
}

TEST(PlanSearchStep, KeepsTurningJustOutsideTheTolerance)
{
  const SearchStepCommand command = planSearchStep(kAngle - kSearchAngleTolerance * 1.5, 0.0, 0, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(command.angular_velocity, kSpeed);
  EXPECT_EQ(command.next_step, 0);
}

TEST(PlanSearchStep, SecondStepTurnsRightToTheNegativeAngle)
{
  const SearchStepCommand command = planSearchStep(kAngle, 0.0, 1, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(command.angular_velocity, -kSpeed);
  EXPECT_EQ(command.next_step, 1);
}

TEST(PlanSearchStep, LastStepReturnsToTheStartHeadingAndFinishes)
{
  const SearchStepCommand turning = planSearchStep(-kAngle, 0.0, 2, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(turning.angular_velocity, kSpeed);
  EXPECT_EQ(turning.next_step, 2);

  const SearchStepCommand done = planSearchStep(0.001, 0.0, 2, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(done.angular_velocity, 0.0);
  EXPECT_EQ(done.next_step, kSearchStepCount);
}

TEST(PlanSearchStep, FullSweepGoesLeftThenRightThenBackToTheStart)
{
  const SweepResult sweep = runSweep(0.3);
  EXPECT_NEAR(sweep.max_offset, kAngle, kSearchAngleTolerance);
  EXPECT_NEAR(sweep.min_offset, -kAngle, kSearchAngleTolerance);
  EXPECT_NEAR(sweep.final_offset, 0.0, kSearchAngleTolerance);
  // 0.35 + 0.70 + 0.35 rad 을 0.05 rad/s 로 돈다: 28 초 = 560 주기 남짓
  EXPECT_GT(sweep.cycles, 500);
  EXPECT_LT(sweep.cycles, 620);
}

// 로봇이 +-pi 근처를 보고 있어도(지도 왼쪽을 보는 도크) 가까운 쪽으로 돈다
TEST(PlanSearchStep, FullSweepWorksAcrossThePiBoundary)
{
  const SweepResult sweep = runSweep(M_PI - 0.1);
  EXPECT_NEAR(sweep.max_offset, kAngle, kSearchAngleTolerance);
  EXPECT_NEAR(sweep.min_offset, -kAngle, kSearchAngleTolerance);
  EXPECT_NEAR(sweep.final_offset, 0.0, kSearchAngleTolerance);
}

TEST(PlanSearchStep, TurnsTheShortWayWhenYawWrapsAround)
{
  // 시작 방향 3.1 rad, 지금 -3.1 rad: 목표(3.1 + 0.35)까지 왼쪽으로 0.27 rad 남았다 (오른쪽으로 6 rad 이 아니다)
  const SearchStepCommand command = planSearchStep(-3.1, 3.1, 0, kAngle, kSpeed);
  EXPECT_DOUBLE_EQ(command.angular_velocity, kSpeed);
}

TEST(CenteringVelocity, NoTurnWhenTheTagIsNearTheImageCentre)
{
  EXPECT_FALSE(centeringVelocity(0.0, 1.0, kSpeed).has_value());
  EXPECT_FALSE(centeringVelocity(kCenteredBearing * 0.99, 1.0, kSpeed).has_value());
  EXPECT_FALSE(centeringVelocity(-kCenteredBearing * 0.99, 1.0, kSpeed).has_value());
}

TEST(CenteringVelocity, TurnsClockwiseWhenTheTagIsOnTheRightOfTheImage)
{
  const auto velocity = centeringVelocity(0.2, 1.0, kSpeed);
  ASSERT_TRUE(velocity.has_value());
  EXPECT_DOUBLE_EQ(*velocity, -kSpeed);
}

TEST(CenteringVelocity, TurnsCounterClockwiseWhenTheTagIsOnTheLeftOfTheImage)
{
  const auto velocity = centeringVelocity(-0.2, 1.0, kSpeed);
  ASSERT_TRUE(velocity.has_value());
  EXPECT_DOUBLE_EQ(*velocity, kSpeed);
}

TEST(CenteringVelocity, UsesBearingNotDistance)
{
  // 같은 0.2 m 옆이라도 3 m 앞이면 중앙 안(0.067), 1 m 앞이면 밖(0.2)
  EXPECT_FALSE(centeringVelocity(0.2, 3.0, kSpeed).has_value());
  EXPECT_TRUE(centeringVelocity(0.2, 1.0, kSpeed).has_value());
}

TEST(CenteringVelocity, IgnoresATagBehindTheCamera)
{
  EXPECT_FALSE(centeringVelocity(0.5, 0.0, kSpeed).has_value());
  EXPECT_FALSE(centeringVelocity(0.5, -1.0, kSpeed).has_value());
}

// 2026-11-16 회귀: 탐색·yaw 고정을 끈 모델(EDU, MH, SM)이 "멈춰서 확인"을 거쳐 도킹이 실패할 수 있었다
TEST(TagSearchState, ModelsWithSearchAndYawLockOffSkipTheStillConfirmation)
{
  TagSearchState state;
  state.reset();
  ASSERT_FALSE(state.seen());
  EXPECT_FALSE(state.needsConfirmation(false, false));
}

TEST(TagSearchState, ConfirmationIsNeededWhenEitherFeatureIsOn)
{
  TagSearchState state;
  state.reset();
  EXPECT_TRUE(state.needsConfirmation(true, false));
  EXPECT_TRUE(state.needsConfirmation(false, true));
  EXPECT_TRUE(state.needsConfirmation(true, true));
}

TEST(TagSearchState, NoConfirmationOnceTheTagWasSeenThisDocking)
{
  TagSearchState state;
  state.reset();
  state.markSeen();
  EXPECT_FALSE(state.needsConfirmation(true, true));
}

// 도킹을 시작하기 전에는(지난 도킹 뒤, 또는 처음) 탐색하지 않는다
TEST(TagSearchState, DoesNotSearchBeforeADockingStarts)
{
  const TagSearchState state;
  EXPECT_TRUE(state.seen());
  EXPECT_FALSE(state.active());
  EXPECT_FALSE(state.needsConfirmation(true, true));
}

TEST(TagSearchState, ResetStartsANewSearchFromTheFirstStep)
{
  TagSearchState state;
  state.reset();
  state.beginSweep(1.2);
  state.advanceTo(kSearchStepCount);
  ASSERT_TRUE(state.sweepFinished());

  state.reset();
  EXPECT_FALSE(state.seen());
  EXPECT_FALSE(state.active());
  EXPECT_EQ(state.step(), 0);
  EXPECT_FALSE(state.sweepFinished());
}

TEST(TagSearchState, BeginSweepRemembersTheStartHeading)
{
  TagSearchState state;
  state.reset();
  state.beginSweep(-2.5);
  EXPECT_TRUE(state.active());
  EXPECT_DOUBLE_EQ(state.originYaw(), -2.5);
}

TEST(TagSearchState, SeeingTheTagEndsTheSearchButKeepsTheSweepProgress)
{
  TagSearchState state;
  state.reset();
  state.beginSweep(0.0);
  state.advanceTo(1);
  state.markSeen();
  EXPECT_TRUE(state.seen());
  EXPECT_FALSE(state.active());
  EXPECT_EQ(state.step(), 1);
}

// 접근 중 태그를 놓쳤을 때 (실기 MH 2026-11-18: 도크 0.53 m 앞에서 놓쳐 904)
TEST(DecideLostTag, FinishesBlindWhenCloseEnough)
{
  EXPECT_EQ(decideLostTag(0.20, 0.5, 0.25, 8.0, 1.0, 4.0), LostTagAction::kBlind);
}

TEST(DecideLostTag, TurnsTowardTheLastTagWhenLostFartherOut)
{
  EXPECT_EQ(decideLostTag(0.53, 0.5, 0.25, 8.0, 1.0, 4.0), LostTagAction::kReacquire);
}

TEST(DecideLostTag, GivesUpWhenTheTagStaysHidden)
{
  EXPECT_EQ(decideLostTag(0.53, 4.5, 0.25, 8.0, 1.0, 4.0), LostTagAction::kFail);
}

TEST(DecideLostTag, DoesNotDriveBlindFromFarAway)
{
  EXPECT_EQ(decideLostTag(1.4, 0.5, 0.25, 8.0, 1.0, 4.0), LostTagAction::kFail);
}

TEST(DecideLostTag, KeepsTheOldBehaviourWhenReacquireIsOff)
{
  EXPECT_EQ(decideLostTag(0.53, 0.5, 0.25, 8.0, 0.0, 4.0), LostTagAction::kFail);
  EXPECT_EQ(decideLostTag(0.20, 9.0, 0.25, 8.0, 1.0, 4.0), LostTagAction::kFail);
}

#pragma once

// 리프트 포트(ㄷ 자 받침대) 라이다 검출과 진입 여유 판정. ROS 없이 테스트한다 (test/test_pocket_detection.cpp)
//
// 포트는 위에서 보면 ㄷ 자다: 좌우 다리 두 개가 나란히 서 있고 로봇은 열린 쪽으로 다리 사이에 들어가 짐을 리프트로 뜬다.
// 2D 스캔에서 두 다리의 안쪽 면을 직선으로 맞춰 포트 중심선(축)과 입구(다리 앞 끝)를 구한다.
// 좌표는 모두 같은 평면 프레임(보통 base_link: x 앞, y 왼쪽)이다.

#include <limits>
#include <string>
#include <vector>

namespace tetra_docking
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

/** 포트 자세: 입구 가운데 점과 포트 안쪽을 향하는 축 방향 */
struct PocketPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PocketParams
{
  /** 두 다리 안쪽 면 사이 간격 (m, 실측) */
  double inner_width{1.0};
  /** 다리 길이: 입구에서 안쪽 끝까지 (m, 실측) */
  double leg_length{1.0};
  /** 사전 자세 기준으로 다리 안쪽 면을 찾는 띠의 반폭 (m). 사전 자세의 좌우 오차보다 커야 한다 */
  double band_half_width{0.10};
  /** 다리 하나로 인정할 최소 점 수 */
  int min_leg_points{8};
  /** 다리 하나로 인정할 최소 길이 (축 방향, m) */
  double min_leg_span{0.25};
  /** 잰 간격이 inner_width 와 이만큼 넘게 다르면 다른 물체로 본다 (m) */
  double max_width_error{0.06};
  /** 두 다리 방향이 이만큼 넘게 어긋나면 다른 물체로 본다 (rad) */
  double max_leg_angle_diff{0.12};
  /** 사전 자세에서 축이 이만큼 넘게 돌아가면 다른 물체로 본다 (rad) */
  double max_yaw_change{0.35};
};

struct PocketDetection
{
  bool found{false};
  /** found 일 때 입구 자세 (입력 점과 같은 프레임) */
  PocketPose entrance;
  /** 잰 안쪽 간격 (m) */
  double width{0.0};
  int left_points{0};
  int right_points{0};
  /** 못 찾았을 때 이유 (로그용) */
  std::string reason;
};

/**
 * 두 다리의 안쪽 면을 찾아 포트 입구 자세를 구한다
 * @param points 스캔 점 (prior 와 같은 프레임)
 * @param prior 포트 입구가 있을 것으로 보는 자세 (등록 위치 또는 지난 검출)
 */
PocketDetection detectPocket(const std::vector<Point2> & points, const PocketPose & prior, const PocketParams & params);

/** 로봇 외곽 (base_link 기준 사각형) */
struct RectFootprint
{
  double front{0.434};
  double back{-0.504};
  double half_width{0.434};
};

/** 로봇 외곽과 주변 점 사이 거리. 점이 없으면 무한대, 외곽 안에 점이 있으면 음수 */
struct SideClearance
{
  double left{std::numeric_limits<double>::infinity()};
  double right{std::numeric_limits<double>::infinity()};
  double front{std::numeric_limits<double>::infinity()};
};

/**
 * @param points base_link 기준 스캔 점
 * @param footprint 로봇 외곽
 * @param side_reach 옆으로 이 거리 밖의 점은 보지 않는다 (m)
 * @param front_reach 앞으로 이 거리 밖의 점은 보지 않는다 (m)
 */
SideClearance footprintClearance(
  const std::vector<Point2> & points, const RectFootprint & footprint, double side_reach, double front_reach);

/**
 * 포트 안으로 들어가도 되는 자세인지. 외곽 꼭짓점을 lookahead 만큼 앞으로 옮겨, 입구를 넘은(포트 축 좌표 >= 0) 꼭짓점이
 * 다리 안쪽 면에서 margin 안으로 들어오면 거부한다. 다리에 스치기 전에 멈추려는 것이다
 * @param robot_in_pocket 포트 입구 프레임의 로봇(base_link) 자세
 * @param inner_width 다리 안쪽 간격 (m)
 */
bool entryPoseClear(
  const PocketPose & robot_in_pocket, const RectFootprint & footprint, double inner_width, double margin, double lookahead);

/** 프레임 b 에서 본 프레임 a 의 자세 pose_in_a 를 b 기준으로 (frame_a_in_b ∘ pose_in_a) */
PocketPose composePose(const PocketPose & frame_a_in_b, const PocketPose & pose_in_a);

/** pose 의 역변환 */
PocketPose invertPose(const PocketPose & pose);

}  // namespace tetra_docking

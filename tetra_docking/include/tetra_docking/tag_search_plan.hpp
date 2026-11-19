#pragma once

#include <optional>

namespace tetra_docking
{

// 탐색은 시작 방향 기준 +각도 -> -각도 -> 시작 방향 순서로 돈다
constexpr int kSearchStepCount = 3;
// 목표 각도에 이만큼 가까우면 다음 방향으로 넘어간다
constexpr double kSearchAngleTolerance = 0.02;
// 태그가 영상 중앙에서 이만큼(카메라 x/z, 약 4.5도) 안에 와야 확인한다. 시야 끝에서 접근을 시작하면 바로 다시 놓친다
constexpr double kCenteredBearing = 0.08;

struct SearchStepCommand
{
  // 이번 주기에 낼 회전 속도 (rad/s). 목표 각도에 닿았으면 0 (한 주기 멈춘다)
  double angular_velocity{0.0};
  // 다음 주기에 쓸 단계. kSearchStepCount 가 되면 탐색이 끝난 것이다
  int next_step{0};
};

/**
 * 탐색 한 주기: 지금 각도와 목표 단계로 낼 회전 속도와 다음 단계를 정한다 (ROS 없이 계산만 한다)
 * @param yaw 지금 로봇 방향 (rad)
 * @param origin_yaw 탐색을 시작할 때의 로봇 방향 (rad)
 * @param step 지금 단계 (0: +각도, 1: -각도, 2: 시작 방향). 0 ~ kSearchStepCount - 1
 * @param search_angle 좌우로 돌아볼 각도 (rad, 양수)
 * @param search_speed 회전 속도 크기 (rad/s)
 */
SearchStepCommand planSearchStep(double yaw, double origin_yaw, int step, double search_angle, double search_speed);

/**
 * 태그를 영상 중앙으로 맞추는 회전 속도. 카메라 광학 프레임 기준: z 정면, x 영상 오른쪽
 * @param tag_x 태그의 카메라 x (m)
 * @param tag_forward 태그의 카메라 z (m)
 * @param search_speed 회전 속도 크기 (rad/s)
 * @return 돌 필요가 없으면(이미 중앙이거나 태그가 카메라 뒤) 값이 없다
 */
std::optional<double> centeringVelocity(double tag_x, double tag_forward, double search_speed);

// 접근 중 태그를 놓쳤을 때 할 일
enum class LostTagAction
{
  kFail,       // 검출 실패로 멈춘다 (도킹 서버가 재시도하거나 실패로 끝낸다)
  kBlind,      // 도크가 가까워 마지막 검출 자세(odom)로 마저 들어간다
  kReacquire,  // 마지막 검출 자세 쪽으로 계속 몰아 태그가 다시 시야에 들어오게 한다 (돌아서 다시 찾기)
};

/**
 * 접근 중 태그를 놓쳤을 때 할 일을 고른다 (ROS 없이 계산만 한다)
 * @param remaining 마지막 검출 도크 자세까지 남은 거리 (m)
 * @param since_detection 마지막 검출 뒤 지난 시간 (s)
 * @param blind_distance 이 거리 안이면 마저 들어간다 (0 이면 끔)
 * @param blind_timeout 마지막 검출이 이보다 오래되면 어떤 것도 하지 않는다 (예전 도킹의 값)
 * @param reacquire_distance 이 거리 안에서 놓치면 다시 찾는다 (0 이면 끔)
 * @param reacquire_time 놓친 뒤 이 시간 안에 다시 보여야 한다 (s)
 */
LostTagAction decideLostTag(
  double remaining, double since_detection, double blind_distance, double blind_timeout,
  double reacquire_distance, double reacquire_time);

// 재시도 때 staging 으로 되돌아가지 않고 제자리에서 다시 접근하는 범위.
// 로봇이 staging 보다 도크에서 조금 멀면 staging 이 로봇 바로 뒤(도크 쪽)에 잡히는데, Jazzy 는 staging 복귀를 전진으로만 해
// 반 바퀴를 돌아야 했고 v_angular_max 0.06 rad/s 로는 dock_approach_timeout(30 s) 안에 못 끝냈다 (M 실기 2026-11-19)
constexpr double kRetryInPlaceBeyondStaging = 0.15;
constexpr double kRetryInPlaceLateral = 0.10;

/** 로봇이 도크 축에서 얼마나 벗어나 있는가 (staging 자세 기준) */
struct ApproachLineOffset
{
  double front{0.0};    // 도크 자세에서 축을 따라 로봇까지 (m, 도크 앞이 +)
  double lateral{0.0};  // 축에서 옆으로 (m)
  double heading{0.0};  // staging 자세 heading 과의 차 (rad)
};

/**
 * 재시도 때 지금 자리에서 바로 다시 접근해도 되는가 (ROS 없이 계산만 한다).
 * staging 보다 도크에 가까우면 staging 이 앞(전진 방향)이라 Jazzy 복귀가 잘 되므로 그대로 물러나게 둔다
 * @param staging_distance 도크 자세에서 staging 까지 (m)
 * @param heading_limit 이 각도 안이면 접근 전 가운데 맞추기·탐색으로 태그를 다시 잡는다 (rad)
 */
bool canRetryInPlace(const ApproachLineOffset & offset, double staging_distance, double heading_limit);

/**
 * 도킹 한 번 동안의 태그 탐색 상태. 도킹 액션 스레드에서만 읽고 쓴다
 */
class TagSearchState
{
public:
  /** 새 도킹을 시작한다: 지난 도킹에서 본 것과 탐색 진행을 지운다 */
  void reset();

  /** 태그를 멈춘 채로 확인했다. 탐색을 끝낸다 */
  void markSeen();

  /** 회전을 시작한다 (탐색 또는 가운데 맞추기). 탐색의 기준 방향은 beginSweep 이 따로 잡는다 */
  void markActive() {active_ = true;}

  /** 좌우 탐색을 시작하며 기준 방향을 기억한다 */
  void beginSweep(double origin_yaw);

  /** 탐색 한 주기의 결과(다음 단계)를 반영한다 */
  void advanceTo(int next_step) {step_ = next_step;}

  /**
   * 접근 전에 멈춰서 태그를 확인해야 하는가.
   * 탐색도 yaw 고정도 끈 설정은 예전처럼 확인 없이 바로 접근한다. 이 확인은 멈춘 채 표본 3 개와 정면 각도를
   * 요구해, 꺼진 모델에서도 돌면 비스듬히 도착하거나 검출이 흔들릴 때 initial_perception_timeout 으로 도킹이 실패했다
   * @param search_enabled 태그 탐색을 켰는가 (tag_search_angle > 0)
   * @param yaw_lock_enabled 도크 yaw 고정을 켰는가 (lock_dock_yaw_to_staging)
   */
  bool needsConfirmation(bool search_enabled, bool yaw_lock_enabled) const;

  bool seen() const {return seen_;}
  bool active() const {return active_;}
  int step() const {return step_;}
  double originYaw() const {return origin_yaw_;}
  /** 좌우를 다 돌아봤다 (더 돌지 않는다) */
  bool sweepFinished() const {return step_ >= kSearchStepCount;}

private:
  // 도킹을 시작하기 전(getStagingPose 가 reset 하기 전)에는 탐색하지 않게 '봤다'로 둔다
  bool seen_{true};
  bool active_{false};
  int step_{0};
  double origin_yaw_{0.0};
};

}  // namespace tetra_docking

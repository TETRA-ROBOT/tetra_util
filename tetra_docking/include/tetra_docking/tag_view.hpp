#pragma once

#include <vector>

#include "tf2/LinearMath/Transform.h"

namespace tetra_docking
{

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};

  bool valid() const {return fx > 0.0 && fy > 0.0 && width > 0 && height > 0;}
};

struct TagView
{
  bool in_view{false};
  double pixel_width{0.0};
};

// staging 자세(로봇이 도크 축 위에서 도크를 정면으로 봄)에서 태그를 카메라가 어떻게 볼지 계산하는 데 쓰는 값
struct StagingViewInputs
{
  CameraIntrinsics intrinsics;
  // base_link 점을 카메라 광학 프레임으로 옮기는 변환 (TF camera_frame <- base_link)
  tf2::Transform optical_from_base;
  // 도크 자세 기준 태그 중심 위치. 도크 자세 = 태그 + translation 이므로 정면 거리는 -translation_x 다
  double tag_ahead_of_dock{0.0};
  double tag_lateral{0.0};
  // base_link 기준 태그 중심 높이 (실제 검출에서 잰다)
  double tag_height{0.0};
  double tag_size{0.1};
  double margin_px{10.0};
  double min_pixels{40.0};
};

struct DockSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * 도크 축 위 base_link 가 도크 자세에서 staging_distance 만큼 떨어져 있을 때 태그 네 모서리를 영상에 투영한다
 */
TagView projectTagAtStaging(const StagingViewInputs & inputs, double staging_distance);

/**
 * preferred 에서 시작해 태그 전체가 여백 안에 들어올 때까지 멀어지고, 태그 폭이 min_pixels 보다 작으면
 * 영상 안에 남는 한 가까워진다. 카메라 정보가 없으면 preferred 를 그대로 돌려준다
 */
double chooseStagingDistance(
  const StagingViewInputs & inputs, double preferred, double min_distance, double max_distance);

/**
 * 도크 자세 표본을 평균한다 (yaw 는 원형 평균). 표본이 평균에서 max_xy / max_yaw 보다 벗어나면 false
 */
bool averageDockSamples(
  const std::vector<DockSample> & samples, double max_xy, double max_yaw, DockSample & mean);

}  // namespace tetra_docking

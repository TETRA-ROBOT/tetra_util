#include "tetra_docking/tag_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"

namespace tetra_docking
{

namespace
{
constexpr double kDistanceStep = 0.05;
}  // namespace

TagView projectTagAtStaging(const StagingViewInputs & inputs, double staging_distance)
{
  TagView view;
  const CameraIntrinsics & k = inputs.intrinsics;
  if (!k.valid()) {
    return view;
  }
  const double half = inputs.tag_size / 2.0;
  const double forward = staging_distance + inputs.tag_ahead_of_dock;
  // 도크 축 좌표(도크 쪽 +, 왼쪽 +)를 base_link 로 옮긴다. 후진이면 로봇이 180도 돌아 서 있다
  const double side = inputs.backward ? -1.0 : 1.0;
  double min_u = std::numeric_limits<double>::max();
  double max_u = std::numeric_limits<double>::lowest();
  bool inside = true;
  // 태그면은 도크 축에 수직이다 (staging 에서 로봇이 태그를 정면으로 본다)
  for (const double dy : {-half, half}) {
    for (const double dz : {-half, half}) {
      const tf2::Vector3 corner = inputs.optical_from_base *
        tf2::Vector3(side * forward, side * (inputs.tag_lateral + dy), inputs.tag_height + dz);
      if (corner.z() <= 0.0) {
        return view;
      }
      const double u = k.fx * corner.x() / corner.z() + k.cx;
      const double v = k.fy * corner.y() / corner.z() + k.cy;
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      inside = inside && u >= inputs.margin_px && u <= k.width - inputs.margin_px &&
        v >= inputs.margin_px && v <= k.height - inputs.margin_px;
    }
  }
  view.in_view = inside;
  view.pixel_width = max_u - min_u;
  return view;
}

double chooseStagingDistance(
  const StagingViewInputs & inputs, double preferred, double min_distance, double max_distance)
{
  if (!inputs.intrinsics.valid()) {
    return preferred;
  }
  double distance = std::clamp(preferred, min_distance, max_distance);
  // 가까우면 태그 위·아래가 잘리고 (카메라 높이·피치), 카메라가 옆으로 치우쳐 있으면 옆이 먼저 잘린다
  while (!projectTagAtStaging(inputs, distance).in_view && distance + kDistanceStep <= max_distance) {
    distance += kDistanceStep;
  }
  // 멀면 태그 픽셀이 줄어 자세(특히 yaw) 추정이 흔들린다. 영상 안에 남는 한 가까이 둔다
  while (projectTagAtStaging(inputs, distance).pixel_width < inputs.min_pixels &&
    distance - kDistanceStep >= min_distance &&
    projectTagAtStaging(inputs, distance - kDistanceStep).in_view)
  {
    distance -= kDistanceStep;
  }
  return distance;
}

bool averageDockSamples(
  const std::vector<DockSample> & samples, double max_xy, double max_yaw, DockSample & mean)
{
  if (samples.empty()) {
    return false;
  }
  double sum_x = 0.0, sum_y = 0.0, sum_sin = 0.0, sum_cos = 0.0;
  for (const auto & sample : samples) {
    sum_x += sample.x;
    sum_y += sample.y;
    sum_sin += std::sin(sample.yaw);
    sum_cos += std::cos(sample.yaw);
  }
  const double count = static_cast<double>(samples.size());
  mean.x = sum_x / count;
  mean.y = sum_y / count;
  mean.yaw = std::atan2(sum_sin, sum_cos);
  for (const auto & sample : samples) {
    if (std::hypot(sample.x - mean.x, sample.y - mean.y) > max_xy ||
      std::abs(angles::normalize_angle(sample.yaw - mean.yaw)) > max_yaw)
    {
      return false;
    }
  }
  return true;
}

}  // namespace tetra_docking

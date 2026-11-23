// 리프트 포트(ㄷ 자) 라이다 검출과 진입 여유 판정 (pocket_detection.hpp 설명 참고)
#include "tetra_docking/pocket_detection.hpp"

#include <algorithm>
#include <cmath>

#include "angles/angles.h"

namespace tetra_docking
{

namespace
{

// 입구 앞으로 이만큼까지 다리 띠로 본다. 사전 자세의 앞뒤 오차를 받으려는 것이다
constexpr double kAxialMargin = 0.15;
// 직선에서 이보다 먼 점은 버리고 다시 맞춘다 (다리 모서리·안쪽 끝 벽 모서리 점)
constexpr double kInlierDistance = 0.03;
// 처음 한 번은 띠를 넓혀 사전 자세(AMCL 의 등록 위치)가 옆으로 틀어져 있어도 다리를 잡는다
constexpr double kFirstPassBandScale = 2.0;
// 다리 앞 끝은 뒤로 점이 kFrontRun 개 넘게 kFrontGap 안 간격으로 이어지는 첫 점이다.
// 다리 앞에 떨어진 바닥 반사 한두 점이 입구를 앞으로 당기지 않게 한다
constexpr double kFrontGap = 0.06;
constexpr size_t kFrontRun = 3;

struct LegFit
{
  bool ok{false};
  double mean_u{0.0};
  double mean_v{0.0};
  /** 사전 축 기준 다리 방향 (rad, -pi/2..pi/2) */
  double angle{0.0};
  std::vector<Point2> points;  // 사전 프레임 (u 축 방향, v 왼쪽)
};

/** 점들을 주성분 방향의 직선으로 맞춘다 */
LegFit fitLine(const std::vector<Point2> & pts, const PocketParams & params)
{
  LegFit fit;
  fit.points = pts;
  if (static_cast<int>(pts.size()) < params.min_leg_points) {
    return fit;
  }
  double su = 0.0;
  double sv = 0.0;
  double min_u = pts.front().x;
  double max_u = pts.front().x;
  for (const auto & p : pts) {
    su += p.x;
    sv += p.y;
    min_u = std::min(min_u, p.x);
    max_u = std::max(max_u, p.x);
  }
  if (max_u - min_u < params.min_leg_span) {
    return fit;
  }
  fit.mean_u = su / pts.size();
  fit.mean_v = sv / pts.size();
  double cuu = 0.0;
  double cuv = 0.0;
  double cvv = 0.0;
  for (const auto & p : pts) {
    const double du = p.x - fit.mean_u;
    const double dv = p.y - fit.mean_v;
    cuu += du * du;
    cuv += du * dv;
    cvv += dv * dv;
  }
  double angle = 0.5 * std::atan2(2.0 * cuv, cuu - cvv);
  // 직선 방향은 부호가 없으므로 축 쪽(-pi/2..pi/2)으로 맞춘다
  if (angle > M_PI / 2) {
    angle -= M_PI;
  } else if (angle < -M_PI / 2) {
    angle += M_PI;
  }
  fit.angle = angle;
  fit.ok = true;
  return fit;
}

/** 한 번 맞춘 뒤 직선에서 먼 점을 버리고 다시 맞춘다 */
LegFit fitLeg(const std::vector<Point2> & pts, const PocketParams & params)
{
  const LegFit first = fitLine(pts, params);
  if (!first.ok) {
    return first;
  }
  const double c = std::cos(first.angle);
  const double s = std::sin(first.angle);
  std::vector<Point2> inliers;
  for (const auto & p : pts) {
    const double distance = std::abs(-s * (p.x - first.mean_u) + c * (p.y - first.mean_v));
    if (distance <= kInlierDistance) {
      inliers.push_back(p);
    }
  }
  return fitLine(inliers, params);
}

/** 축(center, 방향 a)을 따라 잰 다리 점의 앞 끝 좌표 */
double legFrontAlongAxis(const LegFit & leg, double cu, double cv, double a)
{
  std::vector<double> s;
  s.reserve(leg.points.size());
  for (const auto & p : leg.points) {
    s.push_back((p.x - cu) * std::cos(a) + (p.y - cv) * std::sin(a));
  }
  std::sort(s.begin(), s.end());
  for (size_t i = 0; i + kFrontRun < s.size(); ++i) {
    bool continuous = true;
    for (size_t k = i; k < i + kFrontRun; ++k) {
      continuous = continuous && s[k + 1] - s[k] <= kFrontGap;
    }
    if (continuous) {
      return s[i];
    }
  }
  return s.front();
}

PocketDetection detectOnce(const std::vector<Point2> & points, const PocketPose & prior, const PocketParams & params, double band)
{
  PocketDetection result;
  const double c = std::cos(prior.yaw);
  const double s = std::sin(prior.yaw);
  const double half = params.inner_width / 2.0;
  std::vector<Point2> left;
  std::vector<Point2> right;
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    const double dx = p.x - prior.x;
    const double dy = p.y - prior.y;
    const Point2 q{c * dx + s * dy, -s * dx + c * dy};
    // 안쪽 끝 벽(있으면 u = leg_length)의 점이 다리 띠에 섞이면 직선이 기운다. 띠 폭만큼 앞에서 끊는다
    if (q.x < -kAxialMargin || q.x > params.leg_length - band) {
      continue;
    }
    if (std::abs(q.y - half) <= band) {
      left.push_back(q);
    } else if (std::abs(q.y + half) <= band) {
      right.push_back(q);
    }
  }
  result.left_points = static_cast<int>(left.size());
  result.right_points = static_cast<int>(right.size());

  const LegFit lf = fitLeg(left, params);
  const LegFit rf = fitLeg(right, params);
  if (!lf.ok || !rf.ok) {
    result.reason = !lf.ok && !rf.ok ? "both legs not seen" : (!lf.ok ? "left leg not seen" : "right leg not seen");
    return result;
  }
  if (std::abs(lf.angle - rf.angle) > params.max_leg_angle_diff) {
    result.reason = "legs are not parallel";
    return result;
  }
  const double a = 0.5 * (lf.angle + rf.angle);
  if (std::abs(a) > params.max_yaw_change) {
    result.reason = "pocket axis turned too far from the expected pose";
    return result;
  }
  // 두 다리 직선을 같은 축 좌표(두 평균의 가운데)에서 잰다
  const double cu = 0.5 * (lf.mean_u + rf.mean_u);
  const double vl = lf.mean_v + std::tan(a) * (cu - lf.mean_u);
  const double vr = rf.mean_v + std::tan(a) * (cu - rf.mean_u);
  const double cv = 0.5 * (vl + vr);
  result.width = (vl - vr) * std::cos(a);
  if (std::abs(result.width - params.inner_width) > params.max_width_error) {
    result.reason = "leg gap does not match inner_width";
    return result;
  }
  // 입구는 두 다리 앞 끝 중 더 앞쪽: 한쪽 앞 끝이 가려져 안쪽으로 보이면 도크 자세가 깊어져 안쪽 벽에 닿을 수 있다
  const double front = std::min(legFrontAlongAxis(lf, cu, cv, a), legFrontAlongAxis(rf, cu, cv, a));
  const double eu = cu + front * std::cos(a);
  const double ev = cv + front * std::sin(a);

  result.entrance = composePose(prior, PocketPose{eu, ev, a});
  result.found = true;
  return result;
}

}  // namespace

PocketPose composePose(const PocketPose & frame_a_in_b, const PocketPose & pose_in_a)
{
  const double c = std::cos(frame_a_in_b.yaw);
  const double s = std::sin(frame_a_in_b.yaw);
  return {
    frame_a_in_b.x + c * pose_in_a.x - s * pose_in_a.y,
    frame_a_in_b.y + s * pose_in_a.x + c * pose_in_a.y,
    angles::normalize_angle(frame_a_in_b.yaw + pose_in_a.yaw)};
}

PocketPose invertPose(const PocketPose & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y, s * pose.x - c * pose.y, angles::normalize_angle(-pose.yaw)};
}

PocketDetection detectPocket(const std::vector<Point2> & points, const PocketPose & prior, const PocketParams & params)
{
  // 넓은 띠로 대강 찾은 뒤, 그 자세로 다시 좁은 띠를 써서 옆 물체(짐 받침·벽)가 섞이지 않게 한다
  const PocketDetection rough = detectOnce(points, prior, params, params.band_half_width * kFirstPassBandScale);
  if (!rough.found) {
    return rough;
  }
  PocketDetection fine = detectOnce(points, rough.entrance, params, params.band_half_width);
  if (!fine.found) {
    return rough;
  }
  // 두 번째 판정은 첫 결과 기준이라, 사전 자세에서 돈 각도는 다시 잰다
  if (std::abs(angles::normalize_angle(fine.entrance.yaw - prior.yaw)) > params.max_yaw_change) {
    fine.found = false;
    fine.reason = "pocket axis turned too far from the expected pose";
  }
  return fine;
}

SideClearance footprintClearance(
  const std::vector<Point2> & points, const RectFootprint & footprint, double side_reach, double front_reach)
{
  SideClearance clearance;
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    if (p.x >= footprint.back && p.x <= footprint.front) {
      const double gap = std::abs(p.y) - footprint.half_width;
      if (gap > side_reach) {
        continue;
      }
      if (p.y >= 0.0) {
        clearance.left = std::min(clearance.left, gap);
      } else {
        clearance.right = std::min(clearance.right, gap);
      }
    } else if (p.x > footprint.front && std::abs(p.y) <= footprint.half_width) {
      const double gap = p.x - footprint.front;
      if (gap <= front_reach) {
        clearance.front = std::min(clearance.front, gap);
      }
    }
  }
  return clearance;
}

bool entryPoseClear(
  const PocketPose & robot_in_pocket, const RectFootprint & footprint, double inner_width, double margin, double lookahead)
{
  const double c = std::cos(robot_in_pocket.yaw);
  const double s = std::sin(robot_in_pocket.yaw);
  const double limit = inner_width / 2.0 - margin;
  const double xs[2] = {footprint.front + lookahead, footprint.back};
  const double ys[2] = {footprint.half_width, -footprint.half_width};
  for (double x : xs) {
    for (double y : ys) {
      const double u = robot_in_pocket.x + c * x - s * y;
      const double v = robot_in_pocket.y + s * x + c * y;
      if (u >= 0.0 && std::abs(v) > limit) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace tetra_docking

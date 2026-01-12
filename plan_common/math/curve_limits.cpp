
#include <cfloat>
#include <iostream>

#include "plan_common/math/curve_limits.h"
#include "plan_common/math/linear_interpolation.h"

namespace ad_byd {
namespace planning {
bool CurveLimit::Create(const LineCurve1d &lower_boundary,
                        const LineCurve1d &upper_boundary, double s_interval) {
  boundary_map_.clear();
  if (!lower_boundary.IsValid() && !upper_boundary.IsValid()) {
    std::cout << "No valid boundary, create CurveLimit fail!" << std::endl;
    return false;
  }
  // 1. Get boundary range
  double s_begin = DBL_MAX;
  double s_end = -DBL_MAX;
  double begin, end;
  if (!lower_boundary.GetRange(&begin, &end)) {
    return false;
  }
  s_begin = std::min(s_begin, begin);
  s_end = std::max(s_end, end);

  if (!upper_boundary.GetRange(&begin, &end)) {
    return false;
  }
  s_begin = std::min(s_begin, begin);
  s_end = std::max(s_end, end);

  // 2. Resample
  double s = s_begin;
  while (s < s_end + Constants::ZERO) {
    double lower_bound = lower_boundary.GetValue(s);
    double upper_bound = upper_boundary.GetValue(s);
    boundary_map_[s] = std::move(std::make_pair(upper_bound, lower_bound));
    s += s_interval;
  }
  return true;
}

void CurveLimit::SetBoundary(const double &x, const double &lower,
                             const double &upper) {
  if (lower < upper) {
    // remove equal key
    auto it = boundary_map_.lower_bound(x - 1e-5);
    if (it != boundary_map_.end() && std::abs(x - it->first) < 1e-2) {
      boundary_map_.erase(it);
    }
    boundary_map_[x] = std::make_pair(upper, lower);
  }
}

bool CurveLimit::GetBoundary(const double &x, double *lower,
                             double *upper) const {
  if (boundary_map_.empty()) {
    return false;
  }
  if (boundary_map_.size() == 1) {
    if (lower) {
      *lower = boundary_map_.begin()->second.second;
    }
    if (upper) {
      *upper = boundary_map_.begin()->second.first;
    }
    return true;
  }
  auto it0 = boundary_map_.lower_bound(x);
  if (it0 == boundary_map_.begin()) {
    if (lower) {
      *lower = boundary_map_.begin()->second.second;
    }
    if (upper) {
      *upper = boundary_map_.begin()->second.first;
    }
    return true;
  } else if (it0 == boundary_map_.end()) {
    if (lower) {
      *lower = boundary_map_.rbegin()->second.second;
    }
    if (upper) {
      *upper = boundary_map_.rbegin()->second.first;
    }
    return true;
  }
  auto it1 = it0;
  it0--;
  if (lower) {
    *lower = math::lerp(it0->second.second, it0->first, it1->second.second,
                        it1->first, x, true);
  }
  if (upper) {
    *upper = math::lerp(it0->second.first, it0->first, it1->second.first,
                        it1->first, x, true);
  }
  return true;
}
bool CurveLimit::GetBoundary(
    std::vector<ad_byd::planning::Point2d> &lower,
    std::vector<ad_byd::planning::Point2d> &upper) const {
  if (boundary_map_.empty()) {
    return false;
  }
  lower.reserve(boundary_map_.size());
  upper.reserve(boundary_map_.size());
  for (const auto &it : boundary_map_) {
    lower.emplace_back(it.first, it.second.second);
    upper.emplace_back(it.first, it.second.first);
  }
  return true;
}
bool CurveLimit::GetBoundary(std::vector<double> &x, std::vector<double> &lower,
                             std::vector<double> &upper) const {
  if (boundary_map_.empty()) {
    return false;
  }
  x.clear();
  lower.clear();
  upper.clear();
  x.reserve(boundary_map_.size());
  lower.reserve(boundary_map_.size());
  upper.reserve(boundary_map_.size());
  for (const auto &it : boundary_map_) {
    x.push_back(it.first);
    lower.emplace_back(it.second.second);
    upper.emplace_back(it.second.first);
  }
  return true;
}

bool CurveLimit::GetBoundary(std::map<double, double> &left_limit,
                             std::map<double, double> &right_limit) const {
  if (boundary_map_.empty()) {
    return false;
  }
  for (const auto &it : boundary_map_) {
    left_limit[it.first] = it.second.first;
    right_limit[it.first] = it.second.second;
  }
  return true;
}

bool CurveLimit::GetRange(double *begin, double *end) const {
  if (boundary_map_.empty()) {
    return false;
  }
  if (begin) {
    *begin = boundary_map_.begin()->first;
  }
  if (end) {
    *end = boundary_map_.rbegin()->first;
  }
  return true;
}

}  // namespace planning
}  // namespace ad_byd
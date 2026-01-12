

#include "plan_common/maps/lane.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "plan_common/log.h"
#include "plan_common/log.h"
#include "plan_common/math/double.h"
#include "plan_common/math/linear_interpolation.h"

namespace ad_byd {
namespace planning {
using Vec2d = math::Vec2d;
Lane::Lane(const LaneInfo &lane_info, LaneBoundariesConstPtr left_boundary,
           LaneBoundariesConstPtr right_boundary,
           RoadBoundariesConstPtr left_road_boundary,
           RoadBoundariesConstPtr right_road_boundary) {
  lane_info_ = lane_info;
  left_boundary_ = left_boundary;
  right_boundary_ = right_boundary;
  left_road_boundary_ = left_road_boundary;
  right_road_boundary_ = right_road_boundary;
  center_line_.InitializePoints(lane_info.points);
  GenerateSampleWidth(left_boundary_, &sampled_left_width_);
  GenerateSampleWidth(right_boundary_, &sampled_right_width_);
}

bool Lane::IsValid() const {
  return center_line_.points().size() >= 2u && left_boundary_ != nullptr &&
         right_boundary_ != nullptr /*&&
         !left_boundary_->lane_boundaries().empty() &&
         !right_boundary_->lane_boundaries().empty()*/
      ;
}

void Lane::UpdateCenterLinePoints(const std::vector<Point2d> &points) {
  lane_info_.points = points;
  center_line_.InitializePoints(points);
  GenerateSampleWidth(left_boundary_, &sampled_left_width_);
  GenerateSampleWidth(right_boundary_, &sampled_right_width_);
}

bool Lane::IsBoundaryVirtual() const {
  const auto is_virtual = [](const auto boundary) {
    if (boundary == nullptr) return false;
    const auto &boundary_types = boundary->boundary_types();
    if (boundary_types.empty()) return false;
    for (const auto &boundary_type : boundary_types) {
      if (boundary_type.line_type == LineType::VIRTUAL_LANE ||
          boundary_type.line_type == LineType::UNKNOWN) {
        return true;
      }
    }
    return false;
  };
  return is_virtual(left_boundary_) || is_virtual(right_boundary_);
}

void Lane::forward_boundary_type(
    bool is_left, double s,
    std::vector<LaneBoundaryType> &boundary_type) const {
  if (!center_line_.IsValid()) return;
  auto pt = center_line_.GetPointAtS(s);
  forward_boundary_type(is_left, pt, boundary_type);
}

void Lane::forward_boundary_type(
    bool is_left, const Point2d &point,
    std::vector<LaneBoundaryType> &boundary_type) const {
  const LaneBoundariesConstPtr boundary_ptr =
      is_left ? left_boundary_ : right_boundary_;
  if (!center_line_.IsValid() || !boundary_ptr) return;
  double project_s = 0.0, project_l = 0.0;
  boundary_ptr->line_curve().GetProjection(point, &project_s, &project_l);
  const auto &all_boundary_type = boundary_ptr->boundary_types();
  int curr_index = boundary_ptr->GetBoundarySegmentIndex(project_s);
  if (curr_index < 0) return;
  for (int idx = curr_index; idx < static_cast<int>(all_boundary_type.size());
       idx++) {
    const auto &curr_bound_type = all_boundary_type[idx];
    boundary_type.emplace_back(LaneBoundaryType{curr_bound_type.s - project_s,
                                                curr_bound_type.line_type,
                                                curr_bound_type.line_color});
  }
}

bool Lane::IsOnLane(const math::Vec2d &point) const {
  if (!IsValid()) return false;
  double s = 0.0;
  double l = 0.0;
  center_line_.GetProjection(point, &s, &l);
  if (s < 0.0 || s > center_line_.length()) return false;
  double lw = 0.0;
  double rw = 0.0;
  GetWidthFromS(s, &lw, &rw);
  return math::Double::Compare(lw - l, 0.0) !=
             math::Double::CompareType::LESS &&
         math::Double::Compare(l + rw, 0.0) != math::Double::CompareType::LESS;
}

void Lane::GetWidthAtAccumS(double s, double *left_w, double *right_w) const {
  const double lane_width = 3.5;
  *left_w = 0.5 * lane_width;
  *right_w = 0.5 * lane_width;
  if (!center_line_.IsValid()) {
    // LOG_INFO << "no enough points";
    return;
  }
  Point2d start_p, end_p;
  int idx = center_line_.GetIndexByS(s);
  if (idx < 0) return;
  if (idx == 0) idx = 1;
  start_p = lane_info_.points[idx - 1];
  end_p = lane_info_.points[idx];
  math::LineSegment2d base_vec(start_p, end_p);
  if (left_boundary_ && left_boundary_->line_curve().IsValid()) {
    *left_w = CalculateWidth(base_vec, start_p, 1);
  }
  if (right_boundary_ && right_boundary_->line_curve().IsValid()) {
    *right_w = CalculateWidth(base_vec, start_p, -1);
  }
  //   LOG_INFO << "left_w : " << left_w << "right_w : " << right_w;
}

double Lane::GetWidthAtPoint(double x, double y) const {
  double accum_s, l;
  center_line_.GetProjection(Point2d{x, y}, &accum_s, &l);
  return GetWidthAtAccumS(accum_s);
}

double Lane::GetWidthAtAccumS(double s) const {
  double left_width = 0.0, right_width = 0.0;
  GetWidthAtAccumS(s, &left_width, &right_width);
  return left_width + right_width;
}

double Lane::CalculateWidth(const math::LineSegment2d &base, const Point2d &p,
                            int lr) const {
  auto natural_extend = [](const Point2d &p0, const Point2d &p1,
                           const double &length) {
    auto rel_x = p1.x() - p0.x();
    auto rel_y = p1.y() - p0.y();
    auto rel_dis = std::hypot(rel_x, rel_y);
    return Point2d{p0.x() - rel_x / rel_dis * length,
                   p0.y() - rel_y / rel_dis * length};
  };
  double res_l = 1.75;
  math::LineSegment2d normal_vec(
      p, base.rotate_expand(std::copysign(0.5 * M_PI, lr), 5.0));
  LaneBoundariesConstPtr lane_boundary =
      lr > 0 ? left_boundary_ : right_boundary_;
  auto boundary_pts = lane_boundary->line_curve().points();
  if (boundary_pts.size() > 1) {
    const double extend_dist = 5.0;
    // extend front
    auto front_extend_pt =
        natural_extend(boundary_pts.at(0), boundary_pts.at(1), extend_dist);
    boundary_pts.insert(boundary_pts.begin(), front_extend_pt);
    // extend back
    auto back_extend_pt =
        natural_extend(boundary_pts.at(boundary_pts.size() - 1),
                       boundary_pts.at(boundary_pts.size() - 2), extend_dist);
    boundary_pts.emplace_back(back_extend_pt);
  }
  math::Vec2d intersect_p;
  for (int i = 0; i < static_cast<int>(boundary_pts.size()) - 1; ++i) {
    math::LineSegment2d temp_vec(boundary_pts[i], boundary_pts[i + 1]);
    if (normal_vec.HasIntersect(temp_vec)) {
      normal_vec.GetIntersect(temp_vec, &intersect_p);
      res_l = std::hypot(intersect_p.x() - p.x(), intersect_p.y() - p.y());
    }
  }
  return res_l;
}

bool Lane::IsPrecede(const uint64_t lane_id) const {
  auto it = std::find(prev_lane_ids_.begin(), prev_lane_ids_.end(), lane_id);
  return it != prev_lane_ids_.end();
}

bool Lane::IsNext(const uint64_t lane_id) const {
  auto it = std::find(lane_info_.next_lane_ids.begin(),
                      lane_info_.next_lane_ids.end(), lane_id);
  return it != lane_info_.next_lane_ids.end();
}

bool Lane::GetSLWithLimit(const math::Vec2d &query_point,
                          SLPoint *const sl_point) const {
  if (!IsValid()) return false;
  center_line_.GetProjection(query_point, &(sl_point->s), &(sl_point->l));
  if (sl_point->s < 0.0 || sl_point->s > center_line_.length()) {
    sl_point->s = std::clamp(sl_point->s, 0.0, center_line_.length());
    return false;
  }
  return true;
}

bool Lane::GetSLWithoutLimit(const math::Vec2d &query_point,
                             SLPoint *const sl_point) const {
  if (!IsValid()) return false;
  center_line_.GetProjection(query_point, &(sl_point->s), &(sl_point->l));
  return true;
}

bool Lane::GetXYWithoutLimit(const SLPoint &sl_point,
                             math::Vec2d *const xy_point) const {
  if (!IsValid()) return false;
  center_line_.GetPoint(sl_point.s, sl_point.l, xy_point);
  return true;
}

bool Lane::GetHeadingFromS(double query_s, double *heading) const {
  return center_line_.GetHeadingFromS(query_s, heading);
}

bool Lane::GetWidthFromS(double query_s, double *lw, double *rw) const {
  if (sampled_left_width_.size() < 2u || sampled_right_width_.size() < 2u)
    return false;
  if (query_s < 0.0) {
    *lw = sampled_left_width_.front().dist;
    *rw = sampled_right_width_.front().dist;
    return true;
  } else if (query_s > center_line_.length()) {
    *lw = sampled_left_width_.back().dist;
    *rw = sampled_right_width_.back().dist;
    return true;
  }
  *lw = GetDistanceFromSample(query_s, &sampled_left_width_);
  *rw = GetDistanceFromSample(query_s, &sampled_right_width_);
  return true;
}

double Lane::GetDistanceFromSample(
    double query_s, const std::list<SampledWidth> *const sampled_widths) const {
  if (sampled_widths->empty()) {
    return 0.0;
  }
  double dist = sampled_widths->front().dist;
  double s = sampled_widths->front().s;
  for (const auto &sampled_width : *sampled_widths) {
    if (sampled_width.s > query_s) {
      double s_scope = std::max(sampled_width.s - s, 1e-6);
      dist = (sampled_width.dist - dist) * (query_s - s) / s_scope + dist;
      break;
    }
    dist = sampled_width.dist;
    s = sampled_width.s;
  }
  return dist;
}

void Lane::GenerateSampleWidth(const LaneBoundariesConstPtr &boundary,
                               std::list<SampledWidth> *const sample_widths) {
  const auto &boundary_curve = boundary->line_curve();
  // in case junction lane has no boundary
  const double kDefaultHalfLaneWidth = 1.875;
  if (!boundary_curve.IsValid() && lane_info_.junction_id != 0 &&
      sample_widths->empty() && center_line_.IsValid()) {
    double s = 0.0;
    for (int i = 0; i < center_line_.points().size(); i++) {
      if (i >= 1) {
        s += std::hypot((center_line_.points().at(i).x() -
                         center_line_.points().at(i - 1).x()),
                        (center_line_.points().at(i).y() -
                         center_line_.points().at(i - 1).y()));
      }
      SampledWidth sample_width;
      sample_width.s = s;
      sample_width.dist = kDefaultHalfLaneWidth;
      sample_widths->emplace_back(std::move(sample_width));
    }
    return;
  }
  if (!(boundary_curve.IsValid() && center_line_.IsValid() &&
        sample_widths->empty())) {
    return;
  }
  const auto &lane_points = center_line_.points();
  const auto &boundary_points = boundary_curve.points();
  std::size_t lane_pt_idx = 0;
  std::size_t bound_pt_idx = 0;
  double accumulate_s = 0.0;
  math::LineSegment2d lane_seg(lane_points.at(0), lane_points.at(1));
  const std::size_t max_lane_pt_idx = lane_points.size() - 1u;
  const std::size_t max_bound_pt_idx = boundary_points.size();
  const double min_s_dist = 1.0;
  bool has_input_lane_pt = false;
  while (lane_pt_idx < max_lane_pt_idx && bound_pt_idx < max_bound_pt_idx) {
    const auto prod =
        lane_seg.ProductOntoUnit(boundary_points.at(bound_pt_idx));
    const auto proj =
        lane_seg.ProjectOntoUnit(boundary_points.at(bound_pt_idx));
    // process 1st lane center point
    if (!has_input_lane_pt &&
        (sample_widths->empty() ||
         accumulate_s - sample_widths->back().s > min_s_dist) &&
        proj > 0.0 && proj < lane_seg.length() && bound_pt_idx > 0) {
      const auto &start_bound_pt = boundary_points.at(bound_pt_idx - 1);
      double start_prod = lane_seg.ProductOntoUnit(start_bound_pt);
      double start_proj = lane_seg.ProjectOntoUnit(start_bound_pt);
      SampledWidth sample_width;
      sample_width.s = accumulate_s;
      double s_scope = proj - start_proj;
      s_scope =
          s_scope < 0.0 ? std::min(-1e-6, s_scope) : std::max(1e-6, s_scope);
      sample_width.dist =
          std::fabs((-start_proj * (prod - start_prod) / s_scope) + start_prod);
      sample_widths->emplace_back(std::move(sample_width));
      has_input_lane_pt = true;
    } else if (!has_input_lane_pt &&
               ((sample_widths->empty() ||
                 accumulate_s - sample_widths->back().s > min_s_dist)) &&
               proj > 0.0 && proj < lane_seg.length()) {
      SampledWidth sample_width;
      sample_width.s = accumulate_s;
      sample_width.dist = std::fabs(prod);
      sample_widths->emplace_back(std::move(sample_width));
      has_input_lane_pt = true;
    }

    bool bound_in_lane = proj < lane_seg.length();
    if (bound_in_lane &&
        (sample_widths->empty() ||
         accumulate_s + proj - sample_widths->back().s > min_s_dist)) {
      // process boundary point
      SampledWidth sample_width;
      sample_width.s = proj + accumulate_s;
      sample_width.dist = std::fabs(prod);
      sample_widths->emplace_back(std::move(sample_width));
    }

    if (bound_in_lane) {
      ++bound_pt_idx;
    } else if (lane_pt_idx + 1 < max_lane_pt_idx) {
      accumulate_s += lane_seg.length();
      ++lane_pt_idx;
      lane_seg.Reset(lane_points.at(lane_pt_idx),
                     lane_points.at(lane_pt_idx + 1));
      has_input_lane_pt = false;
    } else {
      ++lane_pt_idx;
      break;
    }
  }

  if (sample_widths->empty()) return;
  if (bound_pt_idx >= max_bound_pt_idx) {
    ++lane_pt_idx;
  }

  // process remain lane points
  while (lane_pt_idx < lane_points.size() && lane_pt_idx > 0) {
    lane_seg.Reset(lane_points.at(lane_pt_idx - 1),
                   lane_points.at(lane_pt_idx));
    if (bound_pt_idx >= max_bound_pt_idx) {
      SampledWidth sample_width;
      sample_width.s = accumulate_s + lane_seg.length();
      sample_width.dist = sample_widths->back().dist;
      if (sample_width.s - sample_widths->back().s < min_s_dist &&
          sample_widths->size() > 1u) {
        sample_widths->pop_back();
      }
      ++lane_pt_idx;
      accumulate_s += lane_seg.length();
      sample_widths->emplace_back(std::move(sample_width));
      continue;
    }
    const auto prod =
        lane_seg.ProductOntoUnit(boundary_points.at(bound_pt_idx));
    const auto proj =
        lane_seg.ProjectOntoUnit(boundary_points.at(bound_pt_idx));
    double s_scope = proj + accumulate_s - sample_widths->back().s;
    s_scope =
        s_scope < 0.0 ? std::min(-1e-6, s_scope) : std::max(1e-6, s_scope);
    SampledWidth sample_width;
    sample_width.s = accumulate_s + lane_seg.length();
    if (sample_width.s - sample_widths->back().s < min_s_dist &&
        sample_widths->size() > 1u) {
      sample_widths->pop_back();
    }
    sample_width.dist = (std::fabs(prod) - sample_widths->back().dist) *
                            (sample_width.s - sample_widths->back().s) /
                            s_scope +
                        sample_widths->back().dist;
    sample_widths->emplace_back(std::move(sample_width));
    break;
  }
  // check 1st sample
  const auto cmp_type = math::Double::Compare(sample_widths->front().s, 0.0);
  if (cmp_type == math::Double::CompareType::GREATER) {
    SampledWidth sample_width;
    sample_width.s = 0.0;
    sample_width.dist = sample_widths->front().dist;
    if (sample_widths->front().s < min_s_dist) {
      sample_widths->pop_front();
    }
    sample_widths->emplace_front(std::move(sample_width));
  } else if (cmp_type == math::Double::CompareType::LESS &&
             sample_widths->size() > 1u) {
    double prev_dist = sample_widths->front().dist;
    double prev_s = sample_widths->front().s;
    sample_widths->pop_front();
    while (sample_widths->front().s < 0.0 && sample_widths->size() > 1u) {
      prev_dist = sample_widths->front().dist;
      prev_s = sample_widths->front().s;
      sample_widths->pop_front();
    }
    if (sample_widths->front().s > 0.0) {
      SampledWidth sample_width;
      sample_width.s = 0.0;
      double s_scope = std::max(sample_widths->front().s - prev_s, 1e-6);
      sample_width.dist =
          -prev_s * (sample_widths->front().dist - prev_dist) / s_scope +
          prev_dist;
      sample_widths->emplace_front(std::move(sample_width));
    }

    if (sample_widths->back().s < 0.0) {
      SampledWidth sample_width;
      sample_width.s = 0.0;
      sample_width.dist = sample_widths->back().dist;
      sample_widths->emplace_back(std::move(sample_width));
    }
  }

  // for (const auto &sample : *sample_widths) {
  //   LINFO("Lane %s sample width %.2f, l dist %.2f total lane s %.2f",
  //   lane_info_.id.c_str(), sample.s, sample.dist,
  //         curve_length());
  // }
}

double Lane::LaneFraction(int segment, double segment_fraction) const {
  CHECK_GE(segment_fraction, 0.0);
  CHECK_LE(segment_fraction, 1.0);
  return st::Lerp(center_line_.GetAccuLength()[segment],
                  center_line_.GetAccuLength()[segment + 1], segment_fraction) /
         center_line_.GetAccuLength().back();
}

std::pair<int, double> Lane::SegmentFraction(double lane_fraction) const {
  const auto &s_vec = center_line_.GetAccuLength();
  CHECK_GE(s_vec.size(), 2);
  CHECK_GE(lane_fraction, 0.0);
  CHECK_LE(lane_fraction, 1.0);
  const double cum_length = s_vec.back() * lane_fraction;
  int index =
      std::upper_bound(s_vec.begin(), s_vec.end(), cum_length) - s_vec.begin();
  CHECK_GT(index, 0);
  if (index == s_vec.size()) {
    return {s_vec.size() - 2, 1.0};
  }
  --index;
  CHECK_LT(index + 1, s_vec.size());
  return {index,
          (cum_length - s_vec[index]) / (s_vec[index + 1] - s_vec[index])};
}

bool Lane::GetTangent(double fraction, math::Vec2d *tangent) const {
  const auto [index, _] = SegmentFraction(fraction);
  const math::Vec2d &prev_pt = points()[index];
  const math::Vec2d &next_pt = points()[index + 1];
  *tangent = (next_pt - prev_pt).normalized();

  return true;
}

math::Vec2d Lane::GetTangent(double fraction) const {
  const int index = SegmentFraction(fraction).first;
  CHECK_GE(index, 0);
  CHECK_LT(index + 1, points().size());
  const math::Vec2d prev_to_next = points()[index + 1] - points()[index];

  constexpr double kLengthSqrEpsilon = 1e-9;
  return (prev_to_next.Sqr() < kLengthSqrEpsilon) ? math::Vec2d(0.0, 1.0)
                                                  : prev_to_next.normalized();
}

math::Vec2d Lane::LerpPointFromFraction(double fraction) const {
  if (points().size() == 1) {
    return points()[0];
  }
  const auto index = SegmentFraction(fraction);
  return st::Lerp(points()[index.first], points()[index.first + 1],
                  index.second);
}

bool Lane::GetCustomSpeedLimit(double *custom_speed_limit) const {
  if (!custom_speed_limit) return false;
  if (lane_info_.lane_operation_type & (1LL << 3)) {
    *custom_speed_limit = 30.0 / 3.6;
    return true;
  }
  if (lane_info_.lane_operation_type & (1LL << 4)) {
    *custom_speed_limit = 40.0 / 3.6;
    return true;
  }
  if (lane_info_.lane_operation_type & (1LL << 5)) {
    *custom_speed_limit = 50.0 / 3.6;
    return true;
  }
  if (lane_info_.lane_operation_type & (1LL << 6)) {
    *custom_speed_limit = 60.0 / 3.6;
    return true;
  }
  if (lane_info_.lane_operation_type & (1LL << 7)) {
    *custom_speed_limit = 70.0 / 3.6;
    return true;
  }
  if (lane_info_.lane_operation_type & (1LL << 8)) {
    *custom_speed_limit = 80.0 / 3.6;
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace ad_byd



#include "plan_common/gflags.h"
#include "plan_common/maps/road_boundary.h"
namespace ad_byd {
namespace planning {

RoadBoundary::RoadBoundary(const RoadBoundaryInfo &road_boundary_info) {
  points_.assign(road_boundary_info.points.begin(),
                 road_boundary_info.points.end());
  id_ = road_boundary_info.id;
  type_.boundary_type = road_boundary_info.boundary_type;
  type_.width = road_boundary_info.width;
  line_curve_.InitializePoints(points_);
  curve_length_ = line_curve_.length();
}
void RoadBoundary::InterpolatePoints() {
  std::vector<Point2d> interpolate_points;
  interpolate_points.emplace_back(points_.front());
  const double map_gap_threshold_sqr =
      FLAGS_ad_byd_planning_map_point_distance_threshold *
      FLAGS_ad_byd_planning_map_point_distance_threshold;
  for (int32_t i = 1; i < static_cast<int32_t>(points_.size()); ++i) {
    auto &cur_pt = points_.at(i);
    const auto &last_pt = interpolate_points.back();
    double cur_length = cur_pt.DistanceTo(last_pt);
    int32_t insert_cnt = static_cast<int32_t>(
        cur_length / FLAGS_ad_byd_planning_boundary_interpolate_dist);
    const auto &norm_vector =
        (cur_pt - last_pt) /
        (cur_length + FLAGS_ad_byd_planning_zero_threshold) *
        FLAGS_ad_byd_planning_boundary_interpolate_dist;
    for (int32_t j = 0; j < insert_cnt; ++j) {
      const auto &insert_point = last_pt + norm_vector * (j + 1);
      interpolate_points.emplace_back(insert_point);
    }
    if (cur_pt.DistanceSquareTo(interpolate_points.back()) >
        map_gap_threshold_sqr) {
      interpolate_points.emplace_back(cur_pt);
    }
  }
  line_curve_.InitializePoints(interpolate_points);
  curve_length_ = line_curve_.length();
}

bool RoadBoundary::GetRoadBoundaryPoints(
    const std::vector<Point2d> &center_pts,
    const std::vector<math::LineSegment2d> &road_segs,
    const std::pair<double, double> &lat_dist_range,
    std::vector<Point2d> *road_pts) {
  road_pts->clear();
  if (lat_dist_range.first < lat_dist_range.second) {
    LOG_ERROR << "<GetRoadBoundaryPoints> lat_dist_range errror";
  }
  // Get crossprod_vecs by center_pts and lat_range.
  double kDefaultDist = Constants::DEFAULT_LANE_WIDTH;
  std::vector<math::LineSegment2d> crossprod_vecs;
  for (int i = 1; i < center_pts.size(); i++) {
    math::Vec2d center_vec = center_pts[i] - center_pts[i - 1];
    center_vec.Normalize();
    const math::Vec2d lat_vec(-center_vec.y(), center_vec.x());
    crossprod_vecs.emplace_back(center_pts[i - 1] + kDefaultDist * lat_vec,
                                center_pts[i - 1] - kDefaultDist * lat_vec);
    if (i + 1 == center_pts.size()) {  // terminal vec
      crossprod_vecs.emplace_back(center_pts[i] + kDefaultDist * lat_vec,
                                  center_pts[i] - kDefaultDist * lat_vec);
    }
  }
  // Get intersect_pt and dist_min.
  bool has_in_range = false;
  for (const auto &cross_vec : crossprod_vecs) {
    double dist_min = DBL_MAX;
    Point2d road_pt;
    for (const auto &road_seg : road_segs) {
      Point2d intersect_pt;
      if (cross_vec.GetIntersect(road_seg, &intersect_pt)) {
        const double dist =
            kDefaultDist - cross_vec.start().DistanceTo(intersect_pt);
        if (std::abs(dist) < std::abs(dist_min)) {
          road_pt = intersect_pt;
          dist_min = dist;
        }
      }
    }
    // Check if dist in range.
    if (dist_min < lat_dist_range.first && dist_min > lat_dist_range.second) {
      road_pts->emplace_back(road_pt);
      has_in_range = true;
    }
  }
  return has_in_range;
}
}  // namespace planning
}  // namespace ad_byd
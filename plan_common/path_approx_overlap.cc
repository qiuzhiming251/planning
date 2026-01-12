

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "path_approx_overlap.h"
// IWYU pragma: no_include "Eigen/Core"

#include "path_approx.h"
#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d_util.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/segment_matcher/segment_matcher_kdtree.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"

namespace st {
namespace planning {

namespace {

// Approximates the polygon as a circle and quickly check if we need to compute
// overlap.
bool MaybeOverlap(const Box2d& box, const Polygon2d& polygon,
                  double max_lat_dist) {
  if (Sqr(box.radius() + polygon.CircleRadius() + max_lat_dist) <
      polygon.CircleCenter().DistanceSquareTo(box.center())) {
    return false;
  }
  if (std::abs(box.tangent().CrossProd(polygon.CircleCenter() - box.center())) >
      max_lat_dist + box.half_width() + polygon.CircleRadius()) {
    return false;
  }
  if (std::abs(box.tangent().Dot(polygon.CircleCenter() - box.center())) >
      max_lat_dist + box.half_length() + polygon.CircleRadius()) {
    return false;
  }
  return true;
}

AgentOverlap ConvertToAgentOverlap(
    const PathApprox& path_approx, int segment_index,
    const polygon2d::PolygonBoxOverlap& overlap) {
  const auto& segment = path_approx.segment(segment_index);
  double start_ra_s =
      std::clamp(segment.last_s() - (segment.length() - overlap.in),
                 segment.first_s(), segment.last_s());
  double last_ra_s =
      std::clamp(segment.first_s() + overlap.out, start_ra_s, segment.last_s());
  return AgentOverlap{.first_ra_s = start_ra_s,
                      .last_ra_s = last_ra_s,
                      .ra_heading = segment.heading(),
                      .lat_dist = overlap.lat_dist};
}

double ConvertToHalfPlaneMinRaS(const PathSegment& path_segment, double in) {
  return std::clamp(path_segment.last_s() - (path_segment.length() - in),
                    path_segment.first_s(), path_segment.last_s());
}

bool ClampAgentOverlap(double low, double high, AgentOverlap* overlap) {
  if (low > overlap->last_ra_s || high < overlap->first_ra_s) {
    return false;
  }
  overlap->first_ra_s = std::max(overlap->first_ra_s, low);
  overlap->last_ra_s = std::min(overlap->last_ra_s, high);
  return true;
}

std::optional<std::pair<int, int>> ComputeMinMaxIndexNearPoint(
    const SegmentMatcherKdtree& kd_tree, const Vec2d& search_point,
    double search_radius, int first_index, int last_index) {
  const auto indices = kd_tree.GetSegmentIndexInRadius(
      search_point.x(), search_point.y(), search_radius);
  if (indices.empty()) return std::nullopt;
  const auto [min_index, max_index] =
      std::minmax_element(indices.begin(), indices.end());
  first_index = std::clamp(*min_index, first_index, last_index);
  last_index = std::clamp(*max_index, first_index, last_index);
  return std::pair<int, int>(first_index, last_index);
}

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeadingHelper(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double theta,
    double max_heading_diff) {
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;

  std::vector<AgentOverlap> agent_overlaps;
  double min_abs_dist = std::numeric_limits<double>::max();

  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    polygon2d::PolygonBoxOverlap geom_overlap;
    const auto& path_segment = path_approx.segment(i);
    const double heading_diff = NormalizeAngle(theta - path_segment.heading());
    if (std::abs(heading_diff) > max_heading_diff) {
      // DLOG(INFO) << "Overlapping heading check continue";
      continue;
    }
    if (std::fabs(lat_buffer) < 1e-3 && std::fabs(lon_buffer) < 1e-3) {
      if (!MaybeOverlap(path_segment, polygon, max_lat_dist)) {
        // DLOG(INFO) << "Overlapping1 maybe check continue";
        continue;
      }
      geom_overlap = polygon2d::ComputePolygonBoxOverlap(polygon, path_segment);
    } else {
      const Box2d extended_path_segment(
          path_segment.half_length() + lon_buffer,
          path_segment.half_width() + lat_buffer, path_segment.center(),
          path_segment.heading(), path_segment.tangent());
      if (!MaybeOverlap(extended_path_segment, polygon, max_lat_dist)) continue;
      geom_overlap =
          polygon2d::ComputePolygonBoxOverlap(polygon, extended_path_segment);
      geom_overlap.in -= lon_buffer;
      geom_overlap.out -= lon_buffer;
    }
    // DLOG(INFO) << "i " << i << ", get geom olp " << geom_overlap.in << " "
    //            << geom_overlap.out << ", lon bf " << lon_buffer << " heading
    //            d "
    //            << heading_diff;

    if (geom_overlap.in == geom_overlap.out && geom_overlap.lat_dist == 0.0) {
      continue;
    }

    // Skip if this is not the laterally nearest overlap.
    if (std::abs(geom_overlap.lat_dist) > min_abs_dist) continue;

    min_abs_dist = std::abs(geom_overlap.lat_dist);

    auto agent_overlap = ConvertToAgentOverlap(path_approx, i, geom_overlap);

    if (!ClampAgentOverlap(first_s, last_s, &agent_overlap)) continue;

    agent_overlaps.push_back(agent_overlap);
  }
  // DLOG(INFO) << "Raw overlap : " << agent_overlaps.size();

  // Prune all overlaps that is larger than min_abs_dist.
  agent_overlaps.erase(
      std::remove_if(agent_overlaps.begin(), agent_overlaps.end(),
                     [min_abs_dist](const auto& o) {
                       return std::abs(o.lat_dist) > min_abs_dist;
                     }),
      agent_overlaps.end());
  return agent_overlaps;
}
}  // namespace

std::vector<AgentOverlap> ComputeAgentOverlapsWithBuffer(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
    } else {
      return {};
    }
  }

  return ComputeAgentOverlapsWithBufferAndHeadingHelper(
      path_approx, step_length, first_index, last_index, polygon, max_lat_dist,
      lat_buffer, lon_buffer, /*theta=*/0.0,
      /*max_heading_diff=*/std::numeric_limits<double>::infinity());
}

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff, std::pair<int, int>& min_max_index_res) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
      // DLOG(INFO) << ">> get minmax near point " << polygon.CircleCenter().x()
      //            << " " << polygon.CircleCenter().y() << " // " <<
      //            first_index
      //            << ", " << last_index << ", " << search_radius;
      min_max_index_res = min_max_index.value();
    } else {
      // DLOG(INFO) << "!! cannot find minmax near point "
      //            << polygon.CircleCenter().x() << " "
      //            << polygon.CircleCenter().y() << ", " << search_radius;
      return {};
    }
  }
  return ComputeAgentOverlapsWithBufferAndHeadingHelper(
      path_approx, step_length, first_index, last_index, polygon, max_lat_dist,
      lat_buffer, lon_buffer, theta, max_heading_diff);
}

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
      // DLOG(INFO) << ">> get minmax near point " << polygon.CircleCenter().x()
      //            << " " << polygon.CircleCenter().y() << " // " <<
      //            first_index
      //            << ", " << last_index << ", " << search_radius;
    } else {
      // DLOG(INFO) << "!! cannot find minmax near point "
      //            << polygon.CircleCenter().x() << " "
      //            << polygon.CircleCenter().y() << ", " << search_radius;
      return {};
    }
  }
  return ComputeAgentOverlapsWithBufferAndHeadingHelper(
      path_approx, step_length, first_index, last_index, polygon, max_lat_dist,
      lat_buffer, lon_buffer, theta, max_heading_diff);
}

bool HasPathApproxOverlapWithPolygon(const PathApprox& path_approx,
                                     double step_length, int first_index,
                                     int last_index, const Polygon2d& polygon,
                                     double search_radius) {
  if (path_approx.path_kd_tree() != nullptr) {
    const auto min_max_index = ComputeMinMaxIndexNearPoint(
        *path_approx.path_kd_tree(), polygon.CircleCenter(), search_radius,
        first_index, last_index);
    if (min_max_index.has_value()) {
      std::tie(first_index, last_index) = *min_max_index;
    } else {
      return false;
    }
  }
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;

  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    const auto& path_segment = path_approx.segment(i);
    if (!MaybeOverlap(path_segment, polygon, /*max_lat_dist=*/0.0)) continue;
    const auto geom_overlap =
        polygon2d::ComputePolygonBoxOverlap(polygon, path_segment);
    const AgentOverlap agent_overlap =
        ConvertToAgentOverlap(path_approx, i, geom_overlap);
    if (geom_overlap.in != geom_overlap.out &&
        agent_overlap.first_ra_s <= last_s &&
        agent_overlap.last_ra_s >= first_s) {
      return true;
    }
  }
  return false;
}
}  // namespace planning
}  // namespace st

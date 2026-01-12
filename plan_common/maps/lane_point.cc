

#include "plan_common/maps/lane_point.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <vector>

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/maps/map.h"
#include "plan_common/math/util.h"

namespace st {
namespace mapping {

namespace {

void GetNeighboringVertices(double fraction,
                            const std::vector<double>& cumulative_lengths,
                            const std::vector<Vec2d>& lane_vertices,
                            const std::vector<Vec2d>& lane_point_theta,
                            Vec2d* v0, Vec2d* v1, Vec2d* theta0, Vec2d* theta1,
                            double* alpha) {
  const int num_vertices = lane_vertices.size();

  *alpha = 0.0;
  if (cumulative_lengths.empty()) {
    return;
  }
  const double s = fraction * cumulative_lengths.back();

  CHECK_GE(s, cumulative_lengths.front());
  CHECK_LE(s, cumulative_lengths.back());

  int i = std::upper_bound(cumulative_lengths.begin(), cumulative_lengths.end(),
                           s) -
          cumulative_lengths.begin();
  CHECK_GT(i, 0);
  if (i == num_vertices) --i;
  *v0 = lane_vertices[i - 1];
  *v1 = lane_vertices[i];
  *theta0 = lane_point_theta[i - 1];
  *theta1 = lane_point_theta[i];
  const double s0 = cumulative_lengths[i - 1];
  const double s1 = cumulative_lengths[i];
  *alpha = (s - s0) / (s1 - s0);
  CHECK_GE(*alpha, 0.0);
  CHECK_LE(*alpha, 1.0);
}

void FindNeighboringVerticesWithSuccessorLane(
    const ad_byd::planning::MapConstPtr& map_ptr, ElementId lane_id,
    ElementId succ_lane_id, double fraction, Vec2d* v0, Vec2d* v1,
    Vec2d* theta0, Vec2d* theta1, double* alpha) {
  CHECK(v0 != nullptr);
  CHECK(v1 != nullptr);
  CHECK(alpha != nullptr);

  const std::vector<Vec2d>& lane_vertices =
      map_ptr->GetLaneById(lane_id)->points();
  const std::vector<Vec2d>& succ_lane_vertices =
      map_ptr->GetLaneById(succ_lane_id)->points();

  const int num_vertices = lane_vertices.size();
  const auto& lane_info = map_ptr->GetLaneById(lane_id);
  const auto& cumulative_lengths = lane_info->center_line().GetAccuLength();

  std::vector<Vec2d> lane_point_theta(num_vertices, Vec2d(0.0, 0.0));
  for (int i = 0; i < num_vertices - 1; ++i) {
    lane_point_theta[i] = Vec2d(lane_vertices[i + 1] - lane_vertices[i]);
  }
  lane_point_theta[num_vertices - 1] =
      Vec2d(succ_lane_vertices[1] - lane_vertices[num_vertices - 1]);

  GetNeighboringVertices(fraction, cumulative_lengths, lane_vertices,
                         lane_point_theta, v0, v1, theta0, theta1, alpha);
}

void FindNeighboringVertices(const ad_byd::planning::MapConstPtr& map_ptr,
                             ElementId lane_id, double fraction, Vec2d* v0,
                             Vec2d* v1, Vec2d* theta0, Vec2d* theta1,
                             double* alpha) {
  CHECK(v0 != nullptr);
  CHECK(v1 != nullptr);
  CHECK(alpha != nullptr);

  const std::vector<Vec2d>& lane_vertices =
      map_ptr->GetLaneById(lane_id)->points();
  const int num_vertices = lane_vertices.size();
  const auto& lane_info = map_ptr->GetLaneById(lane_id);
  const auto& cumulative_lengths = lane_info->center_line().GetAccuLength();

  std::vector<Vec2d> lane_point_theta(num_vertices, Vec2d(0.0, 0.0));
  for (int i = 0; i + 1 < num_vertices; ++i) {
    lane_point_theta[i] = Vec2d(lane_vertices[i + 1] - lane_vertices[i]);
    if (i == num_vertices - 2) {
      lane_point_theta[i + 1] = lane_point_theta[i];
    }
  }

  GetNeighboringVertices(fraction, cumulative_lengths, lane_vertices,
                         lane_point_theta, v0, v1, theta0, theta1, alpha);
}

}  // namespace

Vec2d LanePoint::ComputePos(
    const ad_byd::planning::MapConstPtr& map_ptr) const {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVertices(map_ptr, lane_id_, fraction_, &v0, &v1, &theta0,
                          &theta1, &alpha);
  return Lerp(v0, v1, alpha);
}

Vec2d LanePoint::ComputeTangent(
    const ad_byd::planning::MapConstPtr& map_ptr) const {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVertices(map_ptr, lane_id_, fraction_, &v0, &v1, &theta0,
                          &theta1, &alpha);
  return (v1 - v0).normalized();
}

double LanePoint::ComputeLerpTheta(
    const ad_byd::planning::MapConstPtr& map_ptr) const {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVertices(map_ptr, lane_id_, fraction_, &v0, &v1, &theta0,
                          &theta1, &alpha);
  return LerpAngle(theta0.FastAngle(), theta1.FastAngle(), alpha);
}

double LanePoint::ComputeLerpThetaWithSuccessorLane(
    const ad_byd::planning::MapConstPtr& map_ptr,
    ElementId succ_lane_id) const {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVerticesWithSuccessorLane(map_ptr, lane_id_, succ_lane_id,
                                           fraction_, &v0, &v1, &theta0,
                                           &theta1, &alpha);
  return LerpAngle(theta0.FastAngle(), theta1.FastAngle(), alpha);
}

}  // namespace mapping
}  // namespace st

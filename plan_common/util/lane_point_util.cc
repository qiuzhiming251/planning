

#include <algorithm>
#include <vector>

#include "plan_common/util/lane_point_util.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_util.h"
#include "plan_common/math/util.h"

namespace st::planning {
namespace {
void GetNeighboringVertices(double fraction,
                            const std::vector<double>& cumulative_lengths,
                            const std::vector<Vec2d>& lane_vertices,
                            const std::vector<Vec2d>& lane_point_theta,
                            Vec2d* v0, Vec2d* v1, Vec2d* theta0, Vec2d* theta1,
                            double* alpha) {
  const int num_vertices = lane_vertices.size();

  *alpha = 0.0;
  if (cumulative_lengths.size() < 2) {
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
    const PlannerSemanticMapManager& psmm, mapping::ElementId lane_id,
    mapping::ElementId succ_lane_id, double fraction, Vec2d* v0, Vec2d* v1,
    Vec2d* theta0, Vec2d* theta1, double* alpha) {
  CHECK(v0 != nullptr);
  CHECK(v1 != nullptr);
  CHECK(alpha != nullptr);

  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, lane_id, void());
  const std::vector<Vec2d>& lane_vertices = lane_info.points();

  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(succ_lane_info, psmm, succ_lane_id, void());
  const std::vector<Vec2d>& succ_lane_vertices = succ_lane_info.points();

  const int num_vertices = lane_vertices.size();
  const auto& lane_info_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
  if (lane_info_ptr == nullptr) return;
  const auto& cumulative_lengths = lane_info_ptr->center_line().GetAccuLength();

  std::vector<Vec2d> lane_point_theta(num_vertices, Vec2d(0.0, 0.0));
  for (int i = 0; i < num_vertices - 1; ++i) {
    lane_point_theta[i] = Vec2d(lane_vertices[i + 1] - lane_vertices[i]);
  }
  lane_point_theta[num_vertices - 1] =
      Vec2d(succ_lane_vertices[1] - lane_vertices[num_vertices - 1]);

  GetNeighboringVertices(fraction, cumulative_lengths, lane_vertices,
                         lane_point_theta, v0, v1, theta0, theta1, alpha);
}

void FindNeighboringVertices(const PlannerSemanticMapManager& psmm,
                             mapping::ElementId lane_id, double fraction,
                             Vec2d* v0, Vec2d* v1, Vec2d* theta0, Vec2d* theta1,
                             double* alpha) {
  CHECK(v0 != nullptr);
  CHECK(v1 != nullptr);
  CHECK(alpha != nullptr);

  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(lane_info, psmm, lane_id, void());
  const std::vector<Vec2d>& lane_vertices = lane_info.points();

  const int num_vertices = lane_vertices.size();
  const auto& lane_info_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
  if (lane_info_ptr == nullptr) return;
  const auto& cumulative_lengths = lane_info_ptr->center_line().GetAccuLength();

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

Vec2d ComputeLanePointPos(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVertices(psmm, lane_point.lane_id(), lane_point.fraction(),
                          &v0, &v1, &theta0, &theta1, &alpha);
  return Lerp(v0, v1, alpha);
}

Vec2d ComputeLanePointTangent(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVertices(psmm, lane_point.lane_id(), lane_point.fraction(),
                          &v0, &v1, &theta0, &theta1, &alpha);
  return (v1 - v0).normalized();
}

double ComputeLanePointLerpTheta(const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVertices(psmm, lane_point.lane_id(), lane_point.fraction(),
                          &v0, &v1, &theta0, &theta1, &alpha);
  return LerpAngle(theta0.FastAngle(), theta1.FastAngle(), alpha);
}

double ComputeLanePointLerpThetaWithSuccessorLane(
    const PlannerSemanticMapManager& psmm, mapping::ElementId succ_lane_id,
    const mapping::LanePoint& lane_point) {
  Vec2d v0, v1, theta0, theta1;
  double alpha;
  FindNeighboringVerticesWithSuccessorLane(psmm, lane_point.lane_id(),
                                           succ_lane_id, lane_point.fraction(),
                                           &v0, &v1, &theta0, &theta1, &alpha);
  return LerpAngle(theta0.FastAngle(), theta1.FastAngle(), alpha);
}

// absl::StatusOr<Vec2d> ComputeLanePointGlobalPose(
//     const ad_byd::planning::Map& smm, const mapping::LanePoint& lane_point) {
//   const auto lane = smm.GetLaneById(lane_point.lane_id());
//   if (lane == nullptr) {
//     return absl::NotFoundError(
//         absl::StrCat("Failed to find lane id, ", lane_point.lane_id()));
//   }
//   const auto [idx, alpha] =
//       mapping::ComputePosition(lane->segment_points(),
//       lane_point.fraction());
//   return Lerp(lane->segment_points()[idx], lane->segment_points()[idx + 1],
//               alpha);
// }

// absl::StatusOr<Vec2d> ComputeLanePointGlobalHeading(
//     const ad_byd::planning::Map& smm, const mapping::LanePoint& lane_point) {
//   const auto lane = smm.GetLaneById(lane_point.lane_id());
//   if (lane == nullptr) {
//     return absl::NotFoundError(
//         absl::StrCat("Failed to find lane id, ", lane_point.lane_id()));
//   }
//   const auto [idx, alpha] =
//       mapping::ComputePosition(lane->segment_points(),
//       lane_point.fraction());
//   return (lane->segment_points()[idx + 1] - lane->segment_points()[idx])
//       .normalized();
// }

}  // namespace st::planning

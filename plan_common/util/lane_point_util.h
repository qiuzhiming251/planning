

#ifndef AD_BYD_PLANNING_UTIL_LANE_POINT_UTIL_H
#define AD_BYD_PLANNING_UTIL_LANE_POINT_UTIL_H

#include "absl/status/statusor.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/vec.h"
#include "plan_common/maps/planner_semantic_map_manager.h"

namespace st::planning {

Vec2d ComputeLanePointPos(const PlannerSemanticMapManager& psmm,
                          const mapping::LanePoint& lane_point);

Vec2d ComputeLanePointTangent(const PlannerSemanticMapManager& psmm,
                              const mapping::LanePoint& lane_point);

double ComputeLanePointLerpTheta(const PlannerSemanticMapManager& psmm,
                                 const mapping::LanePoint& lane_point);

double ComputeLanePointLerpThetaWithSuccessorLane(
    const PlannerSemanticMapManager& psmm, mapping::ElementId succ_lane_id,
    const mapping::LanePoint& lane_point);

// absl::StatusOr<Vec2d> ComputeLanePointGlobalPose(
//     const ad_byd::planning::Map& smm, const mapping::LanePoint& lane_point);

// absl::StatusOr<Vec2d> ComputeLanePointGlobalHeading(
//     const ad_byd::planning::Map& smm, const mapping::LanePoint& lane_point);

}  // namespace st::planning

#endif  // AD_BYD_PLANNING_UTIL_LANE_POINT_UTIL_H

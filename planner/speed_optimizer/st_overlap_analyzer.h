

#ifndef ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_
#define ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_

#include <vector>

#include "absl/types/span.h"
#include "plan_common/vehicle_shape.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "plan_common/maps/st_boundary.h"

namespace st {
namespace planning {

struct OverlapSourcePriority {
  StOverlapMetaProto::OverlapSource source = StOverlapMetaProto::UNKNOWN_SOURCE;
  StOverlapMetaProto::OverlapPriority priority =
      StOverlapMetaProto::UNKNOWN_PRIORITY;
  std::string priority_reason;
  std::optional<double> time_to_lc_complete;
  std::optional<bool> is_making_u_turn;
  std::optional<bool> is_merging_straight_lane;
  std::optional<bool> is_crossing_straight_lane;
  std::optional<bool> is_unprotected_left_turn;
  std::optional<ad_byd::planning::TurnType> obj_lane_direction;
};

bool IsAnalyzableStBoundary(const StBoundaryRef& st_boundary);

void AnalyzeStOverlaps(
    const DiscretizedPath& path,
    absl::Span<const PathPointSemantic> path_semantics,
    const PlannerSemanticMapManager& psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const DrivePassage& drive_passage,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const LaneChangeStage& lc_stage,
    const std::vector<VehicleShapeBasePtr>& av_shapes, double init_v,
    std::vector<StBoundaryRef>* st_boundaries);

StOverlapMetaProto::OverlapPattern AnalyzeOverlapPattern(
    const StBoundaryRef& st_boundary, const SpacetimeObjectTrajectory& st_traj,
    const DiscretizedPath& path,
    const VehicleGeometryParamsProto& vehicle_geometry_params);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_ST_OVERLAP_ANALYZER_H_

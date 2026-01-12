

#ifndef ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_

#include "absl/status/statusor.h"
#include "plan_common/path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_manager/object_history.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/ref_line/smooth_reference_line_result.h"

namespace st::planning {

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromDrivePassage(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage);

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& plan_start_point,
    const VehicleGeometryParamsProto& vehicle_geom,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const LaneChangeStateProto& lc_state,
    const SmoothedReferenceLineResultMap& smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth_next_left_turn,
    const mapping::LanePath& prev_lane_path_before_lc_from_start =
        mapping::LanePath(),
    PausePushSavedOffsetProto* saved_offset = nullptr,
    absl::flat_hash_set<std::string>* unsafe_object_ids = nullptr,
    bool is_congestion_scene = false,
    const ObjectHistoryManager* obj_history_mgr = nullptr);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_

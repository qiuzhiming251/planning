

#ifndef ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_

#include <map>
#include <vector>

#include "plan_common/constraint_manager.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
//#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/ego_history.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "planner/speed_optimizer/st_close_trajectory.h"
#include "planner/speed_optimizer/st_graph_defs.h"
#include "plan_common/speed/st_speed/vt_speed_limit.h"
#include "plan_common/path_sl_boundary.h"
#include "object_manager/spacetime_trajectory_manager.h"

namespace st::planning {

std::map<SpeedLimitTypeProto::Type, SpeedLimit> GetSpeedLimitMap(
    const DiscretizedPath& discretized_points,
    const std::vector<PathPoint>& st_path_points,
    const PathSlBoundary& path_sl_boundary, double max_speed_limit,
    double av_speed, const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const DrivePassage& drive_passage, const ConstraintManager& constraint_mgr,
    const SpeedFinderParamsProto::SpeedLimitParamsProto& speed_limit_config,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries,
    const PlannerSemanticMapManager* planner_semantic_map_manager,
    double av_max_acc, const Behavior& behavior,
    const std::vector<StCloseTrajectory>& moving_close_trajs,
    const SpacetimeTrajectoryManager& traj_mgr,
    const LaneChangeStateProto lane_change_state,
    const NudgeObjectInfo* nudge_object_info, const EgoHistory* ego_history,
    EgoFrame* curr_ego_frame, double is_narrow_near_large_vehicle,
    bool raise_lane_speed_limit, double spdlimit_curvature_gain_prev,
    double* spdlimit_curvature_gain_ptr,
    std::optional<double>* v2_trigger_distance);

VtSpeedLimit GetExternalVtSpeedLimit(const ConstraintManager& constraint_mgr,
                                     int traj_steps, double time_step);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_

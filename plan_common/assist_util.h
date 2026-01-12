

#ifndef ST_PLANNING_ASSIST_ASSIST_UTIL
#define ST_PLANNING_ASSIST_ASSIST_UTIL

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "plan_common/maps/lane_path.h"
#include "modules/cnoa_pnc/planning/proto/alc.pb.h"
//#include "online_semantic_map.pb.h"
//#include "external_command_info.h"
#include "driving_map_topo.h"
#include "path_sl_boundary.h"
#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/positioning.pb.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "drive_passage.h"
//#include "remote_assist.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st::planning {
using IndexedStationBoundary = std::pair<StationIndex, StationBoundary>;
struct BoundaryInterval {
  std::vector<Vec2d> points;
  StationBoundaryType type;
};
// absl::Status UpdateExternalCmdStatusFromRemoteAssist(
//     const RemoteAssistToCarProto& proto, ExternalCommandStatus* status);

// absl::Status UpdateExternalCmdQueueFromDriverAction(
//     const DriverAction& driver_action, ExternalCommandQueue* queue);

// DriverAction::LaneChangeCommand ProcessLaneChangeCommands(
//     const ExternalCommandQueue& ext_cmd_queue);

void AddBoundariesToIntervals(
    const DrivePassage& drive_passage,
    const std::vector<IndexedStationBoundary>& boundaries,
    std::vector<BoundaryInterval>* intervals);

std::vector<BoundaryInterval> FindSolidBoundaryIntervals(
    const DrivePassage& drive_passage, const FrenetCoordinate& first_point_sl,
    double cutoff_s, const double route_station_unit_step);

absl::StatusOr<bool> CrossedBoundary(const DrivePassage& dp,
                                     const Vec2d& ego_pos);

absl::StatusOr<bool> HasTrajectoryCrossedSolidBoundary(
    const DrivePassage& drive_passage, const PathSlBoundary& sl_boundary,
    const std::vector<ApolloTrajectoryPointProto>& traj_pts,
    const VehicleGeometryParamsProto& vehicle_geom, bool lc_pause,
    const double route_station_unit_step);

absl::StatusOr<ALCState> UpdateAlcState(ALCState state,
                                        const Vec2d& preview_pos,
                                        const Vec2d& ego_pos,
                                        double ego_heading,
                                        const DrivePassage& drive_passage,
                                        const double max_lane_keep_lat_offset);

void ReportPlcEventSignal(ALCState old_state, ALCState new_state,
                          DriverAction::LaneChangeCommand lc_cmd);

}  // namespace st::planning

#endif  // ST_PLANNING_ASSIST_ASSIST_UTIL



#ifndef ONBOARD_PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/types/span.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "object_manager/spacetime_object_trajectory.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "object_scene_recognition.h"
#include "plan_common/math/discretized_path.h"
#include "planner/speed_optimizer/interactive_laterally.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/st_close_trajectory.h"
#include "planner/speed_optimizer/st_graph.h"
namespace st::planning {
struct RightTurnCloseSpeedLimitInput {
  const VehicleGeometryParamsProto* vehicle_geom = nullptr;
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  double current_v = 0.0;
  double current_a = 0.0;
  const DiscretizedPath* path = nullptr;
  const DiscretizedPath* ego_predict_path = nullptr;
  const absl::flat_hash_map<std::string, StBoundaryLatModificationInfo>*
      lat_modification_info_map = nullptr;
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const DrivePassage* drive_passage = nullptr;
};

std::vector<std::optional<SpeedLimit>> GetMovingCloseTrajSpeedLimits(
    absl::Span<const StCloseTrajectory> st_close_trajs, double path_length,
    double av_speed, double time_step, double max_time);

std::optional<SpeedLimit> GetRightTurnCloseSpeedLimit(
    const RightTurnCloseSpeedLimitInput& input,
    const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
    const std::vector<DrivingProcess>& driving_process_seq);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_CLOSE_TRAJECTORY_DECIDER_H_

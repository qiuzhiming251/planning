

#ifndef ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_

#include <optional>
#include <vector>

#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/vt_speed_limit.h"
#include "plan_common/vehicle_shape.h"
#include "planner/speed_optimizer/cipv_object_info.h"
#include "planner/speed_optimizer/object_scene_recognition.h"
#include "planner/speed_optimizer/path_semantic_analyzer.h"
#include "modules/cnoa_pnc/planning/proto/lane_change.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

struct IgnoreDeciderInput {
  const SpeedFinderParamsProto::IgnoreDeciderParamsProto* params = nullptr;
  const DiscretizedPath* path = nullptr;
  const SegmentMatcherKdtree* path_kd_tree = nullptr;
  // Could be empty but not null.
  const std::vector<PathPointSemantic>* path_semantics = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const std::vector<VehicleShapeBasePtr>* av_shapes = nullptr;
  double current_v = 0.0;
  double max_v = 0.0;
  double time_step = 0.0;
  int trajectory_steps = 0;
  LaneChangeStage lc_stage;
  std::vector<DrivingProcess> driving_process_seq;
  double current_acc = 0.0;
  // for special case ignore
  bool is_narrow_near_large_vehicle = false;
  double follow_time_headway;
  int plan_id = 0;
  const LaneChangeStateProto* lane_change_state;
};

enum Merge_Direction { MERGE_NONE = 0, MERGE_LEFT = 1, MERGE_RIGHT = 2 };

void MakeIgnoreAndPreBrakeDecisionForStBoundaries(
    const IgnoreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::optional<VtSpeedLimit>* speed_limit, CipvObjectInfo* cipv_object_info);
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_IGNORE_DECIDER_H_

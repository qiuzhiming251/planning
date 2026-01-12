

#ifndef ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
//#include "decider/decision_manager/traffic_gap_finder.h"
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "plan_common/math/discretized_path.h"
//#include "ml/model_pool.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/drive_passage.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/speed/st_speed/vt_speed_limit.h"
#include "plan_common/util/decision_info.h"
//#include "predictor/container/av_context.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

struct PreDeciderInput {
  const SpeedFinderParamsProto::StBoundaryPreDeciderParamsProto* params =
      nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const absl::flat_hash_set<std::string>* follower_set = nullptr;
  const TrafficGapResult* lane_change_gap = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const VehicleGeometryParamsProto* vehicle_params = nullptr;
  const DrivePassage* drive_passage = nullptr;
  double current_v = 0.0;
  double current_s = 0.0;
  double max_v = 0.0;
  double time_step = 0.0;
  int trajectory_steps = 0;
  //   const ModelPool* planner_model_pool = nullptr;
  //   const prediction::AvContext* planner_av_context = nullptr;
  //   const ObjectsProto* objects_proto = nullptr;
  absl::Time plan_time;
  bool run_act_net_speed_decision = false;
};

void MakePreDecisionForStBoundaries(
    const PreDeciderInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    std::optional<VtSpeedLimit>* speed_limit);

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_

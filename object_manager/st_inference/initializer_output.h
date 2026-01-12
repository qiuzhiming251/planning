

#ifndef ONBOARD_INITIALIZER_OUTPUT_H_
#define ONBOARD_INITIALIZER_OUTPUT_H_

#include <map>
#include <vector>
#include <string>

#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"

#include "plan_common/util/decision_info.h"

namespace st::planning {
struct InitializerOutput {
  absl::flat_hash_set<std::string> follower_set{};
  absl::flat_hash_set<std::string> leader_set{};
  double follower_max_decel = 0.0;
  bool is_lc_pause = false;
  std::vector<ApolloTrajectoryPointProto> traj_points{};
  InitializerStateProto initializer_state{};
  std::map<std::string, ConstraintProto::LeadingObjectProto> leading_trajs{};
  NudgeInfos nudge_info{};
  SpeedResponseStyle speed_response_style = SpeedResponseStyle::SPEED_RESPONSE_CONSERVATIVE;
  PlannerStatusProto::PlannerStatusCode lc_status_code = PlannerStatusProto::OK;
  bool is_init_follow_scene = false;
  std::string lc_lead_obj_id = "none";
  LargeVehicleAvoidStateProto pre_large_vehicle_avoid_state{};
  PausePushSavedOffsetProto saved_offset{};
  // Lane change style decider output
  LaneChangeStyleDeciderResultProto lc_style_decider_result{};
  absl::flat_hash_set<std::string> gaming_lc_obs_set{};
};
}  // namespace st::planning

#endif  // ONBOARD_INITIALIZER_OUTPUT_H_
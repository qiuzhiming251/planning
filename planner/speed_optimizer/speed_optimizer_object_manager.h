

#ifndef ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_
#define ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_

#include <vector>

#include "absl/types/span.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "planner/speed_optimizer/speed_optimizer_object.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/math/discretized_path.h"
#include "plan_common/speed/st_speed/speed_vector.h"
#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner.pb.h"

namespace st::planning {

enum SpeedOptimizerObjectType {
  MOVING_FOLLOW = 0,
  MOVING_LEAD = 1,
  STATIONARY = 2,
};

// Manage the speed optimizer objects those used in speed optimizer. Multiple
// moving st_trajs for the same object_id and the same decision will be
// integrated into a speed optimizer object and collected into the FOLLOW/LEAD
// type. Stationary st_traj will be separately collected into the STATIONARY
// type.
class SpeedOptimizerObjectManager {
 public:
  SpeedOptimizerObjectManager(
      int plan_id,
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      const SpacetimeTrajectoryManager& traj_mgr, const DiscretizedPath& path,
      double av_speed, double plan_total_time, double plan_time_interval,
      const SpeedFinderParamsProto& speed_finder_params,
      const SpeedVector& preliminary_speed,
      const CutinHistoryProto* cutin_history, double current_time,
      LaneChangeStage lc_stage, double coeffi_match_lc_style,
      const LcFinishedTimeProto* lc_finished_time, bool need_accel_for_gap);

  // Return speed optimizer objects with follow decision.
  absl::Span<const SpeedOptimizerObject> MovingFollowObjects() const {
    return objects_.at(SpeedOptimizerObjectType::MOVING_FOLLOW);
  }

  // Return speed optimizer objects with follow decision.
  absl::Span<const SpeedOptimizerObject> MovingLeadObjects() const {
    return objects_.at(SpeedOptimizerObjectType::MOVING_LEAD);
  }

  // Return stationary speed optimizer objects.
  absl::Span<const SpeedOptimizerObject> StationaryObjects() const {
    return objects_.at(SpeedOptimizerObjectType::STATIONARY);
  }

  const std::string& attention_obj_id() const { return attention_obj_id_; }

 private:
  std::vector<std::vector<SpeedOptimizerObject>> objects_;
  std::string attention_obj_id_ = "";
  bool need_accel_for_gap_ = false;
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_SPEED_OPTIMIZER_OBJECT_MANAGER_H_

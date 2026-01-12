

#ifndef ONBOARD_PLANNER_SPEED_DP_SVT_COST_H_
#define ONBOARD_PLANNER_SPEED_DP_SVT_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "planner/speed_optimizer/svt_graph_point.h"

namespace st::planning {
class DpSvtCost {
 public:
  DpSvtCost(const SpeedFinderParamsProto* speed_finder_params, double total_t,
            double total_s, double init_v,
            const std::vector<StBoundaryWithDecision*>*
                sorted_st_boundaries_with_decision,
            const SpacetimeTrajectoryManager& st_traj_mgr);

  std::vector<SvtGraphPoint::StBoundaryDecision>
  GetStBoundaryDecisionsForInitPoint(const SvtGraphPoint& svt_graph_point);
  void GetStBoundaryCostAndDecisions(
      const SvtGraphPoint& prev_svt_graph_point,
      const SvtGraphPoint& svt_graph_point, double av_pseed,
      double* st_boundary_cost,
      std::vector<SvtGraphPoint::StBoundaryDecision>* st_boundary_decisions);

  double GetSpatialPotentialCost(double s) const;

  double GetSpeedLimitCost(double speed, double speed_limit,
                           std::optional<double> lane_merge_speed_limit) const;

  double GetReferenceSpeedCost(double speed, double cruise_speed) const;

  double GetAccelCost(double accel) const;

  void SetFollowDistanceRelSpeedPlf(PiecewiseLinearFunction<double> plf) {
    follow_distance_rel_speed_plf_ = std::move(plf);
  }

 private:
  using SamplingDpSpeedParamsProto =
      SpeedFinderParamsProto::SamplingDpSpeedParamsProto;

  const SpeedFinderParamsProto* speed_finder_params_;
  const SamplingDpSpeedParamsProto* params_;

  PiecewiseLinearFunction<double> follow_distance_rel_speed_plf_;

  // This vector is sorted. The protective st-boundaries are on the front, and
  // the other st-boundaries are on the back.
  const std::vector<StBoundaryWithDecision*>*
      sorted_st_boundaries_with_decision_;
  absl::flat_hash_map<std::string, const StBoundaryWithDecision*>
      original_st_boundary_wd_map_;

  double unit_t_ = 0.0;
  double total_s_ = 0.0;

  absl::flat_hash_map<std::string, int> boundary_map_;
  // Note: boundary_cost_[boundary_index][t] = (s_upper,s_lower).
  std::vector<std::vector<std::pair<double, double>>> boundary_cost_;

  std::vector<double> comfortable_brake_s_;

  double exceed_speed_unit_cost_ = 0.0;
  double low_speed_unit_cost_ = 0.0;

  absl::Mutex boundary_cost_mutex_;
  const SpacetimeTrajectoryManager& st_traj_mgr_;
};
}  // namespace st::planning

#endif  // ONBOARD_PLANNER_SPEED_DP_SVT_COST_H_

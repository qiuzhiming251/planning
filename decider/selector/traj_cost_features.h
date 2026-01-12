#ifndef ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_
#define ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_

#include <algorithm>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"

#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/math/vec.h"
#include "object_manager/st_inference/est_planner_output.h"
#include "object_manager/st_inference/scheduler_output.h"
#include "plan_common/maps/planner_semantic_map_manager.h"
#include "plan_common/maps/route_sections_info.h"
#include "plan_common/plan_common_defs.h"
#include "decider/selector/candidate_stats.h"
#include "decider/selector/common_feature.h"
#include "decider/selector/cost_feature_base.h"
#include "decider/selector/cost_feature_util.h"
#include "decider/selector/discourage_right_most.h"

#include "modules/cnoa_pnc/planning/proto/driving_style.pb.h"
#include "modules/cnoa_pnc/planning/proto/hmi_content.pb.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "modules/cnoa_pnc/planning/proto/trajectory_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"

namespace st {
namespace planning {

using CostVec = CostFeatureBase::CostVec;

class TrajProgressCost : public CostFeatureBase {
 public:
  TrajProgressCost(const SelectorCommonFeature* common_feature,
                   const PlannerSemanticMapManager* psmm,
                   const LaneChangeStyle& lane_change_style,
                   const mapping::LanePath* prev_lp_from_current,
                   ProgressStats stats, int driving_style_gear)
      : CostFeatureBase("progress",
                        {"progress", "follow_slow", "large_slow_working_obj",
                         "junction_object"},
                        /*is_common=*/true, common_feature),
        psmm_(psmm),
        ego_v_(stats.ego_v),
        ego_width_(stats.ego_width),
        lane_speed_map_(std::move(stats.lane_speed_map)),
        slow_working_objs_map_(std::move(stats.slow_working_objs_map)),
        min_slow_working_object_s_(stats.min_slow_working_object_s),
        max_lane_speed_(stats.max_lane_speed),
        min_lane_speed_(stats.min_lane_speed),
        max_init_leader_speed_(stats.max_init_leader_speed),
        lane_keep_init_leader_speed_(stats.lane_keep_init_leader_speed),
        lane_change_style_(lane_change_style),
        is_borrow_case_(common_feature->is_borrow_case),
        prev_lp_from_current_(prev_lp_from_current),
        driving_style_gear_(driving_style_gear) {
    has_nudge_id_ = common_feature->nudge_id != "Invalid" ? true : false;
    is_static_feature_ = false;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  const PlannerSemanticMapManager* psmm_;
  double ego_v_;
  double ego_width_;
  absl::flat_hash_map<TaskIndex, LaneSpeedInfo> lane_speed_map_;
  absl::flat_hash_map<TaskIndex, std::optional<SlowWorkingObject>>
      slow_working_objs_map_;
  absl::flat_hash_map<TaskIndex, std::optional<NudgeObjInfo>> nudge_objs_map_;
  double min_slow_working_object_s_;
  double max_lane_speed_;
  double min_lane_speed_;
  double max_init_leader_speed_;
  double lane_keep_init_leader_speed_;
  LaneChangeStyle lane_change_style_;

  bool is_borrow_case_ = false;
  // bool is_follow_scene_ = false;
  bool has_nudge_id_ = false;
  const mapping::LanePath* prev_lp_from_current_;

  int driving_style_gear_ = 1;
};

class TrajMaxJerkCost : public CostFeatureBase {
 public:
  TrajMaxJerkCost(const SelectorCommonFeature* common_feature,
                  const MotionConstraintParamsProto& motion_constraints,
                  const double vehicle_width)
      : CostFeatureBase("max_jerk",
                        {"max_lon_jerk", "max_lat_jerk", "max_lon_deacc",
                         "lat_away_distance", "split_angle"},
                        /*is_common=*/true, common_feature),
        accel_jerk_constraint_(motion_constraints.max_accel_jerk()),
        decel_jerk_constraint_(motion_constraints.max_decel_jerk()),
        lat_jerk_constraint_(motion_constraints.max_lateral_jerk()),
        ego_width_(vehicle_width) {
    for (int i = 0; i < kTrajectorySteps; ++i) {
      coeffs_[i] = ExpDecayCoeffAtStep(10, 0.6, i);
      coeffs_lat_[i] = ExpDecayCoeffAtStep(M_E, 0.4, i);
    }
    is_static_feature_ = false;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  double accel_jerk_constraint_, decel_jerk_constraint_;
  double lat_jerk_constraint_;
  double coeffs_[kTrajectorySteps];  // Decaying factor w.r.t. time step.
  double coeffs_lat_[kTrajectorySteps];
  double ego_width_;
};

class TrajLaneChangeCost : public CostFeatureBase {
 public:
  TrajLaneChangeCost(const SelectorCommonFeature* common_feature,
                     const PlannerSemanticMapManager* psmm,
                     const mapping::LanePath* prev_lp_from_current,
                     const PlannerTrajectory* prev_traj,
                     const ApolloTrajectoryPointProto& plan_start_point,
                     const LastLcInfoProto& last_lc_info,
                     const LastPassedSplitInfoProto& last_passed_split_info,
                     const bool planner_enable_lane_change_in_intersection,
                     const TurnSignal& last_turn_signal,
                     const TurnSignalReason& last_turn_signal_reason,
                     const st::LaneChangeStage& pre_lc_stage,
                     const double vehicle_width)
      : CostFeatureBase(
            "lane_change",
            {"lane_path_diff", "pose_to_target", "prev_lat_diff",
             "lc_in_curvy_road", "lc_in_intersection", "lc_in_redlight",
             "lc_opposite_direction", "lc_safety_effect",
             "lk_after_turn_signal", "junction_smoothness",
             "passed_split_lc_opposite_direction", "lc_invade_risk",
             "lc_to_bus_lane"},
            /*is_common=*/true, common_feature),
        psmm_(psmm),
        prev_lp_from_current_(prev_lp_from_current),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
        ego_v_(plan_start_point.v()),
        last_lc_info_(last_lc_info),
        last_passed_split_info_(last_passed_split_info),
        planner_enable_lane_change_in_intersection_(
            planner_enable_lane_change_in_intersection),
        last_turn_signal_(last_turn_signal),
        last_turn_signal_reason_(last_turn_signal_reason),
        pre_lc_stage_(pre_lc_stage),
        ego_width_(vehicle_width) {
    std::vector<Vec2d> prev_pts;
    if (prev_traj != nullptr && prev_traj->size() > 1) {
      prev_pts.reserve(prev_traj->size());
      std::transform(prev_traj->begin(), prev_traj->end(),
                     std::back_inserter(prev_pts), [](const auto& traj_pt) {
                       return Vec2dFromApolloTrajectoryPointProto(traj_pt);
                     });
    }
    prev_traj_ff_or_ =
        BuildBruteForceFrenetFrame(prev_pts, /*down_sample_raw_points=*/true);
    is_static_feature_ = false;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  const PlannerSemanticMapManager* psmm_;
  const mapping::LanePath* prev_lp_from_current_;
  Vec2d ego_pos_;
  double ego_v_;

  LastLcInfoProto last_lc_info_;
  LastPassedSplitInfoProto last_passed_split_info_;
  bool planner_enable_lane_change_in_intersection_;
  absl::StatusOr<BruteForceFrenetFrame> prev_traj_ff_or_;
  TurnSignal last_turn_signal_ = TurnSignal::TURN_SIGNAL_NONE;
  TurnSignalReason last_turn_signal_reason_ = TurnSignalReason::TURN_SIGNAL_OFF;
  const st::LaneChangeStage& pre_lc_stage_;
  double ego_width_;
};

class TrajCrossSolidBoundaryCost : public CostFeatureBase {
 public:
  explicit TrajCrossSolidBoundaryCost(
      const SelectorCommonFeature* common_feature,
      const mapping::LanePath* prev_lp_from_current,
      const VehicleGeometryParamsProto& vehicle_geom,
      const PlannerSemanticMapManager& psmm,
      const ApolloTrajectoryPointProto& plan_start_point,
      const bool planner_enable_cross_solid_boundary)
      : CostFeatureBase(
            "cross_solid_boundary",
            {"solid_white", "solid_yellow", "solid_double_yellow", "curb"},
            /*is_common=*/true, common_feature),
        prev_lp_from_current_(prev_lp_from_current),
        ego_length_(vehicle_geom.length()),
        ego_width_(vehicle_geom.width()),
        ego_front_to_ra_(vehicle_geom.front_edge_to_center()),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
        ego_v_(plan_start_point.v()),
        planner_enable_cross_solid_boundary_(
            planner_enable_cross_solid_boundary) {
    psmm_ = &psmm;
    is_static_feature_ = true;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  const mapping::LanePath* prev_lp_from_current_;
  double ego_length_, ego_width_, ego_front_to_ra_;
  Vec2d ego_pos_;
  double ego_v_ = 0.0;
  bool planner_enable_cross_solid_boundary_;
  const PlannerSemanticMapManager* psmm_;
};

class TrajRouteLookAheadCost : public CostFeatureBase {
 public:
  explicit TrajRouteLookAheadCost(
      const SelectorCommonFeature* common_feature, RouteLookAheadStats stats,
      const ApolloTrajectoryPointProto& plan_start_point,
      const PlannerSemanticMapManager& psmm, bool planner_is_bus_model)
      : CostFeatureBase(
            "route_look_ahead",
            {"length_along_route", "reach_destination",
             "preview_beyond_horizon", "discourage_right_most_lane",
             "behind_stalled_object", "merge_lane", "encourage_right_most_lane",
             "neighbor_cones", "bus_lane", "merged_area"},
            /*is_common=*/true, common_feature),
        // driving_dist_map_(std::move(stats.driving_dist_map)),
        len_before_intersection_map_(
            std::move(stats.len_before_intersection_map)),
        len_before_merge_lane_map_(std::move(stats.len_before_merge_lane_map)),
        len_along_route_map_(std::move(stats.len_along_route_map)),
        raw_len_along_route_map_(std::move(stats.raw_len_along_route_map)),
        // lc_num_to_targets_map_(std::move(stats.lc_num_to_targets_map)),
        // lc_num_within_driving_dist_map_(
        //     std::move(stats.lc_num_within_driving_dist_map)),
        is_right_most_lane_map_(std::move(stats.is_right_most_lane_map)),
        front_stalled_obj_map_(std::move(stats.front_stalled_obj_map)),
        has_cross_curb_map_(std::move(stats.has_cross_curb_map)),
        enable_discourage_right_most_cost_(
            stats.enable_discourage_right_most_cost),
        enable_encourage_right_most_cost_(
            stats.enable_encourage_right_most_cost),
        max_len_along_route_(stats.max_length_along_route),
        min_len_along_route_(stats.min_length_along_route),
        traffic_congestion_factor_(stats.traffic_congestion_factor),
        min_lc_num_(stats.min_lc_num),
        max_length_before_merge_lane_(stats.max_length_before_merge_lane),
        is_left_turn_(stats.is_left_turn),
        is_right_turn_(stats.is_right_turn),
        ego_v_(plan_start_point.v()),
        planner_is_bus_model_(planner_is_bus_model),
        is_borrow_case_(common_feature->is_borrow_case),
        func_id_(common_feature->func_id),
        navi_start_(stats.navi_sart),
        psmm_(&psmm) {
    is_static_feature_ = true;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  absl::flat_hash_map<TaskIndex, double> len_before_intersection_map_,
      len_before_merge_lane_map_;
  absl::flat_hash_map<TaskIndex, double> len_along_route_map_,
      raw_len_along_route_map_;
  absl::flat_hash_map<TaskIndex, bool> is_right_most_lane_map_;
  absl::flat_hash_map<TaskIndex, std::optional<StalledObjInfo>>
      front_stalled_obj_map_;
  absl::flat_hash_map<TaskIndex, bool> has_cross_curb_map_;
  bool enable_discourage_right_most_cost_;
  bool enable_encourage_right_most_cost_;
  double max_len_along_route_;
  double min_len_along_route_;
  double traffic_congestion_factor_;
  int min_lc_num_;
  double max_length_before_merge_lane_;
  bool is_left_turn_ = false;
  bool is_right_turn_ = false;

  double ego_v_;
  bool planner_is_bus_model_ = false;

  bool is_borrow_case_ = false;

  st::Behavior_FunctionId func_id_ =
      st::Behavior_FunctionId_NONE;  // function id (only used for CityNOA with
                                     // map)
  ad_byd::planning::NaviPosition navi_start_ = {0, -1.0};
  const PlannerSemanticMapManager* psmm_;
};

// To make selector favor trajectories from non-expanded path boundary.
//
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || .   | ||       || |   . ||       || | ||       || .   |   . ||
// || e   | ||       || e   . ||       || e ||       || .   e   . ||
//     low               high            low               high
class TrajBoundaryExpansionCost : public CostFeatureBase {
 public:
  explicit TrajBoundaryExpansionCost(
      const SelectorCommonFeature* common_feature,
      const ApolloTrajectoryPointProto& plan_start_point)
      : CostFeatureBase("boundary_expansion", {"boundary_expansion"},
                        /*is_common=*/false, common_feature),
        ego_pos_(Vec2dFromApolloTrajectoryPointProto(plan_start_point)) {
    is_static_feature_ = true;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  Vec2d ego_pos_;
};

class PrepareIntentionCost : public CostFeatureBase {
 public:
  explicit PrepareIntentionCost(const SelectorCommonFeature* common_feature)
      : CostFeatureBase("prepare_intention", {"cost_decay"},
                        /*is_common=*/true, common_feature) {
    is_static_feature_ = false;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
};

class DefensiveDrivingCost : public CostFeatureBase {
 public:
  explicit DefensiveDrivingCost(
      const SelectorCommonFeature* common_feature,
      const ApolloTrajectoryPointProto& plan_start_point)
      : CostFeatureBase("defensive_driving", {"lc_truckbehind"},
                        /*is_common=*/true, common_feature),
        ego_v_(plan_start_point.v()) {
    is_static_feature_ = false;
  }

  absl::StatusOr<CostVec> ComputeCostV2(
      const EstPlannerOutput& planner_output, const int idx,
      const SelectorCostInput& cost_input, std::vector<std::string>* extra_info,
      TrajFeatureOutput* traj_feature_output) const override;

 private:
  double ego_v_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_SELECTOR_TRAJ_COST_FEATURES_H_

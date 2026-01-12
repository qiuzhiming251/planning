

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <ostream>
#include <string_view>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"
#include "plan_common/log.h"
#include "gflags/gflags.h"
#include "planner/speed_optimizer/decider/gaming_st_boundary_modifier.h"

// #include "global/trace.h"
// #include "lite/check.h"
// #include "lite/logging.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "object_manager/planner_object.h"
#include "plan_common/maps/st_boundary.h"
#include "plan_common/second_order_trajectory_point.h"
#include "plan_common/trajectory_util.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_defs.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/path_util.h"
// #include "vis/common/color.h"

namespace st::planning {
void GamingModifyStBoundaries(
    const GamingStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects) {
  std::vector<StBoundaryWithDecision> newly_generated_st_boundaries_wd;
  newly_generated_st_boundaries_wd.reserve(
      input.speed_gaming_output->gaming_processed_st_objects.size());

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if (st_boundary_wd.decision_type() == StBoundaryProto::IGNORE ||
        st_boundary_wd.raw_st_boundary()->is_stationary()) {
      continue;
    }
    if (!st_boundary_wd.raw_st_boundary()->traj_id().has_value()) {
      continue;
    }

    size_t pos = st_boundary_wd.raw_st_boundary()->traj_id()->find("|");
    std::string traj_id =
        pos != std::string::npos
            ? st_boundary_wd.raw_st_boundary()->traj_id()->substr(0, pos)
            : st_boundary_wd.raw_st_boundary()->traj_id().value();

    // 如果博弈打超或者忽略，忽略不进dp
    auto it_gaming_decision =
        input.speed_gaming_output->gaming_decision.find(traj_id);
    if (it_gaming_decision !=
            input.speed_gaming_output->gaming_decision.end() &&
        (it_gaming_decision->second == StBoundaryProto::OVERTAKE ||
         it_gaming_decision->second == StBoundaryProto::IGNORE)) {
      // 修改原始st_boundary信息
      const auto modifier_type = StBoundaryModifierProto::SPEED_SINGLE_GAMING;
      st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
      st_boundary_wd.set_decision_reason(
          StBoundaryProto::SPEED_GAMING_DECISION);
      st_boundary_wd.set_ignore_reason(StBoundaryProto::ST_BOUNDARY_MODIFIED);
      st_boundary_wd.set_decision_info(absl::StrCat(
          "ignore overtake modified by ",
          StBoundaryModifierProto::ModifierType_Name(modifier_type)));
      continue;
    }

    auto it =
        input.speed_gaming_output->gaming_processed_st_objects.find(traj_id);
    if (it == input.speed_gaming_output->gaming_processed_st_objects.end()) {
      continue;
    }

    const StBoundary& st_boundary =
        *CHECK_NOTNULL(st_boundary_wd.raw_st_boundary());
    const auto& new_traj = it->second;
    auto st_boundary_output = input.st_graph->MapMovingSpacetimeObject(
        new_traj, /*generate_lane_change_gap=*/false,
        /*calc_moving_close_traj=*/false, /*nudge_object_info=*/nullptr);
    auto& new_st_boundaries = st_boundary_output.st_boundaries;
    const auto modifier_type = StBoundaryModifierProto::SPEED_SINGLE_GAMING;
    for (auto& new_st_boundary : new_st_boundaries) {
      StBoundaryModifierProto modifier;
      modifier.set_modifier_type(modifier_type);
      new_st_boundary->set_id(absl::StrCat(new_st_boundary->id(), "|sg"));
      if (st_boundary.overlap_meta().has_value()) {
        new_st_boundary->set_overlap_meta(*st_boundary.overlap_meta());
      }
      newly_generated_st_boundaries_wd.emplace_back(
          std::move(new_st_boundary), st_boundary_wd.decision_type(),
          st_boundary_wd.decision_reason(),
          absl::StrCat(
              st_boundary_wd.decision_info(), " and keep it after modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(), st_boundary_wd.pass_time(),
          st_boundary_wd.yield_time(), input.path->length());
      newly_generated_st_boundaries_wd.back().set_modifier(std::move(modifier));

      // 修改原始st_boundary信息
      st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
      st_boundary_wd.set_ignore_reason(StBoundaryProto::ST_BOUNDARY_MODIFIED);
      st_boundary_wd.set_decision_info(absl::StrCat(
          "ignore modified by ",
          StBoundaryModifierProto::ModifierType_Name(modifier_type)));
    }
  }

  // Append newly generated st-boundaries with decision to original ones.
  std::move(newly_generated_st_boundaries_wd.begin(),
            newly_generated_st_boundaries_wd.end(),
            std::back_inserter(*st_boundaries_wd));

  // Merge newly processed trajectories with the original ones.
  for (auto& [traj_id, traj] :
       input.speed_gaming_output->gaming_processed_st_objects) {
    // DLOG(INFO) << "after post mod: " << traj_id;
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

void GamingModifyStBoundariesAfterDP(
    const GamingStboundaryModifierInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd,
    std::unordered_map<std::string, SpacetimeObjectTrajectory>*
        processed_st_objects,
    const int& plan_id) {
  std::vector<StBoundaryWithDecision> newly_generated_st_boundaries_wd;
  newly_generated_st_boundaries_wd.reserve(
      input.speed_gaming_output->gaming_processed_st_objects.size());

  for (auto& st_boundary_wd : *st_boundaries_wd) {
    if ((st_boundary_wd.decision_type() == StBoundaryProto::IGNORE &&
         st_boundary_wd.decision_reason() !=
             StBoundaryProto::SPEED_GAMING_DECISION) ||
        st_boundary_wd.raw_st_boundary()->is_stationary()) {
      continue;
    }
    if (!st_boundary_wd.raw_st_boundary()->traj_id().has_value()) {
      continue;
    }

    size_t pos = st_boundary_wd.raw_st_boundary()->traj_id()->find("|");
    std::string traj_id =
        pos != std::string::npos
            ? st_boundary_wd.raw_st_boundary()->traj_id()->substr(0, pos)
            : st_boundary_wd.raw_st_boundary()->traj_id().value();
    auto it =
        input.speed_gaming_output->gaming_processed_st_objects.find(traj_id);
    if (it == input.speed_gaming_output->gaming_processed_st_objects.end()) {
      continue;
    }

    auto it_gaming_decision =
        input.speed_gaming_output->gaming_decision.find(traj_id);

    const StBoundary& st_boundary =
        *CHECK_NOTNULL(st_boundary_wd.raw_st_boundary());
    const auto& new_traj = it->second;
    auto st_boundary_output = input.st_graph->MapMovingSpacetimeObject(
        new_traj, /*generate_lane_change_gap=*/false,
        /*calc_moving_close_traj=*/false, /*nudge_object_info=*/nullptr);
    auto& new_st_boundaries = st_boundary_output.st_boundaries;
    const auto modifier_type = StBoundaryModifierProto::SPEED_MULTI_GAMING;
    for (auto& new_st_boundary : new_st_boundaries) {
      StBoundaryModifierProto modifier;
      modifier.set_modifier_type(modifier_type);
      new_st_boundary->set_id(absl::StrCat(new_st_boundary->id(), "|mg"));
      if (st_boundary.overlap_meta().has_value()) {
        new_st_boundary->set_overlap_meta(*st_boundary.overlap_meta());
      }

      StBoundaryProto::DecisionType new_decision_type =
          it_gaming_decision->second;
      if (new_decision_type == StBoundaryProto::OVERTAKE) {
        bool new_decision_valid = CheckDecisionValid(
            new_st_boundary,
            input.speed_gaming_output->gaming_ego_speed_profile, plan_id);
        if (!new_decision_valid) {
          new_decision_type = StBoundaryProto::IGNORE;
        }
      }
      newly_generated_st_boundaries_wd.emplace_back(
          std::move(new_st_boundary), new_decision_type,
          StBoundaryProto::DecisionReason(
              StBoundaryProto::SPEED_GAMING_DECISION),
          absl::StrCat(
              "modified by ",
              StBoundaryModifierProto::ModifierType_Name(modifier_type)),
          st_boundary_wd.follow_standstill_distance(),
          st_boundary_wd.lead_standstill_distance(), st_boundary_wd.pass_time(),
          st_boundary_wd.yield_time(), input.path->length());
      newly_generated_st_boundaries_wd.back().set_modifier(std::move(modifier));

      // 修改原始st_boundary信息
      st_boundary_wd.set_decision_type(StBoundaryProto::IGNORE);
      st_boundary_wd.set_ignore_reason(StBoundaryProto::ST_BOUNDARY_MODIFIED);
      st_boundary_wd.set_decision_info(absl::StrCat(
          "ignore modified by ",
          StBoundaryModifierProto::ModifierType_Name(modifier_type)));
    }
  }

  // Append newly generated st-boundaries with decision to original ones.
  std::move(newly_generated_st_boundaries_wd.begin(),
            newly_generated_st_boundaries_wd.end(),
            std::back_inserter(*st_boundaries_wd));
  constexpr double kBuffer = 0.4;
  const auto modifier_type = StBoundaryModifierProto::SPEED_MULTI_GAMING;
  std::vector<StBoundaryWithDecision> newly_risk_field_st_boundaries_wd;
  newly_risk_field_st_boundaries_wd.reserve(
      input.speed_gaming_output->gaming_risk_field_zoneinfos.size());
  for (auto& risk_obj_info :
       input.speed_gaming_output->gaming_risk_field_zoneinfos) {
    auto decison_iter =
        input.speed_gaming_output->gaming_decision.find(risk_obj_info.first);
    if (decison_iter == input.speed_gaming_output->gaming_decision.end()) {
      continue;
    }
    auto& risk_field_zone = risk_obj_info.second.risk_field_zoneinfo;
    StBoundaryPoints boundary_points;

    if (decison_iter->second == StBoundaryProto::YIELD) {
      const double cutin_high_s = risk_field_zone.ego_cutin_s;
      const double cutin_lower_s = cutin_high_s - kBuffer;

      const double cutout_high_s = risk_field_zone.ego_cutout_s;
      const double cutout_lower_s = cutout_high_s - kBuffer;
      const double del_s = cutout_high_s - cutin_high_s;
      const double del_t =
          risk_field_zone.agent_cutout_time - risk_field_zone.agent_cutin_time;
      const double v = del_s / std::max(del_t, 1e-6);
      boundary_points.upper_points.emplace_back(
          cutin_high_s, risk_field_zone.agent_cutin_time);
      boundary_points.lower_points.emplace_back(
          cutin_lower_s, risk_field_zone.agent_cutin_time);
      boundary_points.speed_points.emplace_back(
          v, risk_field_zone.agent_cutin_time);
      boundary_points.upper_points.emplace_back(
          cutout_high_s, risk_field_zone.agent_cutout_time);
      boundary_points.lower_points.emplace_back(
          cutout_lower_s, risk_field_zone.agent_cutout_time);
      boundary_points.speed_points.emplace_back(
          v, risk_field_zone.agent_cutout_time);
    } else if (decison_iter->second == StBoundaryProto::OVERTAKE) {
      const double cutin_lower_s = risk_field_zone.ego_cutin_s;
      const double cutin_high_s = cutin_lower_s + kBuffer;

      const double cutout_lower_s = risk_field_zone.ego_cutout_s;
      const double cutout_high_s = cutout_lower_s + kBuffer;
      const double del_s = cutout_high_s - cutin_high_s;
      const double del_t =
          risk_field_zone.agent_cutout_time - risk_field_zone.agent_cutin_time;
      const double v = del_s / std::max(del_t, 1e-6);
      boundary_points.upper_points.emplace_back(
          cutin_high_s, risk_field_zone.agent_cutin_time);
      boundary_points.lower_points.emplace_back(
          cutin_lower_s, risk_field_zone.agent_cutin_time);
      boundary_points.speed_points.emplace_back(
          v, risk_field_zone.agent_cutin_time);
      boundary_points.upper_points.emplace_back(
          cutout_high_s, risk_field_zone.agent_cutout_time);
      boundary_points.lower_points.emplace_back(
          cutout_lower_s, risk_field_zone.agent_cutout_time);
      boundary_points.speed_points.emplace_back(
          v, risk_field_zone.agent_cutout_time);
    }
    auto new_st_boundary = StBoundary::CreateInstance(
        boundary_points,
        ToStBoundaryObjectType(risk_obj_info.second.planner_obj->type()),
        absl::StrCat(risk_obj_info.first, "|mgrf"), 1,
        /*is_stationary=*/false, StBoundaryProto::NON_PROTECTIVE,
        risk_obj_info.second.planner_obj->is_large_vehicle(),
        /*is_traffic_light*/ false, risk_obj_info.second.planner_obj->pose());
    StBoundaryModifierProto modifier;
    modifier.set_modifier_type(modifier_type);

    StBoundaryProto::DecisionType new_decision_type = decison_iter->second;
    if (new_decision_type == StBoundaryProto::OVERTAKE) {
      bool new_decision_valid = CheckDecisionValid(
          new_st_boundary, input.speed_gaming_output->gaming_ego_speed_profile,
          plan_id);
      if (!new_decision_valid) {
        new_decision_type = StBoundaryProto::IGNORE;
      }
    }

    newly_risk_field_st_boundaries_wd.emplace_back(
        std::move(new_st_boundary), new_decision_type,
        StBoundaryProto::DecisionReason(StBoundaryProto::SPEED_GAMING_DECISION),
        StBoundaryModifierProto::ModifierType_Name(modifier_type), 3.5, 4.0,
        0.0, 0.0, input.path->length());
    newly_risk_field_st_boundaries_wd.back().set_modifier(std::move(modifier));
  }

  std::move(newly_risk_field_st_boundaries_wd.begin(),
            newly_risk_field_st_boundaries_wd.end(),
            std::back_inserter(*st_boundaries_wd));

  // Merge newly processed trajectories with the original ones.
  for (auto& [traj_id, traj] :
       input.speed_gaming_output->gaming_processed_st_objects) {
    // DLOG(INFO) << "after post mod: " << traj_id;
    processed_st_objects->insert_or_assign(traj_id, std::move(traj));
  }
}

bool CheckDecisionValid(const StBoundaryRef& st_boundary,
                        const SpeedVector& gaming_ego_speed_profile,
                        const int& plan_id) {
  std::string group_name = Log2DDS::TaskPrefix(plan_id) + "gaming/multi_agent";

  const double time_step = 0.2;
  const double total_time = 8;

  if (st_boundary->IsEmpty() ||
      (gaming_ego_speed_profile.size() <
       static_cast<int>(std::round(total_time / time_step)) + 1)) {
    return false;
  }

  StPoint bottom_left_point = st_boundary->bottom_left_point();
  StPoint bottom_right_point = st_boundary->bottom_right_point();
  double left_point_t = bottom_left_point.t();
  double left_point_s = bottom_left_point.s();
  double right_point_t = bottom_right_point.t();
  double right_point_s = bottom_right_point.s();
  if (right_point_t > total_time)  // 根据轨迹长度截断处理
  {
    right_point_t = total_time;
    std::optional<std::pair<double, double>> obstacle_s_range =
        st_boundary->GetBoundarySRange(right_point_t);
    if (obstacle_s_range == std::nullopt) {
      return false;
    }
    right_point_s = obstacle_s_range->second;
  }

  size_t left_index = static_cast<size_t>(std::round(left_point_t / time_step));
  size_t right_index =
      static_cast<size_t>(std::round(right_point_t / time_step));
  // Log2DDS::LogDataV0(
  //     group_name,
  //     absl::StrCat("left_point_t t: ", left_point_t, " s: ", left_point_s,
  //                  " speed_profile index: ", left_index,
  //                  " s: ", gaming_ego_speed_profile[left_index].s()));
  // Log2DDS::LogDataV0(
  //     group_name,
  //     absl::StrCat("right_point_t t: ", right_point_t, " s: ", right_point_s,
  //                  " speed_profile index: ", right_index,
  //                  " s: ", gaming_ego_speed_profile[right_index].s()));

  if (left_point_s > gaming_ego_speed_profile[left_index].s() &&
      right_point_s > gaming_ego_speed_profile[right_index].s()) {
    Log2DDS::LogDataV0(group_name,
                       absl::StrCat("id ", st_boundary->id(),
                                    " decision is invalid, ignore it. "));
    return false;  // 决策无效，存在碰撞风险
  }
  return true;
}

}  // namespace st::planning


#include "planner/speed_optimizer/cross_blind_decider.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <ostream>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "plan_common/log_data.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"
#include "plan_common/plan_common_defs.h"
#include "planner/speed_optimizer/speed_finder_util.h"
#include "plan_common/speed/st_speed/speed_limit.h"
#include "planner/speed_optimizer/st_close_trajectory.h"
#include "planner/speed_optimizer/st_graph.h"

namespace st::planning {

void CrossCloseDecider(
    const CrossBlindInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision,
    const std::vector<DrivingProcess>& driving_process_seq) {
  if (driving_process_seq.empty()) {
    return;
  }
  const auto& path_points = *input.path;
  const auto driving_zone =
      std::lower_bound(driving_process_seq.begin(), driving_process_seq.end(),
                       path_points.front().s(),
                       [](const DrivingProcess& driving_process, double av_s) {
                         return driving_process.end_s < av_s;
                       });
  if (driving_zone == driving_process_seq.end()) {
    return;
  }
  if (driving_zone->lane_semantic != LaneSemantic::ROAD &&
      driving_zone->lane_semantic != LaneSemantic::INTERSECTION_STRAIGHT) {
    return;
  }
  if (driving_zone->lane_semantic == LaneSemantic::ROAD) {
    Log2DDS::LogDataV0("CrossCloseDecider", "on Straight");
  }
  if (driving_zone->lane_semantic == LaneSemantic::INTERSECTION_STRAIGHT) {
    Log2DDS::LogDataV0("CrossCloseDecider", "on JunctionStraight");
  }
  // constexpr double kBufferOutRightLane = 3.0;
  // if (driving_zone->end_s <
  //     input.vehicle_geom->front_edge_to_center() + kBufferOutRightLane) {
  //   Log2DDS::LogDataV0("GetRightTurnCloseSpeedLimit",
  //                      "rerturn { out turn right lane}");
  //   return;
  // }
  ///
  const auto& st_traj_mgr = *input.st_traj_mgr;
  const auto& moving_spacetime_objects = st_traj_mgr.moving_object_trajs();

  absl::flat_hash_map<std::string, StBoundaryWithDecision*>
      cross_obs_st_boundary_wd_map;
  std::vector<std::pair<std::string, Segment2d>> fov_lines;
  for (auto& st_boundary_wd : *st_boundaries_with_decision) {
    // auto& obj_scene_info = stb_wd.st_boundary()->obj_scenario_info();

    // const StBoundary* st_boundary = st_boundary_wd.st_boundary();
    const auto& st_boundary = *st_boundary_wd.raw_st_boundary();
    auto obj_scene_info = st_boundary.obj_scenario_info();
    if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (st_boundary.object_type() != StBoundaryProto::VEHICLE) {
      continue;
    }
    const auto& overlap_meta = *st_boundary.overlap_meta();
    if (overlap_meta.pattern() == StOverlapMetaProto::CROSS) {
      if (obj_scene_info.lane_semantic != LaneSemantic::ROAD &&
          obj_scene_info.lane_semantic !=
              LaneSemantic ::INTERSECTION_STRAIGHT) {
        continue;
      }
      CHECK(st_boundary.traj_id().has_value());
      const auto& traj_id = st_boundary.traj_id();
      CHECK(st_boundary.object_id().has_value());
      const auto object_id = st_boundary.object_id().value();
      const auto* traj =
          CHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*traj_id));

      const auto& overlap_infos = st_boundary.overlap_infos();
      CHECK(!overlap_infos.empty());
      const auto& first_overlap_info = overlap_infos.front();
      const auto* first_overlap_obj_point =
          traj->states()[first_overlap_info.obj_idx].traj_point;
      const auto first_overlap_obj_heading = first_overlap_obj_point->theta();
      const auto first_overlap_av_middle_point =
          path_points[(first_overlap_info.av_start_idx +
                       first_overlap_info.av_end_idx) /
                      2];
      const auto first_overlap_av_middle_heading =
          first_overlap_av_middle_point.theta();
      constexpr double kOnComingThreshold_upper = 10.0 * M_PI / 18.0;  // 100
      constexpr double kOnComingThreshold_lower = 8.0 * M_PI / 18.0;   // 80

      double heading_diff = std::abs(NormalizeAngle(
          first_overlap_obj_heading - first_overlap_av_middle_heading));
      // 交互点角度过滤
      if (heading_diff < kOnComingThreshold_lower ||
          heading_diff > kOnComingThreshold_upper) {
        continue;  // return false;
      }
      // 当前状态角度过滤
      if (obj_scene_info.delta_heading < kOnComingThreshold_lower ||
          obj_scene_info.delta_heading > kOnComingThreshold_upper) {
        continue;
      }
      double av_time = first_overlap_av_middle_point.s() / input.current_v;
      double obs_time =
          first_overlap_obj_point->s() / traj->planner_object().pose().v();
      if (std::fabs(av_time - obs_time) > 3.0) {
        continue;
      }
      cross_obs_st_boundary_wd_map.emplace(object_id, &st_boundary_wd);
      double frontAxle_x =
          path_points[0].x() +
          input.vehicle_geom->wheel_base() * std::cos(path_points[0].theta());
      double frontAxle_y =
          path_points[0].y() +
          input.vehicle_geom->wheel_base() * std::sin(path_points[0].theta());
      fov_lines.emplace_back(std::make_pair(
          object_id, Segment2d(traj->planner_object().pose().pos(),
                               Vec2d(frontAxle_x, frontAxle_y))));
    }
  }

  const auto& stationary_trajectories = st_traj_mgr.stationary_objects();
  for (auto& [id, fov_line_seg] : fov_lines) {
    std::vector<std::string> blind_obs;
    for (size_t i = 0; i < stationary_trajectories.size(); i++) {
      const auto obj = stationary_trajectories.at(i);
      const std::string object_id(obj.object_id);
      const ObjectProto& object_proto = obj.planner_object.object_proto();
      if (object_proto.has_min_z() && object_proto.has_max_z() &&
          object_proto.has_ground_z() &&
          object_proto.max_z() - object_proto.ground_z() < 4.0) {
        continue;
      }
      const Box2d& obj_box = obj.planner_object.bounding_box();
      if (obj_box.area() < 5.0) {
        continue;
      }
      if (obj_box.HasOverlapWithBuffer(fov_line_seg, 1.5, 0.0)) {
        blind_obs.push_back(object_id);
      }
    }
    //
    if (!blind_obs.empty()) {
      auto cross_st_boundary_wd = cross_obs_st_boundary_wd_map[id];
      Log2DDS::LogDataV0("CrossCloseDecider", absl::StrCat("cross blind:", id));
      auto& obj_scene_info = cross_st_boundary_wd->mutable_raw_st_boundary()
                                 ->mutable_obj_scenario_info();
      // if (!obj_scene_info) continue;
      auto& param = obj_scene_info.obj_decision_param;
      {
        param.dp_follow_lead_ratio = 1.5;
        param.pass_time_additional_buffer = 1.5;
        param.yield_time_additional_buffer = 0.5;
        param.agent_reaction_time = 2.0;
        param.geometry_theory_av_follow_distance = 0.5;
        cross_st_boundary_wd->set_follow_standstill_distance(
            cross_st_boundary_wd->follow_standstill_distance() + 3.0);
        continue;
      }
    }
  }
}

}  // namespace st::planning

#pragma once

#include <string>
#include <vector>

#include "object_manager/object_history.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "plan_common/maps/st_boundary_with_decision.h"
#include "plan_common/log_data.h"
#include "planner/speed_optimizer/st_graph.h"
#include "planner/speed_optimizer/cipv_object_info.h"
#include "planner/speed_optimizer/decider/pre_brake_util.h"
#include "predictor/prediction_defs.h"
#include "modules/cnoa_pnc/planning/proto/vehicle.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder.pb.h"

namespace st::planning {

struct DefensiveSpeedDeciderInput {
  int plan_id = 0;
  double plan_start_v = 0.0;
  double plan_start_a = 0.0;
  double time_step = 0.0;
  int step_num = 0;
  double max_ego_v = 0.0;
  LaneChangeStage lc_stage;
  const DiscretizedPath* path = nullptr;
  const SpacetimeTrajectoryManager* traj_mgr = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const ObjectHistoryManager* obj_history_mgr = nullptr;
  const std::map<std::string, ObjectSlInfo>* obj_sl_map = nullptr;
  const CipvObjectInfo* cipv_info = nullptr;
  const std::map<std::string, ConstraintProto::LeadingObjectProto>*
      leading_trajs = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
};

struct ObjectDerivedInfo {
  double lon_speed = 0.0;
  double lat_fit_speed = 0.0;
  double lon_fit_accel = 0.0;
  double angle_diff_with_dp = 0.0;
  double object_width = 0.0;
  double object_length = 0.0;
  double left_bound_l = 0.0;
  double right_bound_l = 0.0;
  double relative_s_to_av = 0.0;
  double relative_l_to_av = 0.0;
  bool is_on_av_boundary = false;
  double invasion_dist = 0.0;
  FrenetBox frenet_box;
  FrenetCoordinate sl_pos;
  FrenetCoordinate av_sl_pos;

  std::string DebugString() const {
    return absl::StrFormat(
        "lon_speed: %.3f, lat_fit_speed: %.3f, lon_fit_acc: %3f, angle_diff: "
        "%.3f, object_width: %.3f, object_length: %.3f, left_bound_l: %.3f, "
        " right_bound_l: %.3f, "
        " relative_s: %.3f, relative_l: %.3f, on_bound: %s, invasion_dist: %.3f"
        " sl_pos.s: %.3f, sl_pos.l: %.3f, av_sl_pos.s: %.3f, av_sl_pos.l: %.3f",
        lon_speed, lat_fit_speed, lon_fit_accel, r2d(angle_diff_with_dp),
        object_width, object_length, left_bound_l, right_bound_l,
        relative_s_to_av, relative_l_to_av,
        is_on_av_boundary ? "true" : "false", invasion_dist, sl_pos.s, sl_pos.l,
        av_sl_pos.s, av_sl_pos.l);
  }

  inline bool FromRight() const {
    return std::fabs(frenet_box.l_max) < std::fabs(frenet_box.l_min);
  }
};

std::optional<VtSpeedLimit> DecideDefensiveSpeedLimit(
    const DefensiveSpeedDeciderInput& input,
    DefensiveSpeedProto* defensive_speed_state);

}  // namespace st::planning

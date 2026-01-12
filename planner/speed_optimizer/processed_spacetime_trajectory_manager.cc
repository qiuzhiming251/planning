

#include "planner/speed_optimizer/processed_spacetime_trajectory_manager.h"

#include <algorithm>
#include <iterator>
#include <ostream>

#include "plan_common/async/parallel_for.h"
//#include "global/buffered_logger.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "object_manager/trajectory_filter.h"
#include "predictor/prediction.h"
#include "predictor/prediction_util.h"

namespace st {
namespace planning {

bool IsSameDirNearObj(const SpeedFinderInput& input,
                      const SpacetimeObjectTrajectory& traj) {
  if (traj.is_stationary()) {
    return false;
  }
  std::string obj_id = std::string(traj.object_id());
  const auto& obj = input.traj_mgr->FindObjectByObjectId(obj_id);
  if (obj == nullptr) {
    return false;
  }
  if (obj->pose().v() < Kph2Mps(20.0)) {
    return false;
  }
  const auto obj_sl_pos = input.path->XYToSL(obj->pose().pos());
  if (obj_sl_pos.s - obj->bounding_box().length() * 0.5 < 1e-4 ||
      obj_sl_pos.s - obj->bounding_box().length() * 0.5 > 15.0) {
    return false;
  }
  if (std::fabs(obj_sl_pos.l) > 3.0) {
    return false;
  }
  const auto& av_path_pt = input.path->Evaluate(obj_sl_pos.s);
  constexpr double kParallelHeadingThreshold = M_PI / 6.0;
  const double theta_diff =
      std::fabs(NormalizeAngle(av_path_pt.theta() - obj->pose().theta()));
  if (std::fabs(theta_diff) > kParallelHeadingThreshold) {
    return false;
  }

  return true;
}

void ProcessedSpacetimeTrajectoryManager::ModifySpacetimeTrajectory(
    const SpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const SpeedFinderParamsProto& speed_default_params) {
  // modify traj manager
  const auto considered_trajs_ptr = this->mutable_trajectories();
  const auto& leading_objs = (*input.leading_trajs);
  const auto& traffic_gap = (*input.constraint_mgr).TrafficGap();

  for (size_t i = 0; i < considered_trajs_ptr->size(); i++) {
    const auto& st_traj = considered_trajs_ptr->at(i);
    const std::string traj_id(st_traj.traj_id());
    SpacetimeObjectTrajectory modified_st_traj(st_traj);
    if ((*input.leading_trajs).find(traj_id) != (*input.leading_trajs).end()) {
      // get leading traj, and replace it
      const auto& leading_traj = leading_objs.at(traj_id).modified_trajectory();
      if (leading_traj.size() < 1) continue;
      // construct new st_traj from modified_traj(predicted_trajectory_point)
      modified_st_traj.mutable_trajectory()->mutable_points()->clear();
      for (size_t j = 0; j < leading_traj.size(); j++) {
        prediction::PredictedTrajectoryPoint traj_point(leading_traj[j]);
        modified_st_traj.mutable_trajectory()->mutable_points()->emplace_back(
            traj_point);
      }
      VLOG_IF(2, true) << "[process stb] modifying leading: " << traj_id;
    } else if (traffic_gap.leader_id.has_value() &&
               traffic_gap.leader_id == traj_id) {
      modified_st_traj = st_traj;
      VLOG_IF(2, true) << "[process stb] modifying traffic gap leadr: "
                       << traj_id;
    } else if (IsSameDirNearObj(input, st_traj)) {
      modified_st_traj = st_traj;
    } else {
      continue;
    }
    if (!modified_st_traj.trajectory().points().empty() &&
        !modified_st_traj.states().empty()) {
      // refine acc
      double curr_acc = st_traj.planner_object().pose().a();
      double leading_thw = 3.0;
      const absl::StatusOr<FrenetBox> frenet_box_or =
          (*input.drive_passage).QueryFrenetBoxAtContour(st_traj.contour());
      if (frenet_box_or.ok() &&
          frenet_box_or->s_min -
                  vehicle_geometry_params.front_edge_to_center() >
              0.0) {
        leading_thw = (frenet_box_or->s_min -
                       vehicle_geometry_params.front_edge_to_center()) /
                      std::max(0.1, input.plan_start_v);
      }
      const PiecewiseLinearFunction leading_obj_acc_plf(
          PiecewiseLinearFunctionFromProto(
              speed_default_params.leading_obj_acc_plf_params()));
      const PiecewiseLinearFunction leading_obj_acc_factor_plf(
          PiecewiseLinearFunctionFromProto(
              speed_default_params.leading_obj_acc_factor_params()));
      double estimate_acc_ts_sec =
          curr_acc < 0.0 ? leading_obj_acc_plf(leading_thw) : 2.0;
      if (st_traj.planner_object().type() == ObjectType::OT_VEHICLE ||
          st_traj.planner_object().type() == ObjectType::OT_LARGE_VEHICLE) {
        if (input.attention_obj_id == std::string(st_traj.object_id()) ||
            speed_default_params.enable_hard_brake_for_every_obj()) {
          const double acc_factor = leading_obj_acc_factor_plf(curr_acc);
          curr_acc = std::fmax(
              curr_acc * acc_factor,
              curr_acc - speed_default_params.max_leading_acc_extend());
        }
      }
      prediction::RefineTrajByAcc(modified_st_traj.mutable_trajectory(),
                                  curr_acc, estimate_acc_ts_sec);

      // update_states
      const auto& traj_points = modified_st_traj.trajectory().points();
      modified_st_traj.mutable_states()->clear();
      const auto& init_contour = st_traj.contour();
      const auto& init_box = st_traj.bounding_box();
      const auto& init_pt = traj_points.at(0);
      const double init_heading = init_pt.theta();
      const Vec2d init_pos = init_pt.pos();
      const Vec2d box_pos_shift = init_box.center() - init_pos;
      for (size_t j = 0; j < traj_points.size(); j++) {
        const prediction::PredictedTrajectoryPoint& traj_point =
            traj_points.at(j);
        const Vec2d rotation =
            Vec2d::FastUnitFromAngle(traj_point.theta() - init_heading);
        const auto contour = init_contour.Transform(
            init_pos, rotation.x(), rotation.y(), traj_point.pos() - init_pos);
        prediction::PredictionObjectState new_state;
        new_state.traj_point = &traj_point;
        new_state.contour = contour;
        new_state.box =
            Box2d(traj_point.pos() + box_pos_shift, traj_point.theta(),
                  init_box.length(), init_box.width());
        modified_st_traj.mutable_states()->emplace_back(new_state);

        VLOG_IF(2, true) << "[process stb] modifying: " << traj_id
                         << ", t/acc_t " << traj_point.t() << "/"
                         << estimate_acc_ts_sec << ", " << traj_point.a();
      }
      considered_trajs_ptr->at(i) = modified_st_traj;
    }
  }
  this->UpdatePointers(this->stationary_object_trajs().size());
}

}  // namespace planning
}  // namespace st

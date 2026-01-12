

#include "decider/decision_manager/decision_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/math/util.h"
#include "plan_common/math/fast_math.h"
#include "absl/status/statusor.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "object_manager/spacetime_trajectory_manager.h"
#include "modules/cnoa_pnc/planning/proto/affine_transformation.pb.h"
#include "predictor/prediction_object_state.h"
#include "plan_common/plan_common_defs.h"
#include "predictor/predicted_trajectory.h"
#include "predictor/prediction_util.h"
#include "plan_common/util/status_macros.h"

namespace st {
namespace planning {

namespace {

constexpr double kEpsilon = 1e-3;

SpeedProfile IntegrateSpeedProfile(const DrivePassage& drive_passage,
                                   const std::vector<double>& v_s) {
  // Integrate the v-s profile in time.
  CHECK_EQ(drive_passage.size(), v_s.size());
  std::vector<double> path_s(drive_passage.size());
  for (int i = 0; i < drive_passage.size(); ++i) {
    path_s[i] = drive_passage.station(StationIndex(i)).accumulated_s();
  }
  const PiecewiseSqrtFunction<double, double> v_s_plf(path_s, v_s);
  std::vector<double> t(kTrajectorySteps);
  for (int i = 0; i < t.size(); ++i) t[i] = i * kTrajectoryTimeStep;
  std::vector<double> s(t.size());
  double current_s = 0.0;
  for (int i = 0; i < t.size(); ++i) {
    s[i] = current_s;
    const double current_v = v_s_plf(current_s);
    current_s += std::max(current_v, kEpsilon) * kTrajectoryTimeStep;
    current_s = std::min(path_s.back(), current_s);
  }

  // Create the speed profile from the s-t function.
  return SpeedProfile(PiecewiseLinearFunction(std::move(t), std::move(s)));
}

void ApplySpeedConstraintForReferenceSpeedProfile(
    double s_start, double s_end, double v_max,
    const DrivePassage& drive_passage, std::vector<double>* v_s) {
  CHECK_LE(s_start, s_end);
  CHECK_EQ(drive_passage.size(), v_s->size());
  // Max speed dictated by this speed constraint as a function of s.
  // Between s_start and s_end, max speed is v_max. Before that the max
  // speed follows a constant deceleration profile, and after that a
  // constant acceleration profile.
  const double v_max_sqr = Sqr(v_max);
  const auto compute_v_constraint_sqr = [s_start, s_end, v_max_sqr](double s) {
    constexpr double kAccelLimit = 2.0;  // m/s^2.
    if (s < s_start) {
      // Deceleration segment.
      return v_max_sqr + (s_start - s) * (kAccelLimit * 2.0);
    }
    if (s > s_end) {
      // Acceleration segment.
      return v_max_sqr + (s - s_end) * (kAccelLimit * 2.0);
    }
    return v_max_sqr;
  };

  for (int i = 0; i < drive_passage.size(); ++i) {
    const double s = drive_passage.station(StationIndex(i)).accumulated_s();
    const double v_constraint_sqr = compute_v_constraint_sqr(s);
    if (v_constraint_sqr < Sqr((*v_s)[i])) {
      (*v_s)[i] = std::sqrt(v_constraint_sqr);
    }
  }
}
// 补齐预测轨迹时长
void ExtendPredictionTraj(
    const planning::SpacetimeObjectTrajectory& traj,
    planning::ConstraintProto::LeadingObjectProto* leading_obj) {
  // get current perception state
  const auto& obj_cur_pos = traj.planner_object().pose().pos();

  // extend traj
  prediction::PredictedTrajectory extend_traj;
  extend_traj.set_probability(traj.trajectory().probability());
  extend_traj.set_index(traj.trajectory().index());
  bool current_state_insert = false;
  for (size_t i = 0; i < traj.states().size() - 1; ++i) {
    // check current perception state
    Vec2d base_vec = traj.states().at(i + 1).traj_point->pos() -
                     traj.states().at(i).traj_point->pos();
    Vec2d tgt_vec = obj_cur_pos - traj.states().at(i).traj_point->pos();

    const double pj_length = tgt_vec.Dot(base_vec.normalized());
    if (!current_state_insert) {
      if (pj_length < 0) {
        prediction::PredictedTrajectoryPoint pt(traj.planner_object().pose());
        extend_traj.mutable_points()->emplace_back(std::move(pt));
        extend_traj.mutable_points()->emplace_back(
            std::move(*(traj.states().at(i).traj_point)));
        current_state_insert = true;
      } else if (pj_length > 0 && pj_length < base_vec.Length()) {
        prediction::PredictedTrajectoryPoint pt(traj.planner_object().pose());
        extend_traj.mutable_points()->emplace_back(std::move(pt));
        current_state_insert = true;
      } else {
        continue;
      }
    } else {
      extend_traj.mutable_points()->emplace_back(
          std::move(*(traj.states().at(i).traj_point)));
    }
  }
  extend_traj.mutable_points()->emplace_back(
      std::move(*(traj.states().back().traj_point)));

  //  after original predictive time region: 6-8s
  const int origin_traj_size = extend_traj.points().size();
  const int extend_pt_num = prediction::kPredictionPointNum - origin_traj_size;
  if (extend_pt_num > 0) {
    Vec2d cv_pos_t(0.0, 0.0);
    if (extend_traj.points().size() > 10) {
      Vec2d dpos =
          extend_traj.points().back().pos() -
          extend_traj.points().at(extend_traj.points().size() - 10).pos();
      double dt = extend_traj.points().back().t() -
                  extend_traj.points().at(extend_traj.points().size() - 10).t();
      cv_pos_t = prediction::kPredictionTimeStep * dpos / std::max(0.1, dt);

    } else {
      Vec2d dpos = extend_traj.points().back().pos() -
                   extend_traj.points().front().pos();
      double dt =
          extend_traj.points().back().t() - extend_traj.points().front().t();
      cv_pos_t = prediction::kPredictionTimeStep * dpos / std::max(0.1, dt);
    }
    double cv_v = cv_pos_t.Length() / prediction::kPredictionTimeStep;
    for (int i = 0; i < extend_pt_num; ++i) {
      prediction::PredictedTrajectoryPoint traj_pt;
      traj_pt.set_pos(extend_traj.points().back().pos() + cv_pos_t);
      traj_pt.set_s(extend_traj.points().back().s() + cv_pos_t.Length());
      traj_pt.set_theta(fast_math::Atan2(cv_pos_t.x(), cv_pos_t.y()));
      traj_pt.set_kappa(0.0);
      traj_pt.set_t(extend_traj.points().back().t() +
                    prediction::kPredictionTimeStep);
      traj_pt.set_v(cv_v);
      traj_pt.set_a(0.0);
      extend_traj.mutable_points()->emplace_back(std::move(traj_pt));
    }
    DLOG(INFO) << "extend traj size: " << extend_traj.points().size();
  }

  // // refine acc
  // const double curr_acc = traj.planner_object().pose().a();
  // double leading_thw = 3.0;
  // const absl::StatusOr<FrenetBox> frenet_box_or =
  // (*input.drive_passage).QueryFrenetBoxAtContour(st_traj.contour()); if
  // (frenet_box_or.ok() && frenet_box_or->s_min -
  // vehicle_geometry_params.front_edge_to_center() > 0.0) {
  //   leading_thw =
  //       (frenet_box_or->s_min -
  //       vehicle_geometry_params.front_edge_to_center()) / std::max(0.1,
  //       input.plan_start_v);
  // }
  // const PiecewiseLinearFunction
  // leading_obj_acc_plf(std::vector<double>{0.0, 3.0},
  // std::vector<double>{3.0, 2.0}); double estimate_acc_ts_sec =
  // std::clamp(leading_obj_acc_plf(leading_thw), 2.0, 3.0);
  // RefineTrajByAcc(&extend_traj, curr_acc, estimate_acc_ts_sec);

  planning::SpacetimeObjectTrajectory st_obj_traj(
      traj.planner_object(), std::move(extend_traj), traj.traj_index(),
      traj.required_lateral_gap());
  for (const auto& state : st_obj_traj.states()) {
    state.traj_point->ToProto(leading_obj->add_modified_trajectory());
  }
}

}  // namespace

SpeedProfile CreateSpeedProfile(
    double v_now, const DrivePassage& drive_passage,
    const PlannerSemanticMapManager& psmm,
    const absl::Span<const ConstraintProto::SpeedRegionProto>& speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto>& stop_points) {
  const auto dp_size = drive_passage.size();

  std::vector<double> v_s;
  v_s.reserve(dp_size);
  for (int i = 0; i < dp_size; ++i) {
    const double speed_limit_mps =
        drive_passage.station(StationIndex(i)).speed_limit();  // m/s
    v_s.push_back(speed_limit_mps);
  }

  for (const auto& speed_zone : speed_zones) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        speed_zone.start_s(), speed_zone.end_s(), speed_zone.max_speed(),
        drive_passage, &v_s);
  }
  for (const auto& stop_point : stop_points) {
    ApplySpeedConstraintForReferenceSpeedProfile(
        stop_point.s(), /*s_end=*/std::numeric_limits<double>::infinity(),
        /*v_max=*/0.0, drive_passage, &v_s);
  }

  for (int i = 0; i < dp_size; ++i) {
    const double s = drive_passage.station(StationIndex(i)).accumulated_s();
    // Note: why? Refactor later.
    const double v_constraint =
        std::pow(std::abs(s) + pow(std::abs(v_now) / 2.3, 2.5), 0.4) * 2.3;
    (v_s)[i] = std::min((v_s)[i], v_constraint);
  }

  return IntegrateSpeedProfile(drive_passage, v_s);
}

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements) {
  CHECK_GT(elements.size(), 0);
  ConstraintProto::SpeedRegionProto merged_ele = elements[0];
  int begin_idx = 0;
  double min_s = std::numeric_limits<double>::infinity();
  int end_idx = 0;
  double max_s = -std::numeric_limits<double>::infinity();
  for (int i = 0, n = elements.size(); i < n; ++i) {
    const auto& ele = elements[i];
    if (ele.start_s() < min_s) {
      min_s = ele.start_s();
      begin_idx = i;
    }
    if (ele.end_s() > max_s) {
      max_s = ele.end_s();
      end_idx = i;
    }
  }
  merged_ele.set_start_s(min_s);
  merged_ele.set_end_s(max_s);
  *merged_ele.mutable_start_point() = elements[begin_idx].start_point();
  *merged_ele.mutable_end_point() = elements[end_idx].end_point();
  return merged_ele;
}

bool IsLeadingObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_LARGE_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    // case OT_CYCLIST:
    // case OT_TRICYCLIST:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

std::pair<double, double> CalcSlBoundaries(const PathSlBoundary& sl_boundary,
                                           const FrenetBox& frenet_box) {
  if (frenet_box.s_max < sl_boundary.start_s() ||
      frenet_box.s_min > sl_boundary.end_s()) {
    // If the box is completely out of path boundary.
    return {0.0, 0.0};
  }

  const auto [boundary_front_right_l, boundary_front_left_l] =
      sl_boundary.QueryBoundaryL(frenet_box.s_max);
  const double boundary_front_center_l =
      sl_boundary.QueryReferenceCenterL(frenet_box.s_max);

  const auto [boundary_rear_right_l, boundary_rear_left_l] =
      sl_boundary.QueryBoundaryL(frenet_box.s_min);
  const double boundary_rear_center_l =
      sl_boundary.QueryReferenceCenterL(frenet_box.s_min);

  constexpr double kMinHalfBoundaryWidth = 1.1;  // m.
  const double boundary_l_min =
      std::min(std::min(boundary_front_right_l,
                        boundary_front_center_l - kMinHalfBoundaryWidth),
               std::min(boundary_rear_right_l,
                        boundary_rear_center_l - kMinHalfBoundaryWidth));
  const double boundary_l_max =
      std::max(std::max(boundary_front_left_l,
                        boundary_front_center_l + kMinHalfBoundaryWidth),
               std::max(boundary_rear_left_l,
                        boundary_rear_center_l + kMinHalfBoundaryWidth));

  return {boundary_l_max, boundary_l_min};
}

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const DrivePassage& passage,
    ConstraintProto::LeadingObjectProto::Reason reason, bool is_group_tail) {
  ConstraintProto::LeadingObjectProto leading_object;
  leading_object.set_traj_id(std::string(traj.traj_id()));
  leading_object.set_reason(reason);
  leading_object.set_is_group_tail(is_group_tail);

  constexpr double kSampleTimeInterval = 1.0;  // Seconds.
  // Generate ST-constraints based on object current bounding box, for
  // stationary object.
  if (traj.is_stationary()) {
    ASSIGN_OR_RETURN(const auto obj_frenet_box,
                     passage.QueryFrenetBoxAt(traj.bounding_box()),
                     leading_object);
    for (double sample_time = 0.0; sample_time <= kTrajectoryTimeHorizon;
         sample_time += kSampleTimeInterval) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(sample_time);
    }
    if (kTrajectoryTimeHorizon - leading_object.st_constraints().rbegin()->t() >
        kEpsilon) {
      auto* constraint = leading_object.add_st_constraints();
      constraint->set_s(obj_frenet_box.s_min);
      constraint->set_t(kTrajectoryTimeHorizon);
    }

    return leading_object;
  }

  double sample_time = 0.0;
  const double traj_last_time = traj.states().back().traj_point->t();
  // Generate ST-constraints based on spacetime states, for moving object.
  for (const auto& state : traj.states()) {
    // Sample ST-constraints by 1.0s.
    const auto* traj_point = state.traj_point;
    const double t = traj_point->t();
    // Generate ST-constraints at sample time and trajectory last time.
    if (std::abs(t - sample_time) > kEpsilon &&
        std::abs(traj_last_time - t) > kEpsilon) {
      continue;
    }
    // Filter object state out of passage.
    ASSIGN_OR_CONTINUE(const auto obj_frenet_box,
                       passage.QueryFrenetBoxAt(state.box));
    // Filter object state by drive passage direction.
    ASSIGN_OR_CONTINUE(const auto passage_tangent,
                       passage.QueryTangentAngleAtS(obj_frenet_box.s_min));
    const double angle_diff =
        std::abs(NormalizeAngle(passage_tangent - traj_point->theta()));
    if (angle_diff > M_PI_2) continue;

    auto* constraint = leading_object.add_st_constraints();
    constraint->set_s(obj_frenet_box.s_min);
    constraint->set_t(t);

    sample_time += kSampleTimeInterval;
  }

  ExtendPredictionTraj(traj, &leading_object);

  // for debug
  // if (leading_object.traj_id().compare("29345-idx0") == 0) {
  //   for (auto& pt : leading_object.modified_trajectory()) {
  //     DLOG(INFO) << " leading obstacle " << leading_object.traj_id() << " pos
  //     "
  //                << pt.pos().x() << "," << pt.pos().y() << " s " << pt.s()
  //                << " spd: " << pt.v() << " t: " << pt.t();
  //   }
  // }

  return leading_object;
}

bool IsTrafficLightControlledLane(const ad_byd::planning::Lane& lane) {
  return lane.light_status() != ad_byd::planning::LightStatus::NONE_LIGHT;
}

}  // namespace planning
}  // namespace st

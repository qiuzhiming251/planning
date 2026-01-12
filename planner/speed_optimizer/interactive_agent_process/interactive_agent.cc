

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

#include "planner/speed_optimizer/interactive_agent_process/interactive_agent.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/util.h"

namespace st {
namespace planning {

void CvtPoseGlobalToLocal(SecondOrderTrajectoryPoint *local_pose,
                          const SecondOrderTrajectoryPoint *global_pose,
                          const SecondOrderTrajectoryPoint *base_pose) {
  if (nullptr == local_pose || nullptr == global_pose || nullptr == base_pose) {
    return;
  }

  double dx = global_pose->pos().x() - base_pose->pos().x();
  double dy = global_pose->pos().y() - base_pose->pos().y();
  double theta = base_pose->theta();

  double tmp_x = std::cos(theta) * dx + std::sin(theta) * dy;
  double tmp_y = std::cos(theta) * dy - std::sin(theta) * dx;
  double tmp_theta = ad_byd::planning::math::NormalizeAngle(
      global_pose->theta() - base_pose->theta());

  local_pose->set_pos(Vec2d(tmp_x, tmp_y));
  local_pose->set_theta(tmp_theta);
}

void CvtPoseLocalToGlobal(SecondOrderTrajectoryPoint *global_pose,
                          const SecondOrderTrajectoryPoint *local_pose,
                          const SecondOrderTrajectoryPoint *base_pose) {
  if (nullptr == global_pose || nullptr == local_pose || nullptr == base_pose) {
    return;
  }

  double lx = local_pose->pos().x();
  double ly = local_pose->pos().y();
  double theta = base_pose->theta();

  double tmp_x =
      base_pose->pos().x() + std::cos(theta) * lx - std::sin(theta) * ly;
  double tmp_y =
      base_pose->pos().y() + std::cos(theta) * ly + std::sin(theta) * lx;
  double tmp_theta = ad_byd::planning::math::NormalizeAngle(
      local_pose->theta() + base_pose->theta());

  global_pose->set_pos(Vec2d(tmp_x, tmp_y));
  global_pose->set_theta(tmp_theta);
}

void JudgeTurnInfo(bool *is_turn_left, bool *is_turn_right, bool *is_uturn,
                   const SecondOrderTrajectoryPoint *global_start,
                   const SecondOrderTrajectoryPoint *global_end) {
  constexpr double kUTurnHeadingDiff = 0.75 * M_PI;  // 135 deg.
  constexpr double kTurnHeadingDiff = M_PI / 3.0;    // 60 deg.

  if (nullptr == is_turn_left || nullptr == is_turn_right ||
      nullptr == is_uturn) {
    return;
  }

  *is_turn_left = false;
  *is_turn_right = false;
  *is_uturn = false;

  SecondOrderTrajectoryPoint local_pose;

  CvtPoseGlobalToLocal(&local_pose, global_end, global_start);

  double theta_diff =
      std::abs(NormalizeAngle(global_end->theta() - global_start->theta()));

  if (theta_diff >= kUTurnHeadingDiff) {
    *is_uturn = true;
  } else if (theta_diff >= kTurnHeadingDiff) {
    if (local_pose.pos().x() > 0.0) {
      *is_turn_right = true;
    } else if (local_pose.pos().x() < 0.0) {
      *is_turn_left = true;
    }
  }

  return;
}

void InteractiveAgent::ResetInteractiveAgent() {
  id_ = "-1";
  param_.ResetAgentParam();
  type_ = INTERACTIVE_AGENT_TYPE_NUM;
  state_.clear();
  Polygon2d init_polygon;
  SetInitInfo(0.0, 0.0, 0.0, init_polygon, Vec2d(0.0, 0.0));
  obstacle_ = nullptr;
  is_valid_ = false;
  drive_passage_ = nullptr;
  st_boundary_decision_ = nullptr;
}

InteractiveAgent::InteractiveAgent() { ResetInteractiveAgent(); }

void InteractiveAgent::SetEgoInitState(
    const VehicleGeometryParamsProto &vehicle_geom, const DiscretizedPath &path,
    double ego_init_vel, double ego_init_acc,
    const std::vector<VehicleShapeBasePtr> &av_shapes,
    const DrivePassage *drive_passage, const bool is_debug) {
  this->SetAgentID("ego_");
  this->SetEgoParams(vehicle_geom);
  this->SetAgentType(INTERACTIVE_AGENT_EGO);
  this->SetDrivePassage(drive_passage);
  this->SetGameTheoryScenario(GAME_THEORY_SCENARIO_UNKNOWN);

  Polygon2d ego_init_polygon;
  if (av_shapes.size() > 0) {
    const auto &av_shape_ptr = av_shapes[0];
    if (nullptr != av_shape_ptr) {
      ego_init_polygon =
          Polygon2d(av_shape_ptr->GetCornersWithBufferCounterClockwise(
                        /*lat_buffer=*/0.2, /*lon_buffer=*/0.2),
                    /*is_convex=*/true);
    }
  }

  if (path.size() > 0) {
    Vec2d ego_init_pos(path[0].x(), path[0].y());
    this->SetInitInfo(ego_init_vel, ego_init_acc, path[0].theta(),
                      ego_init_polygon, ego_init_pos);
  }

  /* update ego states for find state */

  double last_s = 0.0;
  for (int i = 0; i < path.size(); i++) {
    const auto &av_shape_ptr = av_shapes[i];
    if (nullptr == av_shape_ptr) continue;
    const Polygon2d av_polygon =
        Polygon2d(av_shape_ptr->GetCornersWithBufferCounterClockwise(
                      /*lat_buffer=*/0.2, /*lon_buffer=*/0.2),
                  /*is_convex=*/true);
    if (0 != i) {
      if ((path[i].s() - last_s) < 5.0) continue;
    }
    State tmp_state;
    tmp_state.is_rectangular = true;
    tmp_state.traj_idx = i;
    tmp_state.dist = path[i].s();
    tmp_state.time_stamp = 0.0;
    tmp_state.velocity = ego_init_vel;
    tmp_state.acc = ego_init_acc;
    tmp_state.heading = path[i].theta();

    SecondOrderTrajectoryPoint sl_center_pose, global_ego_pose,
        local_center_pose;
    global_ego_pose.set_pos(Vec2d(path[i].x(), path[i].y()));
    global_ego_pose.set_theta(path[i].theta());

    tmp_state.global_polygon = av_polygon;
    tmp_state.global_center_pose = global_ego_pose;
    CvtGlobalPoseToSlCenterPose(&sl_center_pose, &global_ego_pose, is_debug);
    tmp_state.sl_center_pose = sl_center_pose;

    auto frenet_box = drive_passage->QueryFrenetBoxAtContour(av_polygon, false);

    if (frenet_box.ok()) {
      tmp_state.frenet_box = frenet_box.value();
    }

    tmp_state.sl_vel.s_dt =
        tmp_state.velocity * std::cos(tmp_state.sl_center_pose.theta());
    tmp_state.sl_vel.l_dt =
        tmp_state.velocity * std::sin(tmp_state.sl_center_pose.theta());
    if (is_debug) {
      tmp_state.print();
      printf("\n");
    }
    state_.emplace_back(tmp_state);
    last_s = path[i].s();
  }

  is_valid_ = true;
}

void InteractiveAgent::TransformGlobalToSlCenterPose(
    SecondOrderTrajectoryPoint *sl_center_pose,
    const SecondOrderTrajectoryPoint &global_center_pose,
    const DrivePassage *drive_passage, const bool is_debug) {
  if (nullptr == sl_center_pose) return;
  sl_center_pose->set_pos(Vec2d(0.0, 0.0));
  sl_center_pose->set_theta(0.0);

  if (nullptr == drive_passage) return;

  int cur_station_index =
      drive_passage->FindNearestStationIndex(global_center_pose.pos()).value();

  const Station &cur_station =
      drive_passage->station(StationIndex(cur_station_index));

  double theta = cur_station.tangent().FastAngle();
  SecondOrderTrajectoryPoint target_pose;

  target_pose.set_pos(cur_station.xy());
  target_pose.set_theta(theta);
  if (is_debug) {
    std::cout << "base_global_x: " << target_pose.pos().x()
              << " base_global_y: " << target_pose.pos().y()
              << " base_global_theta: " << target_pose.theta() * 180 / M_PI
              << std::endl;
    std::cout << " obj_global_x: " << global_center_pose.pos().x()
              << " obj_global_y: " << global_center_pose.pos().y()
              << " obj_global_theta: "
              << global_center_pose.theta() * 180 / M_PI << std::endl;
  }
  CvtPoseGlobalToLocal(sl_center_pose, &global_center_pose, &target_pose);
  sl_center_pose->set_pos(
      Vec2d(sl_center_pose->pos().x() + cur_station.accumulated_s(),
            sl_center_pose->pos().y()));
  if (is_debug) {
    std::cout << " local_pose_x: " << sl_center_pose->pos().x()
              << " local_pose_y: " << sl_center_pose->pos().y()
              << " local_pose_theta: " << sl_center_pose->theta() * 180 / M_PI
              << std::endl;
  }
}

void InteractiveAgent::SetStBoundaryWithDecision(
    StBoundaryWithDecision *st_boundary_decision) {
  st_boundary_decision_ = st_boundary_decision;
}

void InteractiveAgent::CvtGlobalPoseToSlCenterPose(
    SecondOrderTrajectoryPoint *sl_center_pose,
    const SecondOrderTrajectoryPoint *global_center_pose, const bool is_debug) {
  if (nullptr == sl_center_pose || nullptr == global_center_pose ||
      nullptr == this->drive_passage())
    return;

  SecondOrderTrajectoryPoint local_center_pose;
  TransformGlobalToSlCenterPose(&local_center_pose, *global_center_pose,
                                this->drive_passage(), is_debug);

  auto frenet_pos =
      this->drive_passage()->QueryFrenetCoordinateAt(global_center_pose->pos());

  if (frenet_pos.ok()) {
    sl_center_pose->set_pos(Vec2d(frenet_pos.value().s, frenet_pos.value().l));
    sl_center_pose->set_theta(local_center_pose.theta());
  } else {
    *sl_center_pose = local_center_pose;
  }

  return;
}

void InteractiveAgent::SetAgentInitState(
    const SpacetimeObjectTrajectory *obstacle,
    const DrivePassage *drive_passage,
    StBoundaryWithDecision *st_boundary_decision, const bool ego_turn_left,
    const bool ego_turn_right, const bool ego_uturn, const bool is_debug) {
  if (nullptr == obstacle || nullptr == drive_passage ||
      nullptr == st_boundary_decision)
    return;

  is_valid_ = true;
  this->SetAgentID(std::string(obstacle->object_id()));
  this->SetMovAgentParams(obstacle);
  this->SetAgentType(INTERACTIVE_AGENT_MOV_AGENT);
  this->SetInitInfo(obstacle->pose().v(), obstacle->planner_object().pose().a(),
                    obstacle->pose().theta(), obstacle->contour(),
                    obstacle->pose().pos());
  this->SetStBoundaryWithDecision(st_boundary_decision);
  this->SetDrivePassage(drive_passage);
  this->UpdateGameTheoryScenario(ego_turn_left, ego_turn_right, ego_uturn);
  this->UpdateCollisionInfo();
  /* update agent states for find state */
  double last_t = 0.0;
  for (int i = 0; i < obstacle->states().size(); i++) {
    if (nullptr == obstacle->states()[i].traj_point) continue;
    if (0 != i) {
      if ((obstacle->states()[i].traj_point->t() - last_t) < 0.5) continue;
    }
    State tmp_state;
    tmp_state.is_rectangular = true;
    tmp_state.traj_idx = i;
    tmp_state.dist = obstacle->states()[i].traj_point->s();
    tmp_state.time_stamp = obstacle->states()[i].traj_point->t();
    tmp_state.velocity = obstacle->states()[i].traj_point->v();
    tmp_state.acc = obstacle->states()[i].traj_point->a();
    tmp_state.heading = obstacle->states()[i].traj_point->theta();

    SecondOrderTrajectoryPoint global_obj_pose, sl_center_pose;
    global_obj_pose.set_pos(obstacle->states()[i].traj_point->pos());
    global_obj_pose.set_theta(obstacle->states()[i].traj_point->theta());
    tmp_state.global_polygon = obstacle->states()[i].contour;
    tmp_state.global_center_pose = global_obj_pose;

    CvtGlobalPoseToSlCenterPose(&sl_center_pose, &global_obj_pose, is_debug);
    tmp_state.sl_center_pose = sl_center_pose;

    auto frenet_box = drive_passage->QueryFrenetBoxAtContour(
        obstacle->states()[i].contour, false);
    if (frenet_box.ok()) {
      tmp_state.frenet_box = frenet_box.value();
    }
    tmp_state.sl_vel.s_dt =
        tmp_state.velocity * std::cos(tmp_state.sl_center_pose.theta());
    tmp_state.sl_vel.l_dt =
        tmp_state.velocity * std::sin(tmp_state.sl_center_pose.theta());
    if (is_debug) {
      tmp_state.print();
      printf("\n");
    }
    state_.emplace_back(tmp_state);
    last_t = obstacle->states()[i].traj_point->t();
  }

  return;
}

void InteractiveAgent::SetInitInfo(const double init_vel, const double init_acc,
                                   const double init_theta,
                                   const Polygon2d init_polygon,
                                   const Vec2d pos) {
  init_info_.init_polygon_ = init_polygon;
  init_info_.init_pose_.set_pos(pos);
  init_info_.init_pose_.set_v(init_vel);
  init_info_.init_pose_.set_a(init_acc);
  init_info_.init_pose_.set_theta(init_theta);
}

void InteractiveAgent::GetLowerUpperIdxAccordingToDist(int *lower_idx,
                                                       int *upper_idx,
                                                       const double dist,
                                                       const int start_idx) {
  if (nullptr == lower_idx || nullptr == upper_idx) return;
  *lower_idx = -1;
  *upper_idx = -1;

  if (0 == state_.size()) return;

  if (dist <= state_[0].dist) {
    *lower_idx = 0;
    *upper_idx = 0;
  } else if (dist >= state_[state_.size() - 1].dist) {
    *lower_idx = state_.size() - 1;
    *upper_idx = state_.size() - 1;
  } else {
    for (int i = start_idx; i < state_.size(); i++) {
      if (state_[i].dist > dist) {
        *upper_idx = i;
        *lower_idx = std::max(i - 1, 0);
        break;
      }
    }
  }

  return;
}

void InteractiveAgent::GetEgoPolygonAccordingToDist(
    Polygon2d *ego_polygon, SecondOrderTrajectoryPoint *global_center_pose,
    int *traj_point_idx, const double dist, int start_idx) {
  if (nullptr == ego_polygon || nullptr == global_center_pose) return;

  int lower_idx, upper_idx;

  GetLowerUpperIdxAccordingToDist(&lower_idx, &upper_idx, dist, start_idx);

  if (lower_idx < 0 || lower_idx >= state_.size() || upper_idx < 0 ||
      upper_idx >= state_.size()) {
    return;
  }

  SecondOrderTrajectoryPoint *low_point, *upper_point;
  low_point = &state_[lower_idx].global_center_pose;
  upper_point = &state_[upper_idx].global_center_pose;
  double lerp_factor =
      LerpFactor(state_[lower_idx].dist, state_[upper_idx].dist, dist);
  double temp_x =
      Lerp(low_point->pos().x(), upper_point->pos().x(), lerp_factor);
  double temp_y =
      Lerp(low_point->pos().y(), upper_point->pos().y(), lerp_factor);
  double temp_theta =
      Lerp(low_point->theta(), upper_point->theta(), lerp_factor);

  SecondOrderTrajectoryPoint target_point;
  target_point.set_pos(Vec2d(temp_x, temp_y));
  target_point.set_theta(temp_theta);

  *global_center_pose = target_point;
  *traj_point_idx = lower_idx;

  const double theta = target_point.theta();
  const Vec2d rac(target_point.pos().x(), target_point.pos().y());
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
  const double half_length = param()->length() / 2.0;
  const double half_width = param()->width() / 2.0;

  const double rac_to_center = half_length - param()->back_edge_to_center();
  const Vec2d center = rac + tangent * rac_to_center;
  VehicleBoxShape target_ego_shape =
      VehicleBoxShape(param()->vehicle_geom(), rac, center, tangent, theta,
                      half_length, half_width);

  *ego_polygon =
      Polygon2d(target_ego_shape.GetCornersWithBufferCounterClockwise(
                    /*lat_buffer=*/0.2, /*lon_buffer=*/0.2),
                /*is_convex=*/true);
}

void InteractiveAgent::GetEgoStateInfoAccordingToDist(
    FrenetBox *frenet_box, Polygon2d *ego_polygon,
    SecondOrderTrajectoryPoint *global_center_pose, int *traj_point_idx,
    const double dist, int start_idx) {
  if (nullptr == frenet_box || nullptr == global_center_pose ||
      nullptr == traj_point_idx)
    return;

  GetEgoPolygonAccordingToDist(ego_polygon, global_center_pose, traj_point_idx,
                               dist, start_idx);

  if (nullptr == drive_passage() || nullptr == ego_polygon) return;

  auto tmp_frenet_box =
      drive_passage()->QueryFrenetBoxAtContour(*ego_polygon, false);
  if (tmp_frenet_box.ok()) {
    *frenet_box = tmp_frenet_box.value();
  }

  return;
}

void InteractiveAgent::GetAgentPolygonAccordingToDist(
    Polygon2d *obj_polygon, SecondOrderTrajectoryPoint *global_center_pose,
    int *traj_point_idx, const double dist, /* default 0 */ int start_idx) {
  if (nullptr == obj_polygon) return;

  int lower_idx, upper_idx;

  GetLowerUpperIdxAccordingToDist(&lower_idx, &upper_idx, dist, start_idx);

  if (lower_idx < 0 || lower_idx >= state_.size() || upper_idx < 0 ||
      upper_idx >= state_.size()) {
    return;
  }

  SecondOrderTrajectoryPoint *low_point, *upper_point;
  low_point = &state_[lower_idx].global_center_pose;
  upper_point = &state_[upper_idx].global_center_pose;
  double lerp_factor =
      LerpFactor(state_[lower_idx].dist, state_[upper_idx].dist, dist);
  double temp_x =
      Lerp(low_point->pos().x(), upper_point->pos().x(), lerp_factor);
  double temp_y =
      Lerp(low_point->pos().y(), upper_point->pos().y(), lerp_factor);
  double temp_theta =
      Lerp(low_point->theta(), upper_point->theta(), lerp_factor);

  SecondOrderTrajectoryPoint target_point;
  target_point.set_pos(Vec2d(temp_x, temp_y));
  target_point.set_theta(temp_theta);
  *global_center_pose = target_point;
  *traj_point_idx = lower_idx;

  Polygon2d lowe_point_global_polygon = state_[lower_idx].global_polygon;

  const Vec2d rotation =
      Vec2d::FastUnitFromAngle(target_point.theta() - low_point->theta());
  *obj_polygon = lowe_point_global_polygon.Transform(
      low_point->pos(), rotation.x(), rotation.y(),
      target_point.pos() - low_point->pos());

  return;
}

void InteractiveAgent::GetAgentStateInfoAccordingToDist(
    FrenetBox *frenet_box, Polygon2d *obj_polygon,
    SecondOrderTrajectoryPoint *global_center_pose, int *traj_point_idx,
    const double dist, int start_idx) {
  if (nullptr == frenet_box) return;

  GetAgentPolygonAccordingToDist(obj_polygon, global_center_pose,
                                 traj_point_idx, dist, start_idx);

  if (nullptr == drive_passage()) return;

  auto tmp_frenet_box =
      drive_passage()->QueryFrenetBoxAtContour(*obj_polygon, false);
  if (tmp_frenet_box.ok()) {
    *frenet_box = tmp_frenet_box.value();
  }
}

void InteractiveAgent::UpdateGameTheoryScenario(const bool ego_turn_left,
                                                const bool ego_turn_right,
                                                const bool ego_uturn) {
  /* todo: analysis scenario */

  bool obj_turn_left, obj_turn_right, obj_uturn;

  game_theory_scenario_t game_theory_scenario =
      GAME_THEORY_SCENARIO_CROSS_STR_STR;

  auto relationship = this->st_boundary_decision()
                          ->st_boundary()
                          ->obj_scenario_info()
                          .relationship;

  /* update obj_turn info */

  const auto obj_start_point = this->obstacle_ptr()->states().front();
  const auto obj_end_point = this->obstacle_ptr()->states().back();

  SecondOrderTrajectoryPoint global_start, global_end;
  global_start.set_pos(obj_start_point.traj_point->pos());
  global_start.set_theta(obj_start_point.traj_point->theta());

  global_start.set_pos(obj_end_point.traj_point->pos());
  global_start.set_theta(obj_end_point.traj_point->theta());

  JudgeTurnInfo(&obj_turn_left, &obj_turn_right, &obj_uturn, &global_start,
                &global_end);

  bool ego_go_straight = (!ego_turn_left) && (!ego_turn_right) && (!ego_uturn);
  bool obj_go_straight = (!obj_turn_left) && (!obj_turn_right) && (!obj_uturn);

  if (Relationship::Cross == relationship) {
    if (obj_turn_left && ego_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_STR_TL;
    } else if (ego_turn_left && obj_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_TL_STR;
    } else if (ego_uturn) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_UTURN_STR;
    } else if (obj_uturn) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_STR_UTURN;
    } else if (obj_turn_right && ego_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_STR_TR;
    } else if (ego_turn_right && obj_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_TR_STR;
    } else if (ego_turn_left && obj_turn_left) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_TL_TL;
    } else if (ego_turn_left && obj_turn_right) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_STR_STR;
    } else if (ego_turn_right && obj_turn_left) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_STR_STR;
    } else if (ego_turn_right && obj_turn_right) {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_TR_TR;
    } else {
      game_theory_scenario = GAME_THEORY_SCENARIO_CROSS_STR_STR;
    }
  } else if (Relationship::Merge == relationship) {
    if (ego_turn_right && obj_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_TR_STR;
    } else if (obj_turn_right && ego_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_STR_TR;
    } else if (ego_turn_left && obj_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_TL_STR;
    } else if (obj_turn_left && ego_go_straight) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_STR_TL;
    } else if (ego_turn_right && obj_turn_right) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_TR_TR;
    } else if (ego_turn_left && obj_turn_left) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_TL_TL;
    } else if (ego_turn_left && obj_turn_right) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_PARALLEL;
    } else if (ego_turn_right && obj_turn_left) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_PARALLEL;
    } else if (ego_uturn) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_UTURN_STR;
    } else if (obj_uturn) {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_STR_UTURN;
    } else {
      game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_PARALLEL;
    }
  } else if (Relationship::OnComing == relationship) {
    game_theory_scenario = GAME_THEORY_SCENARIO_ONCOMMING;
  } else if (Relationship::SameDir == relationship) {
    game_theory_scenario = GAME_THEORY_SCENARIO_MERGE_PARALLEL;
  }

  this->SetGameTheoryScenario(game_theory_scenario);
}

void InteractiveAgent::UpdateCollisionInfo() {
  init_info_.collision_info.ego_dist_to_enter_collision =
      std::numeric_limits<double>::max();
  init_info_.collision_info.ego_dist_to_leave_collision =
      -std::numeric_limits<double>::max();
  init_info_.collision_info.obj_dist_to_enter_collision =
      std::numeric_limits<double>::max();
  init_info_.collision_info.obj_dist_to_leave_collision =
      -std::numeric_limits<double>::max();
  if (nullptr == this->st_boundary_decision_) return;
  if (nullptr == this->obstacle_) return;
  if (nullptr == this->st_boundary_decision_->st_boundary()) return;

  auto overlap_infos =
      this->st_boundary_decision_->st_boundary()->overlap_infos();

  for (const auto &tmp_overlap_info : overlap_infos) {
    CHECK_GT(obstacle_->states().size(), tmp_overlap_info.obj_idx);
    const auto *overlap_agent_traj_point =
        obstacle_->states()[tmp_overlap_info.obj_idx].traj_point;
    init_info_.collision_info.obj_dist_to_enter_collision =
        std::min(overlap_agent_traj_point->s(),
                 init_info_.collision_info.obj_dist_to_enter_collision);
    init_info_.collision_info.obj_dist_to_leave_collision =
        std::max(overlap_agent_traj_point->s(),
                 init_info_.collision_info.obj_dist_to_leave_collision);
  }
  init_info_.collision_info.ego_dist_to_enter_collision =
      this->st_boundary_decision_->st_boundary()->min_s();
  init_info_.collision_info.ego_dist_to_leave_collision =
      this->st_boundary_decision_->st_boundary()->max_s();
}

void InteractiveAgent::SetMovAgentParams(
    const SpacetimeObjectTrajectory *st_traj) {
  if (nullptr == st_traj) return;

  obstacle_ = st_traj;

  ObjectType object_type = st_traj->object_type();
  param_.SetMaxAcc(object_type);
  param_.SetMinAcc(object_type);
  param_.SetInterMaxAcc(object_type);
  param_.SetInterMinAcc(object_type);
  param_.SetMaxVel(object_type);
  param_.SetMinSafeDist(object_type);
  param_.SetLength(5.0);
  param_.SetWidth(2.0);
  param_.SetBackEdgeToCenter(2.5);
  if (st_boundary_decision_ != nullptr &&
      st_boundary_decision_->raw_st_boundary() != nullptr) {
    const auto &obj_sl_info =
        st_boundary_decision_->raw_st_boundary()->obj_sl_info();
    double obj_length = 0.0;
    double obj_width = 0.0;
    if (obj_sl_info.has_value()) {
      const auto &frenet_polygon = obj_sl_info->frenet_polygon;
      obj_length = std::fabs(frenet_polygon.s_max - frenet_polygon.s_min);
      obj_width = std::fabs(frenet_polygon.l_max - frenet_polygon.l_min);
    } else {
      obj_length = st_traj->bounding_box().length();
      obj_width = st_traj->bounding_box().width();
    }
    param_.SetLength(obj_length);
    param_.SetWidth(obj_width);
    param_.wheel_base_ = param_.length_ / 2.0;
    param_.back_edge_to_center_ = param_.length_ / 2.0;
  }
}

void InteractiveAgent::SetEgoParams(
    const VehicleGeometryParamsProto &vehicle_geom) {
  AgentParam ego_params =
      AgentParam(vehicle_geom.length(), vehicle_geom.width(),
                 vehicle_geom.height(), -4.0, 2.0, -4.0, 2.0, 0.0, Kph2Mps(80),
                 0.0, vehicle_geom.back_edge_to_center(), vehicle_geom);
  param_ = ego_params;
}

void AgentParam::ResetAgentParam() {
  length_ = 0.0;
  width_ = 0.0;
  height_ = 0.0;
  wheel_base_ = 0.0;
  min_acc_ = 0.0;
  max_acc_ = 0.0;
  inter_min_acc_ = 0.0;
  inter_max_acc_ = 0.0;
  min_vel_ = 0.0;
  max_vel_ = 0.0;
  min_safe_dist_ = 0.0;
  back_edge_to_center_ = 0.0;
};

AgentParam::AgentParam() { ResetAgentParam(); }

AgentParam::AgentParam(double length, double width, double height,
                       double min_acc, double max_acc, double inter_min_acc,
                       double inter_max_acc, double min_vel, double max_vel,
                       double min_safe_dist, double back_edge_to_center,
                       const VehicleGeometryParamsProto vehicle_geom) {
  length_ = length;
  width_ = width;
  height_ = height;
  min_acc_ = min_acc;
  max_acc_ = max_acc;
  inter_min_acc_ = inter_min_acc;
  inter_max_acc_ = inter_max_acc;
  min_vel_ = min_vel;
  max_vel_ = max_vel;
  wheel_base_ = 0.5 * length;
  min_safe_dist_ = min_safe_dist;
  back_edge_to_center_ = back_edge_to_center;
  vehicle_geom_ = vehicle_geom;
}

AgentParam::AgentParam(const AgentParam &param) {
  length_ = param.length_;
  width_ = param.width_;
  height_ = param.height_;
  wheel_base_ = param.wheel_base_;
  min_acc_ = param.min_acc_;
  max_acc_ = param.max_acc_;
  inter_min_acc_ = param.inter_min_acc_;
  inter_max_acc_ = param.inter_max_acc_;
  min_vel_ = param.min_vel_;
  max_vel_ = param.max_vel_;
  min_safe_dist_ = param.min_safe_dist_;
  back_edge_to_center_ = param.back_edge_to_center_;
  vehicle_geom_ = param.vehicle_geom_;
}

AgentParam &AgentParam::operator=(const AgentParam &param) {
  if (this != &param) {
    length_ = param.length_;
    width_ = param.width_;
    height_ = param.height_;
    wheel_base_ = param.wheel_base_;
    min_acc_ = param.min_acc_;
    max_acc_ = param.max_acc_;
    inter_min_acc_ = param.inter_min_acc_;
    inter_max_acc_ = param.inter_max_acc_;
    min_vel_ = param.min_vel_;
    max_vel_ = param.max_vel_;
    min_safe_dist_ = param.min_safe_dist_;
    back_edge_to_center_ = param.back_edge_to_center_;
    vehicle_geom_ = param.vehicle_geom_;
  }
  return *this;
}

void AgentParam::SetMaxAcc(const ObjectType kind) {
  switch (kind) {
    case ObjectType::OT_VEHICLE:
      max_acc_ = 3.0;
      break;
    case ObjectType::OT_MOTORCYCLIST:
      max_acc_ = 2.0;
      break;
    case ObjectType::OT_PEDESTRIAN:
      max_acc_ = 1.0;
      break;
    case ObjectType::OT_CYCLIST:
      max_acc_ = 1.0;
      break;
    case ObjectType::OT_TRICYCLIST:
      max_acc_ = 1.0;
      break;
    case ObjectType::OT_LARGE_VEHICLE:
      max_acc_ = 2.0;
      break;
    default:
      max_acc_ = 2.0;
      break;
  }
}

void AgentParam::SetMinAcc(const ObjectType kind) {
  switch (kind) {
    case ObjectType::OT_VEHICLE:
      min_acc_ = -3.5;
      break;
    case ObjectType::OT_MOTORCYCLIST:
      min_acc_ = -2.0;
      break;
    case ObjectType::OT_PEDESTRIAN:
      min_acc_ = -2.0;
      break;
    case ObjectType::OT_CYCLIST:
      min_acc_ = -2.0;
      break;
    case ObjectType::OT_TRICYCLIST:
      min_acc_ = -2.0;
      break;
    case ObjectType::OT_LARGE_VEHICLE:
      min_acc_ = -3.0;
      break;
    default:
      min_acc_ = -2.0;
      break;
  }
}

void AgentParam::SetInterMaxAcc(const ObjectType kind) {
  switch (kind) {
    case ObjectType::OT_VEHICLE:
      inter_max_acc_ = 2.0;
      break;
    case ObjectType::OT_MOTORCYCLIST:
      inter_max_acc_ = 1.5;
      break;
    case ObjectType::OT_PEDESTRIAN:
      inter_max_acc_ = 1.0;
      break;
    case ObjectType::OT_TRICYCLIST:
      inter_max_acc_ = 1.0;
      break;
    case ObjectType::OT_CYCLIST:
      inter_max_acc_ = 1.0;
      break;
    case ObjectType::OT_LARGE_VEHICLE:
      inter_max_acc_ = 2.0;
      break;
    default:
      inter_max_acc_ = 2.0;
      break;
  }
}

void AgentParam::SetInterMinAcc(const ObjectType kind) {
  switch (kind) {
    case ObjectType::OT_VEHICLE:
      inter_min_acc_ = -2.0;
      break;
    case ObjectType::OT_MOTORCYCLIST:
      inter_min_acc_ = -1.5;
      break;
    case ObjectType::OT_PEDESTRIAN:
      inter_min_acc_ = -1.0;
      break;
    case ObjectType::OT_TRICYCLIST:
      inter_min_acc_ = -1.0;
      break;
    case ObjectType::OT_CYCLIST:
      inter_min_acc_ = -1.0;
      break;
    case ObjectType::OT_LARGE_VEHICLE:
      inter_min_acc_ = -1.5;
      break;
    default:
      inter_min_acc_ = -2.0;
      break;
  }
}

void AgentParam::SetMaxVel(const ObjectType kind) {
  switch (kind) {
    case ObjectType::OT_VEHICLE:
      max_vel_ = Kph2Mps(80.0);
      break;
    case ObjectType::OT_MOTORCYCLIST:
      max_vel_ = Kph2Mps(50.0);
      break;
    case ObjectType::OT_PEDESTRIAN:
      max_vel_ = Kph2Mps(50.0);
      break;
    case ObjectType::OT_TRICYCLIST:
      max_vel_ = Kph2Mps(6.0);
      break;
    case ObjectType::OT_CYCLIST:
      max_vel_ = Kph2Mps(20.0);
      break;
    case ObjectType::OT_LARGE_VEHICLE:
      max_vel_ = Kph2Mps(80.0);
      break;
    default:
      max_vel_ = Kph2Mps(80.0);
      break;
  }
}

void AgentParam::SetMinSafeDist(const ObjectType kind) {
  switch (kind) {
    case ObjectType::OT_VEHICLE:
      min_safe_dist_ = Kph2Mps(3.0);
      break;
    case ObjectType::OT_MOTORCYCLIST:
      min_safe_dist_ = Kph2Mps(2.0);
      break;
    case ObjectType::OT_PEDESTRIAN:
      min_safe_dist_ = Kph2Mps(2.5);
      break;
    case ObjectType::OT_TRICYCLIST:
      min_safe_dist_ = Kph2Mps(2.0);
      break;
    case ObjectType::OT_CYCLIST:
      min_safe_dist_ = Kph2Mps(2.0);
      break;
    case ObjectType::OT_LARGE_VEHICLE:
      min_safe_dist_ = Kph2Mps(5.0);
      break;
    default:
      min_safe_dist_ = Kph2Mps(2.0);
      break;
  }
}

}  // namespace planning
}  // namespace st

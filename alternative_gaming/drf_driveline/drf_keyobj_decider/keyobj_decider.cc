#include "keyobj_decider.h"

namespace st::planning {

constexpr double kMinimumSpeed = 0.1;
constexpr double kStartAccThresh = 0.5;
constexpr double kEps = 1e-6;
constexpr double kEgoLength = 4.8;
constexpr double kEgoWidth = 2.0;
constexpr double kObjAggressiveStartAcc = 2.0;
constexpr double kQuadraticCoeff = 0.1;
constexpr double kCubicCoeff = 0.01;
constexpr float kOncomingMinYaw = 5 * M_PI / 6.0f;  //[rad]
constexpr float kOncomingMaxYaw = 7 * M_PI / 6.0f;  //[rad]
constexpr double kOncomingCosHeadingThresh = -0.6;
constexpr double kStaticOppoCosHeadingThresh = -0.6;
constexpr float kMaxHwt = 1000.0;
constexpr float kInRightThresh = 0.5;
constexpr float kInLeftThresh = 1.5;
constexpr float kRightTurnKeyObjCosThresh = 0.7;
constexpr float kCrossCosThresh = 0.5;
constexpr float kLeftTurnLeftSideLatPoseThresh = 4.0;
constexpr float kCrossHeadingDiff = M_PI / 6.0;

// TODO(whj): unprotected left turn
void RiskFieldKeyobjDecider::GetOppoKeyobjs(
    const SpacetimeTrajectoryManager* st_mgr, const DiscretizedPath* init_path,
    double ego_v, std::vector<RiskFieldKeyobj>* const oppo_straight_keyvehicles,
    std::vector<RiskFieldKeyobj>* const oppo_straight_keycyclists) {
  auto& av_path = *init_path;
  if (oppo_straight_keyvehicles == nullptr ||
      oppo_straight_keycyclists == nullptr) {
    return;
  }
  const auto& st_trajs = st_mgr->trajectories();
  oppo_straight_keyvehicles->clear();
  oppo_straight_keycyclists->clear();
  oppo_straight_keyvehicles->reserve(st_trajs.size());
  oppo_straight_keycyclists->reserve(st_trajs.size());

  for (const auto& st_traj : st_trajs) {
    if ((IsVehicle(st_traj) || IsCyclist(st_traj) || IsPedestrian(st_traj)) &&
        IsOppoGoStraight(st_traj, av_path) &&
        IsRightOfAvPath(st_traj, av_path)) {
      if (st_traj.states().empty()) {
        continue;
      }
      float obj_s = 0.0;
      float obj_l = 0.0;
      auto obj_frenet =
          av_path.XYToSL(Vec2d(st_traj.trajectory().points().back().pos().x(),
                               st_traj.trajectory().points().back().pos().y()));
      obj_s = obj_frenet.s;
      obj_l = obj_frenet.l;
      if (obj_l < 0.0) {
        continue;
      }
      RiskFieldKeyobj decision_keyobj;
      decision_keyobj.id = std::string(st_traj.object_id());
      decision_keyobj.traj_id = st_traj.traj_id();
      decision_keyobj.speed = st_traj.trajectory().points().front().v();
      decision_keyobj.object_ptr = &st_traj;
      if (IsLargeVehicle(st_traj)) {
        decision_keyobj.obj_type = ObjType::LARGE_VEHICLE;
      } else if (IsCyclist(st_traj) || IsPedestrian(st_traj)) {
        decision_keyobj.obj_type = ObjType::CYCLIST;
      } else {
        decision_keyobj.obj_type = ObjType::NORMAL_VEHICLE;
      }
      if (!IsObjStatic(st_traj)) {
        decision_keyobj.collision_area =
            CalcCollisionAreaByPred(st_traj, av_path);
        if (decision_keyobj.collision_area.has_value()) {
          decision_keyobj.agent_hwt = CalcAgentHwtByCollisionArea(
              st_traj, *decision_keyobj.collision_area);
          decision_keyobj.ego_hwt =
              CalcEgoHwtByCollisionArea(ego_v, *decision_keyobj.collision_area);
          decision_keyobj.collision_dist =
              decision_keyobj.collision_area->obj_yield_point.s();
        } else {
          continue;
        }
      }
      if (decision_keyobj.agent_hwt == std::nullopt) {
        continue;
      }
      decision_keyobj.obj_behavior = ObjBehavior::OPPO_GO_STRAIGHT;
      if (decision_keyobj.obj_type != ObjType::CYCLIST) {
        oppo_straight_keyvehicles->emplace_back(decision_keyobj);
      } else {
        oppo_straight_keycyclists->emplace_back(decision_keyobj);
      }
    }
  }
  // CalculateExactVehicleHwt(oppo_straight_keyvehicles);
  oppo_straight_keyvehicles->shrink_to_fit();
  oppo_straight_keycyclists->shrink_to_fit();
  return;
}

void RiskFieldKeyobjDecider::GetLeftTurnOutsideOncommingKeyobjs(
    const SpacetimeTrajectoryManager* st_mgr, const DiscretizedPath* init_path,
    double ego_v, std::vector<RiskFieldKeyobj>* const outside_risk_keyobjs) {
  //筛选预测轨迹处于外侧并且ttc小于2的对向车
  if (st_mgr == nullptr) {
    return;
  }
  auto& av_path = *init_path;
  const double cos_oncomming_thresh = -0.85;
  const double oncomming_ttc = 2.0;
  const double oncomming_risk_lateral_dist_thresh = 4.0;
  const auto& st_trajs = st_mgr->trajectories();

  for (const auto& st_traj : st_trajs) {
    double init_yaw_diff =
        NormalizeAngle(st_traj.trajectory().points().front().theta() -
                       init_path->front().theta());
    if (IsPedestrian(st_traj)) {
      continue;
    }
    if (st_traj.states().empty()) {
      continue;
    }
    if (std::cos(NormalizeAngle(init_path->front().theta() -
                                st_traj.trajectory().points().front().theta()) >
                 cos_oncomming_thresh))  //只计算对象来车
    {
      continue;
    }
    float obj_start_s = 0.0;
    float obj_start_l = 0.0;
    float obj_end_s = 0.0;
    float obj_end_l = 0.0;
    float ego_start_s = 0.0;
    float ego_start_l = 0.0;
    auto obj_start_point =
        Vec2d(st_traj.trajectory().points().front().pos().x(),
              st_traj.trajectory().points().front().pos().y());
    auto obj_end_point = Vec2d(st_traj.trajectory().points().back().pos().x(),
                               st_traj.trajectory().points().back().pos().y());
    obj_start_s = av_path.XYToSL(obj_start_point).s;
    obj_start_l = av_path.XYToSL(obj_start_point).l;
    obj_end_s = av_path.XYToSL(obj_end_point).s;
    obj_end_l = av_path.XYToSL(obj_end_point).l;
    if (obj_start_s < ego_start_s || obj_start_l > 0.0 || obj_end_l > 0.0 ||
        std::fabs(obj_start_l) > oncomming_risk_lateral_dist_thresh) {
      continue;
    }
    double oncomming_speed = ego_v + st_traj.trajectory().points().front().v();
    if (obj_start_s / oncomming_speed < oncomming_ttc) {
      RiskFieldKeyobj decision_keyobj;
      decision_keyobj.id = std::string(st_traj.object_id());
      decision_keyobj.traj_id = st_traj.traj_id();
      decision_keyobj.speed = st_traj.trajectory().points().front().v();
      decision_keyobj.object_ptr = &st_traj;
      outside_risk_keyobjs->emplace_back(std::move(decision_keyobj));
    }
  }
  return;
}
// TODO(whj): unprotected left turn
// void RiskFieldKeyobjDecider::GetOppoStaticKeyobjs(
//     const SpacetimeTrajectoryManager* st_mgr, const DiscretizedPath*
//     init_path, double ego_v, std::vector<RiskFieldKeyobj>* const
//     oppo_static_keyvehicles, std::vector<RiskFieldKeyobj>* const
//     oppo_static_keycyclists)
// {
//     if (oppo_static_keyvehicles == nullptr || oppo_static_keycyclists ==
//     nullptr)
//     {
//         return;
//     }
//     auto &av_path = *init_path;
//     const auto &st_trajs = st_mgr->trajectories();
//     oppo_static_keyvehicles->clear();
//     oppo_static_keycyclists->clear();
//     oppo_static_keyvehicles->reserve(st_trajs.size());
//     oppo_static_keycyclists->reserve(st_trajs.size());

//     for (const auto& st_traj : st_trajs)
//     {
//         if ((IsVehicle(st_traj) || IsCyclist(st_traj) ||
//         IsPedestrian(st_traj))
//             && IsOppoStaticLeftAvPath(st_traj, av_path))
//         {
//             RiskFieldKeyobj decision_keyobj;
//             decision_keyobj.id = std::string(st_traj.object_id());
//             decision_keyobj.traj_id = st_traj.traj_id();
//             decision_keyobj.speed = st_traj.planner_object().pose().v();
//             decision_keyobj.object_ptr = &st_traj;
//             if (IsLargeVehicle(st_traj))
//             {
//                 decision_keyobj.obj_type = ObjType::LARGE_VEHICLE;
//             }
//             else if (IsCyclist(st_traj) || IsPedestrian(st_traj))
//             {
//                 decision_keyobj.obj_type = ObjType::CYCLIST;
//             }
//             else
//             {
//                 decision_keyobj.obj_type = ObjType::NORMAL_VEHICLE;
//             }

//             decision_keyobj.collision_area = CalcCollisionAreaByPred(st_traj,
//             av_path);//TODO by zx:对于静态障碍物，碰撞点计算要改 if
//             (decision_keyobj.collision_area.has_value())
//             {
//                 decision_keyobj.agent_hwt =
//                     CalcAgentHwtByCollisionArea(st_traj,
//                     *decision_keyobj.collision_area);
//                 decision_keyobj.ego_hwt =
//                     CalcEgoHwtByCollisionArea(ego_v,
//                     *decision_keyobj.collision_area);
//                 decision_keyobj.collision_dist =
//                 decision_keyobj.collision_area->obj_yield_point.s();
//             }
//             else
//             {
//                 continue;
//             }

//             if (decision_keyobj.agent_hwt == std::nullopt)
//             {
//                 continue;
//             }
//             decision_keyobj.obj_behavior = ObjBehavior::OPPO_GO_STRAIGHT;
//             if (decision_keyobj.obj_type != ObjType::CYCLIST)
//             {
//                 oppo_static_keyvehicles->emplace_back(std::move(decision_keyobj));
//             }
//             else
//             {
//                 oppo_static_keycyclists->emplace_back(std::move(decision_keyobj));
//             }
//         }
//     }

//     oppo_static_keyvehicles->shrink_to_fit();
//     oppo_static_keycyclists->shrink_to_fit();
//     return;
// }

bool RiskFieldKeyobjDecider::CheckPoseInLeft(const Vec2d& start_point,
                                             const Vec2d& end_point,
                                             const Vec2d& pose) {
  return (end_point - start_point).CrossProd(pose - start_point) > 0.0;
}

bool RiskFieldKeyobjDecider::IsObjectSameDirectWithEgo(
    const SpacetimeObjectTrajectory st_traj, const PathPoint start_point) {
  const auto pred_traj = st_traj.trajectory();
  const double obj_start_yaw = pred_traj.points().front().theta();
  const double ego_start_yaw = (start_point.theta());
  const double cos_diff_start_yaw =
      std::cos(NormalizeAngle(obj_start_yaw - ego_start_yaw));
  return cos_diff_start_yaw > kRightTurnKeyObjCosThresh;
}

bool RiskFieldKeyobjDecider::IsObjectCrossWithAvPath(
    const SpacetimeObjectTrajectory st_traj, const DiscretizedPath av_path) {
  double obj_end_yaw = st_traj.trajectory().points().back().theta();
  float obj_s = 0.0;
  float obj_l = 0.0;
  obj_s = av_path.XYToSL(st_traj.trajectory().points().front().pos()).s;
  obj_l = av_path.XYToSL(st_traj.trajectory().points().front().pos()).l;
  double query_s = obj_s > 30.0 ? obj_s : 30.0;
  auto query_point = av_path.Evaluate(query_s);
  double query_heading = query_point.theta();
  double heading_diff = NormalizeAngle(query_heading - obj_end_yaw);
  return std::fabs(heading_diff) > kCrossHeadingDiff;
}

void RiskFieldKeyobjDecider::CalSameDirectLeftSideKeyobjs(
    const SpacetimeTrajectoryManager* st_mgr, const DiscretizedPath* init_path,
    double av_half_width, std::vector<RiskFieldKeyobj>* left_side_keyvehicles,
    std::vector<RiskFieldKeyobj>* left_side_keycyclists) {
  if (left_side_keyvehicles == nullptr || left_side_keycyclists == nullptr) {
    return;
  }
  auto& av_path = *init_path;
  auto st_trajs = st_mgr->trajectories();
  left_side_keyvehicles->clear();
  left_side_keycyclists->clear();
  left_side_keyvehicles->reserve(st_trajs.size());
  left_side_keycyclists->reserve(st_trajs.size());
  auto start_point = av_path.front();
  for (const auto& st_traj : st_trajs) {
    if (!IsObjStatic(st_traj) && (IsVehicle(st_traj) || IsCyclist(st_traj)) &&
        IsTotalLeftOfAv(st_traj, av_half_width) &&
        IsObjectSameDirectWithEgo(st_traj, start_point) &&
        IsLeftOfAvPath(st_traj, av_path) &&
        IsObjectCrossWithAvPath(st_traj, av_path)) {
      if (st_traj.trajectory().points().front().pos().y() < 0.0 &&
          !IsTotallyLeftOfAvPath(st_traj, av_path, av_half_width)) {
        continue;
      }
      RiskFieldKeyobj decision_keyobj{};
      decision_keyobj.id = std::string(st_traj.object_id());
      decision_keyobj.traj_id = st_traj.traj_id();
      decision_keyobj.speed = st_traj.trajectory().points().front().v();
      decision_keyobj.object_ptr = &st_traj;
      // 随便给的2.0，内侧车不计算冲突区信息
      decision_keyobj.agent_hwt = std::make_optional<double>(2.0);
      decision_keyobj.ego_hwt = 2.0;
      if (IsVehicle(st_traj)) {
        decision_keyobj.obj_type = ObjType::LARGE_VEHICLE;
        left_side_keyvehicles->push_back(decision_keyobj);
      } else if (IsCyclist(st_traj) &&
                 st_traj.trajectory().points().front().pos().y() <
                     kLeftTurnLeftSideLatPoseThresh) {
        decision_keyobj.obj_type = ObjType::CYCLIST;
        left_side_keycyclists->push_back(decision_keyobj);
      }
    }
  }
}

void RiskFieldKeyobjDecider::GetLeftTurnKeyobjs(
    const SpacetimeTrajectoryManager* st_mgr, const DiscretizedPath* init_path,
    double ego_v, const PathPoint start_point,
    const DrivelineResultProto* last_driveline_result,
    std::vector<RiskFieldKeyobj>* syn_left_turn_keyvehicles,
    std::vector<RiskFieldKeyobj>* syn_left_turn_keycyclists) {
  if (/*oppo_straight_keyvehicles == nullptr || */ syn_left_turn_keyvehicles ==
      nullptr) {
    return;
  }
  auto st_trajs = st_mgr->trajectories();
  auto& av_path = *init_path;
  // oppo_straight_keyvehicles->clear();
  syn_left_turn_keyvehicles->clear();
  syn_left_turn_keycyclists->clear();
  syn_left_turn_keyvehicles->reserve(st_trajs.size());
  syn_left_turn_keycyclists->reserve(st_trajs.size());

  for (const auto& st_traj : st_trajs) {
    if (IsLeftTurnKeyObj(st_traj, av_path, start_point)) {
      RiskFieldKeyobj decision_keyobj;
      decision_keyobj.id = std::string(st_traj.object_id());
      decision_keyobj.traj_id = st_traj.traj_id();
      decision_keyobj.speed = st_traj.trajectory().points().front().v();
      decision_keyobj.object_ptr = &st_traj;
      bool is_last_riskies = false;
      if (last_driveline_result && last_driveline_result->has_most_risk_obj()) {
        is_last_riskies =
            (decision_keyobj.id == last_driveline_result->most_risk_obj())
                ? true
                : false;
      }
      if (IsVehicle(st_traj)) {
        decision_keyobj.obj_type = ObjType::NORMAL_VEHICLE;
      } else if (IsCyclist(st_traj)) {
        decision_keyobj.obj_type = ObjType::CYCLIST;
      } else {
        decision_keyobj.obj_type = ObjType::PEDESTRIAN;
      }
      if (!IsObjStatic(st_traj)) {
        decision_keyobj.collision_area =
            CalcCollisionAreaByPred(st_traj, av_path);
        decision_keyobj.agent_hwt = std::make_optional<double>(2.0);
        decision_keyobj.ego_hwt = 2.0;
      }
      if (decision_keyobj.agent_hwt == std::nullopt) {
        continue;
      }
      decision_keyobj.obj_behavior = ObjBehavior::SYNC_LEFT_TURN;
      if (decision_keyobj.obj_type == ObjType::NORMAL_VEHICLE ||
          decision_keyobj.obj_type == ObjType::LARGE_VEHICLE) {
        // go_staright_keycyclists->emplace_back(decision_keyobj);
        if (IsObjectSameDirectWithEgo(st_traj, start_point) &&
            IsLeftOfAvPath(st_traj, av_path) &&
            (IsFrontOfAvpath(st_traj, av_path) || is_last_riskies)) {
          syn_left_turn_keyvehicles->emplace_back(decision_keyobj);
        }
      } else {
        // oppo_straight_keyvehicles->emplace_back(std::move(decision_keyobj));
      }
    }
  }
  syn_left_turn_keyvehicles->shrink_to_fit();
  return;
}

std::optional<RiskFieldCollisionArea>
RiskFieldKeyobjDecider::CalcCollisionAreaByPred(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  if (st_traj.states().empty()) {
    return std::nullopt;
  }

  auto insert_point = GetCollisionPoint(st_traj, av_path);
  if (!insert_point.has_value()) {
    return std::nullopt;
  }

  double collision_length = GetCollisionLength(st_traj, *insert_point);

  double ego_yield_point_s =
      insert_point->s() - 1.0 * kEgoLength - collision_length;
  double ego_pass_point_s =
      insert_point->s() + 1.0 * kEgoLength + collision_length;
  const auto ego_yield_iter = std::lower_bound(
      av_path.begin(), av_path.end(), ego_yield_point_s,
      [](const auto& point, double value) { return point.s() < value; });
  const auto ego_pass_iter = std::lower_bound(
      av_path.begin(), av_path.end(), ego_pass_point_s,
      [](const auto& point, double value) { return point.s() < value; });

  const auto& pred_trajectory = st_traj.trajectory();
  const auto dis_vec =
      Vec2d(insert_point->x() - pred_trajectory.points().front().pos().x(),
            insert_point->y() - pred_trajectory.points().front().pos().y());
  double obs_collision_dist = dis_vec.norm();
  const double obj_yield_point_s =
      obs_collision_dist - st_traj.bounding_box().half_length();
  const double obj_pass_point_s =
      obs_collision_dist + st_traj.bounding_box().half_length();
  PathPoint obj_yield_point = *insert_point;
  PathPoint obj_pass_point = *insert_point;
  obj_yield_point.set_s(obj_yield_point_s);
  obj_yield_point.set_x(pred_trajectory.points().front().pos().x() +
                        obj_yield_point_s *
                            std::cos(pred_trajectory.points().front().theta()));
  obj_yield_point.set_y(pred_trajectory.points().front().pos().y() +
                        obj_yield_point_s *
                            std::sin(pred_trajectory.points().front().theta()));
  obj_pass_point.set_s(obj_pass_point_s);
  obj_pass_point.set_x(pred_trajectory.points().front().pos().x() +
                       obj_pass_point_s *
                           std::cos(pred_trajectory.points().front().theta()));
  obj_pass_point.set_y(pred_trajectory.points().front().pos().y() +
                       obj_pass_point_s *
                           std::sin(pred_trajectory.points().front().theta()));

  RiskFieldCollisionArea collision_area;
  collision_area.obj_yield_point = obj_yield_point;
  collision_area.obj_pass_point = obj_pass_point;
  collision_area.ego_yield_point =
      ego_yield_iter != av_path.end() ? *ego_yield_iter : *insert_point;
  collision_area.ego_pass_point =
      ego_pass_iter != av_path.end() ? *ego_pass_iter : *insert_point;

  return collision_area;
}

std::optional<double> RiskFieldKeyobjDecider::CalcAgentHwtByCollisionArea(
    const SpacetimeObjectTrajectory st_traj,
    const RiskFieldCollisionArea& collision_area) {
  double start_v = st_traj.trajectory().points().front().v();
  double start_a = EstObjAcc(st_traj);
  if (IsObjStart(st_traj)) {
    start_a = std::max(kObjAggressiveStartAcc, EstObjAcc(st_traj));
  }
  if (start_a < kEps) {
    return collision_area.obj_yield_point.s() / start_v;
  } else {
    return (-start_v +
            std::sqrt(start_v * start_v +
                      2 * start_a * collision_area.obj_yield_point.s())) /
           start_a;
  }
}

double RiskFieldKeyobjDecider::CalcEgoHwtByCollisionArea(
    double ego_v, const RiskFieldCollisionArea& collision_area) {
  return (std::max(collision_area.ego_yield_point.s(), 0.0)) * ego_v /
         (ego_v * ego_v + 0.1);
}
// TODO(whj): unprotected left turn
// void RiskFieldKeyobjDecider::CalculateExactVehicleHwt(
//     std::vector<RiskFieldKeyobj>* oppo_straight_keyvehicles)
// {
//     std::sort(oppo_straight_keyvehicles->begin(),
//     oppo_straight_keyvehicles->end(),
//               [](const auto& left, const auto& right) {
//                   return left.collision_dist < right.collision_dist;
//               });
//     for (int i = 1; i < oppo_straight_keyvehicles->size(); ++i)
//     {
//         double gap_dist = (*oppo_straight_keyvehicles)[i].collision_dist
//                           - (*oppo_straight_keyvehicles)[i -
//                           1].collision_dist
//                           - 0.5 * (*oppo_straight_keyvehicles)[i -
//                           1].object_ptr->bounding_box().length();
//         double hwt =
//             gap_dist
//             /
//             ((*oppo_straight_keyvehicles)[i].object_ptr->trajectory().points().front().v()
//                + 0.1);
//         (*oppo_straight_keyvehicles)[i].agent_hwt = hwt;
//     }
// }

double RiskFieldKeyobjDecider::GetCollisionLength(
    const SpacetimeObjectTrajectory& st_traj, PathPoint& insert_point) {
  const double yaw_diff = NormalizeAngle(
      insert_point.theta() - st_traj.trajectory().points().front().theta());
  double collision_length = st_traj.bounding_box().width();
  if (std::fabs(yaw_diff) > kEps) {
    collision_length = fabs(kEgoWidth / sin(yaw_diff));
  }
  return collision_length;
}

std::optional<PathPoint> RiskFieldKeyobjDecider::GetCollisionPoint(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  constexpr double kSearchBuffer = 0.2;
  constexpr double kPathApproxTolerance = 0.05;
  constexpr double kMaxLatDist = 1.5;
  const auto vehicle_rect =
      CreateOffsetRectFromVehicleGeometry(vehicle_geo_params_);
  std::vector<Vec2d> points;
  points.reserve(av_path.size());
  for (const auto point : av_path) {
    points.emplace_back(point.x(), point.y());
  }
  const auto ego_path_kd_tree = std::make_unique<SegmentMatcherKdtree>(points);
  const auto ego_path_approx = std::make_unique<PathApprox>(BuildPathApprox(
      av_path, vehicle_rect, kPathApproxTolerance, ego_path_kd_tree.get()));
  const double path_step_length = av_path[1].s() - av_path[0].s();

  const Box2d& obj_box = st_traj.bounding_box();
  const double obj_radius =
      obj_box.diagonal() * 0.5 + st_traj.required_lateral_gap();
  const double ego_radius =
      Hypot(std::max(vehicle_geo_params_.front_edge_to_center(),
                     vehicle_geo_params_.back_edge_to_center()),
            vehicle_geo_params_.right_edge_to_center());
  const double search_radius = obj_radius + ego_radius + kSearchBuffer;
  std::optional<std::pair<double, double>> first_overlap_ego_s_range;
  const auto& object_states = st_traj.states();
  size_t obj_state_len = object_states.size();
  //从前往后找第1个碰撞点
  for (size_t i = 0; i < obj_state_len; ++i) {
    const auto agent_overlaps = ComputeAgentOverlapsWithBuffer(
        *ego_path_approx, path_step_length, /*first_index=*/0,
        /*last_index=*/av_path.size() - 1, object_states[i].contour,
        /*max_lat_dist=*/kMaxLatDist, st_traj.required_lateral_gap(),
        st_traj.required_lateral_gap(), search_radius);
    first_overlap_ego_s_range = ConvertToOverlapRange(agent_overlaps);
    if (first_overlap_ego_s_range.has_value()) {
      double collision_s = first_overlap_ego_s_range.value().first;
      auto col_pt = av_path.Evaluate(collision_s);
      return col_pt;
    }
  }
  return std::nullopt;
}
bool RiskFieldKeyobjDecider::IsObjStart(
    const SpacetimeObjectTrajectory st_traj) {
  return EstObjAcc(st_traj) > kStartAccThresh;
}

double RiskFieldKeyobjDecider::EstObjAcc(
    const SpacetimeObjectTrajectory st_traj) {
  const auto pred_traj = st_traj.trajectory();
  if (!pred_traj.points().empty()) {
    const float start_acc =
        (pred_traj.points().back().v() - pred_traj.points().front().v()) /
        (pred_traj.points().back().t() - pred_traj.points().front().t());
    return start_acc;
  }
  return 0.0;
}

std::optional<std::pair<double, double>>
RiskFieldKeyobjDecider::ConvertToOverlapRange(
    absl::Span<const AgentOverlap> agent_overlaps) {
  if (agent_overlaps.empty()) return std::nullopt;
  std::optional<std::pair<double, double>> overlap_range;
  for (const auto& agent_overlap : agent_overlaps) {
    if (agent_overlap.lat_dist != 0.0) continue;
    if (!overlap_range.has_value()) {
      overlap_range =
          std::make_pair(agent_overlap.first_ra_s, agent_overlap.last_ra_s);
    } else {
      overlap_range->first =
          std::min(overlap_range->first, agent_overlap.first_ra_s);
      overlap_range->second =
          std::max(overlap_range->second, agent_overlap.last_ra_s);
    }
  }
  if (overlap_range.has_value() &&
      overlap_range->first >= overlap_range->second) {
    return std::nullopt;
  }
  return overlap_range;
}

bool RiskFieldKeyobjDecider::IsObjStatic(
    const SpacetimeObjectTrajectory& st_traj) {
  return st_traj.is_stationary();
  //   return obj_ptr->pred_trajectories[0].trajectory.front().v < kMinimumSpeed
  //   &&
  //          obj_ptr->pred_trajectories[0].trajectory.back().v < kMinimumSpeed;
}

bool RiskFieldKeyobjDecider::IsRightOfAvPath(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  float obj_s = 0.0;
  float obj_l = 0.0;
  const auto& obs_center_x = st_traj.trajectory().points().front().pos().x();
  const auto& obs_center_y = st_traj.trajectory().points().front().pos().y();
  const auto& obs_heading = st_traj.trajectory().points().front().theta();
  const auto& width = st_traj.bounding_box().width();
  const auto& length = st_traj.bounding_box().length();
  const Box2d obj_box(Vec2d(obs_center_x, obs_center_y), obs_heading, length,
                      width);
  const auto& corners = obj_box.GetCornersCounterClockwise();
  for (const auto& corner_point : corners) {
    obj_s = av_path.XYToSL(corner_point).s;
    obj_l = av_path.XYToSL(corner_point).l;
    if (obj_l < kInRightThresh) {
      return true;
    }
  }
  return false;
}

bool RiskFieldKeyobjDecider::IsLeftOfAvPath(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  float obj_s = 0.0;
  float obj_l = 0.0;
  const auto& obs_center_x = st_traj.planner_object().pose().pos().x();
  const auto& obs_center_y = st_traj.planner_object().pose().pos().y();
  const auto& obs_heading = st_traj.planner_object().pose().theta();
  const auto& width = st_traj.bounding_box().width();
  const auto& length = st_traj.bounding_box().length();
  const Box2d obj_box(Vec2d(obs_center_x, obs_center_y), obs_heading, length,
                      width);
  const auto& corners = obj_box.GetCornersCounterClockwise();
  for (const auto& corner_point : corners) {
    obj_s = av_path.XYToSL(corner_point).s;
    obj_l = av_path.XYToSL(corner_point).l;
    if (obj_l > kInLeftThresh) {
      return true;
    }
  }
  return false;
}
bool RiskFieldKeyobjDecider::IsFrontOfAvpath(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  float obj_s = 0.0;
  float obj_l = 0.0;
  const auto& obs_center_x = st_traj.planner_object().pose().pos().x();
  const auto& obs_center_y = st_traj.planner_object().pose().pos().y();
  const auto& obs_heading = st_traj.planner_object().pose().theta();
  const auto& width = st_traj.bounding_box().width();
  const auto& length = st_traj.bounding_box().length();
  const Box2d obj_box(Vec2d(obs_center_x, obs_center_y), obs_heading, length,
                      width);
  const auto& corners = obj_box.GetCornersCounterClockwise();
  // distance to rear of ego
  float judge_dis = 0.0;
  float max_obj_s = -FLT_MAX;
  for (const auto& corner_point : corners) {
    obj_s = av_path.XYToSL(corner_point).s;
    max_obj_s = std::max(max_obj_s, obj_s);
  }
  return -max_obj_s < judge_dis ? true : false;
}

bool RiskFieldKeyobjDecider::IsTotallyLeftOfAvPath(
    const SpacetimeObjectTrajectory st_traj, const DiscretizedPath av_path,
    double half_width) {
  float obj_s = 0.0;
  float obj_l = 0.0;
  const auto& obs_center_x = st_traj.trajectory().points().front().pos().x();
  const auto& obs_center_y = st_traj.trajectory().points().front().pos().y();
  const auto& obs_heading = st_traj.trajectory().points().front().theta();
  const auto& width = st_traj.bounding_box().width();
  const auto& length = st_traj.bounding_box().length();
  const Box2d obj_box(Vec2d(obs_center_x, obs_center_y), obs_heading, length,
                      width);
  const auto& corners = obj_box.GetCornersCounterClockwise();
  for (const auto& corner_point : corners) {
    obj_s = av_path.XYToSL(corner_point).s;
    obj_l = av_path.XYToSL(corner_point).l;
    if (obj_l < half_width) {
      return false;
    }
  }
  return true;
}

bool RiskFieldKeyobjDecider::IsTotalLeftOfAv(
    const SpacetimeObjectTrajectory st_traj, double av_half_width) {
  const auto& obs_center_x = st_traj.trajectory().points().front().pos().x();
  const auto& obs_center_y = st_traj.trajectory().points().front().pos().y();
  const auto& obs_heading = st_traj.trajectory().points().front().theta();
  const auto& width = st_traj.bounding_box().width();
  const auto& length = st_traj.bounding_box().length();
  const Box2d obj_box(Vec2d(obs_center_x, obs_center_y), obs_heading, length,
                      width);
  const auto& corners = obj_box.GetCornersCounterClockwise();
  for (const auto& corner_point : corners) {
    if (corner_point.y() < av_half_width) {
      return false;
    }
  }
  return true;
}

bool RiskFieldKeyobjDecider::IsOppoGoStraight(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  // 1. 获取对向，直行障碍物为关键障碍物
  bool is_key_obj = true;
  const auto& pred_traj = st_traj.trajectory();
  double obj_end_yaw = pred_traj.points().back().theta();
  double agent_yaw = av_path.front().theta();
  is_key_obj = std::cos(NormalizeAngle(obj_end_yaw - agent_yaw)) <
               kOncomingCosHeadingThresh;
  if (!is_key_obj) {
    return false;
  }
  if (IsObjStatic(st_traj)) {
    return false;
  }

  return is_key_obj;
}

bool RiskFieldKeyobjDecider::IsOppoStaticLeftAvPath(
    const SpacetimeObjectTrajectory& st_traj, const DiscretizedPath& av_path) {
  if (!IsObjStatic(st_traj)) {
    return false;
  }
  // heading condition
  const auto& pred_traj = st_traj.trajectory();
  double obj_end_yaw = st_traj.planner_object().pose().theta();
  double yaw_diff = NormalizeAngle(obj_end_yaw - av_path.front().theta());
  bool heading_condition = std::cos(yaw_diff) < kStaticOppoCosHeadingThresh;
  if (!heading_condition) {
    return false;
  }
  // sl condition
  bool is_left_of_av_path = IsLeftOfAvPath(st_traj, av_path);
  return is_left_of_av_path && heading_condition;
}

bool RiskFieldKeyobjDecider::IsLeftTurnKeyObj(
    const SpacetimeObjectTrajectory st_traj, const DiscretizedPath& init_path,
    PathPoint start_point) {
  if (IsObjStatic(st_traj)) {
    return false;
  }
  const auto pred_traj = st_traj.trajectory();
  const double obj_end_yaw = NormalizeAngle(pred_traj.points().back().theta());
  const double obj_start_yaw =
      NormalizeAngle(pred_traj.points().front().theta());
  const double obj_yaw_diff = NormalizeAngle(obj_end_yaw - obj_start_yaw);
  Vec2d obj_start_pt(pred_traj.points().front().pos().x(),
                     pred_traj.points().front().pos().y());
  double obj_start_l = init_path.XYToSL(obj_start_pt).l;

  bool is_left_turn_key_obj = true;
  // // if (start_point.is_data_valid())
  // {
  Vec2d start_point_start(start_point.x(), start_point.y());
  Vec2d start_point_end(start_point.x() + std::cos(start_point.theta()),
                        start_point.y() + std::sin(start_point.theta()));
  Vec2d obj_pose(pred_traj.points().front().pos().x(),
                 pred_traj.points().front().pos().y());

  is_left_turn_key_obj =
      CheckPoseInLeft(start_point_start, start_point_end, obj_pose);
  // }
  // else
  // {
  //     is_right_turn_key_obj = obj_ptr->perception_pose.position.y < 0.0;
  // }
  if (!is_left_turn_key_obj) {
    return false;
  }

  if (IsVehicle(st_traj)) {
    return (obj_yaw_diff > 0) && obj_start_l > 0.0;
  } else if (IsCyclist(st_traj)) {
    return (obj_yaw_diff > 0) && obj_start_l > 0.0;
  } else {
    return (obj_yaw_diff > 0) && obj_start_l > 0.0 &&
           pred_traj.points().front().v() > 0.5;
  }
}

bool RiskFieldKeyobjDecider::IsVehicle(
    const SpacetimeObjectTrajectory st_traj) {
  return st_traj.object_type() == ObjectType::OT_VEHICLE ||
         st_traj.object_type() == ObjectType::OT_LARGE_VEHICLE;
}

bool RiskFieldKeyobjDecider::IsPedestrian(
    const SpacetimeObjectTrajectory st_traj) {
  return st_traj.object_type() == ObjectType::OT_PEDESTRIAN;
}

bool RiskFieldKeyobjDecider::IsLargeVehicle(
    const SpacetimeObjectTrajectory st_traj) {
  return st_traj.object_type() == ObjectType::OT_LARGE_VEHICLE ||
         st_traj.bounding_box().length() > 9.0;
}

bool RiskFieldKeyobjDecider::IsCyclist(
    const SpacetimeObjectTrajectory st_traj) {
  return st_traj.object_type() == ObjectType::OT_CYCLIST;
}
}  // namespace st::planning
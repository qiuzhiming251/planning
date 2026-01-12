#include "decider/initializer/obstacle_offset_decider.h"

#include <algorithm>

#include "plan_common/log_data.h"
#include "plan_common/math/util.h"
#include "plan_common/util/status_macros.h"

#include "object_manager/spacetime_planner_object_trajectories_filter.h"

namespace st::planning {

absl::StatusOr<bool> ObstacleOffsetDecider::OffsetDecide(
    const DrivePassage &drive_passage,
    const SpacetimePlannerObjectTrajectories &st_planner_object_traj,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const VehicleGeometryParamsProto &vehicle_geometry,
    const std::vector<LeadingGroup> &leading_groups,
    const FrenetCoordinate &ego_sl, const FrenetBox &ego_box,
    const ApolloTrajectoryPointProto &plan_start_point,
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state,
    PathSlBoundary *path_sl_boundary, const LaneChangeStage &prev_lc_stage,
    bool lc_left, const ObjectHistoryManager *obs_history) {
  // 0. 变量初始化
  if (cur_large_vehicle_avoid_state == nullptr || path_sl_boundary == nullptr ||
      obs_history == nullptr) {
    cur_saved_dist_ =
        std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0));
    return false;
  }
  int num_obj_trajs = st_traj_mgr.trajectories().size();
  double ego_half_width = vehicle_geometry.width() * 0.5;
  bool is_lane_change =
      IsLaneChange(prev_lc_stage, cur_large_vehicle_avoid_state, &lc_left);
  double nudge_min = 0.0, nudge_max = 0.0, range_min_lane = 0.0,
         range_max_lane = 0.0;

  // 1. 获取历史信息
  HistoricalNudgeInfo historical_nudge_info =
      GetHistoricalNudgeInfo(cur_large_vehicle_avoid_state);

  // 2. 计算最大/最小偏移
  GetOffsetLimit(drive_passage, ego_sl, historical_nudge_info, ego_half_width,
                 plan_start_point.v(), &nudge_min, &nudge_max, &range_min_lane,
                 &range_max_lane);

  // 3. 初始化候选列表
  std::vector<ObstacleInfo> candidate_nudge_left;
  std::vector<ObstacleInfo> candidate_nudge_right;
  std::vector<ObstacleInfo> all_obs_info;
  candidate_nudge_left.reserve(num_obj_trajs);
  candidate_nudge_right.reserve(num_obj_trajs);
  all_obs_info.reserve(num_obj_trajs);

  double block_s = DBL_MAX, block_v = 0.0;
  AvoidObstacleFilter(
      drive_passage, st_planner_object_traj, st_traj_mgr, leading_groups,
      ego_sl, historical_nudge_info, obs_history, ego_box, plan_start_point,
      ego_half_width, is_lane_change, lc_left, nudge_min, nudge_max, &block_s,
      &block_v, &candidate_nudge_left, &candidate_nudge_right, &all_obs_info);
  // 4. 计算选择的障碍物及offset
  ObstacleInfo selected_obstacle;
  DecideAvoidSituation(drive_passage, ego_sl, historical_nudge_info, ego_box,
                       plan_start_point, ego_half_width, is_lane_change,
                       nudge_min, nudge_max, block_s, block_v,
                       candidate_nudge_left, candidate_nudge_right,
                       all_obs_info, &selected_obstacle);

  // 5. 更新历史信息并计算执行的偏移量
  double executing_avoid_dist = 0.0;
  UpdateAvoidState(selected_obstacle, ego_sl, cur_large_vehicle_avoid_state,
                   historical_nudge_info, &executing_avoid_dist,
                   is_lane_change);

  // 6. 平移中心线
  bool res = path_sl_boundary->ModifyReferenceCenter(drive_passage,
                                                     executing_avoid_dist);

  Log2DDS::LogDataV2(
      "OffsetDecide",
      absl::StrCat(
          "cur avoid target id: ",
          cur_large_vehicle_avoid_state->cur_avoid_veh_id(),
          ", selected obstacle offset: ", selected_obstacle.offset,
          ", cur_l: ", ego_sl.l, ", target_avoid_dist: ", debug_translate_l_,
          ", executing_avoid_dist: ", executing_avoid_dist,
          ", avg_translate_l: ", debug_avg_translate_l_,
          ", saved cnt: ", cur_large_vehicle_avoid_state->saved_dist_cnt(),
          ", cur avoid target dist: ",
          cur_large_vehicle_avoid_state->dist_to_cur_avoid_veh(),
          ", avoid_end_count: ",
          cur_large_vehicle_avoid_state->avoid_end_count(),
          ", offset_min: ", nudge_min, ", offset_max: ", nudge_max));

  if (!res) {
    LOG_ERROR << "Failed to translate reference center line";
    return false;
  }
  return true;
}

// 更新历史信息并计算执行的偏移量
void ObstacleOffsetDecider::UpdateAvoidState(
    const ObstacleInfo &selected_obstacle, const FrenetCoordinate &ego_sl,
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state,
    const HistoricalNudgeInfo &historical_nudge_info,
    double *executing_avoid_dist, bool is_lane_change) {
  if (cur_large_vehicle_avoid_state == nullptr ||
      executing_avoid_dist == nullptr) {
    return;
  }
  int invalid_selected_obstacle_cnt =
      cur_large_vehicle_avoid_state->invalid_input_cnt();
  if (!selected_obstacle.valid) {
    ++invalid_selected_obstacle_cnt;
    // 限幅
    invalid_selected_obstacle_cnt =
        std::max(invalid_selected_obstacle_cnt, 100);
  } else if (selected_obstacle.valid &&
             std::abs(selected_obstacle.offset) > kEpsilon) {
    invalid_selected_obstacle_cnt = 0;
  }
  int avoid_end_count = cur_large_vehicle_avoid_state->avoid_end_count();

  // 保存历史
  double translate_l = 0.0;
  ++cur_saved_dist_.first;
  int cur_saving_idx = (cur_saved_dist_.first - 1) % kMaxDistSavingNum;
  cur_saved_dist_.second[cur_saving_idx] =
      selected_obstacle.valid ? selected_obstacle.offset : 0.0;
  int valid_dist_num = std::min(cur_saved_dist_.first, kMaxDistSavingNum);
  double accumulated_dist =
      std::accumulate(cur_saved_dist_.second.begin(),
                      cur_saved_dist_.second.begin() + valid_dist_num, 0.0);

  translate_l = valid_dist_num > static_cast<int>(kMaxDistSavingNum * 0.5)
                    ? accumulated_dist / double(valid_dist_num)
                    : 0.0;
  debug_avg_translate_l_ = translate_l;
  // 将计算得到的数值近似到一个数值
  translate_l = RoundToNearest(translate_l, historical_nudge_info.nudge_offset);
  debug_translate_l_ = translate_l;
  if (cur_saved_dist_.first > kMaxDistSavingNum &&
      std::abs(translate_l) < kEpsilon) {
    cur_saved_dist_ =
        std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0.0));
  }

  if (std::abs(translate_l) > MaxEndAvoidLateralMoveOffset &&
      selected_obstacle.valid) {
    // 避让结束时预留的最大居中时间，和避让距离相关
    avoid_end_count = kAvoidEndSmoothMaxCnt +
                      static_cast<int>(std::abs(translate_l) / 0.1 * 8);
  }

  cur_large_vehicle_avoid_state->set_avoid_dist(translate_l);
  cur_large_vehicle_avoid_state->set_saved_dist_cnt(cur_saved_dist_.first);
  cur_large_vehicle_avoid_state->mutable_saved_dist()->Clear();
  cur_large_vehicle_avoid_state->mutable_saved_dist()->CopyFrom(
      {cur_saved_dist_.second.begin(), cur_saved_dist_.second.end()});
  cur_large_vehicle_avoid_state->set_invalid_input_cnt(
      invalid_selected_obstacle_cnt);
  if (std::abs(translate_l) > kEpsilon && selected_obstacle.valid) {
    cur_large_vehicle_avoid_state->set_cur_avoid_veh_id(selected_obstacle.id);
    cur_large_vehicle_avoid_state->set_dist_to_cur_avoid_veh(
        selected_obstacle.ds);
    if (translate_l > kEpsilon) {
      cur_large_vehicle_avoid_state->set_avoid_dir(
          LargeVehicleAvoidStateProto_AvoidDir_DIR_LEFT);
    } else if (translate_l < -kEpsilon) {
      cur_large_vehicle_avoid_state->set_avoid_dir(
          LargeVehicleAvoidStateProto_AvoidDir_DIR_RIGHT);
    }
  }

  // 连续十帧无效，清空
  if (invalid_selected_obstacle_cnt > kInvalidInputCntThreshold) {
    cur_large_vehicle_avoid_state->set_cur_avoid_veh_id("");
    cur_large_vehicle_avoid_state->set_dist_to_cur_avoid_veh(-100.0);
  }

  // 正常避让时的输出逻辑
  double base_l = ego_sl.l;
  if ((cur_large_vehicle_avoid_state->avoid_dist() > kEpsilon &&
       ego_sl.l < -kEpsilon) ||
      (cur_large_vehicle_avoid_state->avoid_dist() < -kEpsilon &&
       ego_sl.l > kEpsilon)) {
    base_l = 0.0;
  }

  double delta_avoid_dist =
      cur_large_vehicle_avoid_state->avoid_dist() - base_l;
  delta_avoid_dist = std::clamp(delta_avoid_dist, -MaxAvoidLateralMoveOffset,
                                MaxAvoidLateralMoveOffset);
  double output_avoid_dist_limit =
      std::abs(cur_large_vehicle_avoid_state->avoid_dist());
  *executing_avoid_dist =
      std::clamp(base_l + delta_avoid_dist, -output_avoid_dist_limit,
                 output_avoid_dist_limit);

  // 结束避让时的输出逻辑，从避让到非避让的情况下（executing_avoid_dist应该是一个更小的数值）
  // 变道时不需要该逻辑
  double offset_buffer = 0.1;
  if (is_lane_change || std::fabs(ego_sl.l) > kNudgeOffsetMax + offset_buffer) {
    avoid_end_count = 0;
  }
  if (avoid_end_count > 0 &&
      std::fabs(*executing_avoid_dist) < std::fabs(ego_sl.l) &&
      *executing_avoid_dist * ego_sl.l > -kEpsilon &&
      invalid_selected_obstacle_cnt > 0) {
    --avoid_end_count;
    if (std::abs(ego_sl.l) > MaxEndAvoidLateralMoveOffset) {
      double cnt_to_dist = (int(avoid_end_count - 1) / int(8)) * 0.1 + 0.1;
      *executing_avoid_dist =
          (ego_sl.l > 0) ? std::max(cnt_to_dist, *executing_avoid_dist)
                         : std::min(-cnt_to_dist, *executing_avoid_dist);
    } else {
      // 已经很近了，直接结束
      avoid_end_count = 0;
    }
  }
  cur_large_vehicle_avoid_state->set_avoid_end_count(avoid_end_count);
  if (selected_obstacle.valid && selected_obstacle.type == OT_CONE) {
    cur_large_vehicle_avoid_state->set_is_type_cone(true);
  } else {
    cur_large_vehicle_avoid_state->set_is_type_cone(false);
  }
}

// 判断是否处于换道状态
bool ObstacleOffsetDecider::IsLaneChange(
    const LaneChangeStage &prev_lc_stage,
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state, bool *lc_left) {
  bool is_lane_change = prev_lc_stage == LaneChangeStage::LCS_EXECUTING ||
                        prev_lc_stage == LaneChangeStage::LCS_RETURN ||
                        prev_lc_stage == LaneChangeStage::LCS_PAUSE;
  if (lc_left == nullptr || cur_large_vehicle_avoid_state == nullptr)
    return is_lane_change;
  int lc_state_end_cnt = cur_large_vehicle_avoid_state->lc_state_end_cnt();
  if (is_lane_change) {
    cur_large_vehicle_avoid_state->set_last_lc_is_left(*lc_left);
    lc_state_end_cnt = kLCStateHoldNum;
  }
  // 变道结束后车辆大概率不居中，多维持几帧，防止无效避让
  if (!is_lane_change && lc_state_end_cnt > 0) {
    --lc_state_end_cnt;
    is_lane_change = true;
    *lc_left = cur_large_vehicle_avoid_state->last_lc_is_left();
  }
  cur_large_vehicle_avoid_state->set_lc_state_end_cnt(lc_state_end_cnt);
  if (lc_state_end_cnt == 0) {
    cur_large_vehicle_avoid_state->set_last_lc_is_left(false);
  }
  return is_lane_change;
}

// 获取历史避让信息
HistoricalNudgeInfo ObstacleOffsetDecider::GetHistoricalNudgeInfo(
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state) {
  HistoricalNudgeInfo info;
  if (cur_large_vehicle_avoid_state == nullptr) {
    return info;
  }
  info.nudge_obj_id = cur_large_vehicle_avoid_state->cur_avoid_veh_id();
  info.nudge_offset = cur_large_vehicle_avoid_state->avoid_dist();
  // info.obs_type =
  if (cur_large_vehicle_avoid_state->is_type_cone()) {
    info.obs_type = OT_UNKNOWN_STATIC;
  }

  cur_saved_dist_ =
      std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0));
  cur_saved_dist_.first = cur_large_vehicle_avoid_state->saved_dist_cnt();
  for (int i = 0; i < cur_large_vehicle_avoid_state->saved_dist_size(); ++i) {
    cur_saved_dist_.second[i] = cur_large_vehicle_avoid_state->saved_dist(i);
  }
  return info;
}

// 计算车辆附近的平均曲率
double ObstacleOffsetDecider::CalculateAverageCurvature(
    const DrivePassage &drive_passage, const FrenetCoordinate &ego_sl,
    const double &ego_v) const {
  double average_curvature = 0.0;
  std::vector<double> headings;
  std::vector<double> factors;
  const double preview_t = kMinPreviewTime;
  const double preview_s =
      std::max(ego_sl.s + ego_v * preview_t, kMinPreviewDist);
  for (double accum_s = ego_sl.s - kBackCheckCurveDistance; accum_s < preview_s;
       accum_s += kCalculateCurvatureStep) {
    const auto heading_or = drive_passage.QueryTangentAngleAtS(accum_s);
    if (!heading_or.ok()) {
      continue;
    }
    if (accum_s < ego_sl.s) {
      factors.push_back(2.0 - (ego_sl.s - accum_s) / kBackCheckCurveDistance);
    } else {
      factors.push_back(2.0 - (accum_s - ego_sl.s) / (preview_s - ego_sl.s));
    }
    headings.push_back(heading_or.value());
  }
  if (headings.size() > 1) {
    for (int i = 0; i < headings.size() - 1; ++i) {
      average_curvature +=
          std::fabs(AngleDifference(headings.at(i), headings.at(i + 1))) *
          factors.at(i);
    }
    average_curvature =
        average_curvature / (kCalculateCurvatureStep * (headings.size() - 1));
  }
  return average_curvature;
}

// 获取偏移量约束
void ObstacleOffsetDecider::GetOffsetLimit(
    const DrivePassage &drive_passage, const FrenetCoordinate &ego_sl,
    const HistoricalNudgeInfo &historical_nudge_info,
    const double &ego_half_width, const double &ego_v, double *offset_min,
    double *offset_max, double *range_min_lane, double *range_max_lane) {
  if (offset_min == nullptr || offset_max == nullptr ||
      range_min_lane == nullptr || range_max_lane == nullptr) {
    return;
  }
  std::vector<double> nudge_dist = {0.8, 0.8, 0.6, 0.4, 0.3};
  std::vector<double> safe_speed_ramp = {0.0, 30.0, 60.0, 90.0, 120.0};
  double safe_nudge_dist = ad_byd::planning::math::interp1_inc(
      safe_speed_ramp, nudge_dist, Mps2Kph(ego_v));
  *offset_min = std::fmax(-0.6, -safe_nudge_dist);
  *offset_max = std::fmin(0.6, safe_nudge_dist);
  const double limit_buffer = 0.2;
  const bool not_in_junction =
      drive_passage.lane_seq_info()->dist_to_junction > 0;
  double s_preview = ego_sl.s + ego_v * 2.0;
  double left_lane_l = 0.0;
  double right_lane_l = 0.0;
  // 使用路宽获取初始偏移量约束
  const auto boundary =
      drive_passage.QueryNearestBoundaryLateralOffset(s_preview);
  if (boundary.ok()) {
    left_lane_l = boundary.value().second;
    right_lane_l = boundary.value().first;
  } else {
    left_lane_l = 0.5 * kDefaultLaneWidth;
    right_lane_l = -0.5 * kDefaultLaneWidth;
  }
  *range_min_lane = std::fmin(right_lane_l + ego_half_width, 0.0);
  *range_max_lane = std::fmax(left_lane_l - ego_half_width, 0.0);
  double offset_min_lane =
      std::fmin(right_lane_l + ego_half_width + limit_buffer, 0.0);
  double offset_max_lane =
      std::fmax(left_lane_l - ego_half_width - limit_buffer, 0.0);
  if (not_in_junction && offset_min_lane < -kEpsilon &&
      offset_max_lane > kEpsilon) {
    *offset_min = std::max(*offset_min, offset_min_lane);
    *offset_max = std::min(*offset_max, offset_max_lane);
  }
  // 路沿对偏移量的约束
  bool left_near_curb = false, right_near_curb = false;
  const auto curb_l = drive_passage.QueryCurbOffsetAtS(s_preview);
  if (curb_l.ok()) {
    double left_curb_l = curb_l->second, right_curb_l = curb_l->first;
    left_near_curb = (left_curb_l - left_lane_l) < kLaneNearCurbThreshold;
    right_near_curb = (right_lane_l - right_curb_l) < kLaneNearCurbThreshold;
  }

  if (left_near_curb && not_in_junction) {
    *offset_max = std::min(*offset_max, kNearCurbNudgeOffsetMax);
  }
  if (right_near_curb && not_in_junction) {
    *offset_min = std::max(*offset_min, -kNearCurbNudgeOffsetMax);
  }

  // 曲率对偏移量的约束
  double abs_average_curvature =
      CalculateAverageCurvature(drive_passage, ego_sl, ego_v);
  double offset_min_kappa = -kNudgeOffsetMax,
         offset_max_kappa = kNudgeOffsetMax;
  double kappa_threshold = 0.001;
  double kappa_max = 0.005;
  if (std::fabs(historical_nudge_info.nudge_offset) > kEpsilon) {
    kappa_threshold = 0.0013;
    kappa_max = 0.02;
  }
  std::vector<double> kappa_ramp = {kappa_threshold, kappa_max};
  std::vector<double> offset_ramp = {kNudgeOffsetMax, 0.0};
  if (abs_average_curvature > kappa_threshold) {
    double interp_offset = ad_byd::planning::math::interp1_inc(
        kappa_ramp, offset_ramp, abs_average_curvature);
    offset_min_kappa = -interp_offset;
    offset_max_kappa = interp_offset;
  }
  *offset_min = std::max(*offset_min, offset_min_kappa);
  *offset_max = std::min(*offset_max, offset_max_kappa);
}

// 判断是否为VRU
bool ObstacleOffsetDecider::IsVRUType(ObjectType type) const {
  switch (type) {
    case OT_PEDESTRIAN:
    case OT_MOTORCYCLIST:
    case OT_CYCLIST:
    case OT_TRICYCLIST:
      return true;
    default:
      return false;
  }
}

// 计算预测给出的障碍物在指定时间的横向速度
double ObstacleOffsetDecider::GetObstaclePredLatSpeed(
    const DrivePassage &drive_passage,
    const prediction::PredictedTrajectory &pred_traj, double preview_time) {
  double obstacle_lat_speed = 0.0;
  const auto pred_traj_points = pred_traj.points();
  if (pred_traj_points.size() < 2) return obstacle_lat_speed;
  // 找到第一个时间大于pred_traj_time的点
  prediction::PredictedTrajectoryPoint preview_point;
  for (const auto &point : pred_traj_points) {
    if (point.t() > preview_time) {
      preview_point = point;
      break;
    }
  }
  // 求这个点的横向速度
  const auto obstacle_lane_tangent =
      drive_passage.QueryTangentAtS(preview_point.s());
  if (obstacle_lane_tangent.ok()) {
    obstacle_lat_speed = preview_point.v() *
                         obstacle_lane_tangent->CrossProd(
                             Vec2d::FastUnitFromAngle(preview_point.theta()));
  }
  return obstacle_lat_speed;
}

// 当前位置的横向速度
double ObstacleOffsetDecider::GetObstacleCurLatSpeed(
    const DrivePassage &drive_passage, const double speed, const double theta,
    const double s) {
  double lat_speed = 0.0;
  const auto obstacle_lane_tangent = drive_passage.QueryTangentAtS(s);
  if (obstacle_lane_tangent.ok()) {
    lat_speed = speed * obstacle_lane_tangent->CrossProd(
                            Vec2d::FastUnitFromAngle(theta));
  }
  return lat_speed;
}

// 获取障碍物横纵向速度
void ObstacleOffsetDecider::GetObstacleLongLatSpeed(
    const DrivePassage &drive_passage, const SpacetimeObjectTrajectory &traj,
    ObstacleInfo *obstacle_info) {
  if (obstacle_info == nullptr) return;
  const auto obstacle_lane_tangent =
      drive_passage.QueryTangentAtS(obstacle_info->center_s);
  if (obstacle_lane_tangent.ok()) {
    obstacle_info->vl =
        traj.pose().v() * obstacle_lane_tangent->CrossProd(
                              Vec2d::FastUnitFromAngle(traj.pose().theta()));
    obstacle_info->vs =
        traj.pose().v() * obstacle_lane_tangent->Dot(
                              Vec2d::FastUnitFromAngle(traj.pose().theta()));
  }
}

// 判断障碍物是否在路沿外
bool ObstacleOffsetDecider::CheckObstacleOutsideCurb(
    const DrivePassage &drive_passage, const FrenetBox &obs_box) {
  const auto curb_l = drive_passage.QueryCurbOffsetAtS(obs_box.center_s());
  if (curb_l.ok()) {
    if (obs_box.l_max < curb_l->first || obs_box.l_min > curb_l->second) {
      return true;
    }
  }
  return false;
}

// GetObstacleMeanLOfHistory
bool ObstacleOffsetDecider::GetObstacleMeanLOfHistory(
    const DrivePassage &drive_passage,
    const ObjectHistoryManager *obj_history_mgr,
    const SpacetimeObjectTrajectory &obs_traj, double *center_l,
    double *box_l_min, double *box_l_max, double *lat_speed) {
  if (obj_history_mgr == nullptr || center_l == nullptr ||
      box_l_min == nullptr || box_l_max == nullptr || lat_speed == nullptr) {
    return false;
  }
  double sum_l = 0.0, sum_l_min = 0.0, sum_l_max = 0.0, sum_lat_speed = 0.0;
  int count_l = 0;
  std::string cur_obj_id = obs_traj.planner_object().id();
  const ObjectHistory *obs_history = obj_history_mgr->GetObjHistory(cur_obj_id);
  if (obs_history == nullptr || obs_history->Empty()) {
    return false;
  }
  const std::deque<ObjectFrame> &frames = obs_history->GetFrames();
  bool last_frame = true;
  int64_t first_timestamp = 0, last_timestamp = 0;
  double first_center_l = 0.0, last_center_l = 0.0;
  for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
    const auto &frame = *it;
    const auto &obs_box_proto = frame.object_proto.bounding_box();
    Vec2d obs_center{obs_box_proto.x(), obs_box_proto.y()};
    double speed =
        std::hypot(frame.object_proto.vel().x(), frame.object_proto.vel().y());
    Box2d obs_box{obs_center, obs_box_proto.heading(), obs_box_proto.length(),
                  obs_box_proto.width()};
    ASSIGN_OR_CONTINUE(const auto obs_frenet_box,
                       drive_passage.QueryFrenetBoxAt(obs_box, false));
    if (last_frame) {
      last_frame = false;
      last_timestamp = frame.timestamp;
      last_center_l = obs_frenet_box.center_l();
    }
    first_timestamp = frame.timestamp;
    first_center_l = obs_frenet_box.center_l();
    sum_l += obs_frenet_box.center_l();
    sum_l_min += obs_frenet_box.l_min;
    sum_l_max += obs_frenet_box.l_max;
    // 效果不佳
    // sum_lat_speed +=
    //     GetObstacleCurLatSpeed(drive_passage, speed,
    //     frame.object_proto.yaw(),
    //                            obs_frenet_box.center_s());
    ++count_l;
    if (last_timestamp - first_timestamp > kMaxUsingHistoryLengthUs) {
      break;
    }
  }
  *box_l_min = sum_l_min / std::max(count_l, 1);
  *box_l_max = sum_l_max / std::max(count_l, 1);
  *center_l = sum_l / std::max(count_l, 1);
  double delta_time = (last_timestamp - first_timestamp) / (1e6);
  if (delta_time > kEpsilon) {
    *lat_speed = (last_center_l - first_center_l) / delta_time;
  } else {
    *lat_speed = 0.0;
  }

  if (count_l > 0) {
    return true;
  } else {
    return false;
  }
}

// 计算障碍物的安全offset数值
ObstacleInfo ObstacleOffsetDecider::GetOffsetInfo(
    const DrivePassage &drive_passage, const SpacetimeObjectTrajectory &traj,
    const HistoricalNudgeInfo &historical_nudge_info,
    const ObjectHistoryManager *obs_history, const FrenetBox &obs_frenet_box,
    const FrenetCoordinate &ego_sl, const FrenetBox &ego_box,
    const double &nudge_min, const double &nudge_max, double half_ego_width,
    double ego_v, bool is_lane_change, bool lc_left) {
  ObstacleInfo obstacle_info;

  if (traj.planner_object().id() == "") {
    return obstacle_info;  // 返回空的obstacle
  }
  // 计算历史信息
  double his_avg_obs_center_l = 0.0, his_avg_obs_l_min = 0.0,
         his_avg_obs_l_max = 0.0, his_avg_obs_lat_speed = 0.0;
  bool cal_obs_his_success = GetObstacleMeanLOfHistory(
      drive_passage, obs_history, traj, &his_avg_obs_center_l,
      &his_avg_obs_l_min, &his_avg_obs_l_max, &his_avg_obs_lat_speed);

  // 构建obstacle_info，包含后续计算需要使用的信息
  obstacle_info.valid = true;
  obstacle_info.id = traj.planner_object().id();
  obstacle_info.v = traj.pose().v();
  obstacle_info.l_max =
      cal_obs_his_success ? his_avg_obs_l_max : obs_frenet_box.l_max;
  obstacle_info.l_min =
      cal_obs_his_success ? his_avg_obs_l_min : obs_frenet_box.l_min;
  obstacle_info.center_l =
      cal_obs_his_success ? his_avg_obs_center_l : obs_frenet_box.center_l();
  obstacle_info.s_max = obs_frenet_box.s_max;
  obstacle_info.s_min = obs_frenet_box.s_min;
  obstacle_info.center_s = obs_frenet_box.center_s();
  obstacle_info.type = traj.object_type();
  obstacle_info.length = obs_frenet_box.length();
  obstacle_info.width = obs_frenet_box.width();
  obstacle_info.is_stationary = traj.is_stationary();
  GetObstacleLongLatSpeed(drive_passage, traj, &obstacle_info);

  // 重合时直接置0
  obstacle_info.dl = (obstacle_info.center_l > 0)
                         ? obs_frenet_box.l_min - half_ego_width
                         : -half_ego_width - obs_frenet_box.l_max;
  obstacle_info.dl = obstacle_info.dl < kEpsilon    ? 0.0
                     : (obstacle_info.center_l > 0) ? obstacle_info.dl
                                                    : -obstacle_info.dl;

  // obstacle_info.ds = obstacle_info.center_s - ego_box.center_s();
  obstacle_info.ds = obstacle_info.s_min > ego_box.s_max
                         ? obstacle_info.s_min - ego_box.s_max
                     : obstacle_info.s_max < ego_box.s_min
                         ? obstacle_info.s_max - ego_box.s_min
                         : 0.0;

  double exit_safe_dist_buffer = 0.0;
  const bool not_in_junction =
      drive_passage.lane_seq_info()->dist_to_junction > kEpsilon;
  const double coeff_left_side =
      obstacle_info.center_l > -kEpsilon ? 1.0 : -1.0;

  // 本车偏移中心线的距离，需要考虑本车不居中的情况可以加上，会影响稳定性
  double delta_l_left = 0.0, delta_l_right = 0.0;
  // if (ego_box.l_min < -half_ego_width) {
  //   delta_l_right = -half_ego_width - ego_box.l_min;
  // }
  // if (ego_box.l_max > half_ego_width) {
  //   delta_l_left = ego_box.l_max - half_ego_width;
  // }

  double obs_box_l = (obstacle_info.center_l > kEpsilon)
                         ? obs_frenet_box.l_min - delta_l_left
                         : obs_frenet_box.l_max + delta_l_right;
  double lat_dist = obs_box_l - coeff_left_side * half_ego_width;

  // 根据障碍物属性确定横向的安全距离
  std::vector<double> safe_lat_dist_ramp = {0.7, 0.75, 0.9, 1.2};
  std::vector<double> safe_speed_ramp = {0.0, 15.0, 60.0, 135.0};
  // VRU类型的障碍物
  if (IsVRUType(obstacle_info.type)) {
    if (not_in_junction) {
      std::vector<double> lat_dist_ramp = {0.7, 0.72, 0.8, 1.0};
      safe_lat_dist_ramp = lat_dist_ramp;
    }
    // 普通车车辆类型障碍物
  } else if (obstacle_info.type == OT_VEHICLE) {
    if (not_in_junction) {
      std::vector<double> lat_dist_ramp = {0.7, 0.72, 0.8, 1.0};
      safe_lat_dist_ramp = lat_dist_ramp;
    } else {
      std::vector<double> lat_dist_ramp = {0.8, 0.92, 1.0, 1.2};
      safe_lat_dist_ramp = lat_dist_ramp;
    }
    // 大车
  } else if (obstacle_info.type == OT_LARGE_VEHICLE) {
    // 把较小的货车和大货车区分开
    if (obstacle_info.length > LargeTruckLengthThreshold) {
      exit_safe_dist_buffer = 0.7;
      std::vector<double> lat_dist_ramp = {0.75, 0.95, 1.4, 1.6};
      safe_lat_dist_ramp = lat_dist_ramp;
    } else {
      exit_safe_dist_buffer = 0.5;
      std::vector<double> lat_dist_ramp = {0.75, 0.85, 1.0, 1.2};
      safe_lat_dist_ramp = lat_dist_ramp;
    }
    // 水马或者锥桶
  } else if (obstacle_info.type == OT_BARRIER ||
             obstacle_info.type == OT_CONE) {
    exit_safe_dist_buffer = 0.2;
    std::vector<double> lat_dist_ramp = {0.6, 0.62, 0.65, 0.7};
    safe_lat_dist_ramp = lat_dist_ramp;
  }
  double safe_lat_dist = ad_byd::planning::math::interp1_inc(
      safe_speed_ramp, safe_lat_dist_ramp, Mps2Kph(ego_v));

  if (obstacle_info.center_l * historical_nudge_info.nudge_offset < -kEpsilon) {
    if (obstacle_info.id == historical_nudge_info.nudge_obj_id) {
      exit_safe_dist_buffer =
          obstacle_info.type == OT_LARGE_VEHICLE ? 0.9 : 0.5;
    }
    // else if (obstacle_info.type == OT_LARGE_VEHICLE) {
    //   // 已经在避让中，更容易对同侧大车发起避让
    //   exit_safe_dist_buffer = 0.5;
    // }
  }

  // 连续避让锥桶的情况，增加安全距离
  if (obstacle_info.type == OT_CONE &&
      historical_nudge_info.obs_type == OT_CONE &&
      std::abs(historical_nudge_info.nudge_offset) > kEpsilon) {
    exit_safe_dist_buffer = 0.4;
  }

  double lc_extra_safe_dist =
      (is_lane_change && ((obstacle_info.center_l > 0 && lc_left) ||
                          (obstacle_info.center_l < 0 && !lc_left)))
          ? (std::abs(obstacle_info.ds) < 5.0) ? 0.5 : 0.3
          : 0.0;

  safe_lat_dist += lc_extra_safe_dist;

  // 安全距离
  obstacle_info.safe_lat_dist = safe_lat_dist;
  // 车辆距离
  obstacle_info.lat_dist = lat_dist;

  double pred_traj_time = 1.2;
  double pred_lat_speed =
      cal_obs_his_success ? his_avg_obs_lat_speed : obstacle_info.vl;
  double virtual_dl = obstacle_info.dl + 0.5 * pred_lat_speed;
  bool area_pred = fabs(obstacle_info.dl) < safe_lat_dist + 0.3 &&
                   obstacle_info.ds > -kEpsilon &&
                   obstacle_info.ds < std::max(3.0, ego_v * 0.5);
  // debug
  double dist0 = coeff_left_side *
                 (lat_dist - coeff_left_side * (NudgeEnableLatDist + 0.0));
  double dist1 =
      coeff_left_side *
      (lat_dist - coeff_left_side * (safe_lat_dist + exit_safe_dist_buffer));
  double dist2 =
      coeff_left_side *
      (virtual_dl - coeff_left_side * (safe_lat_dist + exit_safe_dist_buffer));

  // 障碍物的距离小于0.1m，触发borrow，这里暂时只给出Block
  if (coeff_left_side *
          (lat_dist - coeff_left_side * (NudgeEnableLatDist + 0.0)) <
      -kEpsilon) {
    obstacle_info.offset_type = ObstacleInfo::OffsetType::OffsetBlock;
    obstacle_info.offset = 0.0;
    // 稍微远一点
  } else if (coeff_left_side *
                 (lat_dist -
                  coeff_left_side * (safe_lat_dist + exit_safe_dist_buffer)) <
             -kEpsilon) {
    if ((obstacle_info.type == OT_CONE) || (obstacle_info.type == OT_BARRIER) ||
        (obstacle_info.type == OT_UNKNOWN_STATIC)) {
      // 锥桶采用固定的避让距离： 0.6
      obstacle_info.offset = lat_dist - coeff_left_side * 0.6;
      obstacle_info.offset = coeff_left_side > 0
                                 ? std::min(obstacle_info.offset, 0.0)
                                 : std::max(obstacle_info.offset, 0.0);
    } else {
      // 按照距离/安全距离的比例来确定避让距离
      double dist_ratio = coeff_left_side * lat_dist / safe_lat_dist;
      std::vector<double> offset_ramp = {0.6, 0.4, 0.2};
      std::vector<double> dist_ratio_ramp = {0.0, 0.5, 1.0};
      double offset_level = ad_byd::planning::math::interp1_inc(
          dist_ratio_ramp, offset_ramp, dist_ratio);
      // 大幅度避让时保证稳定性
      if (obstacle_info.id == historical_nudge_info.nudge_obj_id &&
          std::abs(historical_nudge_info.nudge_offset) > 0.2) {
        offset_level = std::max(offset_level,
                                std::abs(historical_nudge_info.nudge_offset));
      }
      // 左侧的障碍物会带来负的避让距离，因此需要取反
      obstacle_info.offset = -coeff_left_side * offset_level;
      obstacle_info.offset = coeff_left_side > 0
                                 ? std::max(obstacle_info.offset, nudge_min)
                                 : std::min(obstacle_info.offset, nudge_max);
    }
    obstacle_info.offset_type = ObstacleInfo::OffsetType::OffsetNudge;
    // 对于预测信息的使用，在非换道情况下使用预测信息，障碍物夹角不准确，经常误判暂时取消
    // } else if (area_pred && !is_lane_change &&
    //            coeff_left_side *
    //                    (virtual_dl - coeff_left_side * (safe_lat_dist +
    //                                                     exit_safe_dist_buffer))
    //                                                     <
    //                kEpsilon) {
    //   double dist_ratio = fabs(virtual_dl) / safe_lat_dist;
    //   std::vector<double> offset_ramp = {0.6, 0.4, 0.2};
    //   std::vector<double> dist_ratio_ramp = {0.0, 0.5, 1.0};
    //   double offset_level = ad_byd::planning::math::interp1_inc(
    //       dist_ratio_ramp, offset_ramp, dist_ratio);
    //   obstacle_info.offset = -coeff_left_side * offset_level;
    //   obstacle_info.offset = coeff_left_side > 0
    //                              ? std::max(obstacle_info.offset, nudge_min)
    //                              : std::min(obstacle_info.offset, nudge_max);
    //   obstacle_info.offset_type = ObstacleInfo::OffsetType::OffsetNudge;
  } else {
    obstacle_info.offset_type = ObstacleInfo::OffsetType::OffsetIgnore;
    obstacle_info.offset = 0.0;
  }
  Log2DDS::LogDataV2(
      "GetOffsetInfo",
      absl::StrCat(
          "Obstacle ID: ", obstacle_info.id,
          ", center_l: ", obstacle_info.center_l, ", ds: ", obstacle_info.ds,
          ", dl: ", obstacle_info.dl, ", vs: ", obstacle_info.vs,
          ", vl: ", obstacle_info.vl, ", offset: ", obstacle_info.offset,
          ", offset_type: ", obstacle_info.offset_type, ", safe_lat_dist: ",
          obstacle_info.safe_lat_dist, ", lat_dist: ", obstacle_info.lat_dist,
          ", dist0: ", dist0, ", dist1: ", dist1, ", dist2: ", dist2,
          ", virtual_dl: ", virtual_dl, ", pred_lat_speed: ", pred_lat_speed,
          ", exit_safe_dist_buffer: ", exit_safe_dist_buffer));
  return obstacle_info;
}

// 筛选需要nudge的障碍物
void ObstacleOffsetDecider::UpdateNudgeCandidate(
    const DrivePassage &drive_passage, ObstacleInfo &obstacle_info,
    const SpacetimeObjectTrajectory &traj,
    const std::vector<LeadingGroup> &leading_groups,
    const HistoricalNudgeInfo &historical_nudge_info,
    const ApolloTrajectoryPointProto &plan_start_point,
    const FrenetCoordinate &ego_sl, const FrenetBox &ego_box,
    const bool is_lane_change, const bool lc_left,
    std::vector<ObstacleInfo> *candidate_nudge_left,
    std::vector<ObstacleInfo> *candidate_nudge_right) {
  if (!obstacle_info.valid || candidate_nudge_left == nullptr ||
      candidate_nudge_right == nullptr) {
    return;
  }
  for (const auto &leading_group : leading_groups) {
    if (leading_group.find(traj.traj_id()) != leading_group.end()) {
      return;
    }
  }

  const auto lane_tangent =
      drive_passage.QueryTangentAtS(plan_start_point.path_point().s());
  double ego_vs =
      lane_tangent.ok()
          ? plan_start_point.v() * lane_tangent->Dot(Vec2d::FastUnitFromAngle(
                                       plan_start_point.path_point().theta()))
          : plan_start_point.v();

  const bool in_junction =
      drive_passage.lane_seq_info()->dist_to_junction < kEpsilon;

  double last_nudge_offset = historical_nudge_info.nudge_offset;
  std::string last_nudge_obj_id = historical_nudge_info.nudge_obj_id;
  double offset_preview_ttc =
      obstacle_info.type == OT_LARGE_VEHICLE ? 5.0 : 3.5;
  double offset_preview_time =
      obstacle_info.type == OT_LARGE_VEHICLE ? 1.5 : 1.2;
  double offset_preview_dist_min =
      obstacle_info.type == OT_LARGE_VEHICLE ? 30.0 : 25.0;
  double sustain_preview_time = 2.0;
  double sustain_preview_dist = 45.0;
  // 计算纵向距离条件
  double dist_max = 0.0;
  double delta_vs = ego_vs - obstacle_info.vs;
  // 不在避让中 或者 目标和已经避让的方向不一致
  if (std::fabs(last_nudge_offset) < kEpsilon ||
      last_nudge_offset * obstacle_info.offset < -kEpsilon) {
    dist_max = std::max(std::max(offset_preview_dist_min,
                                 offset_preview_time * plan_start_point.v()),
                        delta_vs * offset_preview_ttc);
  } else {
    // 已经在避让中，适当增加筛选距离，保证连贯性
    dist_max = std::max(std::max(sustain_preview_dist,
                                 sustain_preview_time * plan_start_point.v()),
                        delta_vs * offset_preview_ttc);
  }
  double back_vehicle_sustain_dist = -5.0;
  // std::min(-5.0, -(obstacle_info.s_max - obstacle_info.center_s));

  double dist_min = obstacle_info.id == last_nudge_obj_id
                        ? back_vehicle_sustain_dist
                        : -kEpsilon;
  // 距离条件
  bool lon_condition =
      (obstacle_info.ds > dist_min) && (obstacle_info.ds < dist_max);
  double low_speed_thred = 25.0;
  double exit_speed_buffer = 0.0;
  // 低速场景下更难进入/退出，提升稳定性
  if (Mps2Kph(ego_vs) < low_speed_thred) {
    exit_speed_buffer = obstacle_info.id == last_nudge_obj_id ? 2.5 : -1.5;
  } else {
    exit_speed_buffer = obstacle_info.id == last_nudge_obj_id ? 1.5 : -1.0;
  }
  // 正在避让中且和目标大车平行，不检查速度条件
  bool parallel_obs = obstacle_info.id == last_nudge_obj_id &&
                      std::abs(last_nudge_offset) > kEpsilon &&
                      std::abs(obstacle_info.ds) < kEpsilon;

  bool is_turck_type = obstacle_info.type == OT_LARGE_VEHICLE;

  bool emergency_nudge =
      (fabs(obstacle_info.dl) < 0.8 && fabs(obstacle_info.ds) < 2.0 &&
       in_junction) ||
      (fabs(obstacle_info.dl) < 0.5 && fabs(obstacle_info.ds) < 2.0) ||
      (fabs(obstacle_info.dl) < 0.6 && fabs(obstacle_info.ds) < 3.0 &&
       is_turck_type);
  // 速度条件
  bool speed_condition =
      (plan_start_point.v() > obstacle_info.vs - exit_speed_buffer) ||
      (obstacle_info.vs < plan_start_point.v() + 0.5 &&
       obstacle_info.ds < 5.0) ||
      obstacle_info.is_stationary || emergency_nudge || parallel_obs;

  double prev_traj_time = 3.0;
  double lat_speed =
      GetObstaclePredLatSpeed(drive_passage, traj.trajectory(), prev_traj_time);
  // 取较大的那个值作为实际横向速度
  lat_speed = (lat_speed * obstacle_info.vl > kEpsilon) &&
                      fabs(lat_speed) > fabs(obstacle_info.vl)
                  ? lat_speed
                  : obstacle_info.vl;
  // 切入/切出判断，排除掉该类障碍物
  double cut_in_out_min_lat_speed_threshold = 0.2;
  double cut_in_out_lat_mul = 0.002;
  double cut_in_out_ds_buffer = 3.5;
  double cut_in_out_max_ds_threshold = 5.0;
  double cut_in_out_ds_cal_time = 1.5;
  double ignore_cut_in_check_ttc_threshold = is_turck_type ? 3.0 : 2.0;
  // bool cut_in_out = (std::fabs(lat_speed) >
  //                    std::max(CutInOutlatSpeed -
  //                                 cut_in_out_lat_mul * (obstacle_info.ds
  //                                 - 5.0),
  //                             cut_in_out_min_lat_speed_threshold)) &&
  //                   (obstacle_info.ds >
  //                    std::min(cut_in_out_ds_buffer +
  //                                 cut_in_out_ds_cal_time *
  //                                 plan_start_point.v(),
  //                             cut_in_out_max_ds_threshold)) &&
  //                   !obstacle_info.is_stationary;
  bool cut_in_out = false;
  if (IsCutInObjectTrajectory(drive_passage, is_lane_change,
                              plan_start_point.v(), ego_box, traj)) {
    cut_in_out = true;
  }

  // 变道场景下快速接近的障碍物筛选逻辑
  bool lc_back_veh_risky = false;
  double lc_state_back_vehicle_check_dist_threshold = -15.0;
  double lc_state_back_vehicle_check_ttc_threshold = 2.5;
  double relativ_s_dist = 0.0;
  if (ego_box.s_min > obstacle_info.s_max) {
    relativ_s_dist = obstacle_info.s_max - ego_box.s_min;
  } else if (ego_box.s_max < obstacle_info.s_min) {
    relativ_s_dist = obstacle_info.s_min - ego_box.s_max;
  }
  double relativ_s_speed = obstacle_info.vs - ego_vs;
  // 纵向距离和相对速度条件满足
  if (is_lane_change && relativ_s_dist < kEpsilon &&
      relativ_s_dist > lc_state_back_vehicle_check_dist_threshold &&
      relativ_s_speed > kEpsilon) {
    // ttc条件
    double back_ttc = std::abs(relativ_s_dist) / relativ_s_speed;
    if (back_ttc < lc_state_back_vehicle_check_ttc_threshold) {
      // 横向距离满足
      if ((lc_left && obstacle_info.center_l > kDefaultLaneWidth * 0.5 &&
           obstacle_info.center_l < kDefaultLaneWidth * 1.5) ||
          (!lc_left && obstacle_info.center_l < -kDefaultLaneWidth * 0.5 &&
           obstacle_info.center_l > -kDefaultLaneWidth * 1.5)) {
        lc_back_veh_risky = true;
      }
    }
  }

  // 增加对障碍物超车时间的校验，防止对长时间无法超车的障碍物发起避让
  double overtake_time = -1.0;
  double overtake_time_threshold = is_turck_type ? 8.5 : 6.0;
  double overtake_dist_threshold = 15.0;
  double delta_v = plan_start_point.v() - obstacle_info.vs;
  double delta_s = obstacle_info.ds;
  // 仅在速度条件满足时计算
  if (speed_condition && delta_v > kEpsilon && delta_s > kEpsilon) {
    overtake_time = delta_s / delta_v;
  }
  bool in_nudge = std::fabs(last_nudge_offset) > kEpsilon &&
                  last_nudge_offset * obstacle_info.offset > kEpsilon;
  bool overtake_condition =
      (overtake_time > 0.0 && overtake_time < overtake_time_threshold) ||
      delta_s < overtake_dist_threshold || delta_v < kEpsilon || in_nudge;

  // 当超车时间小于ttc阈值时，不再判断cutin/cutout条件
  if (overtake_time > 0.0 &&
      overtake_time < ignore_cut_in_check_ttc_threshold) {
    cut_in_out = false;
  }

  Log2DDS::LogDataV2(
      "UpdateNudgeCandidate",
      absl::StrCat("id: ", obstacle_info.id, ", overtake_time: ", overtake_time,
                   ", delta_s: ", delta_s, ", delta_v: ", delta_v,
                   ", lon_condition: ", lon_condition, ", speed_condition: ",
                   speed_condition, ", cut_in_out: ", cut_in_out,
                   ", overtake_condition: ", overtake_condition));

  if (!lon_condition || !speed_condition || cut_in_out || !overtake_condition) {
    if (!lc_back_veh_risky) return;
  }
  obstacle_info.emergency_nudge = emergency_nudge ? true : false;
  if (obstacle_info.offset > kEpsilon) {
    candidate_nudge_left->push_back(obstacle_info);
  } else {
    candidate_nudge_right->push_back(obstacle_info);
  }
}

// 筛选函数入口
void ObstacleOffsetDecider::AvoidObstacleFilter(
    const DrivePassage &drive_passage,
    const SpacetimePlannerObjectTrajectories &st_planner_object_traj,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const std::vector<LeadingGroup> &leading_groups,
    const FrenetCoordinate &ego_sl,
    const HistoricalNudgeInfo &historical_nudge_info,
    const ObjectHistoryManager *obs_history, const FrenetBox &ego_box,
    const ApolloTrajectoryPointProto &plan_start_point,
    const double ego_half_width, const bool is_lane_change, const bool lc_left,
    const double nudge_min, const double nudge_max, double *block_s,
    double *block_v, std::vector<ObstacleInfo> *candidate_nudge_left,
    std::vector<ObstacleInfo> *candidate_nudge_right,
    std::vector<ObstacleInfo> *all_obs_info) {
  if (obs_history == nullptr || candidate_nudge_left == nullptr ||
      candidate_nudge_right == nullptr) {
    return;
  }
  const bool not_in_junction =
      drive_passage.lane_seq_info()->dist_to_junction > 0;
  double s_roi_max = std::min(
      std::max(SROIMaxLow, plan_start_point.v() * SROITime), SROIMaxHi);
  // for (const auto &traj : st_planner_object_traj.trajectories) {
  for (const auto &traj : st_traj_mgr.trajectories()) {
    ASSIGN_OR_CONTINUE(const auto obs_frenet_box,
                       drive_passage.QueryFrenetBoxAt(
                           traj.planner_object().bounding_box(), false));
    if (obs_frenet_box.center_s() - ego_sl.s < SROIMin ||
        obs_frenet_box.center_s() - ego_sl.s > s_roi_max) {
      continue;
    }
    if (CheckObstacleOutsideCurb(drive_passage, obs_frenet_box)) {
      continue;
    }

    // 反光片被识别为该类障碍物，检出不稳定，过滤掉
    if (traj.object_type() == OT_UNKNOWN_MOVABLE) {
      continue;
    }

    // 这里先筛掉距离当前中心线比较近的特殊障碍物
    if (!(traj.object_type() == OT_VEHICLE ||
          traj.object_type() == OT_MOTORCYCLIST ||
          traj.object_type() == OT_PEDESTRIAN ||
          traj.object_type() == OT_CYCLIST ||
          traj.object_type() == OT_TRICYCLIST ||
          traj.object_type() == OT_LARGE_VEHICLE)) {
      if (obs_frenet_box.l_min * obs_frenet_box.l_max < kEpsilon ||
          std::fabs(obs_frenet_box.l_min) < kEpsilon ||
          std::fabs(obs_frenet_box.l_max) < kEpsilon) {
        continue;
      }
    }

    // 筛选掉在ego车辆前方的、非变道方向的障碍物
    if (is_lane_change) {
      if (lc_left && obs_frenet_box.l_max < ego_box.l_max &&
          obs_frenet_box.center_s() - ego_box.center_s() > 5.0) {
        continue;
      } else if (!lc_left && obs_frenet_box.l_min > ego_box.l_min &&
                 obs_frenet_box.center_s() - ego_box.center_s() > 5.0) {
        continue;
      }
    }

    // 计算障碍物偏移，这里没有borrow障碍物，感觉可以将距离条件前移
    ObstacleInfo obstacle_info = GetOffsetInfo(
        drive_passage, traj, historical_nudge_info, obs_history, obs_frenet_box,
        ego_sl, ego_box, nudge_min, nudge_max, ego_half_width,
        plan_start_point.v(), is_lane_change, lc_left);
    if (obstacle_info.valid) {
      all_obs_info->emplace_back(obstacle_info);
    }
    if (obstacle_info.offset_type == ObstacleInfo::OffsetType::OffsetNudge) {
      UpdateNudgeCandidate(drive_passage, obstacle_info, traj, leading_groups,
                           historical_nudge_info, plan_start_point, ego_sl,
                           ego_box, is_lane_change, lc_left,
                           candidate_nudge_left, candidate_nudge_right);
    } else if (obstacle_info.offset_type ==
               ObstacleInfo::OffsetType::OffsetBlock) {
      // 约束后续搜索距离
      if (obstacle_info.center_s > ego_sl.s && !IsVRUType(obstacle_info.type)) {
        if (block_s != nullptr && block_v != nullptr &&
            *block_s > obstacle_info.ds) {
          *block_s = obstacle_info.ds;
          *block_v = obstacle_info.vs;
        }
      }
    }
  }
  auto compareByCenterS = [](const ObstacleInfo &a, const ObstacleInfo &b) {
    return a.center_s < b.center_s;
  };
  sort(candidate_nudge_left->begin(), candidate_nudge_left->end(),
       compareByCenterS);
  sort(candidate_nudge_right->begin(), candidate_nudge_right->end(),
       compareByCenterS);
}

bool ObstacleOffsetDecider::IsFarAwayRoadBoundary(
    const DrivePassage &drive_passage, const FrenetCoordinate &ego_sl,
    const ApolloTrajectoryPointProto &plan_start_point, double ego_half_width,
    bool is_left, ObstacleInfo *avoid_obstacle) {
  if (avoid_obstacle == nullptr) return false;
  double road_bound_min_dist = 0.7;
  std::vector<double> safe_dist = {road_bound_min_dist,
                                   road_bound_min_dist + 0.1,
                                   road_bound_min_dist + 0.3};
  std::vector<double> speed_ramp = {30.0, 60.0, 120.0};
  double roadBoundary_safe_dis = ad_byd::planning::math::interp1_inc(
      speed_ramp, safe_dist, Mps2Kph(plan_start_point.v()));
  // 障碍物的位置
  const double s_range_avoid = avoid_obstacle->center_s + 20.0;
  double l_nearest_road = is_left ? std::numeric_limits<double>::max()
                                  : std::numeric_limits<double>::lowest();
  // 从障碍物开始计算，获取一定范围内的点s_range_avoid
  double s_preview = ego_sl.s;
  while (s_preview < s_range_avoid && s_preview < plan_start_point.v() * 5.0) {
    const auto curb_l = drive_passage.QueryCurbOffsetAtS(s_preview);
    if (curb_l.ok()) {
      // 向左避让，检查左边界
      if (is_left) {
        double left_curb_l = curb_l->second;
        l_nearest_road = std::min(l_nearest_road, left_curb_l);
      } else {
        double right_curb_l = curb_l->first;
        l_nearest_road = std::max(l_nearest_road, right_curb_l);
      }
    }
    s_preview += 3.0;
  }
  double dis_to_road = is_left ? (l_nearest_road - ego_half_width)
                               : (l_nearest_road + ego_half_width);
  // 根据到边界的距离，调整避让距离
  double offset_max = is_left ? (dis_to_road - roadBoundary_safe_dis) > 0
                                    ? (dis_to_road - roadBoundary_safe_dis)
                                    : 0.0
                      : (dis_to_road + roadBoundary_safe_dis) < 0
                          ? (dis_to_road + roadBoundary_safe_dis)
                          : 0.0;
  avoid_obstacle->offset = is_left
                               ? std::fmin(avoid_obstacle->offset, offset_max)
                               : std::fmax(avoid_obstacle->offset, offset_max);
  return fabs(avoid_obstacle->offset) > kEpsilon;
}

void ObstacleOffsetDecider::DecideAvoidSituation(
    const DrivePassage &drive_passage, const FrenetCoordinate &ego_sl,
    const HistoricalNudgeInfo &historical_nudge_info, const FrenetBox &ego_box,
    const ApolloTrajectoryPointProto &plan_start_point,
    const double ego_half_width, const bool is_lane_change,
    const double nudge_min, const double nudge_max, const double &block_s,
    const double &block_v,
    const std::vector<ObstacleInfo> &candidate_nudge_left,
    const std::vector<ObstacleInfo> &candidate_nudge_right,
    const std::vector<ObstacleInfo> &all_obs_info,
    ObstacleInfo *selected_obstacle) {
  // nudge in lane
  // 车道内nudge逻辑
  if (selected_obstacle == nullptr) return;
  const auto lane_tangent =
      drive_passage.QueryTangentAtS(plan_start_point.path_point().s());
  double ego_vs =
      lane_tangent.ok()
          ? plan_start_point.v() * lane_tangent->Dot(Vec2d::FastUnitFromAngle(
                                       plan_start_point.path_point().theta()))
          : plan_start_point.v();
  const double expand_range_nudge = 30.0;
  ObstacleInfo l_nudge_ob, r_nudge_ob;
  double block_s_extra_check_s =
      std::abs(historical_nudge_info.nudge_offset) > kEpsilon ? 8.0 : 3.0;

  // 遍历左侧障碍物列表
  for (int idx_nudge_left = 0; idx_nudge_left < candidate_nudge_left.size();
       idx_nudge_left++) {
    double ds =
        candidate_nudge_left[idx_nudge_left].id != "" ? block_v * 0.2 : 0.0;
    double ttc = -1.0;
    if (candidate_nudge_left[idx_nudge_left].ds < -kEpsilon &&
        candidate_nudge_left[idx_nudge_left].vs > ego_vs) {
      double relative_speed = candidate_nudge_left[idx_nudge_left].vs - ego_vs;
      if (relative_speed > kEpsilon) {
        ttc =
            std::abs(candidate_nudge_left[idx_nudge_left].ds) / relative_speed;
      }
    }
    // 障碍物超过了这个范围就不需检查后面的障碍物了
    if (candidate_nudge_left[idx_nudge_left].valid &&
        candidate_nudge_left[idx_nudge_left].ds >
            block_s + ds + block_s_extra_check_s &&
        (ttc < -kEpsilon || ttc > 3.5)) {
      continue;
    }
    if (!l_nudge_ob.valid) {
      // 第一个障碍物直接加入
      l_nudge_ob = candidate_nudge_left[idx_nudge_left];
    } else {
      // 纵向距离稍远，但是需要的横向距离较近，需要更新为横向距离最近的障碍物
      if ((abs(candidate_nudge_left[idx_nudge_left].center_s -
               l_nudge_ob.center_s) < expand_range_nudge) &&
          (abs(candidate_nudge_left[idx_nudge_left].offset) >
           abs(l_nudge_ob.offset))) {
        l_nudge_ob = candidate_nudge_left[idx_nudge_left];
      }
    }
  }

  // 遍历右侧的障碍物
  for (int idx_nudge_right = 0; idx_nudge_right < candidate_nudge_right.size();
       idx_nudge_right++) {
    double ds =
        candidate_nudge_right[idx_nudge_right].valid ? block_v * 0.2 : 0.0;
    double ttc = -1.0;
    if (candidate_nudge_right[idx_nudge_right].ds < -kEpsilon &&
        candidate_nudge_right[idx_nudge_right].vs > ego_vs) {
      double relative_speed =
          candidate_nudge_right[idx_nudge_right].vs - ego_vs;
      if (relative_speed > kEpsilon) {
        ttc = std::abs(candidate_nudge_right[idx_nudge_right].ds) /
              relative_speed;
      }
    }
    if (candidate_nudge_right[idx_nudge_right].valid &&
        candidate_nudge_right[idx_nudge_right].center_s >
            block_s + ds + block_s_extra_check_s &&
        (ttc < -kEpsilon || ttc > 3.5)) {
      continue;
    }
    if (!r_nudge_ob.valid) {
      r_nudge_ob = candidate_nudge_right[idx_nudge_right];
    } else {
      if (abs(candidate_nudge_right[idx_nudge_right].center_s -
              r_nudge_ob.center_s) < expand_range_nudge &&
          (abs(candidate_nudge_right[idx_nudge_right].offset) >
           abs(r_nudge_ob.offset))) {
        r_nudge_ob = candidate_nudge_right[idx_nudge_right];
      }
    }
  }

  const bool is_in_junction =
      drive_passage.lane_seq_info()->dist_to_junction < kEpsilon;
  if (is_in_junction) {
    if (l_nudge_ob.valid && r_nudge_ob.valid) {
      // 两侧的障碍物距离较小，合并障碍物
      bool nudge_ob_close = fabs(r_nudge_ob.center_s - l_nudge_ob.center_s) <
                            std::max(1.0 * plan_start_point.v(), 10.0);
      if (nudge_ob_close) {
        // 取距离更大的
        double offset = l_nudge_ob.offset + r_nudge_ob.offset;
        if (offset > kEpsilon) {
          if (l_nudge_ob.ds < r_nudge_ob.ds) {
            l_nudge_ob.offset = offset;
            r_nudge_ob.valid = false;
          }
        } else if (offset < -kEpsilon) {
          if (r_nudge_ob.ds < l_nudge_ob.ds) {
            r_nudge_ob.offset = offset;
            l_nudge_ob.valid = false;
          }
        }
      }
    }
  }
  // 两侧都有障碍物，优先避让让近的静止障碍物
  if (l_nudge_ob.valid && r_nudge_ob.valid) {
    // 左侧障碍物静止，右侧有速度但是纵向距离更远，忽略该障碍物
    if (std::fabs(l_nudge_ob.v) < kEpsilon && r_nudge_ob.v > kEpsilon &&
        r_nudge_ob.ds > l_nudge_ob.ds) {
      r_nudge_ob.valid = false;
      // 右侧相同的处理
    } else if (std::fabs(r_nudge_ob.v) < kEpsilon && l_nudge_ob.v > kEpsilon &&
               l_nudge_ob.ds > r_nudge_ob.ds) {
      l_nudge_ob.valid = false;
    }
  }

  // 开始避让了且另外一边的障碍物较远，先完成当前避让
  if (l_nudge_ob.valid && r_nudge_ob.valid &&
      std::abs(historical_nudge_info.nudge_offset) > kEpsilon) {
    if (historical_nudge_info.nudge_obj_id == l_nudge_ob.id &&
        r_nudge_ob.center_s - l_nudge_ob.center_s > 20.0) {
      r_nudge_ob.valid = false;
    } else if (historical_nudge_info.nudge_obj_id == r_nudge_ob.id &&
               l_nudge_ob.center_s - r_nudge_ob.center_s > 20.0) {
      l_nudge_ob.valid = false;
    }
  }

  // 合并偏移量，根据正负判断偏移量的方向
  if (l_nudge_ob.valid && r_nudge_ob.valid) {
    double hedge_offset = l_nudge_ob.offset + r_nudge_ob.offset;
    if (hedge_offset > kEpsilon) {
      l_nudge_ob.offset = hedge_offset;
      r_nudge_ob.valid = false;
    } else if (hedge_offset < -kEpsilon) {
      r_nudge_ob.offset = hedge_offset;
      l_nudge_ob.valid = false;
    }
  }

  // 确定最终的nudge障碍物selected_obstacle
  if (l_nudge_ob.valid && !r_nudge_ob.valid) {
    bool far_away_road_bound =
        IsFarAwayRoadBoundary(drive_passage, ego_sl, plan_start_point,
                              ego_half_width, true, &l_nudge_ob);
    if (far_away_road_bound) {
      *selected_obstacle = l_nudge_ob;
    }
  } else if (r_nudge_ob.valid && !l_nudge_ob.valid) {
    bool far_away_road_bound =
        IsFarAwayRoadBoundary(drive_passage, ego_sl, plan_start_point,
                              ego_half_width, false, &r_nudge_ob);
    if (far_away_road_bound) {
      *selected_obstacle = r_nudge_ob;
    }
  } else {
    selected_obstacle->offset = 0.0;
    selected_obstacle->valid = false;
  }
  bool can_nudge = is_collision_free(drive_passage, all_obs_info, ego_box,
                                     historical_nudge_info, ego_vs,
                                     ego_half_width, selected_obstacle);
  if (!can_nudge) {
    selected_obstacle->offset = 0.0;
    selected_obstacle->valid = false;
  }

  // 低速未避让状态下不发起避让
  if (std::abs(historical_nudge_info.nudge_offset) < kEpsilon && ego_vs < 3.0) {
    selected_obstacle->offset = 0.0;
    selected_obstacle->valid = false;
  }

  Log2DDS::LogDataV2(
      "DecideAvoidSituation",
      absl::StrCat(
          "selected_obstacle valid: ", selected_obstacle->valid, ", offset: ",
          selected_obstacle->offset, ", id: ", selected_obstacle->id,
          ", ds: ", selected_obstacle->ds, ", dl: ", selected_obstacle->dl,
          ", vs: ", selected_obstacle->vs, ", vl: ", selected_obstacle->vl,
          ", safe_lat_dist: ", selected_obstacle->safe_lat_dist,
          ", lat_dist: ", selected_obstacle->lat_dist,
          ", center_l: ", selected_obstacle->center_l));
}

// 检查后方快速接近车辆
bool ObstacleOffsetDecider::is_collision_free(
    const DrivePassage &drive_passage,
    const std::vector<ObstacleInfo> &all_obs_info, const FrenetBox &ego_box,
    const HistoricalNudgeInfo &historical_nudge_info, double ego_vs,
    double half_ego_width, ObstacleInfo *selected_obstacle) {
  double BackNoticeObstacleHeadway = 2.0;  // s
  double ParallelObstacleBuffer = 0.0;     // m
  double StartCheckSpeed = 5.0;
  if (!selected_obstacle->valid ||
      std::abs(selected_obstacle->offset) < kEpsilon) {
    return false;
  }
  if (ego_vs < StartCheckSpeed) {
    return true;
  }

  double dist_min =
      std::min(-BackNoticeObstacleHeadway * std::abs(ego_vs), -5.0);
  double dist_max =
      std::max(selected_obstacle->ds + 10.0, 1.0 * std::abs(ego_vs));

  // 遍历all_obs_info，判断横向上是否安全
  for (const auto &checking_obs : all_obs_info) {
    // 为不同类型的车辆设置不同的碰撞距离阈值
    double collision_dist = (checking_obs.type == OT_LARGE_VEHICLE) ? 0.9 : 0.6;
    double back_notice_ttc =
        (checking_obs.type == OT_LARGE_VEHICLE) ? 3.5 : 2.5;
    // 过滤掉selected_obstacle
    if (checking_obs.id == selected_obstacle->id) {
      continue;
    }
    // 横向重叠的障碍物
    if (std::abs(checking_obs.dl) < kEpsilon) {
      continue;
    }
    // 过滤掉非避让方向的障碍物
    if ((selected_obstacle->offset > kEpsilon && checking_obs.dl < kEpsilon) ||
        (selected_obstacle->offset < -kEpsilon &&
         checking_obs.dl > -kEpsilon)) {
      continue;
    }
    //粗筛，过滤掉范围外的障碍物
    if (checking_obs.ds < dist_max && checking_obs.ds > dist_min) {
      // 过滤掉ttc较大的障碍物，仅考虑后方的障碍物，前方的低速障碍物交由前面的避让模块处理
      double ttc = -1.0;
      if (checking_obs.ds < -kEpsilon && checking_obs.vs > ego_vs) {
        double relative_speed = checking_obs.vs - ego_vs;
        if (relative_speed > kEpsilon) {
          ttc = std::abs(checking_obs.ds) / relative_speed;
        }
      }
      if (ttc > back_notice_ttc) {
        continue;
      }

      // 纵向距离筛选，平行障碍物会被保留
      if ((checking_obs.ds < -kEpsilon && checking_obs.vs < ego_vs) ||
          (checking_obs.ds > kEpsilon + ParallelObstacleBuffer &&
           checking_obs.vs > ego_vs)) {
        continue;
      }
      if (checking_obs.dl > kEpsilon) {
        double max_offset =
            checking_obs.l_min - collision_dist - half_ego_width;
        selected_obstacle->offset =
            std::fmin(selected_obstacle->offset, std::fmax(max_offset, 0.0));
      } else if (checking_obs.dl < -kEpsilon) {
        double min_offset =
            checking_obs.l_max + collision_dist + half_ego_width;
        selected_obstacle->offset =
            std::fmax(selected_obstacle->offset, std::fmin(min_offset, 0.0));
      }
    }
  }
  return std::fabs(selected_obstacle->offset) > kEpsilon;
}

}  // namespace st::planning

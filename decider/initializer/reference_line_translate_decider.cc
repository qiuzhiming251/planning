#include "decider/initializer/reference_line_translate_decider.h"

#include <algorithm>

#include "plan_common/log_data.h"
#include "plan_common/math/util.h"
#include "plan_common/util/status_macros.h"

#include "object_manager/spacetime_planner_object_trajectories_filter.h"

namespace st::planning {

void ReferenceLineTranslateDecider::UpdateSavedDist(
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state) {
  cur_saved_dist_ =
      std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0));
  cur_saved_dist_.first = cur_large_vehicle_avoid_state->saved_dist_cnt();
  for (int i = 0; i < cur_large_vehicle_avoid_state->saved_dist_size(); ++i) {
    cur_saved_dist_.second[i] = cur_large_vehicle_avoid_state->saved_dist(i);
  }
}

bool ReferenceLineTranslateDecider::IsLaneChange(
    const LaneChangeStage &prev_lc_stage) const {
  return prev_lc_stage == LaneChangeStage::LCS_EXECUTING ||
         prev_lc_stage == LaneChangeStage::LCS_RETURN ||
         prev_lc_stage == LaneChangeStage::LCS_PAUSE;
}

bool ReferenceLineTranslateDecider::IsAvoiding(
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state) const {
  return (cur_large_vehicle_avoid_state->avoid_stage() ==
              LargeVehicleAvoidStateProto_AvoidStage_STAGE_SLIGHT ||
          cur_large_vehicle_avoid_state->avoid_stage() ==
              LargeVehicleAvoidStateProto_AvoidStage_STAGE_SIGNIFICANT);
}

bool ReferenceLineTranslateDecider::IsLCExcutingLargeVehNearCond(
    const bool &lc_left, const LaneChangeStage &prev_lc_stage) const {
  bool lc_excute_large_veh_near_cond = false;
  if (prev_lc_stage == LCS_EXECUTING) {
    if (lc_left) {
      lc_excute_large_veh_near_cond =
          left_min_obj_abs_dist_ < kLCExcutingLargeVehNearThreshold;
    } else {
      lc_excute_large_veh_near_cond =
          right_min_obj_abs_dist_ < kLCExcutingLargeVehNearThreshold;
    }
  }
  return lc_excute_large_veh_near_cond;
}

double ReferenceLineTranslateDecider::CalculateAverageCurvature(
    const DrivePassage &drive_passage, const FrenetCoordinate &ego_sl,
    const double &ego_v) const {
  double average_curvature = 0.0;
  std::vector<double> headings;
  std::vector<double> factors;
  const double preview_t = kMinPreviewTime;
  const double preview_s =
      std::max(ego_sl.s + ego_v * preview_t, kMinPreviewDist);
  // 从车后20m向车前5s时距计算曲率
  for (double accum_s = ego_sl.s - kBackCheckCurveDistance; accum_s < preview_s;
       accum_s += kCalculateCurvatureStep) {
    const auto heading_or = drive_passage.QueryTangentAngleAtS(accum_s);
    if (!heading_or.ok()) {
      continue;
    }
    // 计算权重，与距离相关，距离当前位置更近的权重更大
    // [远离本车位置，本车当前位置] 对应的权重范围 [1, 2]
    if (accum_s < ego_sl.s) {
      // 车后的权重
      factors.push_back(2.0 - (ego_sl.s - accum_s) / kBackCheckCurveDistance);
    } else {
      // 车前的权重
      factors.push_back(2.0 - (accum_s - ego_sl.s) / (preview_s - ego_sl.s));
    }
    headings.push_back(heading_or.value());
  }
  if (headings.size() > 1) {
    for (int i = 0; i < headings.size() - 1; ++i) {
      // 将d_heading数值乘以权重并累计，不考虑正负方向，只考虑大小关系
      average_curvature +=
          std::fabs(AngleDifference(headings.at(i), headings.at(i + 1))) *
          factors.at(i);
    }
    // 除以步长获取总曲率，再除以累计的曲率数目获取平均曲率
    average_curvature =
        average_curvature / (kCalculateCurvatureStep * (headings.size() - 1));
  }
  return average_curvature;
}

std::tuple<double, LargeVehicleAvoidStateProto::AvoidDir,
           LargeVehicleAvoidStateProto::AvoidStage>
ReferenceLineTranslateDecider::HysteresisControl(
    double l, bool is_avoiding,
    LargeVehicleAvoidStateProto::AvoidStage prev_avoid_stage,
    double average_curvature) const {
  double fabs_l = std::fabs(l);
  double res = 0.0;
  LargeVehicleAvoidStateProto::AvoidStage avoid_stage;
  LargeVehicleAvoidStateProto::AvoidDir avoid_dir;
  if (fabs_l < kEpsilon) {
    res = 0.0;
    avoid_stage = LargeVehicleAvoidStateProto_AvoidStage_STAGE_NONE;
    avoid_dir = LargeVehicleAvoidStateProto_AvoidDir_DIR_NONE;
    return std::make_tuple(res, avoid_dir, avoid_stage);
  } else if (fabs_l <= kLevelSlight) {
    res = kLevelSlight;
    avoid_stage = LargeVehicleAvoidStateProto_AvoidStage_STAGE_SLIGHT;
  } else {
    res = kLevelSignificant;
    avoid_stage = LargeVehicleAvoidStateProto_AvoidStage_STAGE_SIGNIFICANT;
  }
  // 已经在避让中就不考虑曲率，避免反复进入避让
  // 如果加权平均曲率大于0.0008，且小于等于0.002，仅允许轻微避让
  // 如果加权平均曲率大于0.002，不允许避让
  if (!is_avoiding) {
    if (average_curvature > kSmallCurvatureThreshold &&
        average_curvature <= kLargeCurvatureThreshold) {
      // 如果当前避让级别大于轻微避让，则更新为轻微避让
      if (res > kLevelSlight) {
        res = kLevelSlight;
        avoid_stage = LargeVehicleAvoidStateProto_AvoidStage_STAGE_SLIGHT;
      }
    } else if (average_curvature > kLargeCurvatureThreshold) {
      res = 0.0;
      avoid_stage = LargeVehicleAvoidStateProto_AvoidStage_STAGE_NONE;
      avoid_dir = LargeVehicleAvoidStateProto_AvoidDir_DIR_NONE;
    }
  } else {
    // 在避让中，大曲率不允许扩大避让
    // 较大避让暂时不变小，保证稳定
    if (average_curvature > kSmallCurvatureThreshold) {
      if (prev_avoid_stage ==
          LargeVehicleAvoidStateProto_AvoidStage_STAGE_SLIGHT) {
        if (res > kLevelSlight) {
          res = kLevelSlight;
          avoid_stage = LargeVehicleAvoidStateProto_AvoidStage_STAGE_SLIGHT;
        }
      }
    }
  }
  if (l > 0) {
    res = res;
    avoid_dir = LargeVehicleAvoidStateProto_AvoidDir_DIR_LEFT;
  } else {
    res = -res;
    avoid_dir = LargeVehicleAvoidStateProto_AvoidDir_DIR_RIGHT;
  }
  // return l > 0 ? res : -res;
  return std::make_tuple(res, avoid_dir, avoid_stage);
}

SimpleObstacle::VehicleAttribute
ReferenceLineTranslateDecider::GetVehicleAttribute(
    const DrivePassage &drive_passage, const SpacetimeObjectTrajectory &traj,
    const double &ego_v, const FrenetBox &ego_box, const FrenetBox &obs_box,
    const bool &is_lane_change, std::string &cut_in_obj_id) const {
  SimpleObstacle::VehicleAttribute veh_attribute =
      SimpleObstacle::VehicleAttribute::LargeVehicleNoSpecialTreat;
  // cutin障碍物判断
  bool is_cut_in = false;
  if (IsCutInObjectTrajectory(drive_passage, is_lane_change, ego_v, ego_box,
                              traj)) {
    cut_in_obj_id = traj.traj_id();
    is_cut_in = true;
  }
  double obstacle_lat_speed = 0.0;
  const auto obstacle_lane_tangent =
      drive_passage.QueryTangentAtS(traj.pose().s());
  if (obstacle_lane_tangent.ok()) {
    obstacle_lat_speed =
        traj.pose().v() * obstacle_lane_tangent->CrossProd(
                              Vec2d::FastUnitFromAngle(traj.pose().theta()));
  }
  if (abs(obstacle_lat_speed) > kCrossLineLateralVelThreshold) {
    return veh_attribute;
  }
  // 获取障碍物s位置的车道边界
  const auto boundary =
      drive_passage.QueryNearestBoundaryLateralOffset(traj.pose().s());
  if (boundary.ok() && traj.object_type() != OT_LARGE_VEHICLE && !is_cut_in) {
    double obs_left = obs_box.l_max, obs_right = obs_box.l_min,
           boundary_left = boundary.value().second,
           boundary_right = boundary.value().first;
    // 压线判断
    if (obs_right < boundary_left + kCrossLineSlightThreshold &&
        obs_right > ego_box.l_max) {
      veh_attribute = SimpleObstacle::VehicleAttribute::LeftCrossing;
    } else if (obs_left > boundary_right - kCrossLineSlightThreshold &&
               obs_left < ego_box.l_min) {
      veh_attribute = SimpleObstacle::VehicleAttribute::RightCrossing;
    } else {
      veh_attribute = SimpleObstacle::VehicleAttribute::NoCrossing;
    }
  }
  return veh_attribute;
}

// 计算左右两侧障碍物给出的避让距离
double ReferenceLineTranslateDecider::CalculateMaxTranslateL(
    const std::vector<SimpleObstacle> &side_vector, const FrenetBox &ego_box,
    const double &ego_v, const double &half_av_width,
    const std::string &cur_saved_obstacle_id,
    const bool &lc_excute_large_veh_near_cond,
    const double &dist_for_inadvance_avoid, double &cur_avoid_obs_s,
    SimpleObstacle &cur_avoid_obstacle, int &nearest_cross_line,
    bool is_left_side) const {
  if (side_vector.empty()) {
    return 0.0;
  }
  double min_s_obj_v = side_vector.begin()->speed;
  double min_s = side_vector.begin()->frenet_box.s_min;
  if (min_s < cur_avoid_obs_s) {
    cur_avoid_obs_s = min_s;
    cur_avoid_obstacle = *side_vector.begin();
  }
  // 调试暂时使用
  if (side_vector.begin()->vehicle_attribute ==
          SimpleObstacle::VehicleAttribute::LeftCrossing ||
      side_vector.begin()->vehicle_attribute ==
          SimpleObstacle::VehicleAttribute::RightCrossing) {
    nearest_cross_line = 1;
  }

  // 使用速度条件的情况：该障碍物未被避让；是普通的大车障碍物；是压线车
  bool use_speed_cond =
      cur_avoid_obstacle.id != cur_saved_obstacle_id &&
      !lc_excute_large_veh_near_cond &&
      (side_vector.begin()->vehicle_attribute ==
           SimpleObstacle::VehicleAttribute::LargeVehicleNoSpecialTreat ||
       side_vector.begin()->vehicle_attribute ==
           (is_left_side ? SimpleObstacle::VehicleAttribute::LeftCrossing
                         : SimpleObstacle::VehicleAttribute::RightCrossing));
  // 使用位置条件的情况：最近的车辆不是静止车
  bool nearest_veh_stationary =
      side_vector.begin()->vehicle_attribute ==
      SimpleObstacle::VehicleAttribute::StationaryVehicle;

  if ((min_s_obj_v > ego_v && use_speed_cond) ||
      (min_s - ego_box.s_max > dist_for_inadvance_avoid &&
       !nearest_veh_stationary)) {
    cur_avoid_obstacle = SimpleObstacle("", FrenetBox(), 0.0);
    cur_avoid_obs_s = std::numeric_limits<double>::max();
    return 0.0;
  }

  double max_translate_l = 0.0;
  for (int i = 0; i < side_vector.size(); ++i) {
    // 压线小车使用不同的避让距离
    double final_translate_center_line_buffer =
        side_vector[i].vehicle_attribute ==
                (is_left_side ? SimpleObstacle::VehicleAttribute::LeftCrossing
                              : SimpleObstacle::VehicleAttribute::RightCrossing)
            ? kSmallVehicleCrosslineTranslateCenteLineBuffer
            : kTranslateCenteLineBuffer;

    if (i != 0) {
      if (side_vector[i].frenet_box.s_min -
              side_vector[i - 1].frenet_box.s_max >
          kLengthOfGap) {
        break;
      }
    }

    auto frenet_box = side_vector[i].frenet_box;
    double cal_l = is_left_side ? (frenet_box.l_min - half_av_width -
                                   final_translate_center_line_buffer)
                                : (frenet_box.l_max + half_av_width +
                                   final_translate_center_line_buffer);

    if ((is_left_side && max_translate_l > cal_l) ||
        (!is_left_side && max_translate_l < cal_l)) {
      max_translate_l = cal_l;
    }
  }

  return max_translate_l;
}

absl::StatusOr<bool> IsOncomingObjectJudgeByDrivePassage(
    const DrivePassage &passage, const SecondOrderTrajectoryPoint &obj_pose) {
  ASSIGN_OR_RETURN(const auto tangent, passage.QueryTangentAt(obj_pose.pos()));
  const double passage_angle = tangent.Angle();
  const double angle_diff =
      std::abs(NormalizeAngle(passage_angle - obj_pose.theta()));
  return angle_diff > M_PI_2;
}

// 主要函数
absl::StatusOr<bool> ReferenceLineTranslateDecider::Decide(
    const DrivePassage &drive_passage,
    const SpacetimePlannerObjectTrajectories &st_planner_object_traj,
    const VehicleGeometryParamsProto &vehicle_geometry,
    const std::vector<LeadingGroup> &leading_groups,
    const FrenetCoordinate &ego_sl, const FrenetBox &ego_box,
    const double &ego_v,
    LargeVehicleAvoidStateProto *cur_large_vehicle_avoid_state,
    PathSlBoundary *path_sl_boundary, const LaneChangeStage &prev_lc_stage,
    const bool lc_left) {
  // 初始化变量
  UpdateSavedDist(cur_large_vehicle_avoid_state);
  bool is_lane_change = IsLaneChange(prev_lc_stage);
  bool is_avoiding = IsAvoiding(cur_large_vehicle_avoid_state);
  std::string cur_saved_obstacle_id =
      cur_large_vehicle_avoid_state->cur_avoid_veh_id();
  LargeVehicleAvoidStateProto::AvoidStage prev_avoid_stage =
      cur_large_vehicle_avoid_state->avoid_stage();
  double average_curvature =
      CalculateAverageCurvature(drive_passage, ego_sl, ego_v);
  const std::vector<double> speed_vec = {60, 80, 100, 120};
  const std::vector<double> inadvance_dist_vec = {20.0, 23.0, 28.0, 35.0};
  double dist_for_inadvance_avoid = ad_byd::planning::math::interp1_inc(
      speed_vec, inadvance_dist_vec, Mps2Kph(ego_v));
  dist_for_inadvance_avoid = ad_byd::planning::math::Clamp(
      dist_for_inadvance_avoid, kMinDistForInAdvanceAvoid,
      kMaxDistForInAdvanceAvoid);
  if (is_avoiding) {
    dist_for_inadvance_avoid += kExtraDistForInAdvanceAvoidWhenAvoiding;
  }

  // 遍历，获取左右两侧障碍物
  std::vector<SimpleObstacle> left_side_vector, right_side_vector;
  std::string cut_in_obj_id = "";
  for (const auto &traj : st_planner_object_traj.trajectories) {
    bool is_leading_traj = false;
    for (const auto &leading_group : leading_groups) {
      if (leading_group.find(traj.traj_id()) != leading_group.end()) {
        is_leading_traj = true;
      }
    }
    const auto is_oncoming_obj =
        IsOncomingObjectJudgeByDrivePassage(drive_passage, traj.pose());
    if (is_leading_traj || traj.is_stationary() ||
        (!is_oncoming_obj.ok() || is_oncoming_obj.value() == true)) {
      continue;
    }
    ASSIGN_OR_CONTINUE(const auto frenet_box,
                       drive_passage.QueryFrenetBoxAt(
                           traj.planner_object().bounding_box(), false));
    if (frenet_box.s_min - ego_box.s_max > kConsiderDistance ||
        ego_box.s_min - frenet_box.s_max > kDistForBehindAvoid ||
        frenet_box.l_min > kDefaultLaneWidth ||
        frenet_box.l_max < -kDefaultLaneWidth) {
      continue;
    }
    SimpleObstacle::VehicleAttribute veh_attribute =
        GetVehicleAttribute(drive_passage, traj, ego_v, ego_box, frenet_box,
                            is_lane_change, cut_in_obj_id);
    // 对于正在避让的障碍物，不进行车辆属性的校验，近距离车辆的属性不太稳定
    // 压线小车不进行大车属性校验
    bool car_crossing =
        (veh_attribute == SimpleObstacle::VehicleAttribute::LeftCrossing ||
         veh_attribute == SimpleObstacle::VehicleAttribute::RightCrossing);
    if (traj.object_type() != OT_LARGE_VEHICLE &&
        traj.planner_object().id() != cur_saved_obstacle_id && !car_crossing) {
      continue;
    }
    // 记录两侧的障碍物
    if (frenet_box.l_min > 0.0) {
      left_side_vector.emplace_back(SimpleObstacle(traj.planner_object().id(),
                                                   frenet_box, traj.pose().v(),
                                                   veh_attribute));
      left_min_obj_abs_dist_ = std::min(left_min_obj_abs_dist_,
                                        std::fabs(traj.pose().s() - ego_sl.s));
    }
    if (frenet_box.l_max < 0.0) {
      right_side_vector.emplace_back(SimpleObstacle(traj.planner_object().id(),
                                                    frenet_box, traj.pose().v(),
                                                    veh_attribute));
      right_min_obj_abs_dist_ = std::min(right_min_obj_abs_dist_,
                                         std::fabs(traj.pose().s() - ego_sl.s));
    }
  }
  // 对障碍物进行排序
  if (!left_side_vector.empty()) {
    std::stable_sort(left_side_vector.begin(), left_side_vector.end(),
                     [](const auto &a, const auto &b) {
                       return a.frenet_box.s_min < b.frenet_box.s_min;
                     });
  }
  if (!right_side_vector.empty()) {
    std::stable_sort(right_side_vector.begin(), right_side_vector.end(),
                     [](const auto &a, const auto &b) {
                       return a.frenet_box.s_min < b.frenet_box.s_min;
                     });
  }
  // 判断变道方向是否有近距离大车
  bool lc_excute_large_veh_near_cond =
      IsLCExcutingLargeVehNearCond(lc_left, prev_lc_stage);

  // 计算两侧障碍物给出的避让距离
  double half_av_width = vehicle_geometry.width() * 0.5;
  int left_nearest_cross_line = 0, right_nearest_cross_line = 0;
  double cur_avoid_obs_s = std::numeric_limits<double>::max();
  SimpleObstacle cur_avoid_obstacle("", FrenetBox(), 0.0);
  SimpleObstacle left_avoid_obstacle("", FrenetBox(), 0.0);
  SimpleObstacle right_avoid_obstacle("", FrenetBox(), 0.0);
  double max_translate_left_l = CalculateMaxTranslateL(
      left_side_vector, ego_box, ego_v, half_av_width, cur_saved_obstacle_id,
      lc_excute_large_veh_near_cond, dist_for_inadvance_avoid, cur_avoid_obs_s,
      left_avoid_obstacle, left_nearest_cross_line, true);
  double max_translate_right_l = CalculateMaxTranslateL(
      right_side_vector, ego_box, ego_v, half_av_width, cur_saved_obstacle_id,
      lc_excute_large_veh_near_cond, dist_for_inadvance_avoid, cur_avoid_obs_s,
      right_avoid_obstacle, right_nearest_cross_line, false);
  double translate_l = max_translate_left_l + max_translate_right_l;
  if (translate_l > 0) {
    cur_avoid_obstacle = right_avoid_obstacle;
  } else if (translate_l < 0) {
    cur_avoid_obstacle = left_avoid_obstacle;
  }

  // 保存当前的横向偏移量，计算多帧平均值
  ++cur_saved_dist_.first;
  int cur_saving_idx = (cur_saved_dist_.first - 1) % kMaxUsedDistMemory;
  cur_saved_dist_.second[cur_saving_idx] = translate_l;
  int valid_dist_num = std::min(cur_saved_dist_.first, kMaxUsedDistMemory);
  double accumulated_dist =
      std::accumulate(cur_saved_dist_.second.begin(),
                      cur_saved_dist_.second.begin() + valid_dist_num, 0.0);
  translate_l =
      valid_dist_num > 0 ? accumulated_dist / double(valid_dist_num) : 0.0;

  // 计算避让状态
  const auto state = HysteresisControl(translate_l, is_avoiding,
                                       prev_avoid_stage, average_curvature);
  auto val = std::get<0>(state);
  auto dir = std::get<1>(state);
  auto stage = std::get<2>(state);
  auto pre_avoid_stage = cur_large_vehicle_avoid_state->avoid_stage();
  auto pre_avoid_dir = cur_large_vehicle_avoid_state->avoid_dir();
  if (dir == pre_avoid_dir && stage == pre_avoid_stage) {
    cur_large_vehicle_avoid_state->set_successive_count(0);
  } else {
    auto counter = cur_large_vehicle_avoid_state->successive_count();
    counter++;
    cur_large_vehicle_avoid_state->set_successive_count(counter);
  }
  if (cur_large_vehicle_avoid_state->successive_count() > kSuccessiveCountThr) {
    // 避让->非避让
    if (pre_avoid_dir != LargeVehicleAvoidStateProto_AvoidDir_DIR_NONE &&
        dir == LargeVehicleAvoidStateProto_AvoidDir_DIR_NONE) {
      cur_saved_dist_ =
          std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0.0));
      cur_large_vehicle_avoid_state->set_avoid_end_count(kMaxAvoidEndCnt);
    }
    cur_large_vehicle_avoid_state->set_avoid_dist(val);
    cur_large_vehicle_avoid_state->set_successive_count(0);
    cur_large_vehicle_avoid_state->set_avoid_stage(stage);
    cur_large_vehicle_avoid_state->set_avoid_dir(dir);
  }

  // 异常处理
  if (cur_saved_dist_.first > kMaxAvoidLastTime ||
      (cur_saved_dist_.first > kMaxUsedDistMemory &&
       std::fabs(translate_l) < kEpsilon)) {
    cur_saved_dist_ =
        std::make_pair(0, std::vector<double>(kMaxDistSavingNum, 0.0));
  }
  if (dir != LargeVehicleAvoidStateProto_AvoidDir_DIR_NONE) {
    cur_saved_obstacle_id = cur_avoid_obstacle.id;
  } else {
    cur_saved_obstacle_id = "";
  }

  // 保存信息
  cur_large_vehicle_avoid_state->set_saved_dist_cnt(cur_saved_dist_.first);
  cur_large_vehicle_avoid_state->mutable_saved_dist()->Clear();
  cur_large_vehicle_avoid_state->mutable_saved_dist()->CopyFrom(
      {cur_saved_dist_.second.begin(), cur_saved_dist_.second.end()});
  cur_large_vehicle_avoid_state->set_cur_avoid_veh_id(cur_saved_obstacle_id);
  cur_large_vehicle_avoid_state->set_dist_to_cur_avoid_veh(
      cur_avoid_obstacle.frenet_box.s_min - ego_box.s_max);

  double delta_avoid_dist =
      cur_large_vehicle_avoid_state->avoid_dist() - ego_sl.l;
  delta_avoid_dist = std::clamp(delta_avoid_dist, -kMaxLateralMoveOffset,
                                kMaxLateralMoveOffset);
  double output_avoid_dist_limit =
      std::fabs(cur_large_vehicle_avoid_state->avoid_dist());
  // 判断是否避让快结束，避让快结束时限制范围
  int cur_avoid_end_cnt = cur_large_vehicle_avoid_state->avoid_end_count();
  if (cur_avoid_end_cnt > 0) {
    output_avoid_dist_limit = kMaxLateralOffsetThdNearAvoidEnd;
    --cur_avoid_end_cnt;
    cur_large_vehicle_avoid_state->set_avoid_end_count(cur_avoid_end_cnt);
  }
  double executing_avoid_dist = cur_large_vehicle_avoid_state->avoid_dist();

  Log2DDS::LogDataV2(
      "cur avoid state: ",
      absl::StrCat(
          "count: ", cur_large_vehicle_avoid_state->successive_count(), ", ",
          "dir: ", cur_large_vehicle_avoid_state->avoid_dir(), ", ",
          "stage: ", cur_large_vehicle_avoid_state->avoid_stage(), ", ",
          "dist: ", cur_large_vehicle_avoid_state->avoid_dist(), ", ",
          "executing_avoid_dist: ", executing_avoid_dist, ", ",
          "inadvance dist: ", dist_for_inadvance_avoid, ", ",
          "behind dist: ", kDistForBehindAvoid, ", ",
          "saved_dist_cnt: ", cur_large_vehicle_avoid_state->saved_dist_cnt(),
          ", ", "cur_avoid_veh_id: ",
          cur_large_vehicle_avoid_state->cur_avoid_veh_id(), ", ",
          "average_curvature: ", average_curvature, ", ",
          "left_cross_line: ", left_nearest_cross_line, ", ",
          "right_cross_line: ", right_nearest_cross_line, ", ",
          "cutin obj: ", cut_in_obj_id));

  bool res = path_sl_boundary->ModifyReferenceCenter(drive_passage,
                                                     executing_avoid_dist);
  if (!res) {
    LOG_ERROR << "Failed to translate reference center line";
    return false;
  }
  return true;
}

}  // namespace st::planning
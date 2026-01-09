

#include <algorithm>
#include <cmath>
#include <vector>

#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
#include "decider/scheduler/scheduler_util.h"
#include "plan_common/assist_util.h"
#include "plan_common/log_data.h"
#include "plan_common/path_sl_boundary.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/math/geometry/util.h"
#include "plan_common/plan_common_defs.h"
#include "decider/scheduler/path_boundary_builder_helper.h"

namespace st::planning {
namespace {

LaneChangeStage DecideLaneChangeStage(
    const DrivePassage& drive_passage,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const LaneChangeStateProto& prev_lc_state, const double ego_l) {
  const bool target_switched =
      prev_target_lane_path_from_start.front().lane_id() !=
      drive_passage.lane_path().front().lane_id();

  // check the condition when target_lane == before_lc.
  if (((prev_lc_state.stage() == LaneChangeStage::LCS_EXECUTING) ||
       (prev_lc_state.stage() == LaneChangeStage::LCS_PAUSE)) &&
      (!prev_lane_path_before_lc_from_start.IsEmpty()) &&
      (!prev_target_lane_path_from_start.IsEmpty()) &&
      (prev_target_lane_path_from_start.front().lane_id() ==
       prev_lane_path_before_lc_from_start.front().lane_id())) {
    // if target_lane isn't drive_passage, the state is LCS_EXECUTING.
    if (target_switched) {
      Log2DDS::LogDataV2("lc_stage_debug",
                         "loss target,another LCS_EXECUTING;");
      return LaneChangeStage::LCS_EXECUTING;
    }
    // if loss target lane:
    //     1. change to left, ego on the right,
    //     2. or change to right, ego on the left.
    // the state is LCS_EXECUTING.
    else if ((prev_lc_state.lc_left() && (ego_l < 0.0)) ||
             (!prev_lc_state.lc_left() && (ego_l > 0.0))) {
      Log2DDS::LogDataV2("lc_stage_debug", "loss target lane,LCS_EXECUTING;");
      LOG_ERROR << "[lc_stage_debug] LCS_EXECUTING";
      return LaneChangeStage::LCS_EXECUTING;
    }
    // the rest condition is loss lane_before_lc, the state is LCS_RETURN.
    else {
      Log2DDS::LogDataV2("lc_stage_debug", "loss lc lane,LCS_RETURN;");
      LOG_ERROR << "[lc_stage_debug] LCS_RETURN";
      return LaneChangeStage::LCS_RETURN;
    }
  }

  const auto stage_from_before_lc =
      prev_lane_path_before_lc_from_start.IsEmpty() ||
              prev_lane_path_before_lc_from_start.front().lane_id() !=
                  drive_passage.lane_path().front().lane_id()
          ? LaneChangeStage::LCS_EXECUTING
          : LaneChangeStage::LCS_RETURN;
  if (prev_lane_path_before_lc_from_start.IsEmpty()) {
    Log2DDS::LogDataV2("lc_stage_debug", "prev lp before lc is empty");
  } else {
    Log2DDS::LogDataV2(
        "lc_stage_debug",
        absl::StrCat("prev lp before lc is ",
                     prev_lane_path_before_lc_from_start.front().lane_id()));
    Log2DDS::LogDataV2(
        "lc_stage_debug",
        absl::StrCat("dp lp is ", drive_passage.lane_path().front().lane_id()));
  }

  if (!target_switched) {
    if (prev_lc_state.stage() == LaneChangeStage::LCS_PAUSE) {
      // Treat lc pause as executing here since lc safety is checked later.
      Log2DDS::LogDataV2("lc_stage_debug", "use stage_from_before_lc;");
      return stage_from_before_lc;
    }
    Log2DDS::LogDataV2("lc_stage_debug", "use prev_lc_state;");
    return prev_lc_state.stage();
  }

  switch (prev_lc_state.stage()) {
    case LaneChangeStage::LCS_NONE:
    case LaneChangeStage::LCS_WAITING:
    case LaneChangeStage::LCS_RETURN:
      return LaneChangeStage::LCS_EXECUTING;
    case LaneChangeStage::LCS_EXECUTING:
    case LaneChangeStage::LCS_PAUSE:
      return stage_from_before_lc;
    default:
      throw std::runtime_error("switch case on enum unexpected");
  }
}

}  // namespace

LaneChangeStateProto MakeNoneLaneChangeState() {
  LaneChangeStateProto proto;
  proto.set_stage(LaneChangeStage::LCS_NONE);
  // The rest fields remain unavailable.
  return proto;
}

absl::StatusOr<LaneChangeStateProto> MakeLaneChangeState(
    const PlannerSemanticMapManager& psmm, const DrivePassage& drive_passage,
    const ApolloTrajectoryPointProto& start_point,
    const FrenetBox& ego_frenet_box,
    const mapping::LanePath& prev_target_lane_path_from_start,
    const mapping::LanePath& prev_lane_path_before_lc_from_start,
    const mapping::LanePath& preferred_lane_path,
    const LaneChangeStateProto& prev_lc_state,
    const SmoothedReferenceLineResultMap& smooth_res_map, bool should_smooth,
    bool if_miss_navi, bool is_continuous_lc, Behavior behavior,
    const int& target_lane_path_num) {
  if ((behavior.function_id() == Behavior_FunctionId_ACC) ||
      (behavior.function_id() == Behavior_FunctionId_NONE) ||
      (behavior.function_id() == Behavior_FunctionId_LKA &&
       target_lane_path_num == 1)) {
    return MakeNoneLaneChangeState();
  }
  // if start lane is in uturn section, no lane change state.
  if (!drive_passage.lane_path().IsEmpty() && psmm.map_ptr()) {
    const auto& drive_passage_start_lane_id =
        drive_passage.lane_path().front().lane_id();
    const auto& drive_passage_start_lane =
        psmm.map_ptr()->GetLaneById(drive_passage_start_lane_id);
    if (drive_passage_start_lane &&
        drive_passage_start_lane->turn_type() == TurnType::U_TURN) {
      return MakeNoneLaneChangeState();
    }
  }
  // auto drive_passage_lane_id = drive_passage.lane_path().IsEmpty() ?
  //                     mapping::kInvalidElementId :
  //                     drive_passage.lane_path().front().lane_id();
  // auto prev_target_lane_id = prev_target_lane_path_from_start.IsEmpty() ?
  //                     mapping::kInvalidElementId :
  //                     prev_target_lane_path_from_start.front().lane_id();
  // auto before_lc_id = prev_lane_path_before_lc_from_start.IsEmpty() ?
  //                     mapping::kInvalidElementId :
  //                     prev_lane_path_before_lc_from_start.front().lane_id();

  // LOG_ERROR << "[lc-state-debug]drive_passage:" << drive_passage_lane_id
  //         << ", prev_target:" << prev_target_lane_id
  //         <<  ", before_lc:" << before_lc_id;

  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(start_point);
  const auto ego_v = start_point.v();
  const auto ego_theta = start_point.path_point().theta();
  const double ego_v_kph = Mps2Kph(ego_v);

  const std::vector<double> kEenterTargetLaneLatDis = {0.6, 0.5,  0.4, 0.3,
                                                       0.0, -0.3, -0.5};
  const std::vector<double> kEgoVehSpdKphVct = {0.0, 15, 20, 30, 60, 90, 120};

  const auto ego_frenet_center = ego_frenet_box.center();
  double ref_center_l = 0.0;
  if (should_smooth) {
    const auto center_l_vec =
        ComputeSmoothedReferenceLine(psmm, drive_passage, smooth_res_map);
    ref_center_l = center_l_vec.at(
        drive_passage.FindNearestStationIndexAtS(ego_frenet_center.s).value());
  }

  double lc_continus_center_buffer = 0.0;
  if (if_miss_navi) {
    lc_continus_center_buffer = 0.0;  // 1.1;
  }
  if (std::abs(ego_frenet_center.l - ref_center_l) <
      kMaxLaneKeepLateralOffset + lc_continus_center_buffer) {
    // Close to target lane center, no lane change state.
    return MakeNoneLaneChangeState();
  }

  const double ref_l_min = ego_frenet_box.l_min - ref_center_l;
  const double ref_l_max = ego_frenet_box.l_max - ref_center_l;
  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(ego_frenet_center.s);
  // To deal with virtual lanes with no boundaries other than curbs.
  const double lane_boundary_right_offset =
      std::max(boundaries.right->lat_offset, -kDefaultHalfLaneWidth);
  const double lane_boundary_left_offset =
      std::min(boundaries.left->lat_offset, kDefaultHalfLaneWidth);

  double lc_continus_boundary_buffer = 0.0;
  if (if_miss_navi) {
    lc_continus_boundary_buffer = 0.0;  // 0.7;
  }
  if (lane_boundary_right_offset < ref_l_min + lc_continus_boundary_buffer &&
      ref_l_max < lane_boundary_left_offset + lc_continus_boundary_buffer) {
    // Completely within lane path, no lane change state.
    return MakeNoneLaneChangeState();
  }

  ASSIGN_OR_RETURN(
      const bool crossed_boundary, CrossedBoundary(drive_passage, ego_pos),
      _ << "Ego pos " << ego_pos.DebugString() << " is out of drive passage!");

  LaneChangeStateProto lc_state;
  lc_state.set_stage(DecideLaneChangeStage(
      drive_passage, prev_target_lane_path_from_start,
      prev_lane_path_before_lc_from_start, prev_lc_state, ego_frenet_center.l));
  Log2DDS::LogDataV2("lc_derection",
                     absl::StrCat("ego_frenet_center_l", ego_frenet_center.l));
  lc_state.set_lc_left(ego_frenet_center.l < 0.0);
  lc_state.set_crossed_boundary(crossed_boundary);

  double enter_target_lane_dis_thrd = ad_byd::planning::math::interp1_inc(
      kEgoVehSpdKphVct, kEenterTargetLaneLatDis, ego_v_kph);

  lc_state.set_entered_target_lane(
      lc_state.lc_left() ? ref_l_max > enter_target_lane_dis_thrd
                         : ref_l_min < -enter_target_lane_dis_thrd);

  return lc_state;
}

void MakePushState(const DrivePassage& drive_passage,
                   const SpacetimeTrajectoryManager& st_traj_mgr,
                   const FrenetBox& ego_frenet_box,
                   const ad_byd::planning::MapPtr& map,
                   const PlannerObjectManager* obj_mgr,
                   const std::vector<LanePathInfo>& lp_infos,
                   const LanePathInfo& cur_lp_info,
                   const ApolloTrajectoryPointProto& plan_start_point,
                   const bool is_miss_navi, const bool borrow,
                   const std::optional<bool> is_going_force_route_change_left,
                   const ad_byd::planning::PushDirection pre_push_dir,
                   const PushStatusProto* pre_push_status,
                   const absl::flat_hash_set<std::string>* stalled_objects,
                   LaneChangeStateProto* lc_state) {
  if (map == nullptr || pre_push_status == nullptr || lc_state == nullptr ||
      lc_state->stage() != LaneChangeStage::LCS_NONE) {
    return;
  }

  lc_state->set_push_state(PushState::NONE_PUSH);
  const bool last_lc_push = pre_push_dir != PushDirection::Push_None;

  if (!FLAGS_planner_enable_push) {
    Log2DDS::LogDataV2("lc_push_debug",
                       absl::StrCat(" Not push! FLAGS_planner_enable_push: ",
                                    FLAGS_planner_enable_push));
    return;
  }

  if (map->type() != ad_byd::planning::MapType::HD_MAP) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat(" Not push! not hd_map, map_type: ", int(map->type())));
    return;
  }

  constexpr int kForceInValidCountThres = 4;
  if ((last_lc_push &&
       pre_push_status->force_invalid_count() > kForceInValidCountThres) ||
      (!last_lc_push && !is_going_force_route_change_left.has_value())) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat(
            " Not push! is_going_force_route_change_left.has_value(): ",
            is_going_force_route_change_left.has_value()));
    return;
  }

  if (lp_infos.size() <= 1) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat(" Not push! lp_infos.size(): ", lp_infos.size()));
    return;
  }

  const bool force_lc_left = pre_push_status->force_lc_left();
  const double speed_limit =
      (cur_lp_info.lane_seq_info() != nullptr &&
       cur_lp_info.lane_seq_info()->nearest_lane != nullptr)
          ? cur_lp_info.lane_seq_info()->nearest_lane->speed_limit()
          : 33.3;
  Log2DDS::LogDataV2(
      "lc_push_debug",
      absl::StrCat(
          " last_push: ", last_lc_push, " lc_left: ", force_lc_left,
          " plan_start_v: ", plan_start_point.v(),
          " safe_count: ", pre_push_status->push_lane_safe_count(),
          " unsafe_count: ", pre_push_status->push_lane_unsafe_count(),
          " borrow: ", borrow, " is_miss_navi: ", is_miss_navi,
          " speed_limit: ", speed_limit,
          " solid_count: ", pre_push_status->push_lane_solid_count(),
          " dashed_count: ", pre_push_status->push_lane_dashed_count(),
          " force_invalid_count: ", pre_push_status->force_invalid_count()));

  // check ego speed
  if ((last_lc_push && plan_start_point.v() > Kph2Mps(55.0)) ||
      (!last_lc_push && plan_start_point.v() > Kph2Mps(45.0))) {
    Log2DDS::LogDataV2("lc_push_debug",
                       absl::StrCat(" Not push! plan_start_point.v(): ",
                                    plan_start_point.v()));
    return;
  }

  // get lane_seq_info
  auto cur_lane_seq_info = cur_lp_info.lane_seq_info();
  if (cur_lane_seq_info == nullptr) {
    Log2DDS::LogDataV2("lc_push_debug",
                       " Not push! cur_lane_seq_info = nullptr");
    return;
  }

  auto cur_lane = cur_lane_seq_info->nearest_lane;
  if (cur_lane == nullptr) {
    Log2DDS::LogDataV2("lc_push_debug", " Not push! cur_lane = nullptr");
    return;
  }

  auto target_lane =
      force_lc_left ? map->GetLeftLane(cur_lane) : map->GetRightLane(cur_lane);
  if (target_lane == nullptr) {
    Log2DDS::LogDataV2("lc_push_debug", " Not push! target_lane = nullptr");
    return;
  }

  ad_byd::planning::LaneSeqInfoPtr target_lane_seq_info = nullptr;
  for (const auto& lp_info : lp_infos) {
    if (lp_info.lane_seq_info() != nullptr &&
        lp_info.lane_seq_info()->lane_seq != nullptr &&
        lp_info.lane_seq_info()->lane_seq->IsOnLaneSequence(target_lane)) {
      target_lane_seq_info = lp_info.lane_seq_info();
      break;
    }
  }
  if (target_lane_seq_info == nullptr) {
    Log2DDS::LogDataV2("lc_push_debug",
                       " Not push! target_lane_seq_info = nullptr");
    return;
  }

  // check junction
  if (cur_lane_seq_info->dist_to_junction < 25.0 ||
      target_lane_seq_info->dist_to_junction < 25.0) {
    Log2DDS::LogDataV2("lc_push_debug",
                       absl::StrCat(" Not push! cur_dist_to_junction: ",
                                    cur_lane_seq_info->dist_to_junction,
                                    " target_dist_to_junction: ",
                                    target_lane_seq_info->dist_to_junction));
    return;
  }

  // check bus lane
  if (target_lane_seq_info) {
    double dist_to_navi_end = target_lane_seq_info->dist_to_navi_end;
    double dist_to_bus_lane = target_lane_seq_info->dist_to_bus_lane;
    if (dist_to_navi_end > 160.0 && target_lane_seq_info->lc_num <= 1 &&
        dist_to_bus_lane < dist_to_navi_end &&
        dist_to_bus_lane < std::max(5.0 * plan_start_point.v(), 50.0)) {
      Log2DDS::LogDataV2(
          "lc_push_debug",
          absl::StrCat(" Not push! dist_to_bus_lane: ", dist_to_bus_lane));
      return;
    }
  }

  // check unsafe count
  if ((last_lc_push && pre_push_status->push_lane_safe_count() > 10) ||
      (!last_lc_push && pre_push_status->push_lane_unsafe_count() < 3)) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat(" Not push! count not satisfied. safe_count: ",
                     pre_push_status->push_lane_safe_count(), " unsafe_count: ",
                     pre_push_status->push_lane_unsafe_count()));
    return;
  }

  // check solid line
  constexpr int kSolidLineCountThres = 5;
  constexpr int kDashedLineCountThres = 3;
  const double dist_to_lc_solidline =
      force_lc_left ? target_lane_seq_info->dist_to_right_solid_line
                    : target_lane_seq_info->dist_to_left_solid_line;
  if ((last_lc_push &&
       pre_push_status->push_lane_solid_count() >= kSolidLineCountThres) ||
      (!last_lc_push &&
       pre_push_status->push_lane_dashed_count() < kDashedLineCountThres)) {
    Log2DDS::LogDataV2("lc_push_debug",
                       absl::StrCat(" Not push! dist_to_lc_solidline: ",
                                    dist_to_lc_solidline));
    return;
  }

  // 提取车流信息，包括车流速度、是否拥堵、是否存在大车、最近死车的距离
  std::string front_nearest_stall_id = "";
  int traffic_cnt = 0;
  double traffic_speed = DBL_MAX, front_nearest_stall_dis = DBL_MAX;
  bool exist_large_vehicle = false, exist_stalled_vehicle = false,
       is_congestion_scenario = false;
  ExtractTrafficFlowInfo(plan_start_point, obj_mgr, force_lc_left,
                         target_lane_seq_info, last_lc_push, stalled_objects,
                         pre_push_dir, speed_limit, &front_nearest_stall_id,
                         &front_nearest_stall_dis, &traffic_cnt, &traffic_speed,
                         &exist_large_vehicle, &exist_stalled_vehicle,
                         &is_congestion_scenario);

  // 交通拥堵场景
  Log2DDS::LogDataV2(
      "lc_push_debug",
      absl::StrCat(
          "[scenario] is_congestion_scenario: ", is_congestion_scenario,
          " traffic_cnt: ", traffic_cnt, " traffic_speed: ", traffic_speed));

  // 前方死车场景
  const bool is_stalled_obj_scenario =
      (last_lc_push && front_nearest_stall_dis < 50.0) ||
      (!last_lc_push && front_nearest_stall_dis < 40.0);
  Log2DDS::LogDataV2(
      "lc_push_debug",
      absl::StrCat(
          "[scenario] is_stalled_obj_scenario: ", is_stalled_obj_scenario,
          " front_nearest_stall_id: ", front_nearest_stall_id,
          " front_nearest_stall_dis: ", front_nearest_stall_dis));

  // 导航终点场景
  double cur_dist_to_navi_end = cur_lane_seq_info != nullptr
                                    ? cur_lane_seq_info->dist_to_navi_end
                                    : DBL_MAX;
  const bool is_navi_end_scenario =
      (last_lc_push && cur_dist_to_navi_end < 90.0) ||
      (!last_lc_push && cur_dist_to_navi_end < 80.0);
  Log2DDS::LogDataV2(
      "lc_push_debug",
      absl::StrCat("[scenario] is_navi_end_scenario: ", is_navi_end_scenario,
                   " cur_dist_to_navi_end: ", cur_dist_to_navi_end));

  if (!is_congestion_scenario && !is_stalled_obj_scenario &&
      !is_navi_end_scenario) {
    Log2DDS::LogDataV2("lc_push_debug", " Not push! scenario not satisfied.");
    return;
  }

  // 目标车道存在大车时，不push
  if (exist_large_vehicle) {
    Log2DDS::LogDataV2("lc_push_debug", " Not push! exist_large_vehicle!");
    return;
  }

  // 目标车道存在死车时，不push
  if (exist_stalled_vehicle) {
    Log2DDS::LogDataV2("lc_push_debug", " Not push! exist_stalled_vehicle!");
    return;
  }

  // 车流速度高于阈值时，不push
  if ((last_lc_push && traffic_speed > Kph2Mps(55.0)) ||
      (!last_lc_push && traffic_speed > Kph2Mps(45.0))) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat(" Not push! traffic_speed_kph fast: ",
                     traffic_speed * ad_byd::planning::Constants::MPS2KPH));
    return;
  }

  PushState push_state =
      is_congestion_scenario
          ? (force_lc_left ? PushState::CONGESTION_LEFT_PUSH
                           : PushState::CONGESTION_RIGHT_PUSH)
          : (force_lc_left ? PushState::LEFT_PUSH : PushState::RIGHT_PUSH);

  lc_state->set_push_state(push_state);
}

void ExtractTrafficFlowInfo(
    const ApolloTrajectoryPointProto& plan_start_point,
    const PlannerObjectManager* obj_mgr, const bool lc_left,
    const ad_byd::planning::LaneSeqInfoPtr lane_seq_info,
    const bool last_lc_push_state,
    const absl::flat_hash_set<std::string>* stalled_objects,
    const ad_byd::planning::PushDirection pre_push_dir,
    const double speed_limit, std::string* front_nearest_stall_id,
    double* front_nearest_stall_dis, int* traffic_cnt, double* traffic_speed,
    bool* exist_large_vehicle, bool* exist_stalled_vehicle,
    bool* is_congestion_scenario) {
  constexpr int kMaxStallObjNumThreshold = 2;

  if (!obj_mgr || !lane_seq_info || !lane_seq_info->lane_seq ||
      !front_nearest_stall_id || !front_nearest_stall_dis || !traffic_cnt ||
      !traffic_speed || !exist_large_vehicle || !exist_stalled_vehicle ||
      !is_congestion_scenario) {
    Log2DDS::LogDataV2("lc_push_debug", "[traffic_flow] check nullptr failed!");
    return;
  }

  *front_nearest_stall_id = "";
  *front_nearest_stall_dis = DBL_MAX;
  *traffic_cnt = 0;
  *traffic_speed = speed_limit;
  *exist_large_vehicle = false;
  *exist_stalled_vehicle = false;
  *is_congestion_scenario = false;

  const auto laneseq = lane_seq_info->lane_seq;
  double ego_s = DBL_MAX, ego_l = DBL_MAX;
  const double ego_v = plan_start_point.v();
  const Vec2d ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  laneseq->GetProjectionDistance(ego_pos, &ego_s, &ego_l);

  const double required_s_max =
      std::min(std::max(80.0, ego_v * 5.0), lane_seq_info->dist_to_navi_end);
  const double required_s_min = std::min(-30.0, -ego_v * 2.0);
  const int required_obs_cnt = last_lc_push_state ? 4 : 5;
  const double required_obs_avg_speed = last_lc_push_state ? 50.0 : 45.0;
  Log2DDS::LogDataV2(
      "lc_push_debug",
      absl::StrCat("[traffic_flow] required_s_max: ", required_s_max,
                   " required_s_min: ", required_s_min,
                   " required_obs_cnt: ", required_obs_cnt,
                   " required_obs_avg_speed_kph: ", required_obs_avg_speed));

  absl::flat_hash_map<std::string, SecondOrderTrajectoryPoint>
      traffic_flow_objects;  // 参与车流计算的障碍物
  absl::flat_hash_map<std::string, SecondOrderTrajectoryPoint>
      target_large_objects;  // 目标车道的大车
  absl::flat_hash_map<std::string, SecondOrderTrajectoryPoint>
      target_stalled_objects;  // 目标车道的死车

  std::string traffic_flow_objects_debug;
  std::string target_large_objects_debug;
  std::string target_stalled_objects_debug;
  std::string front_nearest_obs_id = "";
  double front_nearest_obs_dis = DBL_MAX;
  for (const auto& obs : obj_mgr->planner_objects()) {
    if (obs.type() != OT_VEHICLE && obs.type() != OT_MOTORCYCLIST &&
        obs.type() != OT_TRICYCLIST && obs.type() != OT_LARGE_VEHICLE) {
      continue;
      // TODO take bicycle into consideration
    }
    double obs_s = DBL_MAX, obs_l = DBL_MAX;
    const double obs_v = obs.pose().v();
    const double obs_length = obs.bounding_box().length();
    laneseq->GetProjectionDistance(obs.pose().pos(), &obs_s, &obs_l);

    // 找到自车道前方最近的车辆
    if ((obs.type() == OT_VEHICLE || obs.type() == OT_LARGE_VEHICLE) &&
        (obs_s - 0.5 * obs_length > ego_s + 3.9) &&
        (std::abs(obs_l - ego_l) < 2.2)) {
      double obs_dis = (obs_s - 0.5 * obs_length) - (ego_s + 3.9);
      if (obs_dis < front_nearest_obs_dis) {
        front_nearest_obs_id = obs.id();
        front_nearest_obs_dis = obs_dis;
      }
    }

    // 目标车道交通流车辆
    if ((obs_s + 0.5 * obs_length > ego_s + required_s_min) &&
        (obs_s - 0.5 * obs_length < ego_s + required_s_max)) {
      if (std::abs(obs_l) < 2.2) {
        if ((!traffic_flow_objects.contains(obs.id())) &&
            (!stalled_objects || !stalled_objects->contains(obs.id()))) {
          traffic_flow_objects.emplace(obs.id(), obs.pose());
          traffic_flow_objects_debug +=
              absl::StrCat("id:", obs.id(), ",v:", obs_v, "; ");
        }
      }
    }

    // 自车道前方交通流车辆
    if ((obs_s - 0.5 * obs_length > ego_s + 3.9) &&
        (obs_s - 0.5 * obs_length < ego_s + required_s_max)) {
      if (std::abs(obs_l - ego_l) < 2.2) {
        if ((!traffic_flow_objects.contains(obs.id())) &&
            (!stalled_objects || !stalled_objects->contains(obs.id()))) {
          traffic_flow_objects.emplace(obs.id(), obs.pose());
          traffic_flow_objects_debug +=
              absl::StrCat("id:", obs.id(), ",v:", obs_v, "; ");
        }
      }
    }

    const double lon_debounce_buffer = last_lc_push_state ? 5.0 : 0.0;
    const double lat_debounce_buffer = last_lc_push_state ? 0.2 : 0.0;
    // 目标车道大车，纵向范围内，或TTC范围内的大车
    if (obs.is_large_vehicle()) {
      if (std::abs(obs_l) < 2.2 - lat_debounce_buffer) {
        if ((obs_s + 0.5 * obs_length > ego_s - 10.0 + lon_debounce_buffer) &&
            (obs_s - 0.5 * obs_length < ego_s + 13.9 - lon_debounce_buffer)) {
          if (!target_large_objects.contains(obs.id())) {
            target_large_objects.emplace(obs.id(), obs.pose());
            target_large_objects_debug +=
                absl::StrCat("id:", obs.id(), ",v:", obs_v, "; ");
          }
        } else if (std::abs(obs_v - ego_v) > 1e-3) {
          double ttc = (obs_s - ego_s) / (ego_v - obs_v);
          if (ttc > 0.0 && ttc < 3.0) {
            if (!target_large_objects.contains(obs.id())) {
              target_large_objects.emplace(obs.id(), obs.pose());
              target_large_objects_debug += absl::StrCat(
                  "id:", obs.id(), ",v:", obs_v, ",ttc:", ttc, "; ");
            }
          }
        }
      }
    }

    // 目标车道死车
    if ((obs.type() == OT_VEHICLE || obs.type() == OT_LARGE_VEHICLE) &&
        (stalled_objects != nullptr && stalled_objects->contains(obs.id()))) {
      if ((obs_s + 0.5 * obs_length > ego_s + lon_debounce_buffer) &&
          (obs_s - 0.5 * obs_length <
           ego_s + required_s_max - lon_debounce_buffer)) {
        if (std::abs(obs_l) < 2.2 - lat_debounce_buffer) {
          if (!target_stalled_objects.contains(obs.id())) {
            target_stalled_objects.emplace(obs.id(), obs.pose());
            target_stalled_objects_debug +=
                absl::StrCat("id:", obs.id(), ",v:", obs_v, "; ");
          }
        }
      }
    }
  }

  Log2DDS::LogDataV2("lc_push_debug",
                     absl::StrCat("[traffic_flow] traffic_flow_objects: ",
                                  traffic_flow_objects_debug));
  Log2DDS::LogDataV2("lc_push_debug",
                     absl::StrCat("[traffic_flow] target_large_vehicles: ",
                                  target_large_objects_debug));
  Log2DDS::LogDataV2("lc_push_debug",
                     absl::StrCat("[traffic_flow] target_stalled_vehicles: ",
                                  target_stalled_objects_debug));

  if (stalled_objects != nullptr &&
      stalled_objects->contains(front_nearest_obs_id)) {
    *front_nearest_stall_dis = front_nearest_obs_dis;
    *front_nearest_stall_id = front_nearest_obs_id;
  }

  if (!target_large_objects.empty()) {
    *exist_large_vehicle = true;
  }

  if (!target_stalled_objects.empty()) {
    *exist_stalled_vehicle = true;
  }

  // 计算车辆平均速度
  *traffic_cnt = traffic_flow_objects.size();
  if (!traffic_flow_objects.empty()) {
    double speed_sum = 0.0;
    for (const auto& [id, pose] : traffic_flow_objects) {
      speed_sum += pose.v();
    }
    *traffic_speed = speed_sum / traffic_flow_objects.size();
  }

  Log2DDS::LogDataV2(
      "lc_push_debug",
      absl::StrCat("[traffic_flow] traffic_speed: ", *traffic_speed,
                   " traffic_cnt: ", *traffic_cnt,
                   " exist_large_vehicle: ", *exist_large_vehicle,
                   " exist_stalled_vehicle: ", *exist_stalled_vehicle,
                   " front_nearest_stall_id: ", *front_nearest_stall_id,
                   " front_nearest_stall_dis: ", *front_nearest_stall_dis,
                   " front_nearest_obs_id: ", front_nearest_obs_id,
                   " front_nearest_obs_dis: ", front_nearest_obs_dis));

  // 进入拥堵push后不再进行拥堵判断
  if (pre_push_dir == ad_byd::planning::PushDirection::Push_Congestion_Left ||
      pre_push_dir == ad_byd::planning::PushDirection::Push_Congestion_Right) {
    *is_congestion_scenario = true;
  } else if (target_stalled_objects.size() >= kMaxStallObjNumThreshold) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat("[traffic_flow] target_stalled_cnt fail: ",
                     target_stalled_objects.size(),
                     " kMaxStallObjNumThreshold: ", kMaxStallObjNumThreshold));
    *is_congestion_scenario = false;
  } else if (traffic_flow_objects.size() < required_obs_cnt) {
    Log2DDS::LogDataV2("lc_push_debug",
                       absl::StrCat("[traffic_flow] traffic_cnt fail: ",
                                    traffic_flow_objects.size(),
                                    " required_obs_cnt: ", required_obs_cnt));
    *is_congestion_scenario = false;
  } else if (*traffic_speed >
             required_obs_avg_speed * ad_byd::planning::Constants::KPH2MPS) {
    Log2DDS::LogDataV2(
        "lc_push_debug",
        absl::StrCat(
            "[traffic_flow] traffic_speed fail: ", *traffic_speed,
            " required_obs_avg_speed: ",
            required_obs_avg_speed * ad_byd::planning::Constants::KPH2MPS));
    *is_congestion_scenario = false;
  } else {
    *is_congestion_scenario = true;
  }
}

}  // namespace st::planning

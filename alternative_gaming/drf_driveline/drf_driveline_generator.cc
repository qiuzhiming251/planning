#include "drf_driveline_generator.h"

namespace st::planning {
constexpr double kMinVirtualTimeForLonRisk = 1.0;
constexpr double kMinVirtualTimeForLatRisk = 1.5;
constexpr double kMinVirtualTimeForEnvRisk = 1.0;
constexpr double kMaxVirtualTimeForEnvRisk = 2.0;
constexpr double kMinTargetAForLonRisk = -1.0;
constexpr double kMaxTargetAForLonRisk = 2.0;
constexpr double kMinTargetAForLatRisk = -1.0;
constexpr double kMaxTargetAForLatRisk = 2.0;
constexpr double kMinTargetAForEnvRisk = -2.0;
constexpr double kMaxTargetAForEnvRisk = 2.0;
constexpr double kMinTargetAForEnvRiskInPP = -1.5;
constexpr double kEpsilon = 1e-03;
constexpr double kMaxKappa = 0.2;
constexpr double kMinKappa = -0.12;
constexpr double kMaxDkappa = 0.1;
constexpr double kMaxDkappaForEnvRisk = 0.15;
constexpr double kMaxRtcDkappa = 0.3;
constexpr double kMaxLateralJerk = 1.2;
constexpr double kMaxRtcLateralJerk = 4.0;
constexpr double kSimulationDt = 0.2;
constexpr double kSimulationDt2 = 0.04;
constexpr double kSimulationDt3 = 0.008;
constexpr double kSimulationDt4 = kSimulationDt3 * kSimulationDt;
constexpr double kAccDccSwitchDt = 0.2;
constexpr double kMaxJForAcc = 2.0;
constexpr double kMaxJForDcc = 10.0;
constexpr double kMinJForAcc = -10.0;
constexpr double kMinJForDcc = -4.0;
constexpr double kSimulationHorizon = 8.0;
constexpr double kOneSixth = 0.16666666666;
constexpr double kOneThird = 0.33333333333;
constexpr double kOneEighth = 0.125;
constexpr double kMinLookAheadDist = 10.0;
constexpr double kMaxLookAheadDist = 15.0;
constexpr double kMinVehicleSpeed = 2.0;
constexpr double kSpeedLimit = 8.0;
constexpr double kAccLimit = 1.0;
constexpr double kPurePursuitAcc = 1.0;
constexpr double kEndEpsilonSquare = 9.0;  // m
constexpr double kMaxLateralAcc = 2.0;
constexpr double kMaxFrictionAcc = 2.0;
constexpr double kMinAccForCurvatureSpeedLimit = -2.0;
constexpr double kMinJerkForCurvatureSpeedLimit = -1.5;
constexpr double kMinCurvatureSpeedLimit = 3.0;
constexpr double kPurePursuitVel = 5.0;
constexpr double kMinLatConstrainVel = 3.0;
constexpr double kMinRiskValue = -1.0e6;
constexpr double kConsiderHwt = 8.0;
constexpr double kConsiderHwtGap = 4.0;
constexpr double kLatVirtualTimeCoeff = 0.5;
constexpr double kStartPointEnvBuffer = -1.5;
constexpr double kVeticalOffset = -1.0;
constexpr double kFastRtcVirtualTime = 0.2;
constexpr double kParallelSinThresh = 0.3;
constexpr double kParallelCosThresh = 0.966;
constexpr double kLeftNudgeDrivelineKeypointsLength = 15.0;
constexpr double kRightNudgeDrivelineKeypointsLength = 10.0;
constexpr double kRightNudgeKeypointsStartPointHeadingDiffThresh = 0.6;
constexpr double kDrivelineKeypointsResolution = 1.0;

constexpr double kBackWardCosHeadingDiff = 0.1;

double SquareDist(double x1, double y1, double x2, double y2) {
  double dx = x1 - x2;
  double dy = y1 - y2;
  return dx * dx + dy * dy;
}

void SolveSimpleQP(double A, double B, double C, double D, double E, double F,
                   double G, bool has_ieq_constraint, double* x, double* y) {
  double x_star = A > std::numeric_limits<double>::epsilon() ? -C / A : 0.0f;
  double y_star = B > std::numeric_limits<double>::epsilon() ? -D / B : 0.0f;

  if (!has_ieq_constraint) {
    *x = x_star;
    *y = y_star;
    return;
  }

  if (E * x_star + F * y_star <= G) {
    *x = x_star;
    *y = y_star;
    return;
  }

  double E_A = A > std::numeric_limits<double>::epsilon() ? E / A : 0.0f;
  double F_B = B > std::numeric_limits<double>::epsilon() ? F / B : 0.0f;

  double lambda_multiplier = -(C * E_A + D * F_B + G) / (E * E_A + F * F_B);
  *x = -lambda_multiplier * E_A + x_star;
  *y = -lambda_multiplier * F_B + y_star;
}

absl::StatusOr<DRFDrivelineOutput>
DRFDynamicDrivelineGenerator::RunRiskFieldDrivelineGenerator(
    const DRFDrivelineInput& input) {
  TIMELINE("RunRiskFieldDrivelineGenerator");
  std::string name =
      Log2DDS::TaskPrefix(input.plan_id) + "RunRiskFieldDrivelineGenerator";
  SCOPED_TRACE(name.c_str());
  DrivelineResultProto driveline_result;
  DRFDrivelineOutput output;

  if (input.init_path->empty()) {
    debug_.append("initializer path is empty \n");
    Log2DDS::LogDataV2(name, debug_);
    driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
    debug_.append("clear strat_pt\n");
    driveline_result.clear_start_point();
    output.driveline_result = std::move(driveline_result);
    return output;
  }

  Vec2d ego_xy =
      Vec2d(input.init_path->front().x(), input.init_path->front().y());
  double ego_s = input.drive_passage->frenet_frame()->XYToSL(ego_xy).s - 30.0;
  debug_ += absl::StrCat(" ego_s:", ego_s, "\n");
  bool is_left_turn = false;
  bool is_closed_junction = false;
  if (input.drive_passage->lane_seq_info() &&
      input.drive_passage->lane_seq_info()->lane_seq) {
    const auto& lanes = input.drive_passage->lane_seq_info()->lane_seq->lanes();
    double dis_to_turn = 100.0;
    const double dis_to_junction =
        input.drive_passage->lane_seq_info()->dist_to_junction;
    for (const auto& lane : lanes) {
      if (!lane->IsValid()) {
        continue;
      }
      Vec2d lane_end_point_xy(lane->center_line().end_point().x(),
                              lane->center_line().end_point().y());
      double lane_end_point_s =
          input.drive_passage->frenet_frame()->XYToSL(lane_end_point_xy).s -
          30.0;
      if (lane->turn_type() == ad_byd::planning::TurnType::LEFT_TURN &&
          lane_end_point_s - ego_s > -1.0) {
        debug_.append(
            absl::StrCat("left end_point_s:", lane_end_point_s, "\n"));
        is_left_turn = true;
        double dis_to_centerline = lane->center_line().GetDistance(
            input.init_path->front().x(), input.init_path->front().y());
        dis_to_turn = std::max(dis_to_centerline, dis_to_junction);
        debug_.append(absl::StrCat("key left turn lane_id: ", lane->id(), "\n",
                                   "dis_to_junction  : ", dis_to_junction, "\n",
                                   "dis_to_ceter_lane: ", dis_to_centerline,
                                   "\n"));
      }
      if (dis_to_turn < 20.0) {
        is_closed_junction = true;
      }
      if (is_left_turn && is_closed_junction) {
        break;
      }
    }
  }

  if (!is_left_turn) {
    debug_.append("ego is not turn left!");
    Log2DDS::LogDataV2(name, debug_);
    driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
    debug_.append("clear strat_pt\n");
    driveline_result.clear_start_point();
    output.driveline_result = std::move(driveline_result);
    return output;
  } else {
    debug_.append("ego is turn left!\n");
  }

  if (!is_closed_junction) {
    debug_.append("ego is not in junction, I'm sleeping!");
    Log2DDS::LogDataV2(name, debug_);
    // return absl::UnavailableError("ego is not in junction.");
    driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
    debug_.append("clear strat_pt\n");
    driveline_result.clear_start_point();
    output.driveline_result = std::move(driveline_result);
    return output;
  } else {
    debug_.append("ego is in junction, I'm working!\n");
  }
  // if pre_result有start pt：直接赋给当前的result
  // else pre_result没有start pt：遍历选点
  //为防止找不到点，初始化tmp点为当前位置点
  PathPoint start_point_tmp = input.init_path->front();
  if (input.last_driveline_result &&
      input.last_driveline_result->has_start_point()) {
    debug_.append("already have strat_pt\n");
    StartPointProto* start_pt_tmp = driveline_result.mutable_start_point();
    start_pt_tmp->set_x(input.last_driveline_result->start_point().x());
    start_pt_tmp->set_y(input.last_driveline_result->start_point().y());
    start_pt_tmp->set_h(input.last_driveline_result->start_point().h());
    start_point_tmp.set_x(input.last_driveline_result->start_point().x());
    start_point_tmp.set_y(input.last_driveline_result->start_point().y());
    start_point_tmp.set_theta(input.last_driveline_result->start_point().h());
    // StationIndex station_index =
    // input.drive_passage->FindNearestStationIndex(Vec2d(start_pt_tmp->x(),start_pt_tmp->y()));
  } else {
    //遍历找路口起点
    //若当前点在路口，前一点必定不在路口，就设置当前点，break
    debug_.append("try2find strat_pt\n");
    const auto stations = input.drive_passage->stations();
    for (int i = 0; i < input.drive_passage->size(); i++) {
      auto start_station = stations[StationIndex(i)];
      if (start_station.accumulated_s() < ego_s) {
        continue;
      }
      if (start_station.station_info().is_in_intersection) {
        StartPointProto* start_pt_tmp = driveline_result.mutable_start_point();
        start_pt_tmp->set_x(start_station.xy().x());
        start_pt_tmp->set_y(start_station.xy().y());
        start_pt_tmp->set_h(
            atan2(start_station.tangent().y(), start_station.tangent().x()));
        start_point_tmp.set_x(start_station.xy().x());
        start_point_tmp.set_y(start_station.xy().y());
        start_point_tmp.set_theta(
            atan2(start_station.tangent().y(), start_station.tangent().x()));
        break;
      }
    }
  }
  const PathPoint start_point = start_point_tmp;
  Vec2d start_xy = Vec2d(start_point.x(), start_point.y());
  double start_s =
      input.drive_passage->frenet_frame()->XYToSL(start_xy).s - 30.0;
  debug_ += absl::StrCat(" start_point.x:", start_point.x(),
                         "\n start_point.y:", start_point.y(), "\n");
  const auto& prefix = Log2DDS::TaskPrefix(input.plan_id);
  const auto& init_point = input.plan_start_point->start_point;
  VehicleState av_state;
  TrajPointToAvState(init_point, &av_state);
  debug_.append("run riskfield driveline \n");
  SpacetimeTrajectoryManager modified_traj_mgr;
  // RefinePredTraj(*input.traj_mgr, &modified_traj_mgr);
  debug_.append("modified obj traj theta \n");
  std::vector<RiskFieldKeyobj> oppo_straight_keyvehicles;
  std::vector<RiskFieldKeyobj> oppo_straight_keycyclists;
  std::vector<RiskFieldKeyobj> syn_left_turn_keyvehicles;
  std::vector<RiskFieldKeyobj> syn_left_turn_keycyclists;
  std::vector<RiskFieldKeyobj> outside_risk_keyobjs;
  RiskFieldKeyobjDecider risk_keyobj(*vehicle_geo_params_);
  // risk_keyobj.GetLeftTurnKeyobjs(&modified_traj_mgr, input.init_path,
  // av_state.v,
  //                                start_pt, &syn_left_turn_keyvehicles,
  //                                &syn_left_turn_keycyclists);
  if (input.last_driveline_result) {
    debug_.append(absl::StrCat(
        "last most risk obj: ", input.last_driveline_result->most_risk_obj(),
        "\n"));
  }
  risk_keyobj.GetLeftTurnKeyobjs(input.traj_mgr, input.init_path, av_state.v,
                                 start_point, input.last_driveline_result,
                                 &syn_left_turn_keyvehicles,
                                 &syn_left_turn_keycyclists);

  // for (const auto& yield_id : yield_ids)
  // {
  //     debug_ += "*********yield_id: " + yield_id + "**************\n";
  // }
  // debug_ += "\n";
  debug_.append("syn_left_turn_keyvehicles:  ");
  ;
  for (const auto& keyobj : syn_left_turn_keyvehicles) {
    debug_.append(absl::StrCat(keyobj.id, " "));
  }
  debug_.append("\n");

  // risk_keyobj.GetLeftTurnOutsideOncommingKeyobjs(input.traj_mgr,
  // input.init_path, av_state.v,
  //                                                &outside_risk_keyobjs);
  // if (!outside_risk_keyobjs.empty())
  // {
  //     debug_.append("outside_risk_keyobjs is not empty");
  //     Log2DDS::LogDataV2(name, debug_);
  //     driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
  //     output.driveline_result = std::move(driveline_result);
  //     return output;
  // }
  // debug_.append("outside_oncomming_keyobjs:  \n");
  // for (const auto& keyobj : outside_risk_keyobjs)
  // {
  //     debug_ .append(absl::StrCat(keyobj.id," "));
  // }
  // debug_.append("\n");

  risk_keyobj.GetOppoKeyobjs(input.traj_mgr, input.init_path, av_state.v,
                             &oppo_straight_keyvehicles,
                             &oppo_straight_keycyclists);
  debug_.append("oppo_straight_keyvehicles:  ");
  for (const auto& keyobj : oppo_straight_keyvehicles) {
    debug_.append(absl::StrCat(keyobj.id, " "));
  }
  debug_.append("\n");
  debug_.append("oppo_straight_keycyclists:  ");
  for (const auto& keyobj : oppo_straight_keycyclists) {
    debug_.append(absl::StrCat(keyobj.id, " "));
    oppo_straight_keyvehicles.push_back(keyobj);
  }
  debug_.append("\n");

  // RiskField driveline调整
  // 有图 go point 搜索方案
  std::optional<PathPoint> go_point_opt;
  RoadInfo road_info;
  const auto stations = input.drive_passage->stations();
  bool has_go_point = false;
  if (input.drive_passage->empty()) {
    debug_.append("drive_passage is not empty");
    Log2DDS::LogDataV2(name, debug_);
    driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
    output.driveline_result = std::move(driveline_result);
    return output;
  }
  int size_station = input.drive_passage->size();
  for (int i = 0; i < size_station; i++) {
    auto station = stations[StationIndex(i)];
    if (station.accumulated_s() < ego_s) {
      continue;
    }
    if (station.station_info().is_in_intersection) {
      has_go_point = true;
    } else if (!station.station_info().is_in_intersection && has_go_point) {
      auto go_station = stations[StationIndex(i - 1)];
      debug_ += absl::StrCat(" go_station.x:", go_station.xy().x(),
                             " go_station.y:", go_station.xy().y(), "\n");
      PathPoint go_point_tmp;
      go_point_tmp.set_x(go_station.xy().x());
      go_point_tmp.set_y(go_station.xy().y());
      go_point_tmp.set_theta(
          atan2(go_station.tangent().y(), go_station.tangent().x()));
      go_point_opt = go_point_tmp;
      debug_ += absl::StrCat(" go_point.x:", go_point_opt.value().x(),
                             " go_point.y:", go_point_opt.value().y(), "\n");
      break;
    }
  }

  if (!go_point_opt.has_value()) {
    // 无图 go point 搜索方案
    debug_.append("Searching Wihtout Map:");
    bool has_large_curvature = false;
    const double CURVATURE_THRESHOLD = 0.015;
    const double CURVATURE_LOWER_THRESHOLD = 0.0015;
    if (input.drive_passage->empty()) {
      debug_.append("drive_passage is empty");
      Log2DDS::LogDataV2(name, debug_);
      driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
      output.driveline_result = std::move(driveline_result);
      return output;
    }
    int size_station = input.drive_passage->size();
    // 至少需要3个点才能计算曲率
    for (int i = 2; i < size_station; i++) {
      auto station = stations[StationIndex(i)];
      if (station.accumulated_s() < ego_s) {
        continue;
      }
      // 获取连续三个点用于计算曲率
      auto p_prev = stations[StationIndex(i - 2)];
      auto p_curr = stations[StationIndex(i - 1)];
      auto p_next = stations[StationIndex(i)];
      // 提取坐标
      double x0 = p_prev.xy().x();
      double y0 = p_prev.xy().y();
      double x1 = p_curr.xy().x();
      double y1 = p_curr.xy().y();
      double x2 = p_next.xy().x();
      double y2 = p_next.xy().y();
      // 计算曲率
      // 基于三点计算曲率公式: k = 2|(x2 - x0)(y1 - y0) - (x1 - x0)(y2 - y0)| /
      // [(x2 - x0)² + (y2 - y0)²]^(3/2)
      double dx1 = x1 - x0;
      double dy1 = y1 - y0;
      double dx2 = x2 - x0;
      double dy2 = y2 - y0;
      double cross_product = dx2 * dy1 - dx1 * dy2;
      double distance_sq = dx2 * dx2 + dy2 * dy2;
      // 避免除以零
      if (distance_sq < 1e-6) {
        continue;
      }
      double curvature =
          2.0 * std::abs(cross_product) / std::pow(distance_sq, 1.5);
      // 检测到进入路口后又出现大曲率（出路口特征）
      if (curvature > CURVATURE_THRESHOLD) {
        has_large_curvature = true;
      }
      if (has_large_curvature && curvature < CURVATURE_LOWER_THRESHOLD) {
        // 将该点作为go point
        auto go_station = p_curr;
        PathPoint go_point_tmp;
        go_point_tmp.set_x(go_station.xy().x());
        go_point_tmp.set_y(go_station.xy().y());
        go_point_tmp.set_theta(
            atan2(go_station.tangent().y(), go_station.tangent().x()));
        go_point_opt = go_point_tmp;
        debug_ += absl::StrCat(
            "Found go point via curvature.\n x:", go_point_opt.value().x(),
            " y:", go_point_opt.value().y(), " curvature:", curvature, "\n");
        break;
      }
    }
  }

  if (!go_point_opt.has_value()) {
    debug_.append("do not have go point");
    Log2DDS::LogDataV2(name, debug_);
    driveline_result.set_driveline_status(DrivelineStatus::INIT_CHECK_FAILED);
    output.driveline_result = std::move(driveline_result);
    return output;
  }
  auto go_point = go_point_opt.value();
  constexpr double start_point_buffer = -1.5;
  constexpr double go_point_buffer = 0.5;
  Vec2d go_point_y_unit(-sin(go_point.theta()), cos(go_point.theta()));
  road_info.go_point.set_x(go_point.x() +
                           go_point_buffer * go_point_y_unit.x());
  road_info.go_point.set_y(go_point.y() +
                           go_point_buffer * go_point_y_unit.y());
  road_info.go_point_heading = NormalizeAngle(go_point.theta());
  Vec2d start_point_y_unit(-sin(start_point.theta()), cos(start_point.theta()));
  road_info.start_point.set_x(start_point.x() +
                              start_point_buffer * start_point_y_unit.x());
  road_info.start_point.set_y(start_point.y() +
                              start_point_buffer * start_point_y_unit.y());
  road_info.start_point_heading = NormalizeAngle(start_point.theta());
  road_info.start_point_is_valid = true;
  if (!road_info.start_point_is_valid) {
    debug_.append("start point is not valid\n");
  }
  std::unordered_set<std::string> syn_yield_obj_ids;
  std::unordered_set<std::string> oppo_yield_obj_ids;

  PiecewiseLinearFunction<double> min_long_look_forward_acc_plf =
      PiecewiseLinearFunction<double>({5.0, 10.0, 15.0, 30.0, 40.0},
                                      {-0.5, -0.3, 0.0, 0.3, 0.3});

  PiecewiseLinearFunction<double> min_long_acc_plf =
      PiecewiseLinearFunction<double>({5.0, 10.0, 15.0, 30.0, 40.0},
                                      {-1.0, -0.3, 0.0, 0.3, 0.5});

  PiecewiseLinearFunction<double> min_lat_acc_plf =
      PiecewiseLinearFunction<double>({5.0, 10.0, 15.0, 30.0, 40.0},
                                      {-1.0, -0.3, 0.0, 0.3, 0.5});

  // PiecewiseLinearFunction<double> left_nudge_compensate_acc_plf =
  //     PiecewiseLinearFunction<double>({-0.84 * M_PI, -0.5 * M_PI}, {-0.5,
  //     0.0});
  // left_nudge_compensate_acc_ = left_nudge_compensate_acc_plf(
  //     NormalizeAngle(road_info.go_point_heading -
  //     road_info.start_point_heading));
  PiecewiseLinearFunction<double> oppo_right_nudge_compensate_acc_plf =
      PiecewiseLinearFunction<double>({0.5 * M_PI, 0.84 * M_PI}, {0.5, 0.0});
  oppo_right_nudge_compensate_acc_ =
      oppo_right_nudge_compensate_acc_plf(NormalizeAngle(
          road_info.go_point_heading - road_info.start_point_heading));
  debug_.append(absl::StrCat("sync right nudge compensate acc ",
                             oppo_right_nudge_compensate_acc_, "\n"));
  PiecewiseLinearFunction<double> sync_right_nudge_compensate_acc_plf =
      PiecewiseLinearFunction<double>({0.5 * M_PI, 0.84 * M_PI}, {0.0, 0.0});
  sync_right_nudge_compensate_acc_ =
      sync_right_nudge_compensate_acc_plf(NormalizeAngle(
          road_info.go_point_heading - road_info.start_point_heading));
  debug_.append(absl::StrCat("oppo right nudge compensate acc ",
                             oppo_right_nudge_compensate_acc_, "\n"));

  PiecewiseLinearFunction<double> oppo_left_nudge_compensate_acc_plf =
      PiecewiseLinearFunction<double>({0.5 * M_PI, 0.84 * M_PI}, {-0.5, 0.0});
  oppo_left_nudge_compensate_acc_ =
      oppo_left_nudge_compensate_acc_plf(NormalizeAngle(
          road_info.go_point_heading - road_info.start_point_heading));
  debug_.append(absl::StrCat("sync right nudge compensate acc ",
                             oppo_left_nudge_compensate_acc_, "\n"));
  PiecewiseLinearFunction<double> sync_left_nudge_compensate_acc_plf =
      PiecewiseLinearFunction<double>({0.5 * M_PI, 0.84 * M_PI}, {-0.5, 0.0});
  sync_left_nudge_compensate_acc_ =
      sync_left_nudge_compensate_acc_plf(NormalizeAngle(
          road_info.go_point_heading - road_info.start_point_heading));
  debug_.append(absl::StrCat("oppo right nudge compensate acc ",
                             sync_left_nudge_compensate_acc_, "\n"));
  // PiecewiseLinearFunction<double> forward_left_nudge_compensate_acc_plf =
  //     PiecewiseLinearFunction<double>({-0.84 * M_PI, -0.5 * M_PI}, {-0.5,
  //     0.0});
  // forward_left_nudge_compensate_acc_ = forward_left_nudge_compensate_acc_plf(
  //     NormalizeAngle(road_info.go_point_heading -
  //     road_info.start_point_heading));
  debug_.append(absl::StrCat("left nudge compensate acc ",
                             sync_left_nudge_compensate_acc_, "\n"));
  // PiecewiseLinearFunction<double> forward_right_nudge_compensate_acc_plf =
  //     PiecewiseLinearFunction<double>({0.5 * M_PI, 0.84 * M_PI}, {0.0, 0.0});
  // forward_right_nudge_compensate_acc_ =
  // forward_right_nudge_compensate_acc_plf(
  //     NormalizeAngle(road_info.go_point_heading -
  //     road_info.start_point_heading));

  enm_risk_virtual_time_ = 1.0;
  use_longitude_risk_ = true;
  if (input.last_driveline_result) {
    debug_.append(absl::StrCat(
        "last trigger failed counter: ",
        input.last_driveline_result->trigger_failed_counter(), "\n"));
  }

  bool is_syn_trigger = false;
  bool is_oppo_trigger = false;
  if (drf_trigger_.TriggerLtRiskFieldDriveline(
          syn_left_turn_keyvehicles, syn_left_turn_keycyclists,
          *input.obs_history, *input.init_path, input.last_driveline_result,
          &driveline_result, &syn_yield_obj_ids)) {
    is_syn_trigger = true;
  }

  if (drf_trigger_.TriggerRiskField(oppo_straight_keyvehicles,
                                    *input.obs_history, *input.init_path,
                                    input.last_driveline_result,
                                    &driveline_result, &oppo_yield_obj_ids) &&
      (ego_s > start_s)) {
    is_oppo_trigger = true;
  }
  if (is_syn_trigger || is_oppo_trigger) {
    if (syn_yield_obj_ids.empty()) {
      use_longitude_risk_ = false;
    }
    std::unordered_set<std::string> risk_obj_ids = syn_yield_obj_ids;
    for (auto& keyobj : syn_left_turn_keyvehicles) {
      if (syn_yield_obj_ids.find(keyobj.id) != syn_yield_obj_ids.end()) {
        keyobj.obj_decision_type = ObjDecisionType::YIELD;
      } else {
        keyobj.obj_decision_type = ObjDecisionType::PASS;
      }
    }

    for (auto& keyobj : oppo_straight_keyvehicles) {
      if (oppo_yield_obj_ids.find(keyobj.id) != oppo_yield_obj_ids.end()) {
        keyobj.obj_decision_type = ObjDecisionType::YIELD;
      } else {
        keyobj.obj_decision_type = ObjDecisionType::PASS;
      }
    }
    std::vector<ApolloTrajectoryPointProto> drf_driveline;
    std::vector<Vec2d> rf_driveline_keypoints;
    std::string most_risk_obj_id;
    // add obj_lat_decision
    // TODO 具体的变量名确定之后再改
    for (auto& p : syn_left_turn_keyvehicles) {
      p.obj_lat_decision = ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE;
    }
    for (auto& p : oppo_straight_keyvehicles) {
      p.obj_lat_decision = ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE;
    }
    if (GenerateRiskFieldDriveline(
            av_state, road_info, *input.init_path, syn_left_turn_keyvehicles,
            oppo_straight_keyvehicles, syn_yield_obj_ids, oppo_yield_obj_ids,
            &most_risk_obj_id, &drf_driveline, &rf_driveline_keypoints,
            debug_)) {
      //构造输出
      // DRFDrivelineOutput output;
      driveline_result.set_most_risk_obj(most_risk_obj_id);
      driveline_result.set_most_risk_counter(0);
      driveline_result.set_driveline_status(DrivelineStatus::GENERATE_SUCCEED);
      output.dynamic_drive_line = std::move(drf_driveline);
      output.driveline_result = driveline_result;
      Log2DDS::LogDataV2(name, debug_);
    } else {
      driveline_result.set_driveline_status(DrivelineStatus::GENERATE_FAILED);
      output.driveline_result = std::move(driveline_result);
      debug_.append("generate driveline failed");
      Log2DDS::LogDataV2(name, debug_);
    }
  } else {
    debug_.append("don't trigge risk field");
    Log2DDS::LogDataV2(name, debug_);
    output.driveline_result = std::move(driveline_result);
  }
  return output;
}
bool DRFDynamicDrivelineGenerator::GenerateRiskFieldDriveline(
    const VehicleState& av_state, const RoadInfo& road_info,
    const DiscretizedPath& init_path,
    const std::vector<RiskFieldKeyobj>& syn_leftturn_keyobjs,
    const std::vector<RiskFieldKeyobj>& oppo_straight_keyobjs,
    const std::unordered_set<std::string>& yield_obj_ids,
    const std::unordered_set<std::string>& oppo_yield_obj_ids,
    std::string* most_risk_obj_id,
    std::vector<ApolloTrajectoryPointProto>* const rf_driveline,
    std::vector<Vec2d>* const rf_driveline_keypoints, std::string& debug) {
  if (rf_driveline == nullptr || !IsRoadInfoValid(road_info, av_state.theta)) {
    debug.append("this two condition \n");
    return false;
  }
  // Params involving av
  const double av_front_edge_to_center =
      vehicle_geo_params_->front_edge_to_center();
  const double av_back_edge_to_center =
      vehicle_geo_params_->back_edge_to_center();
  const double av_width = vehicle_geo_params_->width();
  const double av_length = vehicle_geo_params_->length();
  const double av_half_length = 0.5 * av_length;
  const double av_half_width = 0.5 * av_width;

  int N = static_cast<int>(kSimulationHorizon / kSimulationDt);

  VehicleState av_init_state = av_state;
  std::vector<VehicleState> drf_traj = {av_init_state};
  double t = 0.0;

  // Get the key obstacle pred-traj ptr
  const SpacetimeObjectTrajectory* key_obj_ptr;
  double key_obj_length = 0.0;
  double key_obj_width = 0.0;
  //找lat risk前五的障碍物
  auto most_risk_syn_keyobjs = ChoseMostRiskKeyobjs(
      syn_leftturn_keyobjs, av_state, yield_obj_ids,
      ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE, debug);
  auto most_risk_oppo_keyobjs = ChoseMostRiskKeyobjs(
      oppo_straight_keyobjs, av_state, oppo_yield_obj_ids,
      ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE, debug);
  if ((!most_risk_syn_keyobjs.has_value() || most_risk_syn_keyobjs->empty()) &&
      (!most_risk_oppo_keyobjs.has_value() ||
       most_risk_oppo_keyobjs->empty())) {
    debug_.append("do not have any most_risk_keyobj\n");
    return false;
  }
  constexpr int kKeyObjectNums = 5;
  std::vector<RiskFieldKeyobj> most_risk_keyobjs;
  most_risk_keyobjs.reserve(kKeyObjectNums);
  // 当两类obj都有值时，while循环到全部遍历或者满足5个
  // 当仅某类obj有值时，将该类obj赋值到目标中
  if ((most_risk_syn_keyobjs.has_value() && !most_risk_syn_keyobjs->empty()) &&
      (most_risk_oppo_keyobjs.has_value() &&
       !most_risk_oppo_keyobjs->empty())) {
    int num_of_objs = 0;
    int index_of_syn_obj = 0;
    int index_of_oppo_obj = 0;
    while (num_of_objs < kKeyObjectNums) {
      // 两类均有obj未遍历
      if (index_of_syn_obj < most_risk_syn_keyobjs->size() &&
          index_of_oppo_obj < most_risk_oppo_keyobjs->size()) {
        auto& syn_pair = (*most_risk_syn_keyobjs)[index_of_syn_obj];
        auto& oppo_pair = (*most_risk_oppo_keyobjs)[index_of_oppo_obj];
        if (syn_pair.first > oppo_pair.first) {
          most_risk_keyobjs.push_back(*syn_pair.second);
          index_of_syn_obj++;
        } else {
          most_risk_keyobjs.push_back(*oppo_pair.second);
          index_of_oppo_obj++;
        }
        num_of_objs++;
      } else if (index_of_syn_obj < most_risk_syn_keyobjs->size()) {
        auto& syn_pair = (*most_risk_syn_keyobjs)[index_of_syn_obj];
        most_risk_keyobjs.push_back(*syn_pair.second);
        index_of_syn_obj++;
        num_of_objs++;
      } else if (index_of_oppo_obj < most_risk_oppo_keyobjs->size()) {
        auto& oppo_pair = (*most_risk_oppo_keyobjs)[index_of_oppo_obj];
        most_risk_keyobjs.push_back(*oppo_pair.second);
        index_of_oppo_obj++;
        num_of_objs++;
      } else {
        break;
      }
    }
  } else if ((most_risk_syn_keyobjs.has_value() &&
              !most_risk_syn_keyobjs->empty())) {
    for (const auto& p : *most_risk_syn_keyobjs) {
      most_risk_keyobjs.push_back(*p.second);
      if (most_risk_keyobjs.size() >= kKeyObjectNums) {
        break;
      }
    }
  } else {
    for (const auto& p : *most_risk_oppo_keyobjs) {
      most_risk_keyobjs.push_back(*p.second);
      if (most_risk_keyobjs.size() >= kKeyObjectNums) {
        break;
      }
    }
  }
  const auto& highest_risk_obj = most_risk_keyobjs.front();
  *most_risk_obj_id = highest_risk_obj.id;
  debug_.append(
      absl::StrCat("most_risk_keyobj id: ", highest_risk_obj.id, "\n"));
  int pred_traj_size = 0;
  key_obj_ptr = highest_risk_obj.object_ptr;
  const auto& key_obj_traj_points = key_obj_ptr->trajectory().points();
  bool has_key_obj_in_front = key_obj_traj_points.size() > 2;
  double pred_dt = 0.2;
  if (has_key_obj_in_front) {
    pred_dt = key_obj_traj_points.at(1).t() - key_obj_traj_points.at(0).t();
  }
  if (pred_dt < 0.05) {
    has_key_obj_in_front = false;
  }

  if (!has_key_obj_in_front) {
    debug_.append("do not have key_obj_in_front");
    return false;
  }

  pred_traj_size = static_cast<int>(key_obj_traj_points.size());
  const auto& shape = key_obj_ptr->bounding_box();
  key_obj_length = std::max(shape.length(), 4.0);
  key_obj_width = shape.width();

  // Simulation. Warning: here the kSimulationDt must be interger-multiple times
  // of pred_dt
  const auto& drive_line_end_point = init_path.back();
  double ocp_driveline_end_point_heading = drive_line_end_point.theta();
  Vec2d ocp_end_pt(drive_line_end_point.x(), drive_line_end_point.y());
  double square_dist_to_drive_line_end_point = 0.0;
  double square_dist_to_go_point = 0.0;
  auto ocp_driveline_road_info = road_info;
  ocp_driveline_road_info.go_point = ocp_end_pt;

  ocp_driveline_road_info.go_point_heading = ocp_driveline_end_point_heading;
  auto go_point_lat_decision =
      CalcGopointEnvRiskLatDecision(ocp_driveline_road_info);
  auto start_point_lat_decision =
      CalcStartpointEnvRiskLatDecision(ocp_driveline_road_info);
  // if (go_point_lat_decision ==
  // ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE)
  // {
  //     debug_ += "go_point_lat_decision left nudge\n";
  // }
  // else
  // {
  //     debug_ += "go_point_lat_decision right nudge\n";
  // }

  // if (start_point_lat_decision ==
  // ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE)
  // {
  //     debug_ += "start_point_lat_decision left nudge\n";
  // }
  // else if (start_point_lat_decision ==
  // ObjectDecisionType::OBJECT_DECISION_TYPE_NONE)
  // {
  //     debug_ += "start_point_lat_decision no nudge\n";
  // }
  // else
  // {
  //     debug_ += "start_point_lat_decision right nudge\n";
  // }
  if (go_point_lat_decision != start_point_lat_decision) {
    debug_.append(
        "start point and go point lat decision are not consistency\n");
    return false;
  }
  // for(const auto& key_obj : most_risk_keyobjs)
  // {
  //     std::string obj_behavior;
  //     if(key_obj.obj_behavior == ObjBehavior::SYNC_LEFT_TURN){
  //         obj_behavior = "SYNC_LEFT_TURN";
  //     }else if(key_obj.obj_behavior == ObjBehavior::OPPO_GO_STRAIGHT){
  //         obj_behavior = "OPPO_GO_STRAIGHT";
  //     }
  //     debug_.append(absl::StrCat("obj_id: ",key_obj.id, "obj_behavior:
  //     ",obj_behavior));
  // }
  for (int i = 0; i < N; i++) {
    const auto& av_state_i = drf_traj.at(i);
    square_dist_to_drive_line_end_point =
        SquareDist(av_state_i.x, av_state_i.y, drive_line_end_point.x(),
                   drive_line_end_point.y());
    square_dist_to_go_point =
        SquareDist(av_state_i.x, av_state_i.y, road_info.go_point.x(),
                   road_info.go_point.y());
    // Break when state i have near go point
    if (square_dist_to_drive_line_end_point < kEndEpsilonSquare ||
        square_dist_to_go_point < kEndEpsilonSquare) {
      break;
    }
    // Box2d(const Vec2d& center, double heading, double half_length_front,
    // double half_length_back,
    //     double half_width_left, double half_width_right);
    const Box2d av_box_i(Vec2d(av_state_i.x, av_state_i.y), av_state_i.theta,
                         av_length, av_width);
    // Consider the agent(obstacle), the first key-vehicle
    t = static_cast<double>(i) * kSimulationDt;
    // debug_ += "********************************************  \n";
    // To Do, calculate the action for pure pursuit:
    VehicleAction pure_pursuit_action =
        CalcActionByPurePursuit(av_state_i, init_path, &debug);
    VehicleAction action_i = pure_pursuit_action;
    // To Do:
    ActionRange drf_action_range;
    int pred_idx = static_cast<int>(t / pred_dt);
    if (pred_idx <
        pred_traj_size)  // TODO：有问题，每个预测轨迹点不一定一样,且静止障碍物就没有轨迹点
    {
      for (const auto& key_obj : most_risk_keyobjs) {
        // debug_ += "simulation with obj " + key_obj.id + "\n";
        key_obj_ptr = key_obj.object_ptr;
        const auto& pred_pt = key_obj_ptr->trajectory().points().at(pred_idx);
        double obs_center_x = pred_pt.pos().x();
        double obs_center_y = pred_pt.pos().y();
        double obs_heading = pred_pt.theta();
        double obs_v = pred_pt.v();
        const Box2d obj_box_i(Vec2d(obs_center_x, obs_center_y), obs_heading,
                              key_obj_length, key_obj_width);
        if (key_obj.obj_decision_type == ObjDecisionType::YIELD) {
          use_lookforward_risk_ = true;
          use_longitude_risk_ = true;
          use_lat_virtual_time_ = true;
        } else {
          use_lookforward_risk_ = false;
          use_longitude_risk_ = false;
          use_lat_virtual_time_ = false;
        }
        drf_action_range = CalcActionRangeForOneAgent(
            road_info, av_box_i, av_state_i, pure_pursuit_action, obj_box_i,
            obs_v, key_obj.obj_behavior, key_obj.obj_lat_decision,
            go_point_lat_decision, start_point_lat_decision, t, &debug);
        debug_ += absl::StrCat("pp_res: dkappa: ", action_i.dkappa,
                               " target_kappa: ", action_i.target_kappa);
        action_i = AdjustActionByRiskActionRange(action_i, drf_action_range);
        debug_ += absl::StrCat(" adjust_res: dkappa: ", action_i.dkappa,
                               " target_kappa: ", action_i.target_kappa, "\n");
      }
    } else {
      break;
    }
    AddLateralAccConstraint(av_state_i, action_i);
    // To Do end;
    VehicleState av_state_i_plus = CalcNextVehicleState(av_state_i, action_i);
    drf_traj.emplace_back(std::move(av_state_i_plus));
  }
  const auto& end_state = drf_traj.back();
  square_dist_to_drive_line_end_point =
      SquareDist(end_state.x, end_state.y, drive_line_end_point.x(),
                 drive_line_end_point.y());
  // Use pp to track path until near basic driveline endpoint
  int count = 0;
  constexpr int kMaxIterTime = 100;
  Vec2d pp_road_info_y_unit(-sin(road_info.go_point_heading),
                            cos(road_info.go_point_heading));
  double go_point_free_buffer = 0.0;
  if (go_point_lat_decision ==
      ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
    go_point_free_buffer = 1.0;
  } else {
    go_point_free_buffer = -1.5;
  }

  // Convert vehicle state to driveline
  for (const auto& key_obj : most_risk_keyobjs) {
    if (key_obj.obj_decision_type == ObjDecisionType::YIELD) {
      if (start_point_lat_decision ==
          ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
        CalcRightNudgeDrivelineKeypoints(drf_traj, rf_driveline_keypoints,
                                         road_info.start_point_heading);
      }
      break;
    }
  }
  TrajToDriveline(drf_traj, rf_driveline);
  double dt = 0.1;
  ReSampleDriveline(*rf_driveline, dt);
  if (rf_driveline->size() < 10 ||
      rf_driveline->back().path_point().s() < 3.0) {
    debug_.append("driveline leq 3.0 or point leq 10 \n");
    return false;
  }
  return true;
}

// PurePursuitAction is the action calculated from the purpursuit algorithm,
// would be an input for the simulation
ActionRange DRFDynamicDrivelineGenerator::CalcActionRangeForOneAgent(
    const RoadInfo& road_info, const Box2d& av_box,
    const VehicleState& av_state, const VehicleAction& PurePursuitAction,
    const Box2d& agent_box, double agent_speed, const ObjBehavior& obj_behavior,
    const ObjectDecisionType& obj_lateral_decision,
    const ObjectDecisionType& go_point_lat_decision,
    const ObjectDecisionType& start_point_lat_decision, double cur_simu_time,
    std::string* debug) {
  // Compute the pose-info in the vcs of av
  const auto agent_pose_info = RiskField::CalcAgentPoseInfo(av_box, agent_box);
  const auto enm_info = RiskField::CalEnvInfo(road_info);
  const double av_speed = av_state.v;

  // Consider enm risk
  const auto go_point_enm_risk = RiskField::CalGoPointEnvRisk(
      av_box, av_speed, enm_info, go_point_lat_decision, enm_risk_virtual_time_,
      debug);
  const auto action_range_go_point_enm_risk = CalcActionRangeForEnvRisk(
      go_point_enm_risk, av_state, PurePursuitAction, go_point_lat_decision,
      kMinTargetAForEnvRisk, debug);
  const auto start_point_enm_risk = RiskField::CalStartPointEnvRisk(
      av_box, av_speed, enm_info, start_point_lat_decision,
      enm_risk_virtual_time_, debug);
  const auto action_range_start_point_enm_risk = CalcActionRangeForEnvRisk(
      start_point_enm_risk, av_state, PurePursuitAction,
      start_point_lat_decision, kMinTargetAForEnvRisk, debug);
  double go_point_risk_val = go_point_enm_risk.risk;
  // Consider the longitudinal risk:
  // To Do, alse consider the case for PASS not only yield
  ActionRange action_range_lon_risk;
  action_range_lon_risk.max_jerk = 10.0;
  action_range_lon_risk.min_jerk = -10.0;
  action_range_lon_risk.max_dkappa = kMaxDkappa;
  action_range_lon_risk.min_dkappa = -kMaxDkappa;
  if (use_longitude_risk_) {
    const auto lon_risk = RiskField::CalcAgentLongitudinalRiskForCrossYield(
        av_box, av_speed, agent_pose_info, agent_speed, debug);
    const auto& min_long_acc_plf = PiecewiseLinearFunction<double>(
        {5.0, 10.0, 15.0, 30.0, 40.0},
        {-0.5, -0.3, 0.0, 0.3, 0.3});  // Value comfirmed
    action_range_lon_risk = CalcActionRangeForLongitudinalCrossYield(
        lon_risk, go_point_risk_val, av_state, agent_pose_info, agent_speed,
        PurePursuitAction, obj_lateral_decision, obj_behavior, min_long_acc_plf,
        0.0, debug);
  }
  // Consider lookforward risk
  VehicleState look_forward_av_state = av_state;
  look_forward_av_state.kappa = PurePursuitAction.target_kappa;

  // Consider the lateral risk:
  const auto lat_risk = RiskField::CalcAgentLateralRisk(
      av_box, av_speed, agent_pose_info, agent_speed, obj_lateral_decision,
      obj_behavior, use_lat_virtual_time_, debug);
  auto action_range_lat_risk = CalcActionRangeForLateralRisk(
      lat_risk, go_point_risk_val, av_state, agent_pose_info, agent_speed,
      PurePursuitAction, obj_lateral_decision, obj_behavior, debug);
  // debug_ += "lon_risk: " + lon_risk.Debug();
  // debug_ += "action_range_lon_risk: " + action_range_lon_risk.Debug();
  // debug_ += "max_jerk: " + std::to_string(action_range_lon_risk.max_jerk) +
  //           " min_jerk: " + std::to_string(action_range_lon_risk.min_jerk) +
  //           "\n";
  // debug_ += "look_forward_range: " +
  // action_range_look_forward_lon_risk.Debug(); debug_ +=
  // "look_forward_pose_info: " + look_forward_pose_info.Debug(); debug_ +=
  // "lat_risk: " + lat_risk.Debug(); debug_ == "look_forward_risk: " +
  // look_forward_lat_risk.Debug(); debug_ += "action_range_lat_risk: " +
  // action_range_lat_risk.Debug(); debug_ += "max_jerk: " +
  // std::to_string(action_range_lat_risk.max_jerk) +
  //           " min_jerk: " + std::to_string(action_range_lat_risk.min_jerk) +
  //           " max_dkappa: " +
  //           std::to_string(action_range_lat_risk.max_dkappa) + " min_dkappa:
  //           " + std::to_string(action_range_lat_risk.min_dkappa) + "\n";
  ActionRange action_range;
  action_range.max_dkappa = std::min(action_range_lon_risk.max_dkappa,
                                     action_range_lat_risk.max_dkappa);
  action_range.min_dkappa = std::max(action_range_lon_risk.min_dkappa,
                                     action_range_lat_risk.min_dkappa);
  action_range.max_jerk =
      std::min(action_range_lon_risk.max_jerk, action_range_lat_risk.max_jerk);
  action_range.min_jerk =
      std::max(action_range_lon_risk.min_jerk, action_range_lat_risk.min_jerk);

  if (use_lookforward_risk_ && use_longitude_risk_) {
    const auto look_forward_pose_info = RiskField::CalcLookForwardPoseInfo(
        look_forward_av_state, av_box, agent_box, cur_simu_time,
        enm_info.go_point_heading);
    const auto look_forward_risk =
        RiskField::CalcAgentLookForwardLongitudinalRiskForCrossYield(
            av_box, av_speed, look_forward_av_state.kappa,
            look_forward_pose_info, agent_speed, debug);
    const auto& min_long_look_forward_acc_plf = PiecewiseLinearFunction<double>(
        {5.0, 10.0, 15.0, 30.0, 40.0},
        {-0.5, -0.3, 0.0, 0.3, 0.3});  // Value confirmed
    const auto action_range_look_forward_lon_risk =
        CalcActionRangeForLookForwardLongitudinalCrossYield(
            look_forward_risk, go_point_risk_val, av_state,
            look_forward_pose_info, agent_speed, PurePursuitAction,
            obj_lateral_decision, obj_behavior, min_long_look_forward_acc_plf,
            look_forward_av_state.kappa, debug);

    action_range.max_dkappa = std::min(
        action_range_look_forward_lon_risk.max_dkappa, action_range.max_dkappa);
    action_range.min_dkappa = std::max(
        action_range_look_forward_lon_risk.min_dkappa, action_range.min_dkappa);
    action_range.max_jerk = std::min(
        action_range_look_forward_lon_risk.max_jerk, action_range.max_jerk);
    action_range.min_jerk = std::max(
        action_range_look_forward_lon_risk.min_jerk, action_range.min_jerk);
  }
  action_range = AdjustActionByEnvRiskActionRange(
      go_point_enm_risk, go_point_lat_decision, action_range,
      action_range_go_point_enm_risk);
  action_range = AdjustActionByEnvRiskActionRange(
      start_point_enm_risk, start_point_lat_decision, action_range,
      action_range_start_point_enm_risk);
  return action_range;
}

ActionRange
DRFDynamicDrivelineGenerator::CalcActionRangeForLongitudinalCrossYield(
    const Risk& lon_risk, double go_point_risk_val,
    const VehicleState& av_state, const AgentPoseInfo& agent_pose_info,
    double agent_speed, const VehicleAction& PurePursuitAction,
    const ObjectDecisionType& obj_lateral_decision,
    const ObjBehavior& obj_behavior,
    const PiecewiseLinearFunction<double>& min_acc_plf,
    double look_forward_kappa, std::string* debug) {
  ActionRange lon_action_range;
  lon_action_range.max_jerk = 10.0;
  lon_action_range.min_jerk = -10.0;
  lon_action_range.max_dkappa = kMaxDkappa;
  lon_action_range.min_dkappa = -kMaxDkappa;
  // If there is no risk, a wide range will be the output
  if (lon_risk.risk >= 0.0) {
    const double& risk = lon_risk.risk;
    const double& risk_x = lon_risk.risk_x;
    const double& risk_v = lon_risk.risk_v;
    const double& risk_theta = lon_risk.risk_theta;
    const double& risk_t = lon_risk.risk_t;
    const double& d1 = agent_pose_info.av_distance_to_conflict_zone;
    const double& v1 = av_state.v;
    const double& d2 = agent_pose_info.agent_distance_to_leave_conflict_zone;
    const double& v2 = agent_speed;
    const double& pure_pursuit_kappa = PurePursuitAction.target_kappa;
    const double target_dot_theta =
        (pure_pursuit_kappa - look_forward_kappa) * v1;
    const double dot_theta = v1 * av_state.kappa;

    double target_kappa_min = kMinKappa;
    double target_kappa_max = kMaxKappa;
    double dkappa_min = -kMaxDkappa;
    double dkappa_max = kMaxDkappa;
    double agent_leave_conflict_zone_time = d2 / std::max(0.5, v2);
    double virtual_time = std::min(d1 / std::max(kMinVehicleSpeed, v1),
                                   d2 / std::max(kMinVehicleSpeed, v2));
    virtual_time = std::max(virtual_time, kMinVirtualTimeForLonRisk);
    double dot_risk = -risk / virtual_time;
    // We hope that risk_x * v + risk_v * a + risk_theta * dotTheta + risk_t <=
    // dot_risk
    double risk_res = dot_risk - risk_x * v1 - risk_t;
    double target_a = (risk_res - risk_theta * target_dot_theta) * risk_v /
                      (risk_v * risk_v + kEpsilon);
    target_a = std::min(kMaxTargetAForLonRisk, target_a);
    const double longitude_free_space = -go_point_risk_val;
    double compensate_target_acc =
        GetCompensateTargetAcc(obj_lateral_decision, obj_behavior);
    target_a = std::max(
        min_acc_plf(longitude_free_space) + compensate_target_acc, target_a);

    const PiecewiseLinearFunction<double>
        vritual_time_abs_max_critical_kappa_plf({0.0, 2.0, 3.0, 4.0, 5.0},
                                                {0.01, 0.0, 0.00, 0.00, 0.0});
    double abs_max_critical_kappa =
        vritual_time_abs_max_critical_kappa_plf(agent_leave_conflict_zone_time);
    if (risk_v * target_a + risk_theta * target_dot_theta >= risk_res) {
      double critical_dot_theta = (risk_res - risk_v * target_a) * risk_theta /
                                  (risk_theta * risk_theta + kEpsilon);
      double critical_target_kappa =
          look_forward_kappa +
          critical_dot_theta / std::max(v1, kMinVehicleSpeed);
      critical_target_kappa = std::max(critical_target_kappa, kMinKappa);
      critical_target_kappa = std::min(critical_target_kappa, kMaxKappa);
      if (risk_theta <= -kEpsilon) {
        target_kappa_min =
            std::min(critical_target_kappa, abs_max_critical_kappa);
      } else if (risk_theta >= kEpsilon) {
        target_kappa_max =
            std::max(critical_target_kappa, -abs_max_critical_kappa);
      } else {
        ;
      }
    }

    dkappa_min =
        CalcTargetDkappa(av_state, target_kappa_min, 0.5 * virtual_time);
    dkappa_max =
        CalcTargetDkappa(av_state, target_kappa_max, 0.5 * virtual_time);
    // Consider the lateral jerk

    // To Do, calculate the min and max jerk
    if (obj_lateral_decision ==
        ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
      lon_action_range.max_jerk =
          CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
      lon_action_range.min_jerk = -10.0;
      lon_action_range.max_dkappa = dkappa_max;
      lon_action_range.min_dkappa = std::min(dkappa_min, 0.0);
    } else {
      lon_action_range.max_jerk =
          CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
      lon_action_range.min_jerk = -10.0;
      lon_action_range.max_dkappa = std::max(dkappa_max, 0.0);
      lon_action_range.min_dkappa = dkappa_min;
    }
  }

  return lon_action_range;
}

ActionRange DRFDynamicDrivelineGenerator::
    CalcActionRangeForLookForwardLongitudinalCrossYield(
        const Risk& lon_risk, double go_point_risk_val,
        const VehicleState& av_state, const AgentPoseInfo& agent_pose_info,
        double agent_speed, const VehicleAction& PurePursuitAction,
        const ObjectDecisionType& obj_lateral_decision,
        const ObjBehavior& obj_behavior,
        const PiecewiseLinearFunction<double>& min_acc_plf,
        double look_forward_kappa, std::string* debug) {
  ActionRange lon_action_range;
  lon_action_range.max_jerk = 10.0;
  lon_action_range.min_jerk = -10.0;
  lon_action_range.max_dkappa = kMaxDkappa;
  lon_action_range.min_dkappa = -kMaxDkappa;
  // If there is no risk, a wide range will be the output
  if (lon_risk.risk >= 0.0) {
    const double& risk = lon_risk.risk;
    const double& risk_x = lon_risk.risk_x;
    const double& risk_v = lon_risk.risk_v;
    const double& risk_theta = lon_risk.risk_theta;
    const double& risk_t = lon_risk.risk_t;
    const double& d1 = agent_pose_info.av_distance_to_conflict_zone;
    const double& v1 = av_state.v;
    const double& d2 = agent_pose_info.agent_distance_to_leave_conflict_zone;
    const double& v2 = agent_speed;
    const double& pure_pursuit_kappa = PurePursuitAction.target_kappa;
    const double target_dot_theta =
        (pure_pursuit_kappa - look_forward_kappa) * v1;
    const double dot_theta = v1 * av_state.kappa;

    double target_kappa_min = kMinKappa;
    double target_kappa_max = kMaxKappa;
    double dkappa_min = -kMaxDkappa;
    double dkappa_max = kMaxDkappa;
    double virtual_time = std::min(d1 / std::max(kMinVehicleSpeed, v1),
                                   d2 / std::max(kMinVehicleSpeed, v2));
    virtual_time = std::max(virtual_time, kMinVirtualTimeForLonRisk);
    double dot_risk = -risk / virtual_time;
    // We hope that risk_x * v + risk_v * a + risk_theta * dotTheta + risk_t <=
    // dot_risk
    double risk_res = dot_risk - risk_x * v1 - risk_t;
    double target_a = (risk_res - risk_theta * target_dot_theta) * risk_v /
                      (risk_v * risk_v + kEpsilon);
    target_a = std::min(kMaxTargetAForLonRisk, target_a);
    const double longitude_free_space = -go_point_risk_val;
    double compensate_target_acc =
        GetCompensateTargetAcc(obj_lateral_decision, obj_behavior);
    target_a = std::max(
        min_acc_plf(longitude_free_space) + compensate_target_acc, target_a);

    if (risk_v * target_a + risk_theta * target_dot_theta >= risk_res) {
      double critical_dot_theta = (risk_res - risk_v * target_a) * risk_theta /
                                  (risk_theta * risk_theta + kEpsilon);
      double critical_target_kappa =
          look_forward_kappa +
          critical_dot_theta / std::max(v1, kMinVehicleSpeed);
      critical_target_kappa = std::max(critical_target_kappa, kMinKappa);
      critical_target_kappa = std::min(critical_target_kappa, kMaxKappa);
      if (risk_theta <= -kEpsilon) {
        critical_target_kappa = std::min(0.0, critical_target_kappa);
        target_kappa_min = critical_target_kappa;
      } else if (risk_theta >= kEpsilon) {
        critical_target_kappa = std::max(0.0, critical_target_kappa);
        target_kappa_max = critical_target_kappa;
      } else {
        ;
      }
    }

    dkappa_min =
        CalcTargetDkappa(av_state, target_kappa_min, 0.5 * virtual_time);
    dkappa_max =
        CalcTargetDkappa(av_state, target_kappa_max, 0.5 * virtual_time);
    // Consider the lateral jerk

    // To Do, calculate the min and max jerk
    if (obj_lateral_decision ==
        ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
      lon_action_range.max_jerk =
          CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
      lon_action_range.min_jerk = -10.0;
      lon_action_range.max_dkappa = dkappa_max;
      lon_action_range.min_dkappa = std::min(dkappa_min, 0.0);
    } else {
      lon_action_range.max_jerk =
          CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
      lon_action_range.min_jerk = -10.0;
      lon_action_range.max_dkappa = std::max(dkappa_max, 0.0);
      lon_action_range.min_dkappa = dkappa_min;
    }
  }

  return lon_action_range;
}

ActionRange DRFDynamicDrivelineGenerator::CalcActionRangeForLateralRisk(
    const Risk& lat_risk, double go_point_risk_val,
    const VehicleState& av_state, const AgentPoseInfo& agent_pose_info,
    double agent_speed, const VehicleAction& PurePursuitAction,
    const ObjectDecisionType& obj_lateral_decision,
    const ObjBehavior& obj_behavior, std::string* debug) {
  ActionRange lat_action_range;
  lat_action_range.max_jerk = 10.0;
  lat_action_range.min_jerk = -10.0;
  lat_action_range.max_dkappa = kMaxDkappa;
  lat_action_range.min_dkappa = -kMaxDkappa;
  // If there is no risk, a wide range will be the output
  if (lat_risk.risk >= 0.0) {
    const double& risk = lat_risk.risk;
    const double& risk_x = lat_risk.risk_x;
    const double& risk_v = lat_risk.risk_v;
    const double& risk_theta = lat_risk.risk_theta;
    const double& risk_t = lat_risk.risk_t;
    const double& d1 = agent_pose_info.av_distance_to_conflict_zone;
    const double& v1 = av_state.v;
    const double& d2 = agent_pose_info.agent_distance_to_leave_conflict_zone;
    const double& v2 = agent_speed;
    const double& pure_pursuit_kappa = PurePursuitAction.target_kappa;
    const double target_dot_theta = pure_pursuit_kappa * v1;
    const double dot_theta = v1 * av_state.kappa;
    double target_kappa_min = kMinKappa;
    double target_kappa_max = kMaxKappa;
    double dkappa_min = -kMaxDkappa;
    double dkappa_max = kMaxDkappa;
    double virtual_time = lat_risk.virtual_time;
    virtual_time = std::max(virtual_time, kMinVirtualTimeForLatRisk);
    double dot_risk = -1.2 * risk / virtual_time;
    // We hope that risk_x * v + risk_v * a + risk_theta * dotTheta + risk_t <=
    // dot_risk
    double risk_res = dot_risk - risk_x * v1 - risk_t;
    double target_a = (risk_res - risk_theta * target_dot_theta) * risk_v /
                      (risk_v * risk_v + kEpsilon);
    target_a = std::min(kMaxTargetAForLatRisk, target_a);
    const double longitude_free_space = -go_point_risk_val;
    const auto& min_acc_plf = PiecewiseLinearFunction<double>(
        {0.0, 5.0, 10.0, 15.0, 30.0, 40.0},
        {-0.5, -0.3, 0.2, 0.2, 0.3, 0.3});  // Value confirmed
    double compensate_target_acc =
        GetCompensateTargetAcc(obj_lateral_decision, obj_behavior);
    target_a = std::max(
        min_acc_plf(longitude_free_space) + compensate_target_acc, target_a);

    const PiecewiseLinearFunction<double>
        vritual_time_abs_max_critical_kappa_plf({0.0, 3.0, 4.0, 5.0},
                                                {0.05, 0.05, 0.00, 0.0});
    const PiecewiseLinearFunction<double>
        nudge_point_distance_abs_max_critical_kappa_plf({-1.5, -1.0, -0.5, 0.0},
                                                        {0.0, 0.0, 0.05, 0.05});
    if (risk_v * target_a + risk_theta * target_dot_theta >= risk_res) {
      double critical_dot_theta = (risk_res - risk_v * target_a) * risk_theta /
                                  (risk_theta * risk_theta + kEpsilon);
      double critical_target_kappa =
          critical_dot_theta / std::max(v1, kMinVehicleSpeed);
      critical_target_kappa = std::max(critical_target_kappa, kMinKappa);
      critical_target_kappa = std::min(critical_target_kappa, kMaxKappa);
      critical_target_kappa *= lat_risk.shrinkage_coeff;
      double abs_max_vritual_time_critical_kappa =
          vritual_time_abs_max_critical_kappa_plf(lat_risk.virtual_time);
      double abs_max_nudge_distance_critical_kappa =
          nudge_point_distance_abs_max_critical_kappa_plf(
              lat_risk.lat_distance_T);
      double abs_max_critical_kappa =
          std::min(abs_max_vritual_time_critical_kappa,
                   abs_max_nudge_distance_critical_kappa);
      if (risk_theta <= -kEpsilon) {
        target_kappa_min =
            std::min(critical_target_kappa, abs_max_critical_kappa);
      } else if (risk_theta >= kEpsilon) {
        target_kappa_max =
            std::max(critical_target_kappa, -abs_max_critical_kappa);
      }
      // debug_ += "lat_risk_range, critical_target_kappa " +
      // std::to_string(critical_target_kappa) +
      //           " critical_dot_theta: " + std::to_string(critical_dot_theta)
      //           + " dot_risk: " + std::to_string(dot_risk) + " risk_theta " +
      //           std::to_string(risk_theta) + " target_a "
      //           + std::to_string(target_a) + " risk_v " +
      //           std::to_string(risk_v) + " shrinkage_coeff: " +
      //           std::to_string(lat_risk.shrinkage_coeff) + "\n";
    }

    dkappa_min =
        CalcTargetDkappa(av_state, target_kappa_min, 0.5 * virtual_time);
    dkappa_max =
        CalcTargetDkappa(av_state, target_kappa_max, 0.5 * virtual_time);
    // Consider the lateral jerk

    // To Do, calculate the min and max jerk
    lat_action_range.max_jerk =
        CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
    lat_action_range.min_jerk = -10.0;
    lat_action_range.max_dkappa = dkappa_max;
    lat_action_range.min_dkappa = dkappa_min;
  }

  return lat_action_range;
}

ActionRange DRFDynamicDrivelineGenerator::CalcActionRangeForEnvRisk(
    const Risk& enm_risk, const VehicleState& av_state,
    const VehicleAction& pure_pursuit_action,
    const ObjectDecisionType& enm_lateral_decision, double target_a,
    std::string* debug) {
  ActionRange enm_action_range;
  enm_action_range.max_jerk = 10.0;
  enm_action_range.min_jerk = -10.0;
  enm_action_range.max_dkappa = kMaxDkappa;
  enm_action_range.min_dkappa = -kMaxDkappa;
  // If there is no risk, a wide range will be the output
  if (enm_risk.risk >= 0.0) {
    const double& risk = enm_risk.risk;
    const double& risk_x = enm_risk.risk_x;
    const double& risk_v = enm_risk.risk_v;
    const double& risk_theta = enm_risk.risk_theta;
    const double& risk_t = enm_risk.risk_t;
    const double& v1 = av_state.v;
    const double& pure_pursuit_kappa = pure_pursuit_action.target_kappa;
    // const double target_dot_theta = pure_pursuit_kappa * v1;
    const double dot_theta = v1 * av_state.kappa;
    double target_kappa_min = kMinKappa;
    double target_kappa_max = kMaxKappa;
    double dkappa_min = -kMaxDkappa;
    double dkappa_max = kMaxDkappa;
    double virtual_time = enm_risk.virtual_time;
    virtual_time = std::max(virtual_time, kMinVirtualTimeForEnvRisk);
    virtual_time = std::min(virtual_time, kMaxVirtualTimeForEnvRisk);
    double dot_risk = -risk / virtual_time;
    // We hope that risk_x * v + risk_v * a + risk_theta * dotTheta + risk_t <=
    // dot_risk
    double risk_res = dot_risk - risk_x * v1 - risk_t;
    // const double target_dot_theta = (risk_res - risk_v * target_a) *
    // risk_theta / (risk_theta * risk_theta + kEpsilon); double target_a =
    // (risk_res - risk_theta * target_dot_theta) * risk_v / (risk_v * risk_v +
    // kEpsilon); target_a = std::min(kMaxTargetAForEnvRisk, target_a); target_a
    // = std::max(kMinTargetAForEnvRisk, target_a);

    // if (risk_v * target_a + risk_theta * target_dot_theta >= risk_res) {
    double critical_dot_theta = (risk_res - risk_v * target_a) * risk_theta /
                                (risk_theta * risk_theta + kEpsilon);
    double critical_target_kappa = critical_dot_theta / std::max(v1, 0.1);
    critical_target_kappa = std::max(critical_target_kappa, kMinKappa);
    critical_target_kappa = std::min(critical_target_kappa, kMaxKappa);
    if (risk_theta <= -kEpsilon) {
      critical_target_kappa = std::max(0.0, critical_target_kappa);
      target_kappa_min = critical_target_kappa;
    } else if (risk_theta >= kEpsilon) {
      critical_target_kappa = std::min(0.0, critical_target_kappa);
      target_kappa_max = critical_target_kappa;
    } else {
      ;
    }
    // debug_ += "enm_risk: " + enm_risk.Debug() + "\n";
    // debug_ += "enm_risk, target_a: " + std::to_string(target_a) + "
    // critical_target_kappa " +
    //           std::to_string(critical_target_kappa) + " \n";
    // }

    dkappa_min =
        CalcTargetDkappa(av_state, target_kappa_min, 0.5 * virtual_time);
    dkappa_max =
        CalcTargetDkappa(av_state, target_kappa_max, 0.5 * virtual_time);
    // Consider the lateral jerk

    // To Do, calculate the min and max jerk
    if (enm_lateral_decision ==
        ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
      enm_action_range.max_jerk =
          CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
      enm_action_range.min_jerk = -10.0;
      enm_action_range.max_dkappa = std::max(dkappa_max, 0.01);
      enm_action_range.min_dkappa = dkappa_min;
    } else if (enm_lateral_decision ==
               ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
      enm_action_range.max_jerk =
          CalcTargetJ(av_state.a, target_a, 0.5 * virtual_time);
      enm_action_range.min_jerk = -10.0;
      enm_action_range.max_dkappa = dkappa_max;
      enm_action_range.min_dkappa = std::min(dkappa_min, -0.01);
    } else {
    }
  }

  return enm_action_range;
}

VehicleState DRFDynamicDrivelineGenerator::CalcNextVehicleState(
    const VehicleState& vehicle_state, const VehicleAction& vehicle_action) {
  VehicleState av_state_i_plus;
  const auto& av_state_i = vehicle_state;
  const auto& action_i = vehicle_action;
  const double& x = av_state_i.x;
  const double& y = av_state_i.y;
  const double& theta = av_state_i.theta;
  const double cos_theta = cos(theta);
  const double sin_theta = sin(theta);
  const double& kappa = av_state_i.kappa;
  const double& v = av_state_i.v;
  const double& a = av_state_i.a;
  const double& dkappa = action_i.dkappa;
  const double& j = action_i.jerk;

  double kappa_next = kappa + dkappa * kSimulationDt;
  double theta_next =
      theta + (kappa * v) * kSimulationDt +
      0.5 * (a * kappa + v * dkappa) * kSimulationDt2 +
      (kOneSixth * kappa * j + kOneThird * a * dkappa) * kSimulationDt3 +
      kOneEighth * dkappa * j * kSimulationDt4;
  double v_next = v + a * kSimulationDt + 0.5 * j * kSimulationDt2;
  v_next = std::min(8.0, v_next);
  v_next = std::max(v_next, 0.0);
  double a_next = a + j * kSimulationDt;
  if (v_next <= 0.1 || v_next + kEpsilon >= kSpeedLimit) {
    a_next = 0.0;
  }
  a_next = std::min(a_next, kAccLimit);
  double cos_theta_next = cos(theta_next);
  double sin_theta_next = sin(theta_next);
  double x_next =
      x + 0.5 * (v * cos_theta + v_next * cos_theta_next) * kSimulationDt;
  double y_next =
      y + 0.5 * (v * sin_theta + v_next * sin_theta_next) * kSimulationDt;
  av_state_i_plus.x = x_next;
  av_state_i_plus.y = y_next;
  av_state_i_plus.theta = theta_next;
  av_state_i_plus.kappa = kappa_next;
  av_state_i_plus.v = v_next;
  av_state_i_plus.a = a_next;

  return av_state_i_plus;
}

VehicleAction DRFDynamicDrivelineGenerator::CalcActionByPurePursuit(
    const VehicleState& vehicle_state, const DiscretizedPath& path,
    std::string* debug) {
  // ********PP from python file********
  const double lat_acc_max = 1.0;
  const double init_index = 0;
  const double kappa_ego = vehicle_state.kappa;
  const double a_ego = vehicle_state.a;
  const double enable_large_ddtheta = false;
  // res = [target_dot_theta_for_pp, ddot_theta_ego_i, target_a_ego_i, if_succ]
  auto pp_info = RfPurePursuit::CalcPathTrackingAction(
      path, vehicle_state, path.length(), init_index, lat_acc_max, debug);

  // Get final vehicle action
  VehicleAction vehicle_action;
  vehicle_action.dkappa = pp_info.pp_dkappa;
  vehicle_action.target_kappa = pp_info.pp_target_kappa;
  vehicle_action.target_a = pp_info.pp_target_acc;
  vehicle_action.jerk = CalcTargetJ(vehicle_state.a, vehicle_action.target_a,
                                    0.5 * pp_info.pp_virtual_time);
  // debug_ += "pp_target_kappa: " + std::to_string(vehicle_action.target_kappa)
  // +
  //           " pp_dkappa: " + std::to_string(vehicle_action.dkappa) + "\n";
  return vehicle_action;
}

double DRFDynamicDrivelineGenerator::CalcTargetJ(double a, double target_a,
                                                 double virtual_time) {
  double j = 0.0;
  double acc_dcc_switch_dt = 0.1;
  if ((a <= -0.1) && (target_a >= 0.1)) {
    j = -a / kAccDccSwitchDt;
  } else if ((a >= 0.1) && (target_a <= -0.1)) {
    j = -a / kAccDccSwitchDt;
  } else {
    j = (target_a - a) / std::max(virtual_time, 0.1);
    if (a >= -0.1) {
      j = std::min(j, kMaxJForAcc);
      j = std::max(j, kMinJForAcc);
    } else {
      j = std::min(j, kMaxJForDcc);
      j = std::max(j, kMinJForDcc);
    }
  }
  return j;
}

double DRFDynamicDrivelineGenerator::CalcTargetDkappa(
    const VehicleState& av_state, double target_kappa, double virtual_time) {
  double cur_kappa = av_state.kappa;
  bool need_fast_rtc = (cur_kappa < -1.0e-3 && target_kappa > 1.0e-3) ||
                       (cur_kappa > 1.0e-3 && target_kappa < -1.0e-3);

  double target_dkappa = virtual_time > std::numeric_limits<double>::epsilon()
                             ? (target_kappa - cur_kappa) / virtual_time
                             : 0.0f;
  if (need_fast_rtc) {
    target_dkappa = -cur_kappa / kFastRtcVirtualTime;
  }
  if (need_fast_rtc) {
    target_dkappa = std::min(kMaxRtcDkappa, target_dkappa);
    target_dkappa = std::max(-kMaxRtcDkappa, target_dkappa);
  } else {
    target_dkappa = std::min(kMaxDkappa, target_dkappa);
    target_dkappa = std::max(-kMaxDkappa, target_dkappa);
  }
  AddLateralJerkConstraint(av_state, need_fast_rtc, &target_dkappa);
  return target_dkappa;
}

void DRFDynamicDrivelineGenerator::ConnectOcpDriveline(
    const DiscretizedPath& ocp_driveline, std::vector<VehicleState>* rf_traj) {
  if (rf_traj == nullptr || rf_traj->empty()) {
    return;
  }
  const auto& cur_state = rf_traj->back();
  float cur_state_s = 0.0, cur_state_l = 0.0;
  const double step = 3.0;
  auto sl = ocp_driveline.XYToSL(Vec2d(cur_state.x, cur_state.y));
  cur_state_s = sl.s;
  cur_state_l = sl.l;

  double end_state_s = ocp_driveline.length();
  int step_count = std::floor(end_state_s - cur_state_s) / step;
  for (int i = 1; i < step_count; ++i) {
    float ocp_point_heading = 0.0;
    auto ocp_pt = ocp_driveline.Evaluate(cur_state_s + i * step);
    VehicleState next_state;
    next_state.x = ocp_pt.x();
    next_state.y = ocp_pt.y();
    next_state.theta = ocp_pt.theta();
    next_state.v = rf_traj->back().v;
    rf_traj->push_back(next_state);
  }
}

void DRFDynamicDrivelineGenerator::TrajToDriveline(
    const std::vector<VehicleState>& traj,
    std::vector<ApolloTrajectoryPointProto>* rf_driveline) {
  if (rf_driveline == nullptr) {
    return;
  }
  rf_driveline->reserve(traj.size());
  double cur_t = 0.0;
  double cur_s = 0.0;
  for (const auto& av_state : traj) {
    ApolloTrajectoryPointProto traj_point;
    auto path_point = traj_point.mutable_path_point();
    path_point->set_x(av_state.x);
    path_point->set_y(av_state.y);
    path_point->set_theta(av_state.theta);
    path_point->set_kappa(av_state.kappa);
    path_point->set_s(cur_s);
    traj_point.set_relative_time(cur_t);
    traj_point.set_v(av_state.v);
    traj_point.set_a(av_state.a);

    cur_t += kSimulationDt;
    cur_s += rf_driveline->empty()
                 ? 0.0
                 : std::sqrt(SquareDist(av_state.x, av_state.y,
                                        rf_driveline->back().path_point().x(),
                                        rf_driveline->back().path_point().y()));
    rf_driveline->push_back(traj_point);
  }
}

void DRFDynamicDrivelineGenerator::CalcRightNudgeDrivelineKeypoints(
    const std::vector<VehicleState>& traj,
    std::vector<Vec2d>* rf_driveline_keypoints, double start_point_heading) {
  if (rf_driveline_keypoints == nullptr) {
    return;
  }

  rf_driveline_keypoints->reserve(rf_driveline_keypoints->size() + 16);
  const auto& vehicle_state = traj.front();
  if (fabs(vehicle_state.theta - start_point_heading) >
      kRightNudgeKeypointsStartPointHeadingDiffThresh) {
    return;
  }

  Vec2d driveline_point;
  driveline_point.set_x(vehicle_state.x);
  driveline_point.set_y(vehicle_state.y);
  double last_s = 0.0;
  rf_driveline_keypoints->push_back(driveline_point);
  for (const auto& vehicle_state : traj) {
    const auto last_keypoint = rf_driveline_keypoints->back();
    Vec2d driveline_point;
    driveline_point.set_x(vehicle_state.x);
    driveline_point.set_y(vehicle_state.y);
    double cur_s = last_s + std::hypot(vehicle_state.x - last_keypoint.x(),
                                       vehicle_state.y - last_keypoint.y());
    if (cur_s - last_s < kDrivelineKeypointsResolution) {
      continue;
    }
    last_s = cur_s;
    if (cur_s > kRightNudgeDrivelineKeypointsLength ||
        fabs(vehicle_state.theta - start_point_heading) >
            kRightNudgeKeypointsStartPointHeadingDiffThresh) {
      break;
    }
    rf_driveline_keypoints->emplace_back(std::move(driveline_point));
  }
}

void DRFDynamicDrivelineGenerator::AddLateralJerkConstraint(
    const VehicleState& av_state, bool need_fast_rtc, double* dkappa) {
  if (dkappa == nullptr) {
    return;
  }
  const double max_lateral_jerk =
      need_fast_rtc ? kMaxRtcLateralJerk : kMaxLateralJerk;
  const double kappa = av_state.kappa;
  const double v = std::max(av_state.v, kMinLatConstrainVel);
  const double a = av_state.a;
  double dkappa_max_for_lateral_jerk =
      (max_lateral_jerk - 2.0 * kappa * a) / std::max(v * v, 1.0);
  double dkappa_min_for_lateral_jerk =
      (-max_lateral_jerk - 2.0 * kappa * a) / std::max(v * v, 1.0);
  *dkappa = std::min(dkappa_max_for_lateral_jerk, *dkappa);
  *dkappa = std::max(dkappa_min_for_lateral_jerk, *dkappa);
}

void DRFDynamicDrivelineGenerator::AddLateralAccConstraint(
    const VehicleState& av_state, VehicleAction& vehicle_action) {
  const double& kappa = av_state.kappa;
  const double v = std::max(av_state.v, kMinLatConstrainVel);
  const double& a = av_state.a;
  double min_jerk_for_lat_acc = kMinJerkForCurvatureSpeedLimit;
  min_jerk_for_lat_acc =
      std::max(min_jerk_for_lat_acc,
               (kMinAccForCurvatureSpeedLimit - a) / kSimulationDt);
  double next_min_speed_for_lat_acc =
      v + a * kSimulationDt + 0.5 * min_jerk_for_lat_acc * kSimulationDt2;
  next_min_speed_for_lat_acc =
      std::max(next_min_speed_for_lat_acc, kMinCurvatureSpeedLimit);
  double max_kappa_for_lat_acc = kMaxLateralAcc / (next_min_speed_for_lat_acc *
                                                   next_min_speed_for_lat_acc);
  double max_dkappa = (max_kappa_for_lat_acc - kappa) / kSimulationDt;
  double min_dkappa = (-max_kappa_for_lat_acc - kappa) / kSimulationDt;
  vehicle_action.dkappa = std::min(vehicle_action.dkappa, max_dkappa);
  vehicle_action.dkappa = std::max(vehicle_action.dkappa, min_dkappa);
}

void DRFDynamicDrivelineGenerator::AddFrictionCircleConstraint(
    const VehicleState& av_state, VehicleAction& vehicle_action) {}

VehicleAction DRFDynamicDrivelineGenerator::AdjustActionByRiskActionRange(
    const VehicleAction& action, const ActionRange& action_range) {
  VehicleAction new_action = action;
  const double& pp_dkappa = action.dkappa;
  const double& rf_dkappa_min = action_range.min_dkappa;
  const double& rf_dkappa_max = action_range.max_dkappa;
  if ((pp_dkappa >= rf_dkappa_min) && (pp_dkappa <= rf_dkappa_max)) {
    new_action.dkappa = pp_dkappa;
    new_action.jerk = std::min(new_action.jerk, action_range.max_jerk);
    new_action.jerk = std::max(new_action.jerk, action_range.min_jerk);
  } else if (pp_dkappa < rf_dkappa_min) {
    new_action.dkappa = rf_dkappa_min;
    new_action.jerk = std::min(new_action.jerk, action_range.max_jerk);
    new_action.jerk = std::max(new_action.jerk, action_range.min_jerk);
  } else {
    new_action.dkappa = rf_dkappa_max;
    new_action.jerk = std::min(new_action.jerk, action_range.max_jerk);
    new_action.jerk = std::max(new_action.jerk, action_range.min_jerk);
  }

  return new_action;
}

ActionRange DRFDynamicDrivelineGenerator::AdjustActionByEnvRiskActionRange(
    const Risk& enm_risk, const ObjectDecisionType& enm_lat_decision,
    const ActionRange& action_range, const ActionRange& enm_risk_action_range) {
  // To Do, calculate the min and max jerk
  ActionRange new_action_range = action_range;
  if (enm_risk.risk > 0.0 &&
      enm_lat_decision == ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
    new_action_range.max_dkappa =
        std::max(action_range.max_dkappa, enm_risk_action_range.max_dkappa);
    new_action_range.min_dkappa =
        std::max(action_range.min_dkappa, enm_risk_action_range.min_dkappa);
    new_action_range.max_jerk =
        std::min(action_range.max_jerk, enm_risk_action_range.max_jerk);
    new_action_range.min_jerk =
        std::max(action_range.min_jerk, enm_risk_action_range.min_jerk);
  } else if (enm_risk.risk > 0.0 &&
             enm_lat_decision ==
                 ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
    new_action_range.max_dkappa =
        std::min(action_range.max_dkappa, enm_risk_action_range.max_dkappa);
    new_action_range.min_dkappa =
        std::min(action_range.min_dkappa, enm_risk_action_range.min_dkappa);
    new_action_range.max_jerk =
        std::min(action_range.max_jerk, enm_risk_action_range.max_jerk);
    new_action_range.min_jerk =
        std::max(action_range.min_jerk, enm_risk_action_range.min_jerk);
  } else {
  }

  return new_action_range;
}

void DRFDynamicDrivelineGenerator::TrajPointToAvState(
    const ApolloTrajectoryPointProto& point, VehicleState* av_state) {
  if (av_state == nullptr) {
    return;
  }
  av_state->x = point.path_point().x();
  av_state->y = point.path_point().y();
  av_state->theta = point.path_point().theta();
  av_state->kappa = point.path_point().kappa();
  av_state->v = point.v();
  av_state->a = point.a();
}

std::optional<std::vector<std::pair<double, const RiskFieldKeyobj*>>>
DRFDynamicDrivelineGenerator::ChoseMostRiskKeyobjs(
    const std::vector<RiskFieldKeyobj>& keyobjs, const VehicleState& av_state,
    const std::unordered_set<std::string>& yield_obj_ids,
    const ObjectDecisionType& obj_lat_decision, std::string& debug) {
  // return keyobjs.front();
  const double av_front_edge_to_center =
      vehicle_geo_params_->front_edge_to_center();
  const double av_back_edge_to_center =
      vehicle_geo_params_->back_edge_to_center();
  const double av_width = vehicle_geo_params_->width();
  const double av_length = vehicle_geo_params_->length();
  const double av_half_length = 0.5 * av_length;
  const double av_half_width = 0.5 * av_width;

  const Box2d av_box(Vec2d(av_state.x, av_state.y), av_state.theta, av_length,
                     av_width);

  const double& av_speed = av_state.v;

  bool has_key_obj = false;
  std::map<double, const RiskFieldKeyobj*, std::greater<double>>
      risk_keyvehicle_map;
  std::vector<std::pair<double, const RiskFieldKeyobj*>> risk_keyvehicle_vec;
  for (const auto& keyobj : keyobjs) {
    if (keyobj.agent_hwt.has_value()) {
      double hwt_gap = keyobj.agent_hwt.value() - keyobj.ego_hwt;
      if (yield_obj_ids.find(keyobj.id) != yield_obj_ids.end()) {
        has_key_obj = true;
        const auto& pred_pt = keyobj.object_ptr->planner_object().pose();
        double obs_center_x = pred_pt.pos().x();
        double obs_center_y = pred_pt.pos().y();
        double obs_heading = pred_pt.theta();
        double obs_v = pred_pt.v();
        const Box2d agent_box(
            Vec2d(obs_center_x, obs_center_y), obs_heading,
            keyobj.object_ptr->planner_object().bounding_box().length(),
            keyobj.object_ptr->planner_object().bounding_box().width());

        const auto agent_pose_info =
            RiskField::CalcAgentPoseInfo(av_box, agent_box);

        // const auto lon_risk = CalcAgentLongitudinalRiskForCrossYield(av_box,
        // av_speed, agent_pose_info, pred_pt.v, debug); Consider the lateral
        // risk:
        if (keyobj.obj_decision_type == ObjDecisionType::YIELD) {
          const auto lat_risk = RiskField::CalcAgentLateralRisk(
              av_box, av_speed, agent_pose_info, obs_v, obj_lat_decision,
              keyobj.obj_behavior, true, &debug);
          const double risk = lat_risk.risk;
          // risk_keyvehicle_map[risk] = &keyobj;
          risk_keyvehicle_vec.emplace_back(std::pair{risk, &keyobj});
        } else {
          const auto lat_risk = RiskField::CalcAgentLateralRisk(
              av_box, av_speed, agent_pose_info, obs_v, obj_lat_decision,
              keyobj.obj_behavior, false, &debug);
          const double risk = lat_risk.risk;
          // risk_keyvehicle_map[risk] = &keyobj;
          risk_keyvehicle_vec.emplace_back(std::pair{risk, &keyobj});
        }
      }
    }
  }

  if (has_key_obj) {
    std::sort(risk_keyvehicle_vec.begin(), risk_keyvehicle_vec.end(),
              [](const std::pair<double, const RiskFieldKeyobj*>& a,
                 const std::pair<double, const RiskFieldKeyobj*>& b) {
                return a.first > b.first;
              });
    // constexpr int kKeyObjectNums = 5;
    // std::vector<RiskFieldKeyobj> highest_risk_objs;
    // highest_risk_objs.reserve(kKeyObjectNums);
    // // for (const auto& [risk, keyobj_ptr] : risk_keyvehicle_map) {
    // //   highest_risk_objs.push_back(*keyobj_ptr);
    // //   debug_ += "risk obs id " + keyobj_ptr->id + " risk is " +
    // std::to_string(risk) + "\n";
    // //   if (highest_risk_objs.size() >= kKeyObjectNums) {
    // //     break;
    // //   }
    // // }
    // for (const auto& p : risk_keyvehicle_vec)
    // {
    //     highest_risk_objs.push_back(*p.second);
    //     if (highest_risk_objs.size() >= kKeyObjectNums)
    //     {
    //         break;
    //     }
    // }
    // return highest_risk_objs;
    return risk_keyvehicle_vec;
  }
  return std::nullopt;
}

void DRFDynamicDrivelineGenerator::ReSampleDriveline(
    std::vector<ApolloTrajectoryPointProto>& dr_line, double dt) {
  int cout = kSimulationDt / dt;
  std::vector<ApolloTrajectoryPointProto> new_dr_line;
  new_dr_line.reserve(dr_line.size() * cout);
  for (int i = 0; i + 1 < dr_line.size(); i++) {
    new_dr_line.push_back(dr_line[i]);
    auto& pt1 = dr_line[i];
    auto& pt2 = dr_line[i + 1];
    ApolloTrajectoryPointProto traj_point;
    auto path_point = traj_point.mutable_path_point();
    path_point->set_x((pt1.path_point().x() + pt2.path_point().x()) * 0.5);
    path_point->set_y((pt1.path_point().y() + pt2.path_point().y()) * 0.5);
    path_point->set_theta(
        (pt1.path_point().theta() + pt2.path_point().theta()) * 0.5);
    path_point->set_kappa(
        (pt1.path_point().kappa() + pt2.path_point().kappa()) * 0.5);
    path_point->set_s((pt1.path_point().s() + pt2.path_point().s()) * 0.5);
    traj_point.set_relative_time((pt1.relative_time() + pt2.relative_time()) *
                                 0.5);
    traj_point.set_v((pt1.v() + pt2.v()) * 0.5);
    traj_point.set_a((pt1.a() + pt2.a()) * 0.5);
    new_dr_line.push_back(traj_point);
  }
  new_dr_line.push_back(dr_line.back());
  dr_line = new_dr_line;
}
ObjectDecisionType DRFDynamicDrivelineGenerator::CalcGopointEnvRiskLatDecision(
    const RoadInfo& road_info) {
  const Vec2d go_point_start = road_info.go_point;
  double cos_go_point_heading = std::cos(road_info.go_point_heading);
  double sin_go_point_heading = std::sin(road_info.go_point_heading);
  const Vec2d go_point_end(go_point_start.x() + cos_go_point_heading,
                           go_point_start.y() + sin_go_point_heading);
  // 如果start point有效，使用start point判断绕行方向
  auto go_vecotr = go_point_end - go_point_start;
  if (road_info.start_point_is_valid) {
    if (go_vecotr.CrossProd(road_info.start_point - go_point_start) > 0.0) {
      return ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE;
    } else {
      return ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE;
    }
  }
  // 如果start point无效，通过主车的位置判断绕行方向
  if (go_vecotr.CrossProd(Vec2d(0.0, 0.0) - go_point_start) > 0.0) {
    return ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE;
  } else {
    return ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE;
  }
}

ObjectDecisionType
DRFDynamicDrivelineGenerator::CalcStartpointEnvRiskLatDecision(
    const RoadInfo& road_info) {
  if (!road_info.start_point_is_valid) {
    return ObjectDecisionType::OBJECT_DECISION_TYPE_NONE;
  }
  const Vec2d start_point_start = road_info.start_point;
  double cos_start_point_heading = std::cos(road_info.start_point_heading);
  double sin_start_point_heading = std::sin(road_info.start_point_heading);
  const Vec2d start_point_end(start_point_start.x() + cos_start_point_heading,
                              start_point_start.y() + sin_start_point_heading);
  auto start_vec = start_point_end - start_point_start;
  if (start_vec.CrossProd(road_info.go_point - start_point_start) > 0.0) {
    return ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE;
  } else {
    return ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE;
  }
}

bool DRFDynamicDrivelineGenerator::IsRoadInfoValid(const RoadInfo& road_info,
                                                   const double& ego_heading) {
  //如果start point和go point近乎平行，抑制此时超让driveline触发
  double heading_diff =
      road_info.go_point_heading - road_info.start_point_heading;
  double heading_diff_ego = road_info.go_point_heading - ego_heading;
  if (cos(heading_diff) > kParallelCosThresh ||
      cos(heading_diff_ego) > kParallelCosThresh) {
    debug_.append(absl::StrCat("go start heading_diff: ", heading_diff,
                               " \n cos(heading_diff): ", cos(heading_diff),
                               "\n"));
    debug_.append(absl::StrCat(
        "go ego heading_diff: ", heading_diff_ego,
        " \n cos(heading_diff_ego): ", cos(heading_diff_ego), "\n"));
    return false;
  }
  // if (std::fabs(sin(road_info.go_point_heading)) > kParallelSinThresh)
  // {
  //     return false;
  // }
  return true;
}

bool DRFDynamicDrivelineGenerator::IsPointBackWard(const VehicleState& av_state,
                                                   double x, double y) {
  double av_heading = av_state.theta;
  double av_to_point_heading = atan2(y - av_state.y, x - av_state.x);
  return cos(av_to_point_heading - av_heading) < kBackWardCosHeadingDiff;
}

double DRFDynamicDrivelineGenerator::GetCompensateTargetAcc(
    const ObjectDecisionType& obj_lateral_decision,
    const ObjBehavior& obj_behavior) {
  double compensate_target_acc = 0.0;
  if (obj_lateral_decision ==
      ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE) {
    compensate_target_acc = (obj_behavior == ObjBehavior::SYNC_LEFT_TURN)
                                ? sync_left_nudge_compensate_acc_
                                : oppo_left_nudge_compensate_acc_;
  } else if (obj_lateral_decision ==
             ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE) {
    compensate_target_acc = (obj_behavior == ObjBehavior::SYNC_LEFT_TURN)
                                ? sync_right_nudge_compensate_acc_
                                : oppo_right_nudge_compensate_acc_;
  } else {
  }
  return compensate_target_acc;
}
// double DRFDynamicDrivelineGenerator::GetLookForwardCompensateTargetAcc(
//     const ObjectDecisionType& obj_lateral_decision)
// {
//     double compensate_target_acc = 0.0;
//     if (obj_lateral_decision ==
//     ObjectDecisionType::OBJECT_DECISION_TYPE_LEFT_NUDGE)
//     {
//         compensate_target_acc = oppo_left_nudge_compensate_acc_;
//     }
//     else if (obj_lateral_decision ==
//     ObjectDecisionType::OBJECT_DECISION_TYPE_RIGHT_NUDGE)
//     {
//         compensate_target_acc = oppo_right_nudge_compensate_acc_;
//     }
//     else
//     {
//     }
//     return compensate_target_acc;
// }

void DRFDynamicDrivelineGenerator::RefinePredTraj(
    const SpacetimeTrajectoryManager& origin_traj_mgr,
    SpacetimeTrajectoryManager* const modified_traj_mgr) {
  *modified_traj_mgr = origin_traj_mgr;
  auto mutable_trajs_ptr = modified_traj_mgr->mutable_trajectories();
  for (size_t i = 0; i < mutable_trajs_ptr->size(); i++) {
    //修改预测原始theta值
    auto& st_traj = mutable_trajs_ptr->at(i);
    prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
    auto points_ptr = modified_traj.mutable_points();
    if (points_ptr->size() < 2) {
      continue;
    }
    for (size_t i = 0; i + 1 < points_ptr->size(); i++) {
      const double dx =
          points_ptr->at(i + 1).pos().x() - points_ptr->at(i).pos().x();
      const double dy =
          points_ptr->at(i + 1).pos().y() - points_ptr->at(i).pos().y();
      if (dx * dx + dy * dy < 0.25)  //低速不修
      {
        continue;
      }
      double theta = NormalizeAngle(std::atan2(dy, dx));
      points_ptr->at(i).set_theta(theta);
    }
  }
}
}  // namespace st::planning
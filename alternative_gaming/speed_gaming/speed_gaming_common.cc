/**
 * @file speed_gaming_common.cc
 * @author zhang.xing (zhang.xing43@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "speed_gaming_common.h"

namespace st::planning {
constexpr double kFrontToJunctionDistBuffer = 30.0f;
constexpr double kMergeInAngelDiff = M_PI / 6.0;  // rad
constexpr double kEps = 1.0e-6;
constexpr double kDelta = 1e-3;
constexpr float kMsToS = 0.001f;
constexpr float kKph2Mps = 1.0 / 3.6;
constexpr double kComfortLatAccUpper = 1.6;
constexpr double kUseTableCurvatureLower = 0.001;             // (1/m)
constexpr double kUseTableCurvatureUpper = 1.0 / 100.0;       // (1/m)
constexpr double kUseUrbanTableCurvatureUpper = 1.0 / 100.0;  // (1/m)
const std::vector<std::pair<double, double>> KRadius_speed_table = {
    {5.0, 15.0},   {10.0, 25.0},  {30.0, 30.0},  {40.0, 35.0},   {50.0, 40.0},
    {100.0, 50.0}, {150.0, 60.0}, {200.0, 65.0}, {250.0, 70.0},  {300.0, 75.0},
    {350.0, 85.0}, {400.0, 90.0}, {500.0, 95.0}, {600.0, 100.0}, {700.0, 110.0},
    {800.0, 120.0}};  // {1000.0, 130.0}}; //{m,kph}
constexpr double kEndSforSpeedLimitPlfFallback = 200;  // m.
constexpr double kApproxSpeedLimitEps = 0.5;
constexpr double kEpsilon = 1e-6;
constexpr double kTrajectoryTimeStep = 0.1;  // 100ms.
constexpr bool bPredictTrajRefine = false;

SpeedVector SpeedGamingCommon::ConstructConstAccelSpeedProfile(
    const TrajectoryPoint &init_point, double accel, double speed_limit,
    const PiecewiseLinearFunction<double> &speed_limit_plf, double timestep,
    double time_horizon) {
  SpeedVector speed_profile;
  double max_speed_limit = std::max(init_point.v(), speed_limit);
  double curr_v = init_point.v();
  double curr_a = accel;
  double t = 0.0;
  double curr_s = init_point.s();
  while (t <= time_horizon) {
    SpeedPoint speed_point;
    speed_point.set_s(curr_s);
    speed_point.set_v(curr_v);
    speed_point.set_a(curr_a);
    speed_point.set_t(t);
    speed_profile.push_back(std::move(speed_point));

    // Update.
    t += timestep;
    curr_s = curr_s + curr_v * timestep + curr_a * Sqr(timestep) * 0.5;
    speed_limit = std::min(speed_limit_plf(curr_s), max_speed_limit);
    curr_v = std::clamp(curr_v + timestep * curr_a, 0.0, speed_limit);
    if (std::abs(curr_v - speed_limit) < kEps) {
      curr_a = 0.0;
    }
  }
  return speed_profile;
}

DiscretizedPath SpeedGamingCommon::GeneratePathBasedTraj(
    const SpacetimeObjectTrajectory *traj, double max_s, bool extend) {
  DiscretizedPath path;
  for (const auto &traj_state : traj->states()) {
    PathPoint path_point;
    path_point.set_x(traj_state.traj_point->pos().x());
    path_point.set_y(traj_state.traj_point->pos().y());
    path_point.set_theta(traj_state.traj_point->theta());
    path_point.set_kappa(traj_state.traj_point->kappa());
    path_point.set_s(traj_state.traj_point->s());
    path.emplace_back(std::move(path_point));
  }
  if (!extend) {
    return path;
  }
  constexpr double kStep = 1.0;
  const int end_index = std::floor(max_s / kStep);
  for (int i = 0; i < end_index; ++i) {
    PathPoint path_point;
    double s = path.back().s() + kStep;
    double x = path.back().x() + kStep * std::cos(path.back().theta());
    double y = path.back().y() + kStep * std::sin(path.back().theta());
    double yaw = path.back().theta();
    double k = 0.0;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_s(s);
    path_point.set_theta(yaw);
    path_point.set_kappa(k);
    if (s > max_s) {
      break;
    }
    path.emplace_back(std::move(path_point));
  }
  return path;
}

double SpeedGamingCommon::CalcAgentCrossAngle(
    const DiscretizedPath &av_path,
    const SpacetimeObjectTrajectory *agent_pred_traj, double agent_cutout_time,
    double ego_cutin_s) {
  return CalcEgoCrossAngle(av_path, agent_pred_traj, agent_cutout_time,
                           ego_cutin_s);
}

double SpeedGamingCommon::CalcEgoCrossAngle(
    const DiscretizedPath &av_path,
    const SpacetimeObjectTrajectory *agent_pred_traj, double agent_cutin_time,
    double ego_cutout_s) {
  prediction::PredictedTrajectoryPoint cutin_traj_point;
  if (!SpeedGamingCommon::GetPredictedPointByTimeFromTrajectory(
          agent_pred_traj, agent_cutin_time, &cutin_traj_point)) {
    return 0.5 * M_PI;
  }
  PathPoint av_point = av_path.Evaluate(ego_cutout_s);
  return NormalizeAngle(av_point.theta() - cutin_traj_point.theta());
}

SpacetimeObjectTrajectory SpeedGamingCommon::genThetaValidTraj(
    const SpacetimeObjectTrajectory &st_traj) {
  if (!bPredictTrajRefine) {
    return st_traj;
  }
  prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
  auto points_ptr = modified_traj.mutable_points();
  if (points_ptr->size() < 2) {
    return st_traj;
  }
  points_ptr->at(0).set_theta(st_traj.planner_object().pose().theta());
  for (size_t i = 1; i + 1 < points_ptr->size(); i++) {
    const double dx =
        points_ptr->at(i + 1).pos().x() - points_ptr->at(i).pos().x();
    const double dy =
        points_ptr->at(i + 1).pos().y() - points_ptr->at(i).pos().y();
    const double ds = std::sqrt(dx * dx + dy * dy);
    double theta = NormalizeAngle(fast_math::Atan2(dy, dx));
    double last_theta =
        i == 0 ? points_ptr->at(0).theta() : points_ptr->at(i - 1).theta();
    double dtheta = theta - last_theta;
    double kappa = std::abs(dtheta / std::max(0.1, ds));
    if ((dx * dx + dy * dy < 5) && kappa > 0.3)  //低速用融合
    {
      theta = points_ptr->at(i - 1).theta();
    }
    // std::string debug_info1 =
    //     "theta " + std::to_string(i) + " is" + std::to_string(theta) + " ";
    // Log2DDS::LogDataV2("_task0_gaming/input", debug_info1);
    points_ptr->at(i).set_theta(theta);
  }
  points_ptr->back().set_theta(points_ptr->at(points_ptr->size() - 2).theta());
  auto res = st_traj.CreateTrajectoryMutatedInstance(modified_traj);
  return res;
}

bool SpeedGamingCommon::GetPredictedPointByTimeFromTrajectory(
    const SpacetimeObjectTrajectory *PredictedTraj, double time,
    prediction::PredictedTrajectoryPoint *traj_point) {
  if (traj_point == nullptr) {
    return false;
  }

  if (PredictedTraj->trajectory().points().empty()) {
    return false;
  }

  //假设预测轨迹等时间间隔，TODO check
  const double step_time = PredictedTraj->trajectory().points()[1].t() -
                           PredictedTraj->trajectory().points()[0].t();
  if (step_time <= 0) {
    return false;
  }

  size_t index = static_cast<size_t>(floor(time / step_time));

  if (index >= PredictedTraj->trajectory().points().size()) {
    const auto &end_point = PredictedTraj->trajectory().points().back();
    double dti = time - end_point.t();
    double si = end_point.s();
    double vi = end_point.v();
    double ai = std::max(end_point.a(), 0.0);
    *traj_point = end_point;
    traj_point->set_t(time);
    traj_point->set_s(si + vi * dti + 0.5 * ai * dti * dti);
    traj_point->set_v(std::max(vi + ai * dti, 0.0));
    traj_point->set_a(0.0);

    return true;
  }
  const auto &point = PredictedTraj->trajectory().points()[index];
  double dti = time - point.t();
  double si = point.s();
  double vi = point.v();
  double ai = point.a();
  *traj_point = point;
  traj_point->set_t(time);
  traj_point->set_s(si + vi * dti + 0.5 * ai * dti * dti);
  traj_point->set_v(std::max(vi + ai * dti, 0.0));
  traj_point->set_a(ai);
  return true;
}

PiecewiseLinearFunction<double> SpeedGamingCommon::GetObjCurveSpeedLimit(
    const SpacetimeObjectTrajectory *pred_traj) {
  const auto &traj_points = pred_traj->trajectory().points();
  int n = 5;
  if (traj_points.size() < 2 * n + 1) {
    // 这里返回的可以是一个恒定速度120kph的函数，基本不限速
    return PiecewiseLinearFunction<double>({0, kEndSforSpeedLimitPlfFallback},
                                           {33.3, 33.3});
  }

  std::vector<double> s_vec;
  std::vector<double> v_limit_vec;
  s_vec.reserve(traj_points.size());
  v_limit_vec.reserve(traj_points.size());

  double kappa = 0.0;
  for (int i = n; i < traj_points.size() - n; i += n) {
    s_vec.push_back(traj_points[i].s());
    // 三点法计算曲率
    kappa =
        CalKappaByThreePoints(traj_points[i - n].pos(), traj_points[i].pos(),
                              traj_points[i + n].pos());
    v_limit_vec.push_back(CalCurveSpeedlimit(kappa));
  }
  // 额外处理第1个和最后1个点
  s_vec.insert(s_vec.begin(), traj_points.front().s());
  v_limit_vec.insert(v_limit_vec.begin(), v_limit_vec.front());
  s_vec.push_back(traj_points.back().s());
  v_limit_vec.push_back(v_limit_vec.back());

  return PiecewiseLinearFunction<double>(s_vec, v_limit_vec);
}

double SpeedGamingCommon::CalKappaByThreePoints(const Vec2d &p1,
                                                const Vec2d &p2,
                                                const Vec2d &p3) {
  //计算三边长度
  double a = (p1 - p2).Length();
  double b = (p1 - p3).Length();
  double c = (p2 - p3).Length();
  // 计算半周长
  double s = (a + b + c) / 2.0;
  // 计算三角形面积
  double area = sqrt(s * (s - a) * (s - b) * (s - c));
  // 计算kappa
  return std::min(0.2, (4.0 * area) / std::max(a * b * c, 1e-6));
}

double SpeedGamingCommon::CalCurveSpeedlimit(const double k) {
  double des_speed =
      std::sqrt(kComfortLatAccUpper / std::max(std::fabs(k), 1e-6));
  // use curv speed table to get des speed if possible
  // if (std::fabs(k) > kUseTableCurvatureLower && std::fabs(k) <
  // kUseUrbanTableCurvatureUpper)
  {
    des_speed = TableInterpolate(KRadius_speed_table, 1.0 / std::fabs(k)) / 3.6;
  }
  return des_speed;
}

double SpeedGamingCommon::TableInterpolate(
    const std::vector<std::pair<double, double>> &table, double x) {
  // Use std::lower_bound to find the first element not less than x
  auto it = std::lower_bound(
      table.begin(), table.end(), x,
      [](const std::pair<double, double> &p, double x) { return p.first < x; });

  // Check if x is within the range of the data
  if (it == table.end()) {
    return table.back().second;
  }
  if (it == table.begin()) {
    return table.front().second;
  }

  // Get the indices of the interval [x1, x2]
  size_t index = std::distance(table.begin(), it);
  return Lerp(table[index - 1].second, table[index - 1].first,
              table[index].second, table[index].first, x);
}

SpeedVector SpeedGamingCommon::ConvertPredTrajToSpeedData(
    const SpacetimeObjectTrajectory *pred_traj, double time_step,
    double time_horizon) {
  SpeedVector speed_data;
  const auto &traj_points = pred_traj->trajectory().points();
  if (traj_points.empty()) {
    return speed_data;
  }
  for (int i = 0; i < traj_points.size() - 1; i++) {
    const auto &point = traj_points[i];
    SpeedPoint speed_point;
    speed_point.set_s(point.s());
    speed_point.set_v(point.v());
    speed_point.set_a(point.a());
    speed_point.set_t(point.t());

    double time_diff = traj_points[i + 1].t() - point.t();
    if (time_diff > 0.0) {
      speed_point.set_j((traj_points[i + 1].a() - point.a()) / time_diff);
    } else {
      speed_point.set_j(0.0);
    }
    speed_data.push_back(speed_point);
  }

  SpeedPoint last_point;
  const auto &point = traj_points.back();
  last_point.set_s(point.s());
  last_point.set_v(point.v());
  last_point.set_a(point.a());
  last_point.set_t(point.t());
  last_point.set_j(0.0);
  speed_data.push_back(last_point);

  // resample to 0.2s interval
  SpeedVector final_speed_data;
  double cur_t = 0.0;
  while (cur_t < speed_data.back().t() + 1e-3 || cur_t < time_horizon + 1e-3) {
    auto spt = speed_data.EvaluateByTime(cur_t);
    if (spt.has_value()) {
      final_speed_data.push_back(spt.value());
    } else if (!final_speed_data.empty()) {
      const auto &point = final_speed_data.back();
      SpeedPoint pt{};
      pt.set_s(point.s() + point.v() * time_step +
               0.5 * point.a() * time_step * time_step);
      pt.set_v(point.v() + point.a() * time_step);
      pt.set_a(point.a());
      pt.set_t(cur_t);
      pt.set_j(0.0);
      final_speed_data.emplace_back(std::move(pt));
    }
    cur_t += time_step;
  }
  return final_speed_data;
}

bool SpeedGamingCommon::UpdateConflictZoneInfoByEgoSpeedProfile(
    const SpeedVector &ego_speed_data,
    GamingConflictZoneInfo *riskfield_conflict_info) {
  auto &riskfield_conflict_zone_in_agent_view =
      riskfield_conflict_info->conflict_zone_in_agent_view;
  auto &riskfield_conflict_zone_in_ego_view =
      riskfield_conflict_info->conflict_zone_in_ego_view;

  const auto ego_cutin_speed_point_in_agent_view = ego_speed_data.EvaluateByS(
      riskfield_conflict_zone_in_agent_view.ego_cutin_s, true);
  const auto ego_cutout_speed_point_in_agent_view = ego_speed_data.EvaluateByS(
      riskfield_conflict_zone_in_agent_view.ego_cutout_s, true);
  if (!ego_cutin_speed_point_in_agent_view.has_value() ||
      !ego_cutout_speed_point_in_agent_view.has_value()) {
    return false;
  }
  riskfield_conflict_zone_in_agent_view.ego_cutin_time =
      ego_cutin_speed_point_in_agent_view.value().t();
  riskfield_conflict_zone_in_agent_view.ego_cutout_time =
      ego_cutout_speed_point_in_agent_view.value().t();

  const auto ego_cutin_speed_point_in_ego_view = ego_speed_data.EvaluateByS(
      riskfield_conflict_zone_in_ego_view.ego_cutin_s, true);
  const auto ego_cutout_speed_point_in_ego_view = ego_speed_data.EvaluateByS(
      riskfield_conflict_zone_in_ego_view.ego_cutout_s, true);
  if (!ego_cutin_speed_point_in_ego_view.has_value() ||
      !ego_cutout_speed_point_in_ego_view.has_value()) {
    return false;
  }
  riskfield_conflict_zone_in_ego_view.ego_cutin_time =
      ego_cutin_speed_point_in_ego_view.value().t();
  riskfield_conflict_zone_in_ego_view.ego_cutout_time =
      ego_cutout_speed_point_in_ego_view.value().t();
  return true;
}

bool SpeedGamingCommon::UpdateConflictZoneInfoByAgentSpeedProfile(
    const SpeedVector &agent_speed_data,
    GamingConflictZoneInfo *riskfield_conflict_info) {
  auto &riskfield_conflict_zone_in_agent_view =
      riskfield_conflict_info->conflict_zone_in_agent_view;
  auto &riskfield_conflict_zone_in_ego_view =
      riskfield_conflict_info->conflict_zone_in_ego_view;

  const auto agent_cutin_speed_point_in_agent_view =
      agent_speed_data.EvaluateByS(
          riskfield_conflict_zone_in_agent_view.agent_cutin_s, true);
  const auto agent_cutout_speed_point_in_agent_view =
      agent_speed_data.EvaluateByS(
          riskfield_conflict_zone_in_agent_view.agent_cutout_s, true);
  if (!agent_cutin_speed_point_in_agent_view.has_value() ||
      !agent_cutout_speed_point_in_agent_view.has_value()) {
    return false;
  }
  riskfield_conflict_zone_in_agent_view.agent_cutin_time =
      agent_cutin_speed_point_in_agent_view.value().t();
  riskfield_conflict_zone_in_agent_view.agent_cutout_time =
      agent_cutout_speed_point_in_agent_view.value().t();

  const auto agent_cutin_speed_point_in_ego_view = agent_speed_data.EvaluateByS(
      riskfield_conflict_zone_in_ego_view.agent_cutin_s, true);
  const auto agent_cutout_speed_point_in_ego_view =
      agent_speed_data.EvaluateByS(
          riskfield_conflict_zone_in_ego_view.agent_cutout_s, true);
  if (!agent_cutin_speed_point_in_ego_view.has_value() ||
      !agent_cutout_speed_point_in_ego_view.has_value()) {
    return false;
  }
  riskfield_conflict_zone_in_ego_view.agent_cutin_time =
      agent_cutin_speed_point_in_ego_view.value().t();
  riskfield_conflict_zone_in_ego_view.agent_cutout_time =
      agent_cutout_speed_point_in_ego_view.value().t();
  return true;
}

void SpeedGamingCommon::ObjSpeedDataPostProcess(SpeedVector &origin_speed_data,
                                                double time_step,
                                                double min_jerk,
                                                double max_jerk) {
  if (origin_speed_data.empty()) {
    return;
  }

  double beta = 0.4;
  double target_finish_time = 2.5;

  SpeedIdmState cur_state;
  cur_state.ego_s = origin_speed_data[0].s();
  cur_state.ego_v = origin_speed_data[0].v();
  cur_state.ego_a = origin_speed_data[0].a();

  for (int i = 1; i < origin_speed_data.size(); i++) {
    double target_acc =
        (origin_speed_data.back().v() - cur_state.ego_v) / target_finish_time;
    double target_jerk =
        CalcJerkByAcc(cur_state.ego_a, target_acc, 2.0 * beta, time_step);
    target_jerk = std::clamp(target_jerk, min_jerk, max_jerk);

    cur_state = UpdateNextEgoState(cur_state, target_jerk, time_step);

    origin_speed_data[i].set_s(cur_state.ego_s);
    origin_speed_data[i].set_v(cur_state.ego_v);
    origin_speed_data[i].set_a(cur_state.ego_a);
  }
}

double SpeedGamingCommon::CalcJerkByAcc(double cur_acc, double target_acc,
                                        double pJerk, double time_step1) {
  if (pJerk < 0.0) {
    return 0.0;
  }

  double jerk = 0.0;
  constexpr double kFastAccChangeThresh = 0.1;
  if (cur_acc > kFastAccChangeThresh &&
      target_acc <= -kFastAccChangeThresh) {  // 快速加转减
    jerk = -cur_acc * time_step1 / (time_step1 * time_step1 + kEps);
  } else if (cur_acc < -kFastAccChangeThresh &&
             target_acc > kFastAccChangeThresh) {
    jerk = -cur_acc * time_step1 / (time_step1 * time_step1 + kEps);
  } else {
    jerk = pJerk * (target_acc - cur_acc);  // K 是与beta有关的系数
  }
  return jerk;
}
double SpeedGamingCommon::CalcSaturatedJerkByAcc(
    double cur_acc, double target_acc, double pJerk, double time_step1,
    const double jerk_lower_bound, const double jerk_upper_bound) {
  if (pJerk < 0.0) {
    return 0.0;
  }
  double jerk = 0.0;
  constexpr double kFastAccChangeThresh = 0.1;
  constexpr double kFastAccChangeGapThresh = -0.4;
  if (cur_acc > kFastAccChangeThresh &&
      target_acc <= -kFastAccChangeThresh) {  // fast_acc_to_dec
    jerk = -cur_acc * time_step1 / (time_step1 * time_step1 + kEps);
  } else if (cur_acc < -kFastAccChangeThresh &&
             target_acc > kFastAccChangeThresh) {  // fast_dec_to_acc
    jerk = -cur_acc * time_step1 / (time_step1 * time_step1 + kEps);
  } else if (cur_acc < -kFastAccChangeThresh &&
             ((cur_acc - target_acc) <
              kFastAccChangeGapThresh)) {  // quick brake releasing
    jerk = (std::min(0.0, target_acc) - cur_acc) * time_step1 /
           (time_step1 * time_step1 + kEps);
  } else {
    jerk = pJerk * (target_acc - cur_acc);
    jerk = std::min(jerk_upper_bound, std::max(jerk_lower_bound, jerk));
  }
  return jerk;
}

SpeedIdmState SpeedGamingCommon::UpdateNextEgoState(
    const SpeedIdmState &cur_state, double jerk, double time_step) {
  // 根据当前的位置、速度、加速度和加加速度更新新的状态
  SpeedIdmState next_state = cur_state;
  double time_step1 = time_step;
  double time_step2 = time_step1 * time_step1;
  double time_step3 = time_step2 * time_step1;
  double next_s = cur_state.ego_s + cur_state.ego_v * time_step1 +
                  0.5 * cur_state.ego_a * time_step2 +
                  0.16666666 * jerk * time_step3;
  double next_v =
      cur_state.ego_v + cur_state.ego_a * time_step1 + 0.5 * jerk * time_step2;
  double next_a = cur_state.ego_a + jerk * time_step1;
  double next_j = jerk;
  // 保证不倒车
  if (next_s < cur_state.ego_s) {
    next_s = cur_state.ego_s;
    next_v = 0.0;
    next_a = 0.0;
    next_j = 0.0;
  }
  next_state.ego_s = next_s;
  next_state.ego_v = next_v;
  next_state.ego_a = next_a;
  next_state.ego_j = next_j;
  return next_state;
}

std::vector<TrajectoryPoint> SpeedGamingCommon::GenerateTrajByPathAndSpeedData(
    const DiscretizedPath &av_path, const SpeedVector &av_speed_profile) {
  std::vector<TrajectoryPoint> traj;
  if (av_path.empty() || av_speed_profile.empty()) {
    return traj;
  }

  const double time_horizon = 10.0;

  auto speed_iter = av_speed_profile.begin();
  double curr_s = speed_iter->s();
  double curr_t = speed_iter->t();
  // 遍历生成轨迹
  while (curr_t <= time_horizon && speed_iter != av_speed_profile.end() &&
         curr_s < av_path.back().s()) {
    // 根据 curr_s 找到路径点
    auto path_point = av_path.Evaluate(curr_s);
    TrajectoryPoint traj_point;
    traj_point.set_pos({path_point.x(), path_point.y()});
    traj_point.set_theta(path_point.theta());
    traj_point.set_t(curr_t);
    traj_point.set_s(curr_s);
    traj_point.set_v(speed_iter->v());
    traj_point.set_a(speed_iter->a());
    traj.push_back(traj_point);

    ++speed_iter;
    curr_s = speed_iter->s();
    curr_t = speed_iter->t();
  }
  return traj;
}

std::map<SpeedLimitTypeProto::Type, SpeedLimit>
SpeedGamingCommon::GetSpeedLimitMap(
    const DiscretizedPath &discretized_points, double max_speed_limit,
    double av_speed, const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params,
    const DrivePassage &drive_passage) {
  std::map<SpeedLimitTypeProto::Type, SpeedLimit> speed_limit_map;
  SpeedLimit lane_speed_limit = GenerateLaneSpeedLimit(
      discretized_points, max_speed_limit, av_speed, drive_passage);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_LANE,
                          std::move(lane_speed_limit));

  SpeedLimit curvature_speed_limit =
      GenerateCurvatureSpeedLimit(discretized_points, veh_drive_params,
                                  veh_geo_params, max_speed_limit, av_speed);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_CURVATURE,
                          std::move(curvature_speed_limit));

  SpeedLimit combination_speed_limit =
      GenerateCombinationSpeedLimit(speed_limit_map, max_speed_limit);
  speed_limit_map.emplace(SpeedLimitTypeProto_Type_COMBINATION,
                          std::move(combination_speed_limit));

  return speed_limit_map;
}

SpeedLimit SpeedGamingCommon::GenerateLaneSpeedLimit(
    const DiscretizedPath &path_points, double max_speed_limit, double av_speed,
    const DrivePassage &drive_passage) {
  const auto get_speed_limit = [&drive_passage, max_speed_limit,
                                av_speed](const PathPoint &path_point) {
    constexpr double kLCSoftAcc = 0.3;             // m/s^2
    constexpr double kExceedLimitThreshold = 0.5;  // m/s.
    const PiecewiseLinearFunction<double> kMaxComfortDecelToSpeedDiff = {
        {1.0, 7.0}, {-0.2, -1.1}};
    const double av_speed_sqr = Sqr(av_speed - kExceedLimitThreshold);
    const auto speed_limit =
        drive_passage.QuerySpeedLimitAt(Vec2d(path_point.x(), path_point.y()));
    if (!speed_limit.ok()) {
      return max_speed_limit;
    }
    const double comfortable_decel =
        kMaxComfortDecelToSpeedDiff(av_speed - *speed_limit);
    const double comfortable_brake_speed_sqr =
        av_speed_sqr + 2.0 * comfortable_decel * path_point.s();
    if (comfortable_brake_speed_sqr > Sqr(*speed_limit)) {
      return std::sqrt(comfortable_brake_speed_sqr);
    }
    return *speed_limit;
  };

  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  const int num_points = path_points.size();
  speed_limit_ranges.reserve(num_points);
  // first: s second: v
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  double last_sample_s = 0.0;
  for (int i = 1; i < num_points; ++i) {
    // Only check the lane speed limit at every meter to save computation.
    constexpr double kSpeedLimitSampleRange = 1.0;  // Meters.
    if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
        i == num_points - 1) {
      last_sample_s = path_points[i].s();
      if (const double curr_speed_limit = get_speed_limit(path_points[i]);
          std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
              kApproxSpeedLimitEps ||
          i == num_points - 1) {
        CHECK_GT(path_points[i].s(), prev_speed_limit_point.first);
        speed_limit_ranges.push_back(
            {.start_s = prev_speed_limit_point.first,
             .end_s = path_points[i].s(),
             .speed_limit = prev_speed_limit_point.second,
             .info = SpeedLimitTypeProto::Type_Name(
                 SpeedLimitTypeProto_Type_LANE)});
        prev_speed_limit_point =
            std::make_pair(path_points[i].s(), curr_speed_limit);
      }
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit SpeedGamingCommon::GenerateCurvatureSpeedLimit(
    const DiscretizedPath &path_points,
    const VehicleDriveParamsProto &veh_drive_params,
    const VehicleGeometryParamsProto &veh_geo_params, double max_speed_limit,
    double av_speed) {
  const std::function<double(const PathPoint &)> get_kappa =
      [](const PathPoint &pt) { return pt.kappa(); };

  // --- Params preparation
  // The new curvature speed limit equation is
  // \frac{a}{\kappa^{b} + c} + d.
  // The derivative of lat acc is
  // -\frac{2ab\kappa^{b}(a+d(\kappa^{b}+c))}{(\kappa^{b}+c)^{3}}+(\frac{a}{\kappa^{b}+c})^{2}
  // Use the tangent at kSmallKappaThres to calc speed limit for the points
  // whose kappa is less than kSmallKappaThres to avoid hard brake.
  constexpr double kSmallKappaThres = 0.003;  // m^-1.
  constexpr double kCurvatureEps = 0.000001;  // Avoid division by zero.
  const double a = 0.47;
  const double b = 0.74;
  const double c = 0.008;
  const double d = 2.1;

  const double k_power_b = std::pow(kSmallKappaThres, b);
  const double k_power_b_plus_c = k_power_b + c;
  const double small_kappa_speed_limit_sqr = Sqr(a / k_power_b_plus_c + d);
  const double small_kappa_lat_acc_derivative =
      std::max(0.0, -(2.0 * a * b * k_power_b * (a + d * k_power_b_plus_c)) /
                            Cube(k_power_b_plus_c) +
                        small_kappa_speed_limit_sqr);
  const double intercept =
      std::max(0.0, -kSmallKappaThres * small_kappa_lat_acc_derivative +
                        small_kappa_speed_limit_sqr * kSmallKappaThres);

  const double max_allowed_kappa = std::tan(veh_drive_params.max_steer_angle() /
                                            veh_drive_params.steer_ratio()) /
                                   veh_geo_params.wheel_base();
  const double av_speed_sqr = Sqr(av_speed);

  // --- get_speed_limit[]
  const auto get_speed_limit = [&](double max_kappa, double s) {
    const double fabs_kappa = std::fabs(max_kappa);
    double speed_limit = 0.0;
    if (fabs_kappa > max_allowed_kappa) {
      speed_limit = 2.0;
    } else if (fabs_kappa >= kSmallKappaThres) {
      speed_limit = a / (std::pow(fabs_kappa, b) + c) + d;
    } else {
      const double lat_acc =
          fabs_kappa * small_kappa_lat_acc_derivative + intercept;
      speed_limit = std::sqrt(lat_acc / (fabs_kappa + kCurvatureEps));
    }
    constexpr double kComfortableBrakeAcc = -1.5;  // m/ss.
    const double comfortable_brake_speed_sqr =
        av_speed_sqr + 2.0 * kComfortableBrakeAcc * s;
    if (comfortable_brake_speed_sqr > Sqr(speed_limit)) {
      speed_limit = std::sqrt(comfortable_brake_speed_sqr);
    }
    return speed_limit;
  };

  // --- Main loop.
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_points.size());
  const double radius = 5.0;
  // Deque element: [s, v].
  std::deque<std::pair<double, double>> dq;
  std::vector<double> max_kappas;
  max_kappas.reserve(path_points.size());
  int k = 0;
  for (int i = 0; i < path_points.size(); ++i) {
    const auto &path_point = path_points[i];
    const double start_s = path_point.s() - radius;
    const double end_s = path_point.s() + radius;
    while (!dq.empty() && dq.front().first < start_s) {
      dq.pop_front();
    }
    while (k < path_points.size() && path_points[k].s() <= end_s) {
      const double value = get_kappa(path_points[k]);
      while (!dq.empty() && dq.back().second < std::fabs(value)) {
        dq.pop_back();
      }
      dq.emplace_back(path_points[k++].s(), value);
    }
    max_kappas.push_back(dq.front().second);
  }

  const double init_limit = get_speed_limit(max_kappas[0], path_points[0].s());
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), init_limit);

  for (int i = 1; i < path_points.size(); ++i) {
    const double curr_speed_limit =
        get_speed_limit(max_kappas[i], path_points[i].s());
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == path_points.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = SpeedLimitTypeProto::Type_Name(
               SpeedLimitTypeProto_Type_CURVATURE)});
      prev_speed_limit_point =
          std::make_pair(path_points[i].s(), curr_speed_limit);
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit SpeedGamingCommon::GenerateCombinationSpeedLimit(
    const std::map<SpeedLimitTypeProto::Type, SpeedLimit> &speed_limit_map,
    double max_speed_limit) {
  // FUNC_QTRACE();
  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_ranges;
  int cnt = 0;
  for (const auto &[_, speed_limit] : speed_limit_map) {
    cnt += speed_limit.speed_limit_ranges().size();
  }
  speed_limit_ranges.reserve(cnt);
  for (const auto &[_, speed_limit] : speed_limit_map) {
    for (const auto &range : speed_limit.speed_limit_ranges()) {
      speed_limit_ranges.push_back(range);
    }
  }
  CHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

absl::Status SpeedGamingCommon::CombinePathAndSpeed(
    const DiscretizedPath &path_data, bool forward,
    const SpeedVector &speed_data,
    std::vector<ApolloTrajectoryPointProto> *trajectory) {
  // FUNC_QTRACE();

  CHECK_NOTNULL(trajectory);

  for (int i = 0; i < speed_data.size(); ++i) {
    // VLOG(3) << "speed_data:[" << i << "] = " << speed_data[i].DebugString();
  }
  CHECK_GT(path_data.size(), 1);
  CHECK_GT(speed_data.size(), 1);

  trajectory->clear();
  double t = 0.0;
  while (t < speed_data.TotalTime()) {
    const auto speed_point = speed_data.EvaluateByTime(t);
    if (!speed_point.has_value()) {
      const auto error_msg =
          absl::StrFormat("Fail to evaluate speed vector at time %.2f", t);
      LOG_WARN << error_msg;
      return absl::InternalError(error_msg);
    }

    PathPoint path_point;
    if (path_data.length() < kEpsilon) {
      path_point = path_data.front();
    } else {
      path_point = path_data.Evaluate(speed_point->s());
    }

    ApolloTrajectoryPointProto &traj_point = trajectory->emplace_back();
    if (!forward) {
      path_point.set_s(-path_point.s());
      path_point.set_theta(NormalizeAngle(path_point.theta() + M_PI));
      path_point.set_kappa(-path_point.kappa());
    }
    *(traj_point.mutable_path_point()) = path_point;
    traj_point.set_v(forward ? speed_point->v() : -speed_point->v());
    traj_point.set_a(forward ? speed_point->a() : -speed_point->a());
    traj_point.set_j(forward ? speed_point->j() : -speed_point->j());
    const double yaw_rate = speed_point->v() * path_point.kappa();
    traj_point.set_yaw_rate(forward ? yaw_rate : -yaw_rate);
    traj_point.set_relative_time(t);
    traj_point.set_is_extend(false);

    t += kTrajectoryTimeStep;
  }

  // Extend points if s is too short.
  RETURN_IF_ERROR(
      ExtendTrajectoryLength(path_data, forward, speed_data, trajectory));

  return absl::OkStatus();
}

absl::Status SpeedGamingCommon::ExtendTrajectoryLength(
    const DiscretizedPath &path_data, bool forward,
    const SpeedVector &speed_data,
    std::vector<ApolloTrajectoryPointProto> *trajectory) {
  if (trajectory->empty()) {
    const auto error_msg =
        absl::StrFormat("Fail to extend trajectory because of empty");
    LOG_WARN << error_msg;
    return absl::InternalError(error_msg);
  }
  if (speed_data.empty()) {
    const auto error_msg =
        absl::StrFormat("Fail to evaluate speed vector because of empty");
    LOG_WARN << error_msg;
    return absl::InternalError(error_msg);
  }
  const double kTrajMinLength = 5.0;
  const double kStopVThreshold = 0.1;
  const double kExtendDistStep = 0.1;
  auto speed_point = speed_data.back();
  double t = trajectory->back().relative_time();
  double s = speed_point.s();
  while (trajectory->back().path_point().s() <
             trajectory->front().path_point().s() + kTrajMinLength &&
         trajectory->back().v() < kStopVThreshold) {
    t += kTrajectoryTimeStep;
    s += kExtendDistStep;
    speed_point.set_s(s);
    PathPoint path_point;
    if (path_data.length() < kEpsilon) {
      path_point = path_data.front();
    } else {
      path_point = path_data.Evaluate(speed_point.s());
    }
    ApolloTrajectoryPointProto &extend_point = trajectory->emplace_back();
    if (!forward) {
      path_point.set_s(-path_point.s());
      path_point.set_theta(NormalizeAngle(path_point.theta() + M_PI));
      path_point.set_kappa(-path_point.kappa());
    }
    *(extend_point.mutable_path_point()) = path_point;
    extend_point.set_v(forward ? speed_point.v() : -speed_point.v());
    extend_point.set_a(forward ? speed_point.a() : -speed_point.a());
    extend_point.set_j(forward ? speed_point.j() : -speed_point.j());
    const double yaw_rate = speed_point.v() * path_point.kappa();
    extend_point.set_yaw_rate(forward ? yaw_rate : -yaw_rate);
    extend_point.set_relative_time(t);
    extend_point.set_is_extend(true);
  }
  return absl::OkStatus();
}
std::optional<double> SpeedGamingCommon::FindYawByT(
    const std::vector<TrajectoryPoint> &traj, double t) {
  if (traj.size() < 2) {
    return std::nullopt;
  }

  if (t > traj.back().t() || t < traj.front().t()) {
    return std::nullopt;
  }

  const auto it_lower = std::lower_bound(
      traj.begin(), traj.end(), t,
      [](const TrajectoryPoint &tp, double t) { return tp.t() < t; });

  if (it_lower == traj.begin()) return traj.front().theta();
  if (it_lower == traj.end()) return traj.back().theta();
  const auto &p0 = *(it_lower - 1);
  const auto &p1 = *it_lower;
  const double t0 = p0.t();
  const double t1 = p1.t();
  const double alpha = LerpFactor(t0, t1, t);
  return LerpAngle(p0.theta(), p1.theta(), alpha);
}

void SpeedGamingCommon::ModifyFollowTrajAndSimResult(
    const DiscretizedPath *ego_path, const DrivePassage *drive_passage,
    double time_horizon, double time_step,
    SpacetimeTrajectoryManager *const modified_traj_mgr,
    std::unordered_map<std::string, GamingSimResult>
        *const follow_sim_results) {
  const PiecewiseLinearFunction<double, double> kVelToMaxObjectAccelPlf(
      {Kph2Mps(30.0), Kph2Mps(60.0), Kph2Mps(90.0)}, {2.0, 1.5, 1.0});
  const PiecewiseLinearFunction<double, double> kAccelToAccelDurationPlf(
      {-3.5, -2.0, -0.5, 0, 0.3, 1.0, 2.0},
      {3.0, 1.5, 2.0, 2.5, 2.0, 1.5, 2.5});
  constexpr double kIgnoreAccelDistanceThres = 80.0;  // m.
  constexpr double kMinPerceptionAcc = -5.0;          // m/s^2.
  auto mutable_trajs_ptr = modified_traj_mgr->mutable_trajectories();
  for (size_t i = 0; i < mutable_trajs_ptr->size(); i++) {
    auto &st_traj = mutable_trajs_ptr->at(i);
    auto iter = follow_sim_results->find(st_traj.traj_id());
    if (iter == follow_sim_results->end() ||
        iter->second.interaction_type == InteractionType::kStaticOccupy) {
      continue;
    }
    const auto &object_pose = st_traj.planner_object().pose();
    const double object_vel = object_pose.v();
    double object_accel = 0.0;
    const auto obj_sl = ego_path->XYToSL(object_pose.pos());
    if (obj_sl.s < kIgnoreAccelDistanceThres) {
      object_accel = std::clamp(object_pose.a(), kMinPerceptionAcc,
                                kVelToMaxObjectAccelPlf(object_vel));
    }
    const double a_duration = kAccelToAccelDurationPlf(object_accel);
    int point_num = std::floor(time_horizon / time_step) + 1;
    const auto prediction_speed = prediction::GenerateSpeedProfileByConstAccel(
        object_vel, object_accel, point_num, time_step, a_duration);
    iter->second.obj_speed_data = prediction_speed;
    auto prediction_path = prediction::PredictedTrajectoryPointsToPathPoints(
        st_traj.trajectory().points());
    if (prediction_path.length() < prediction_speed.TotalLength() &&
        drive_passage != nullptr) {
      ExtendPathAlongDrivePassage(
          *drive_passage, prediction_speed.TotalLength(), &prediction_path);
    }
    iter->second.obj_traj =
        GenerateTrajByPathAndSpeedData(prediction_path, prediction_speed);

    std::optional<SpeedPoint> cutin_speed_point = prediction_speed.EvaluateByS(
        iter->second.sim_conflict_zone_in_ego_view.agent_cutin_s, true);
    std::optional<SpeedPoint> cutout_speed_point = prediction_speed.EvaluateByS(
        iter->second.sim_conflict_zone_in_ego_view.agent_cutout_s, true);
    if (cutin_speed_point.has_value()) {
      iter->second.sim_conflict_zone_in_ego_view.agent_cutin_time =
          cutin_speed_point->t();
      iter->second.sim_conflict_zone_in_agent_view.agent_cutin_time =
          cutin_speed_point->t();
    }
    if (cutout_speed_point.has_value()) {
      iter->second.sim_conflict_zone_in_ego_view.agent_cutout_time =
          cutout_speed_point->t();
      iter->second.sim_conflict_zone_in_agent_view.agent_cutout_time =
          cutout_speed_point->t();
    }

    std::vector<prediction::PredictedTrajectoryPoint>
        prediction_trajectory_points;
    if (!prediction::CombinePathAndSpeed(prediction_path, prediction_speed,
                                         time_step,
                                         &prediction_trajectory_points)
             .ok()) {
      return;
    }
    prediction::PredictedTrajectory modified_traj = st_traj.trajectory();
    *modified_traj.mutable_points() = std::move(prediction_trajectory_points);
    st_traj = st_traj.CreateTrajectoryMutatedInstance(modified_traj);
  }
}

void SpeedGamingCommon::ExtendPathAlongDrivePassage(
    const DrivePassage &drive_passage, double length, DiscretizedPath *path) {
  CHECK_NOTNULL(path);
  constexpr double kPathInterval = 3.0;  // m.
  const auto end_point_sl = drive_passage.QueryFrenetCoordinateAt(
      Vec2d(path->back().x(), path->back().y()));
  if (!end_point_sl.ok()) return;
  const double end_s = end_point_sl->s;
  const double end_l = end_point_sl->l;
  double path_end_s = path->back().s();
  double ds = 0.0;
  while (path_end_s < length) {
    path_end_s += kPathInterval;
    ds += kPathInterval;
    const double curr_s = end_s + ds;
    ASSIGN_OR_BREAK(const Vec2d point,
                    drive_passage.QueryPointXYAtSL(curr_s, end_l));
    ASSIGN_OR_BREAK(const double theta,
                    drive_passage.QueryTangentAngleAtS(curr_s));
    auto extend_point = path->back();
    extend_point.set_x(point.x());
    extend_point.set_y(point.y());
    extend_point.set_theta(theta);
    extend_point.set_s(path_end_s);
    path->push_back(std::move(extend_point));
  }
  return;
}

}  // namespace st::planning

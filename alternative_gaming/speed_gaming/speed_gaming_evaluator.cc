/***********************************************************
 * @file     speed_gaming_evaluator.cc
 * @author   tianrui.liu
 * @date     2025.03.31
 * @brief    The implementation of speed gaming evaluator
 * @version  1.0
 ***********************************************************/

#include "speed_gaming_evaluator.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

namespace st::planning {
constexpr double kSpeedThresholdForEgoAccCost = 2.2;

void SpeedGamingEvaluator::Init(
    const double obj_length, const double obj_width, const double obj_offset,
    const double ego_length, const double ego_width, const double ego_offset,
    const PiecewiseLinearFunction<double> &ego_speed_limit_plf,
    const PiecewiseLinearFunction<double> &obj_speed_limit_plf,
    const ObjectType obj_type, const GamingSimResult &gaming_sim_result) {
  obj_length_ = obj_length;
  obj_width_ = obj_width;
  obj_offset_ = obj_offset;
  ego_length_ = ego_length;
  ego_width_ = ego_width;
  ego_offset_ = ego_offset;
  ego_speed_limit_plf_ = ego_speed_limit_plf;
  obj_speed_limit_plf_ = obj_speed_limit_plf;
  obj_type_ = obj_type;
  gaming_sim_result_ = gaming_sim_result;
}

double SpeedGamingEvaluator::Evaluate() {
  double cost = 0.0;
  if (gaming_sim_result_.ego_speed_data.empty() ||
      gaming_sim_result_.obj_speed_data.empty()) {
    debug_ += absl::StrCat("Invalid speed data size, ego ",
                           gaming_sim_result_.ego_speed_data.size(), " obj ",
                           gaming_sim_result_.obj_speed_data.size(), "\n");
    cost = 999999.9;
    return cost;
  }

  // Smoothness
  // Switch off jerk smoothness cost (temporarily)
  // double ego_jerk_smoothness = CalculateJerkSmoothness(GamingCostType::kEgo);
  double ego_jerk_smoothness = 0.0;
  double ego_acc_smoothness = CalculateAccSmoothness(GamingCostType::kEgo);
  if (gaming_sim_result_.ego_speed_data[0].v() < kSpeedThresholdForEgoAccCost &&
      gaming_sim_result_.ego_lon_type == LonGamingDecisionType::kOvertake) {
    ego_acc_smoothness = 0;
  }

  // double obj_jerk_smoothness = CalculateJerkSmoothness(GamingCostType::kObj);
  double obj_jerk_smoothness = 0.0;
  double obj_acc_smoothness = CalculateAccSmoothness(GamingCostType::kObj);

  // Efficiency
  double ego_speed_efficiency = CalculateSpeedEfficiency(GamingCostType::kEgo);
  double ego_travel_dis_efficiency =
      CalculateTravelEfficiency(GamingCostType::kEgo);
  double obj_speed_efficiency = CalculateSpeedEfficiency(GamingCostType::kObj);
  double obj_travel_dis_efficiency =
      CalculateTravelEfficiency(GamingCostType::kObj);

  // Break to stop
  const int ego_stop_count = CheckBreakToStop(GamingCostType::kEgo);
  const int obj_stop_count = CheckBreakToStop(GamingCostType::kObj);
  debug_ += absl::StrCat("Stop Count:ego ", ego_stop_count, ",obj ",
                         obj_stop_count, "\n");
  bool is_collision_result = false;
  // Safety
  double safety_cost = CalculateStaticSafetyCost(is_collision_result);

  // Paradox
  double paradox_cost = 0.0;
  if (gaming_sim_result_.ego_lon_type == LonGamingDecisionType::kOvertake &&
      gaming_sim_result_.interaction_type == InteractionType::kCross) {
    const double ego_conflict_duration =
        gaming_sim_result_.sim_conflict_zone_in_ego_view.ego_cutout_time -
        gaming_sim_result_.sim_conflict_zone_in_ego_view.ego_cutin_time;
    const double overlap_time =
        gaming_sim_result_.sim_conflict_zone_in_ego_view.ego_cutout_time -
        gaming_sim_result_.sim_conflict_zone_in_ego_view.agent_cutin_time;
    const bool is_enter_conflict_zone_time_diff_paradox =
        overlap_time / std::max(0.1, ego_conflict_duration) > 0.3;
    const bool is_conflict_zone_time_valid =
        (gaming_sim_result_.sim_conflict_zone_in_ego_view.agent_cutin_time +
             1e-5 >
         0.0) &&
        (gaming_sim_result_.sim_conflict_zone_in_ego_view.ego_cutin_time +
             1e-5 >
         0.0);
    const bool is_enter_conflict_zone_time_paradox =
        (gaming_sim_result_.sim_conflict_zone_in_ego_view.agent_cutin_time <
         0.01) &&
        (gaming_sim_result_.sim_conflict_zone_in_ego_view.ego_cutin_time > 1.5);
    if (is_conflict_zone_time_valid &&
        (is_enter_conflict_zone_time_diff_paradox ||
         is_enter_conflict_zone_time_paradox)) {
      paradox_cost = 2000;
    }
  }

  const double final_ego_jerk_smoothness_cost =
      std::min(200.0, w_ego_jerk_smoothness_ * ego_jerk_smoothness);
  const double final_obj_jerk_smoothness_cost =
      std::min(100.0, w_obj_jerk_smoothness_ * obj_jerk_smoothness);
  const double final_ego_acc_smoothness_cost =
      std::min(300.0, w_ego_acc_smoothness_ * ego_acc_smoothness);
  const double final_obj_acc_smoothness_cost =
      std::min(300.0, w_obj_acc_smoothness_ * obj_acc_smoothness);
  const double final_ego_speed_efficiency_cost =
      std::min(300.0, w_ego_speed_efficiency_ * ego_speed_efficiency);
  const double final_obj_speed_efficiency_cost =
      std::min(300.0, w_obj_speed_efficiency_ * obj_speed_efficiency);
  const double final_ego_travel_dis_efficiency_cost =
      std::min(200.0, w_ego_travel_dis_efficiency_ * ego_travel_dis_efficiency);
  const double final_obj_travel_dis_efficiency_cost =
      std::min(50.0, w_obj_travel_dis_efficiency_ * obj_travel_dis_efficiency);
  const double final_safety_cost = w_safety_ * safety_cost;
  const double final_ego_stop_cost =
      w_ego_break_to_stop_ * (ego_stop_count > 0 ? 1 : 0);
  const double final_obj_stop_cost =
      w_obj_break_to_stop_ * (obj_stop_count > 0 ? 1 : 0);
  const double final_both_stop_cost =
      w_both_break_to_stop_ *
      ((ego_stop_count > 0 && obj_stop_count > 0) ? 1 : 0);

  debug_ += absl::StrCat("ego_jerk_smooth_cost ",
                         final_ego_jerk_smoothness_cost, "\n");
  debug_ += absl::StrCat("obj_jerk_smooth_cost ",
                         final_obj_jerk_smoothness_cost, "\n");
  debug_ +=
      absl::StrCat("ego_acc_smooth_cost ", final_ego_acc_smoothness_cost, "\n");
  debug_ +=
      absl::StrCat("obj_acc_smooth_cost ", final_obj_acc_smoothness_cost, "\n");
  debug_ += absl::StrCat("ego_speed_efficiency_cost ",
                         final_ego_speed_efficiency_cost, "\n");
  debug_ += absl::StrCat("obj_speed_efficiency_cost ",
                         final_obj_speed_efficiency_cost, "\n");
  debug_ += absl::StrCat("ego_travel_dis_efficiency_cost ",
                         final_ego_travel_dis_efficiency_cost, "\n");
  debug_ += absl::StrCat("obj_travel_dis_efficiency_cost ",
                         final_obj_travel_dis_efficiency_cost, "\n");
  debug_ += absl::StrCat("safety_cost ", final_safety_cost, "\n");
  debug_ += absl::StrCat("ego_stop_cost ", final_ego_stop_cost, "\n");
  debug_ += absl::StrCat("obj_stop_cost ", final_obj_stop_cost, "\n");
  debug_ += absl::StrCat("both_stop_cost ", final_both_stop_cost, "\n");
  debug_ += absl::StrCat("paradox_cost ", paradox_cost, "\n");
  cost += final_ego_jerk_smoothness_cost + final_obj_jerk_smoothness_cost +
          final_ego_acc_smoothness_cost + final_obj_acc_smoothness_cost +
          final_ego_speed_efficiency_cost + final_obj_speed_efficiency_cost +
          final_ego_travel_dis_efficiency_cost +
          final_obj_travel_dis_efficiency_cost + final_safety_cost +
          final_ego_stop_cost + final_obj_stop_cost + final_both_stop_cost +
          paradox_cost;
  return cost;
}

double SpeedGamingEvaluator::GetWeight(size_t t) {
  if (t >= w_index_.size()) {
    return 0.0;
  }
  return w_index_[t];
}

double SpeedGamingEvaluator::CalculateAccSmoothness(GamingCostType cost_type) {
  const SpeedVector &speed_data = cost_type == GamingCostType::kEgo
                                      ? gaming_sim_result_.ego_speed_data
                                      : gaming_sim_result_.obj_speed_data;
  double acc_cost = 0.0;
  const double acc_comfort_max = 1.0;
  const double comfort_acc_level_1 = 2.5;
  const double comfort_acc_level_2 = 4.0;
  size_t n = std::floor(speed_gaming_params_->planning_horizon /
                        speed_gaming_params_->time_step) +
             1;

  for (size_t i = 1; i < std::min(n, speed_data.size()); ++i) {
    const double current_acc = speed_data[i].a();

    if (std::abs(current_acc) < acc_comfort_max) {
      acc_cost += GetWeight(i) * (current_acc * current_acc);
    } else if (std::abs(current_acc) < comfort_acc_level_1) {
      acc_cost += GetWeight(i) * (current_acc * current_acc) * 5.0;
    } else if (std::abs(current_acc) < comfort_acc_level_2) {
      acc_cost += GetWeight(i) * (current_acc * current_acc) * 10.0;
    } else {
      acc_cost += GetWeight(i) * (current_acc * current_acc) * 20.0;
    }
  }
  return acc_cost;
}

double SpeedGamingEvaluator::CalculateJerkSmoothness(GamingCostType cost_type) {
  const SpeedVector &speed_data = cost_type == GamingCostType::kEgo
                                      ? gaming_sim_result_.ego_speed_data
                                      : gaming_sim_result_.obj_speed_data;
  double jerk_cost = 0.0;
  const double comfort_acc_level_1 = 2.5;
  const double comfort_acc_level_2 = 4.0;
  size_t n = std::floor(speed_gaming_params_->planning_horizon /
                        speed_gaming_params_->time_step) +
             1;

  for (size_t i = 1; i < std::min(n, speed_data.size()); ++i) {
    const double acc_diff = speed_data[i].a() - speed_data[i - 1].a();
    const double t_diff = speed_data[i].t() - speed_data[i - 1].t();
    const double estimate_jerk = acc_diff * t_diff / (t_diff * t_diff + 1e-3);
    jerk_cost += GetWeight(i) * estimate_jerk * estimate_jerk;
  }
  return jerk_cost;
}

double SpeedGamingEvaluator::CalculateSpeedEfficiency(
    GamingCostType cost_type) {
  const PiecewiseLinearFunction<double> speed_limit_plf =
      cost_type == GamingCostType::kEgo ? ego_speed_limit_plf_
                                        : obj_speed_limit_plf_;
  const SpeedVector &speed_data = cost_type == GamingCostType::kEgo
                                      ? gaming_sim_result_.ego_speed_data
                                      : gaming_sim_result_.obj_speed_data;

  double average_speed = 0.0;
  size_t n = std::floor(speed_gaming_params_->planning_horizon /
                        speed_gaming_params_->time_step) +
             1;

  const auto last_point =
      n < speed_data.size() ? speed_data[n - 1] : speed_data.back();
  average_speed = (last_point.s() - speed_data.front().s()) /
                  (std::abs(last_point.t() - speed_data.front().t()) + 1e-3);

  // const double speed_target =
  //     std::min(std::max(speed_data.front().v(), 5.0), 20.0); // TODO: get
  //     road speed limit
  double speed_target;
  if (speed_limit_plf.x().size() < 2) {
    speed_target = std::min(std::max(speed_data.front().v(), 5.0), 20.0);
    // TODO: optimalize this accrording to the obj type
  } else {
    const double delta_s = 0.5;
    double s = 0.0, total_time = 0.0;
    double v0 = speed_limit_plf(0.0);
    double v1 = 0.0;
    size_t i = 0;
    while (total_time < last_point.t() &&
           s < 120.0 * 1.2 / 3.6 * speed_gaming_params_->planning_horizon) {
      i++;
      s += delta_s;
      v1 = speed_limit_plf.Evaluate(s);
      total_time +=
          delta_s / ((v0 + v1) / 2.0 + 1e-3);  // Avoid division by zero
      v0 = v1;
    }
    speed_target = s / total_time;

    if (obj_type_ == ObjectType::OT_CYCLIST) {
      speed_target = speed_target > std::max(7.0, speed_data.front().v())
                         ? std::max(7.0, speed_data.front().v())
                         : speed_target;
    } else if (obj_type_ == ObjectType::OT_TRICYCLIST) {
      speed_target = speed_target > std::max(7.0, speed_data.front().v())
                         ? std::max(7.0, speed_data.front().v())
                         : speed_target;
    } else if (obj_type_ == ObjectType::OT_PEDESTRIAN) {
      speed_target = speed_target > std::max(2.0, speed_data.front().v())
                         ? std::max(2.0, speed_data.front().v())
                         : speed_target;
    }
  }

  debug_ += cost_type == GamingCostType::kEgo ? "ego" : "obj";
  debug_ += absl::StrCat(" average_speed:", average_speed,
                         " speed_target:", speed_target, "\n");
  const double k_speed = 2.0;
  const double speed_efficiency_cost =
      1.0 /
      (1.0 + std::exp(k_speed * (average_speed - speed_target) / speed_target));
  return speed_efficiency_cost;
}

int SpeedGamingEvaluator::CheckBreakToStop(GamingCostType cost_type) {
  const std::vector<TrajectoryPoint> &traj = cost_type == GamingCostType::kEgo
                                                 ? gaming_sim_result_.ego_traj
                                                 : gaming_sim_result_.obj_traj;
  size_t n = std::min(
      traj.size(), static_cast<size_t>(speed_gaming_params_->planning_horizon /
                                       speed_gaming_params_->time_step) +
                       1);
  bool has_stopped = false;
  int stop_count = 0;
  for (size_t i = 0; i < n; ++i) {
    if (traj[i].v() < 0.25) {
      has_stopped = true;
      stop_count++;
    }
  }
  return stop_count;
}

double SpeedGamingEvaluator::CalculateTravelEfficiency(
    GamingCostType cost_type) {
  const SpeedVector &speed_data = cost_type == GamingCostType::kEgo
                                      ? gaming_sim_result_.ego_speed_data
                                      : gaming_sim_result_.obj_speed_data;
  const auto last_point =
      speed_data.EvaluateByTime(speed_gaming_params_->planning_horizon);
  if (!last_point.has_value()) {
    return 0.0;
  }

  const double travel_distance = last_point->s() - speed_data.front().s();
  const double k_distance = 1.5;
  const double target_distance = 20.0;
  const double travel_efficiency_cost =
      1.0 / (1.0 + std::exp(k_distance * (travel_distance - target_distance) /
                            target_distance));  // Sigmoid

  // cost_type == GamingCostType::kEgo ? debug_ += "ego" : debug_ += "obj";
  // debug_ += absl::StrCat(" travel_distance " , travel_distance, "\n");
  // debug_ += absl::StrCat(" target_distance " , target_distance, "\n");

  return travel_efficiency_cost;
}

double SpeedGamingEvaluator::CalculateStaticSafetyCost(
    bool &is_collision_result) {
  size_t n = std::min(gaming_sim_result_.ego_traj.size(),
                      gaming_sim_result_.obj_traj.size());
  n = std::min(n, static_cast<size_t>(speed_gaming_params_->planning_horizon /
                                      speed_gaming_params_->time_step) +
                      1);

  double min_distance = 999.9;
  int collision_index = 1000;

  double safe_stationary_cost = -1.0;

  for (size_t i = 0; i < n; ++i) {
    double temp_distance = CalculateMinBoxDistance(i);
    min_distance = std::min(min_distance, temp_distance);
    if (min_distance < 1e-1) {
      collision_index = i;
      break;
    }
    double safe_stationary_cost_temp =
        -2.0 * (std::log(temp_distance - 0.1) - 0.6);
    double decay_factor = std::exp(static_cast<double>(i) * (-0.05));
    safe_stationary_cost_temp *= decay_factor;
    safe_stationary_cost =
        std::max(safe_stationary_cost, safe_stationary_cost_temp);
  }

  debug_ += absl::StrCat("min rectangle dist ", min_distance, "\n");
  if (min_distance < 1e-1) {
    debug_ += absl::StrCat("collision at index ", collision_index, "\n");
    double decay_factor_for_collision = std::exp(-0.1 * collision_index);
    safe_stationary_cost =
        std::max(safe_stationary_cost, decay_factor_for_collision * 10.0);
    safe_stationary_cost += 2;  // Add a penalty for collision
    is_collision_result = true;
    return safe_stationary_cost;
  }

  return std::min(std::max(safe_stationary_cost, 0.0), 2.0);
}

double SpeedGamingEvaluator::CalculateMinBoxDistance(const size_t i) {
  const auto ego_traj = gaming_sim_result_.ego_traj;
  const double ego_x = ego_traj[i].pos().x();
  const double ego_y = ego_traj[i].pos().y();
  const double ego_yaw = ego_traj[i].theta();
  const auto obj_traj = gaming_sim_result_.obj_traj;
  const double obj_x = obj_traj[i].pos().x();
  const double obj_y = obj_traj[i].pos().y();
  const double obj_yaw = obj_traj[i].theta();

  // 计算三角函数值
  const double cos_ego_yaw = std::cos(ego_yaw);
  const double sin_ego_yaw = std::sin(ego_yaw);
  const double cos_obj_yaw = std::cos(obj_yaw);
  const double sin_obj_yaw = std::sin(obj_yaw);

  // 计算自车和障碍物的四个顶点
  std::vector<std::pair<double, double>> ego_vertices = {
      {ego_x + ego_offset_ - ego_length_ / 2.0, ego_y - ego_width_ / 2.0},
      {ego_x + ego_offset_ + ego_length_ / 2.0, ego_y - ego_width_ / 2.0},
      {ego_x + ego_offset_ + ego_length_ / 2.0, ego_y + ego_width_ / 2.0},
      {ego_x + ego_offset_ - ego_length_ / 2.0, ego_y + ego_width_ / 2.0}};
  std::vector<std::pair<double, double>> obj_vertices = {
      {obj_x + obj_offset_ - obj_length_ / 2.0, obj_y - obj_width_ / 2.0},
      {obj_x + obj_offset_ + obj_length_ / 2.0, obj_y - obj_width_ / 2.0},
      {obj_x + obj_offset_ + obj_length_ / 2.0, obj_y + obj_width_ / 2.0},
      {obj_x + obj_offset_ - obj_length_ / 2.0, obj_y + obj_width_ / 2.0}};

  // 旋转自车和障碍物的顶点
  for (auto &vertex : ego_vertices) {
    double dx = vertex.first - ego_x;
    double dy = vertex.second - ego_y;
    double rotated_x = dx * cos_ego_yaw - dy * sin_ego_yaw;
    double rotated_y = dx * sin_ego_yaw + dy * cos_ego_yaw;
    vertex.first = rotated_x + ego_x;
    vertex.second = rotated_y + ego_y;
  }
  for (auto &vertex : obj_vertices) {
    double dx = vertex.first - obj_x;
    double dy = vertex.second - obj_y;
    double rotated_x = dx * cos_obj_yaw - dy * sin_obj_yaw;
    double rotated_y = dx * sin_obj_yaw + dy * cos_obj_yaw;
    vertex.first = rotated_x + obj_x;
    vertex.second = rotated_y + obj_y;
  }

  std::vector<std::pair<double, double>> axes;
  // 自车的分离轴
  for (size_t m = 0; m < ego_vertices.size() / 2; ++m) {
    size_t n = (m + 1) % ego_vertices.size();
    double dx = ego_vertices[n].first - ego_vertices[m].first;
    double dy = ego_vertices[n].second - ego_vertices[m].second;
    // 计算垂直向量作为分离轴
    axes.emplace_back(-dy, dx);
  }

  // 障碍物的分离轴
  for (size_t m = 0; m < obj_vertices.size() / 2; ++m) {
    size_t n = (m + 1) % obj_vertices.size();
    double dx = obj_vertices[n].first - obj_vertices[m].first;
    double dy = obj_vertices[n].second - obj_vertices[m].second;
    // 计算垂直向量作为分离轴
    axes.emplace_back(-dy, dx);
  }

  // 对每个分离轴进行投影
  std::optional<double> axis_distance;
  for (const auto &axis : axes) {
    double min_ego = std::numeric_limits<double>::max();
    double max_ego = std::numeric_limits<double>::lowest();
    double min_obj = std::numeric_limits<double>::max();
    double max_obj = std::numeric_limits<double>::lowest();

    // 自车顶点投影
    for (const auto &vertex : ego_vertices) {
      double projection =
          vertex.first * axis.first + vertex.second * axis.second;
      min_ego = std::min(min_ego, projection);
      max_ego = std::max(max_ego, projection);
    }

    // 障碍物顶点投影
    for (const auto &vertex : obj_vertices) {
      double projection =
          vertex.first * axis.first + vertex.second * axis.second;
      min_obj = std::min(min_obj, projection);
      max_obj = std::max(max_obj, projection);
    }

    if (max_ego < min_obj) {
      axis_distance = min_obj - max_ego;
    } else if (max_obj < min_ego) {
      axis_distance = min_ego - max_obj;
    }

    if (axis_distance.has_value()) {
      break;
    }
  }

  if (!axis_distance.has_value()) {
    debug_ += "Min distance is 0 \n";
    return 0.0;
  }

  // 计算最短距离
  double min_distance = std::numeric_limits<double>::max();
  for (size_t m = 0; m < ego_vertices.size(); ++m) {
    for (size_t n = 0; n < obj_vertices.size(); ++n) {
      size_t k = (n + 1) % obj_vertices.size();
      std::pair<double, double> &point = ego_vertices[m];
      std::pair<double, double> &start = obj_vertices[n];
      std::pair<double, double> &end = obj_vertices[k];

      double temp_distance = CalculatePointToSegmentDistance(point, start, end);
      min_distance = std::min(min_distance, temp_distance);
    }
  }
  for (size_t m = 0; m < obj_vertices.size(); ++m) {
    for (size_t n = 0; n < ego_vertices.size(); ++n) {
      size_t k = (n + 1) % ego_vertices.size();
      std::pair<double, double> &point = obj_vertices[m];
      std::pair<double, double> &start = ego_vertices[n];
      std::pair<double, double> &end = ego_vertices[k];

      double temp_distance = CalculatePointToSegmentDistance(point, start, end);
      min_distance = std::min(min_distance, temp_distance);
    }
  }

  // 返回最小距离
  // debug_ += absl::StrCat("Min distance at index " , i , " is " ,
  //     min_distance , "\n");
  return min_distance;
}

double SpeedGamingEvaluator::CalculatePointToSegmentDistance(
    const std::pair<double, double> &point,
    const std::pair<double, double> &start,
    const std::pair<double, double> &end) {
  // 线段向量
  std::pair<double, double> seg_vec = {end.first - start.first,
                                       end.second - start.second};
  // 点到线段起点的向量
  std::pair<double, double> point_vec = {point.first - start.first,
                                         point.second - start.second};

  // 线段长度的平方
  double seg_length_sq =
      seg_vec.first * seg_vec.first + seg_vec.second * seg_vec.second;
  if (std::abs(seg_length_sq) < 1e-2) {
    // 线段退化为一个点
    return std::sqrt(point_vec.first * point_vec.first +
                     point_vec.second * point_vec.second);
  }

  // 计算投影比例
  double t =
      (point_vec.first * seg_vec.first + point_vec.second * seg_vec.second) /
      seg_length_sq;
  t = std::max(0.0, std::min(1.0, t));  // 将 t 限制在 [0, 1] 范围内

  // 计算投影点
  std::pair<double, double> projection = {start.first + t * seg_vec.first,
                                          start.second + t * seg_vec.second};

  // 计算点到投影点的距离
  double dx = point.first - projection.first;
  double dy = point.second - projection.second;
  return std::sqrt(dx * dx + dy * dy);
}
}  // namespace st::planning
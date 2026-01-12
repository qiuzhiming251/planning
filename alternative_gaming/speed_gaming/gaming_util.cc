#include "gaming_util.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <utility>

namespace st {
namespace planning {

namespace {

constexpr double kEpsilon = 0.01;

// 获取当前时间戳（秒，double）
double GetCurrentTimestamp() {
  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  auto seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(duration);
  return seconds.count();
}

void UpdateCooperationMap(
    ObjectCooperationInfo* info, std::string obs_id,
    std::unordered_map<std::string, ObjectCooperationInfo>&
        object_cooperation_maps,
    double now_stamp, double velocity, double ideal_yield_acc,
    double ideal_pass_acc) {
  info->add_observe_timestamp_list(now_stamp);
  info->add_observe_velocity_list(velocity);
  // info->add_observe_acc_list(acc);
  info->add_ideal_yield_acc_list(ideal_yield_acc);
  info->add_ideal_pass_acc_list(ideal_pass_acc);

  auto* timestamps = info->mutable_observe_timestamp_list();
  auto* velocities = info->mutable_observe_velocity_list();
  // auto* accs = info->mutable_observe_acc_list();
  auto* ideal_yield_accs = info->mutable_ideal_yield_acc_list();
  auto* ideal_pass_accs = info->mutable_ideal_pass_acc_list();

  assert(timestamps->size() == velocities->size());
  // assert(timestamps->size() == accs->size());
  assert(timestamps->size() == ideal_yield_accs->size());
  assert(timestamps->size() == ideal_pass_accs->size());

  const int max_size = 10;
  const double max_age = 5.0;  //  second

  while (!timestamps->empty()) {
    if (timestamps->size() > max_size) {
      timestamps->erase(timestamps->begin());
      velocities->erase(velocities->begin());
      // accs->erase(accs->begin());
      ideal_yield_accs->erase(ideal_yield_accs->begin());
      ideal_pass_accs->erase(ideal_pass_accs->begin());
      continue;
    }

    double oldest_stamp = (*timestamps)[0];
    if (now_stamp - oldest_stamp > max_age) {
      timestamps->erase(timestamps->begin());
      velocities->erase(velocities->begin());
      // accs->erase(accs->begin());
      ideal_yield_accs->erase(ideal_yield_accs->begin());
      ideal_pass_accs->erase(ideal_pass_accs->begin());
      continue;
    }
    break;
  }

  // 维护map
  for (auto it = object_cooperation_maps.begin();
       it != object_cooperation_maps.end();) {
    if (it->first == obs_id) {
      ++it;
      continue;
    }
    const auto& ts_list = it->second.observe_timestamp_list();
    if (!ts_list.empty() &&
        (now_stamp - ts_list.Get(ts_list.size() - 1) < 5.0)) {
      ++it;
      continue;
    } else {
      it = object_cooperation_maps.erase(it);
    }
  }
}

std::vector<double> CalculateAcceleration(ObjectCooperationInfo* info,
                                          std::ostringstream& oss) {
  if (info == nullptr || info->observe_velocity_list_size() < 2) {
    return {};
  }
  const auto& timestamps_list = info->observe_timestamp_list();
  const auto& velocity_list = info->observe_velocity_list();

  int n = static_cast<int>(info->observe_timestamp_list_size());
  // oss << "acc comput start size:" << n << "\n";
  std::vector<double> accelerations(n, 0.0);
  for (int i = 1; i < n - 1; ++i) {
    const double dt_sec = static_cast<double>(timestamps_list.Get(i + 1) -
                                              timestamps_list.Get(i - 1));
    // oss << "i:" << i << " dt_sec:" << dt_sec << " acc:"
    //     << (velocity_list.Get(i + 1) - velocity_list.Get(i - 1)) / dt_sec
    //     << "\n";
    if (dt_sec <= 0.0) {
      accelerations[i] = 0.0;
      continue;
    }
    accelerations[i] =
        (velocity_list.Get(i + 1) - velocity_list.Get(i - 1)) / dt_sec;
  }

  // 两端点
  if (n > 1) {
    double dt0 =
        static_cast<double>(timestamps_list.Get(1) - timestamps_list.Get(0));
    if (dt0 > 0.0) {
      accelerations[0] = (velocity_list.Get(1) - velocity_list.Get(0)) / dt0;
    }

    double dtn = static_cast<double>(timestamps_list.Get(n - 1) -
                                     timestamps_list.Get(n - 2));
    if (dtn > 0.0) {
      accelerations[n - 1] =
          (velocity_list.Get(n - 1) - velocity_list.Get(n - 2)) / dtn;
    }
  }
  return accelerations;
}

ObjectCooperationInfo* FindOrCreateCooperationInfo(
    std::unordered_map<std::string, ObjectCooperationInfo>&
        object_cooperation_maps,
    const std::string& obj_id) {
  auto it = object_cooperation_maps.find(obj_id);
  if (it != object_cooperation_maps.end()) {
    return &it->second;
  }
  auto [new_it, inserted] =
      object_cooperation_maps.emplace(obj_id, ObjectCooperationInfo{});
  return &new_it->second;
}

// 函数计算相对误差
std::vector<double> CalculateRelativeCooperate(
    const std::vector<double>& observed, const std::vector<double>& ideal) {
  std::vector<double> relative_cooperate;
  double epsilon = 0.5;
  for (size_t i = 0; i < observed.size(); ++i) {
    epsilon = 0.5;
    if (ideal[i] < -1.0) {
      epsilon = 0.1;
    } else if (ideal[i] < -0.2) {
      epsilon = 0.5;
    } else {
      epsilon = 0.8;
    }
    const double temp = std::max(observed[i] - ideal[i], 0.0) /
                        (std::abs(std::max(-ideal[i], 0.0)) + epsilon);
    const double cooperate = 1.0 - std::clamp(temp, 0.05, 0.95);
    relative_cooperate.push_back(cooperate);
  }
  return relative_cooperate;
}

}  // anonymous namespace

// yield & pass coop grad.
std::pair<double, double> calcObsCooperationGrad(
    GamingSimResult* yield_simu, GamingSimResult* pass_simu, std::string obs_id,
    SpacetimeObjectTrajectory* pred_traj,
    std::unordered_map<std::string, ObjectCooperationInfo>&
        object_cooperation_maps,
    std::string* cur_debug_info, IdmSimulateState* simulate_state) {
  ObjectCooperationInfo* obs_info =
      FindOrCreateCooperationInfo(object_cooperation_maps, obs_id);
  std::ostringstream oss;
  // oss << "calc coops grad id:" << obs_id << "\n";
  auto start_time = GetCurrentTimestamp();
  auto vel2d = pred_traj->planner_object().velocity();
  double speed = std::sqrt(vel2d.x() * vel2d.x() + vel2d.y() * vel2d.y());
  // auto calc_ideal_a_avg = [](const SpeedVector& list) -> double {
  //   const size_t n = std::min<size_t>(8, list.size());
  //   if (n == 0) return 0.0;
  //   double sum = 0.0;
  //   for (size_t i = 0; i < n; ++i) {
  //     sum += list[i].a();
  //   }
  //   return sum / static_cast<double>(n);
  // };
  // double ideal_yield_acc = calc_ideal_a_avg(yield_simu->obj_speed_data);
  // double ideal_pass_acc = calc_ideal_a_avg(pass_simu->obj_speed_data);
  double ideal_yield_acc = 0.0;
  double ideal_pass_acc = 0.0;
  if (simulate_state != nullptr) {
    ideal_yield_acc = simulate_state->agent_ideal_acc_in_yield_mode;
    ideal_pass_acc = simulate_state->agent_ideal_acc_in_pass_mode;
  }
  // oss << "calc ideal_yield_acc:" << ideal_yield_acc << "\n"
  //     << "calc ideal_pass_acc:" << ideal_pass_acc << "\n"
  //     << "now time:" << start_time << "\n"
  //     << "speed:" << speed << "\n";
  // oss << "update before:" << obs_info->DebugString() << "\n";
  UpdateCooperationMap(obs_info, obs_id, object_cooperation_maps, start_time,
                       speed, ideal_yield_acc, ideal_pass_acc);
  // oss << "update after:" << obs_info->DebugString() << "\n";
  std::vector<double> acc_computed = CalculateAcceleration(obs_info, oss);
  // for (auto i : acc_computed) {
  //   oss << "acc computed:" << i;
  // }
  // oss << "acc_computed size: " << acc_computed.size() << "\n";
  std::vector<double> ideal_yield_accs, ideal_pass_accs;
  ideal_yield_accs.reserve(acc_computed.size());
  ideal_pass_accs.reserve(acc_computed.size());

  const int n = obs_info->observe_timestamp_list_size();
  for (int i = 0; i < n; ++i) {
    ideal_yield_accs.emplace_back(obs_info->ideal_yield_acc_list(i));
    ideal_pass_accs.emplace_back(obs_info->ideal_pass_acc_list(i));
  }

  auto calculateCooperationGrad =
      [](const std::vector<double>& observed,
         const std::vector<double>& ideal) -> double {
    auto relative_cooperate = CalculateRelativeCooperate(observed, ideal);
    if (ideal.empty()) {
      return 0.5;
    }
    double sum = std::accumulate(relative_cooperate.begin(),
                                 relative_cooperate.end(), 0.0) /
                 static_cast<double>(relative_cooperate.size());
    return std::clamp(sum, 0.05, 0.5);
  };
  // for (const auto& [obj_id, coop_info] : object_cooperation_maps) {
  //   if (obj_id == obs_id) {
  //     oss << "cooperation_calc_out_obj_id: " << obj_id << "\n"
  //         << coop_info.DebugString() << "\n";
  //   }
  // }
  // oss << "acc_computed size: " << acc_computed.size() << "\n"
  //     << " ideal_yield_accs size:" << ideal_yield_accs.size() << "\n"
  //     << " ideal_pass_accs size:" << ideal_pass_accs.size() << "\n";
  // for (auto i : acc_computed) {
  //   oss << "acc_computed:" << i << "\n";
  // }
  // for (auto i : ideal_yield_accs) {
  //   oss << "yield acc:" << i << "\n";
  // }
  // for (auto i : ideal_pass_accs) {
  //   oss << "pass acc:" << i << "\n";
  // }
  // *cur_debug_info += oss.str();
  bool use_pass_calc = true;
  if (!use_pass_calc) {
    return std::make_pair(
        calculateCooperationGrad(acc_computed, ideal_yield_accs), 0.5);
  } else {
    return std::make_pair(
        calculateCooperationGrad(acc_computed, ideal_yield_accs),
        calculateCooperationGrad(acc_computed, ideal_pass_accs));
  }
}

// void DumpStGraphBoundary(
//     int plan_id,
//     const std::vector<StBoundaryWithDecision>& st_boundaries_with_decision,
//     const SpeedFinderDebugProto& speed_finder_proto) {
//   const auto get_stb_points = [](const StBoundary* st_boundary) {
//     std::vector<StPoint> st_points;
//     if (st_boundary == nullptr) return st_points;
//     const auto& lower_points = st_boundary->lower_points();
//     const auto& upper_points = st_boundary->upper_points();
//     if (lower_points.empty() || upper_points.empty()) {
//       return st_points;
//     }
//     st_points.reserve(lower_points.size() + upper_points.size());
//     for (int i = 0; i < lower_points.size(); ++i) {
//       st_points.push_back(lower_points[i]);
//     }
//     for (int i = upper_points.size() - 1; i >= 0; --i) {
//       st_points.push_back(upper_points[i]);
//     }
//     return st_points;
//   };

//   const auto get_decision_suffix = [](StBoundaryProto::DecisionType type) {
//     switch (type) {
//       case StBoundaryProto::FOLLOW:
//         return "_F";
//       case StBoundaryProto::YIELD:
//         return "_Y";
//       case StBoundaryProto::OVERTAKE:
//         return "_O";
//       case StBoundaryProto::IGNORE:
//         return "_I";
//       case StBoundaryProto::UNKNOWN:
//         return "_U";
//     }
//     return "_U";
//   };

//   const auto get_soft_bound_idx = [](const auto& times, double t) {
//     return std::distance(times.begin(),
//                          std::lower_bound(times.begin(), times.end(), t));
//   };

//   for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
//     const auto& raw_st_boundary =
//     st_boundary_with_decision.raw_st_boundary(); const auto raw_st_points =
//     get_stb_points(raw_st_boundary);

//     // Dump st_boundary points.
//     const auto decision = st_boundary_with_decision.decision_type();
//     const auto decision_suffix = get_decision_suffix(decision);
//     const auto color =
//         st_boundary_with_decision.decision_type() == StBoundaryProto::IGNORE
//             ? Log2DDS::kTiffanyBlue
//             : Log2DDS::kOrange;
//     if (!raw_st_points.empty()) {
//       Log2DDS::LogChartV2(
//           Log2DDS::TaskPrefix(plan_id) + "st_raw",
//           raw_st_boundary->id() + decision_suffix, color, true,
//           raw_st_points,
//           [](const StPoint& p) -> double { return p.t(); },
//           [](const StPoint& p) -> double { return p.s() - 0.5; });
//     }

//     const auto& st_boundary = st_boundary_with_decision.st_boundary();
//     const auto st_points = get_stb_points(st_boundary);
//     if (!st_points.empty()) {
//       Log2DDS::LogChartV2(
//           Log2DDS::TaskPrefix(plan_id) + "st",
//           st_boundary->id() + decision_suffix, color, true, st_points,
//           [](const StPoint& p) -> double { return p.t(); },
//           [](const StPoint& p) -> double { return p.s(); });
//     }

//     // Dump debug_info.
//     Log2DDS::LogDataV2(
//         Log2DDS::TaskPrefix(plan_id) + "stbinfo_" + st_boundary->id(),
//         MakeStBoundaryDebugInfo(st_boundary_with_decision));

//     // Dump st_boundary soft bound.
//     if (decision == StBoundaryProto::UNKNOWN ||
//         decision == StBoundaryProto::IGNORE) {
//       continue;
//     }
//     const auto id =
//         GetStBoundaryIntegrationId(*st_boundary_with_decision.st_boundary());
//     const auto& speed_opt_debug = speed_finder_proto.speed_optimizer();
//     CHECK(decision == StBoundaryProto::YIELD ||
//           decision == StBoundaryProto::OVERTAKE ||
//           decision == StBoundaryProto::FOLLOW);
//     const auto* soft_bound_data =
//         (decision == StBoundaryProto::YIELD ||
//          decision == StBoundaryProto::FOLLOW)
//             ? FindOrNull(speed_opt_debug.soft_s_upper_bound(), id)
//             : FindOrNull(speed_opt_debug.soft_s_lower_bound(), id);
//     if (!soft_bound_data || soft_bound_data->time_size() < 2) continue;

//     const double min_t = st_boundary->min_t();
//     const double max_t = st_boundary->max_t();
//     const int start_idx = get_soft_bound_idx(soft_bound_data->time(), min_t);
//     const int soft_bound_size = soft_bound_data->time_size();
//     std::vector<StPoint> stb_st;
//     stb_st.reserve(soft_bound_size);

//     // First point.
//     const auto stb_start_pt =
//         st_boundary->GetBoundarySRange(soft_bound_data->time(start_idx));
//     if (!stb_start_pt.has_value()) continue;
//     const double stb_start_point_s = (decision == StBoundaryProto::YIELD ||
//                                       decision == StBoundaryProto::FOLLOW)
//                                          ? stb_start_pt->second
//                                          : stb_start_pt->first;
//     stb_st.emplace_back(stb_start_point_s, soft_bound_data->time(start_idx));

//     int end_idx = 0.0;
//     for (int i = start_idx; i < soft_bound_size; ++i) {
//       const double curr_time = soft_bound_data->time(i);
//       if (!InRange(curr_time, min_t, max_t)) break;
//       end_idx = i;
//       stb_st.emplace_back(soft_bound_data->value(i), curr_time);
//     }

//     // Back point.
//     const auto stb_end_pt =
//         st_boundary->GetBoundarySRange(soft_bound_data->time(end_idx));
//     if (!stb_end_pt.has_value()) return;
//     const double stb_end_point_s = (decision == StBoundaryProto::YIELD ||
//                                     decision == StBoundaryProto::FOLLOW)
//                                        ? stb_end_pt->second
//                                        : stb_end_pt->first;
//     stb_st.emplace_back(stb_end_point_s, soft_bound_data->time(end_idx));

//     const auto group_name = Log2DDS::TaskPrefix(plan_id) + "st_softbound";
//     Log2DDS::LogChartV2(
//         group_name, st_boundary->id() + "_softbound", Log2DDS::kGray, false,
//         stb_st, [](const StPoint& p) -> double { return p.t(); },
//         [](const StPoint& p) -> double { return p.s(); });
//   }
// }
void DumpConflictZoneGraph(std::string& group_name, std::string& key,
                           const GamingConflictZone& sim_conflict_zone) {
  SpeedVector obj_stBoundary;

  SpeedPoint cutin_point;
  SpeedPoint cutout_point;
  double cutin_s = sim_conflict_zone.ego_cutin_s;
  double cutin_t = sim_conflict_zone.agent_cutin_time;
  cutin_point.set_s(cutin_s);
  cutin_point.set_t(cutin_t);
  obj_stBoundary.emplace_back(cutin_point);

  double cutout_s = sim_conflict_zone.ego_cutout_s;
  double cutout_t = sim_conflict_zone.agent_cutout_time;
  cutout_point.set_s(cutout_s);
  cutout_point.set_t(cutout_t);
  obj_stBoundary.emplace_back(cutout_point);

  Log2DDS::LogChartV2(
      group_name, "boundary/" + key, Log2DDS::kGray, false, obj_stBoundary,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });
}

void DumpConflictZoneGraph(std::string& group_name, const std::string& key,
                           const Log2DDS::Color& color,
                           const GamingConflictZone& sim_conflict_zone,
                           const LonGamingDecisionType decision_type) {
  std::string lon_decision;
  if (decision_type == LonGamingDecisionType::kYield) {
    lon_decision = "Y";
  } else if (decision_type == LonGamingDecisionType::kOvertake) {
    lon_decision = "O";
  } else if (decision_type == LonGamingDecisionType::kNone) {
    lon_decision = "N";
  }

  SpeedVector obj_stBoundary;

  SpeedPoint cutin_point;
  SpeedPoint cutout_point;
  double cutin_s = sim_conflict_zone.ego_cutin_s;
  double cutin_t = sim_conflict_zone.agent_cutin_time;
  cutin_point.set_s(cutin_s);
  cutin_point.set_t(cutin_t);
  obj_stBoundary.emplace_back(cutin_point);

  double cutout_s = sim_conflict_zone.ego_cutout_s;
  double cutout_t = sim_conflict_zone.agent_cutout_time;
  cutout_point.set_s(cutout_s);
  cutout_point.set_t(cutout_t);
  obj_stBoundary.emplace_back(cutout_point);

  Log2DDS::LogChartV2(
      group_name + "/st", "boundary/" + key + "-" + lon_decision, color, false,
      obj_stBoundary, [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });
}

void DumpSimResultGraph(std::string& group_name, const std::string& obj_id,
                        const Log2DDS::Color& color,
                        const GamingSimResult& sim_result) {
  SpeedVector ego_speed_data = sim_result.ego_speed_data;
  GamingConflictZone conflict_zone = sim_result.sim_conflict_zone_in_ego_view;

  DumpConflictZoneGraph(group_name, obj_id, color, conflict_zone,
                        sim_result.ego_lon_type);

  DumpGamingSpeedVectorGraph(group_name, obj_id, color, ego_speed_data);
}

// void DumpVtGraphSpeedLimit(int plan_id, const SpeedFinderDebugProto&
// speed_finder_proto)
// {
//     const std::unordered_map<SpeedLimitTypeProto::Type, Log2DDS::Color> types
//     = {
//         {SpeedLimitTypeProto_Type_LANE, Log2DDS::kBlack}};
//     auto group_name = Log2DDS::TaskPrefix(plan_id) + "vt_limit";
//     for (const auto& [type, color] : types)
//     {
//         const auto type_name = SpeedLimitTypeProto::Type_Name(type);
//         const auto* data =
//             FindOrNull(speed_finder_proto.speed_optimizer().speed_limit(),
//             type_name);
//         if (data == nullptr)
//         {
//             continue;
//         }
//         CHECK_EQ(data->time_size(), data->value_size());
//         constexpr double kMaxPlotSpeedLimit = 40.0; // m/s
//         constexpr double kEpsilon = 1.0e-6;
//         SpeedVector speed_limit;
//         speed_limit.reserve(data->time_size());
//         std::vector<Log2DDS::ChartInfos> infos;
//         infos.reserve(data->time_size());
//         for (int i = 0; i < static_cast<int>(data->time_size()); ++i)
//         {
//             const auto time = data->time(i);
//             const auto value = data->value(i);
//             speed_limit.emplace_back(time, 0.0, std::min(value,
//             kMaxPlotSpeedLimit), 0.0, 0.0); Log2DDS::ChartInfos info;
//             info.set_name("info");
//             info.set_value(data->info(i));
//             infos.emplace_back(info);
//         }
//         Log2DDS::LogChartV2(
//             group_name, type_name, color, false, speed_limit,
//             [](const SpeedPoint& p) -> double { return p.t(); },
//             [](const SpeedPoint& p) -> double { return p.v(); }, infos);
//     }
// }

void DumpGamingSpeedVectorGraph(std::string& group_name, const std::string& key,
                                const Log2DDS::Color& color,
                                const SpeedVector& gaming_speed) {
  // gaming s-t
  Log2DDS::LogChartV2(
      group_name + "/st", "ego/" + key, color, false, gaming_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.s(); });

  // gaming v-t
  Log2DDS::LogChartV2(
      group_name + "/vt", "ego/" + key, color, false, gaming_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.v(); }, /*infos=*/{});

  // gaming a-t
  Log2DDS::LogChartV2(
      group_name + "/at", "ego/" + key, color, false, gaming_speed,
      [](const SpeedPoint& p) -> double { return p.t(); },
      [](const SpeedPoint& p) -> double { return p.a(); });
}

std::string ConvertEgoData2Log(const DiscretizedPath& av_path) {
  std::ostringstream log_stream;
  log_stream << "********************** av_path ***********************\n";
  log_stream << "x\ty\ttheta\ts\n";
  for (const auto& point : av_path) {
    log_stream << std::fixed << std::setprecision(5)
               << absl::StrCat(point.x(), "\t", point.y(), "\t", point.theta(),
                               "\t", point.s(), "\n");
  }
  return log_stream.str();
}

std::string ConvertEgoSpeedData2Log(const SpeedVector& speed_data) {
  std::ostringstream log_stream;

  log_stream
      << "********************** ego_speed_data ***********************\n";
  log_stream << "s | v | a | j | t\n";
  for (auto& point : speed_data) {
    log_stream << std::fixed << std::setprecision(5)
               << absl::StrCat(point.s(), "\t", point.v(), "\t", point.a(),
                               "\t", point.j(), "\t", point.t(), "\n");
  }
  return log_stream.str();
}

std::string ConvertObjSpeedData2Log(const SpeedVector& speed_data) {
  std::ostringstream log_stream;

  log_stream
      << "********************** obj_speed_data ***********************\n";
  log_stream << "s | v | a | j | t\n";
  for (auto& point : speed_data) {
    log_stream << std::fixed << std::setprecision(5)
               << absl::StrCat(point.s(), "\t", point.v(), "\t", point.a(),
                               "\t", point.j(), "\t", point.t(), "\n");
  }
  return log_stream.str();
}

std::string ConvertEgoSpeedDataAndObjSpeedData2Log(
    const GamingSimResult& sim_result) {
  std::string log_str;

  log_str += ConvertEgoSpeedData2Log(sim_result.ego_speed_data);
  log_str += ConvertObjSpeedData2Log(sim_result.obj_speed_data);

  return log_str;
}
std::string ConvertObjData2Log(const GamingSimResult& sim_result) {
  std::ostringstream log_stream;
  log_stream << "obj_traj:x y yaw s v a t\n";
  for (int i = 0; i < sim_result.obj_traj.size(); i++) {
    const auto& point = sim_result.obj_traj[i];
    log_stream << std::fixed << std::setprecision(2) << point.pos().x() << " "
               << point.pos().y() << " " << point.theta() << " " << point.s()
               << " " << point.v() << " " << point.a() << " " << point.t()
               << "\n";
  }
  return log_stream.str();
}

std::string ConvertEgoData2Log(const GamingSimResult& sim_result) {
  std::ostringstream log_stream;
  log_stream << "ego_traj:x y yaw s v a t\n";
  for (int i = 0; i < sim_result.ego_traj.size(); i++) {
    const auto& point = sim_result.ego_traj[i];
    log_stream << std::fixed << std::setprecision(2) << point.pos().x() << " "
               << point.pos().y() << " " << point.theta() << " " << point.s()
               << " " << point.v() << " " << point.a() << " " << point.t()
               << "\n";
  }
  return log_stream.str();
}
}  // namespace planning
}  // namespace st

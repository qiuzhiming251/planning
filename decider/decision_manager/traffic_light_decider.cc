

#include "decider/decision_manager/traffic_light_decider.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "plan_common/log_data.h"
#include "plan_common/speed/ad_byd_speed/constant_acc_builder.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/log.h"
//#include "global/logging.h"
//#include "lite/check.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/geometry/halfplane.h"
#include "plan_common/math/util.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "modules/cnoa_pnc/planning/proto/piecewise_linear_function.pb.h"
#include "modules/cnoa_pnc/planning/proto/traffic_light_info.pb.h"
#include "plan_common/plan_common_defs.h"
//#include "planner/speed_optimizer/speed_point.h"
#include "plan_common/util/map_util.h"
#include "plan_common/util/vehicle_geometry_util.h"

namespace st {
namespace planning {

namespace {
constexpr double kTrafficLightStandoff = 1.5;           // m.
constexpr double kTrafficLightOutFovStandoffMin = 0.0;  // m.
constexpr double kTrafficLightOutFovStandoffMax = 3.5;  // m.
constexpr double kTrafficLightindicationDistThrs = 20;  // m.
constexpr double kEpsilon = 1e-5;
constexpr double kExtraLeftWaitStopBuffer = 1.5;
}  // namespace

using TrafficLightStatusMap = ad_byd::planning::TrafficLightStatusMap;
using FsdTrafficLightDeciderInfo = ad_byd::planning::FsdTrafficLightDeciderInfo;
using StopLineReason = ad_byd::planning::StopLineReason;
using StopLineInterface = ad_byd::planning::StopLineInterface;
using LightStatus = ad_byd::planning::LightStatus;
using CompositeTurnType = ad_byd::planning::CompositeTurnType;
using LanePtr = ad_byd::planning::LanePtr;

bool LeftWaitCheck(const mapping::ElementId left_wait_lane,
                   const mapping::ElementId focus_lane,
                   const LightStatus left_wait_lane_light,
                   const LightStatus focus_lane_light, bool has_set_leftwait,
                   bool ego_is_in_leftwait) {
  if (left_wait_lane != mapping::kInvalidElementId &&
      left_wait_lane_light == LightStatus::RED_LIGHT) {
    return false;
  }
  if ((left_wait_lane != mapping::kInvalidElementId && has_set_leftwait) ||
      (ego_is_in_leftwait && has_set_leftwait)) {
    return true;
  }
  if (left_wait_lane != mapping::kInvalidElementId &&
      focus_lane != mapping::kInvalidElementId &&
      left_wait_lane_light == LightStatus::GREEN_LIGHT &&
      (focus_lane_light == LightStatus::RED_LIGHT ||
       focus_lane_light == LightStatus::FAIL_DETECTION)) {
    return true;
  }
  return false;
}

double ComputeDisToStopLine(
    const DrivePassage& passage, ad_byd::planning::StopLinePtr stop_line,
    const ApolloTrajectoryPointProto& plan_start_point) {
  double stop_line_s = std::numeric_limits<double>::max();
  double start_point_s = std::numeric_limits<double>::max();
  const auto& start_point = plan_start_point.path_point();
  if (stop_line && !(stop_line->id() == 0) && stop_line->points().size() > 1) {
    const double stop_line_x =
        (stop_line->points().front().x() + stop_line->points().back().x()) *
        0.5;
    const double stop_line_y =
        (stop_line->points().front().y() + stop_line->points().back().y()) *
        0.5;
    if (passage.lane_seq_info() && passage.lane_seq_info()->lane_seq) {
      const auto& laneseq = passage.lane_seq_info()->lane_seq;
      laneseq->GetProjectionDistance({stop_line_x, stop_line_y}, &stop_line_s);
      laneseq->GetProjectionDistance({start_point.x(), start_point.y()},
                                     &start_point_s);
      if (stop_line_s == std::numeric_limits<double>::max() ||
          start_point_s == std::numeric_limits<double>::max()) {
        return std::numeric_limits<double>::max();
      }
      return stop_line_s - start_point_s;
    } else {
      std::vector<std::string> tl_input_debug;
      tl_input_debug.emplace_back(" NO lane_seq !");
      Log2DDS::LogDataV2("fsd-traffic", tl_input_debug);
      return std::numeric_limits<double>::max();
    }
  } else {
    std::vector<std::string> tl_input_debug;
    tl_input_debug.emplace_back(" stop_line size short !");
    Log2DDS::LogDataV2("fsd-traffic", tl_input_debug);
    return std::numeric_limits<double>::max();
  }
}

bool IfNeedGreenBlinkLightBrake(
    double front_edge_to_center_dis, double dis_to_stopline,
    const ApolloTrajectoryPointProto& plan_start_point,
    const StopLineReason& last_stopline_reason, uint32_t light_count) {
  std::vector<std::string> tl_output_debug;
  // Quick Judgement Condition
  if (last_stopline_reason == StopLineReason::REASON_GREENBLINK) {
    tl_output_debug.emplace_back(" last_stopline_reason is REASON_GREENBLINK ");
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    return true;
  }
  // Judgement Condition based on Time Window
  constexpr double kKeepTimeThr = 3.5;  // Seconds.
  constexpr double kYellowKeepTimeThr = 1.0;
  double keep_time =
      std::clamp(light_count + kYellowKeepTimeThr, 0.0, kKeepTimeThr);
  double keep_distance =
      std::max(plan_start_point.v() * keep_time +
                   0.5 * plan_start_point.a() * keep_time * keep_time,
               0.0);
  if (keep_distance < dis_to_stopline) {
    tl_output_debug.push_back(absl::StrCat("keep_time", keep_time));
    tl_output_debug.push_back(absl::StrCat("keep_distance", keep_distance));
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    return true;
  }
  return false;
}

bool IfNeedYellowLightBrake(double front_edge_to_center_dis,
                            double dis_to_stopline,
                            const ApolloTrajectoryPointProto& plan_start_point,
                            const StopLineReason& last_stopline_reason,
                            bool has_green_to_yellow,
                            double has_green_to_yellow_time,
                            double plan_start_time) {
  std::vector<std::string> tl_output_debug;
  if (last_stopline_reason == StopLineReason::REASON_LIGHT_YELLOW) {
    tl_output_debug.emplace_back(
        " last_stopline_reason is REASON_LIGHT_YELLOW ");
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    return true;
  }
  if (!has_green_to_yellow) {
    tl_output_debug.emplace_back("!has_green_to_yellow");
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    return true;
  }

  double head_to_stopline = dis_to_stopline - front_edge_to_center_dis;
  double target_a = -4.0;
  double target_v = 0.0;
  const double dt = 5.0;
  auto speed_builder = std::make_shared<ad_byd::planning::ConstantAccBuilder>();
  ad_byd::planning::SpeedPoint start_sp(0.0, 0.0, plan_start_point.v(),
                                        plan_start_point.a(),
                                        plan_start_point.j());
  ad_byd::planning::SpeedPoint target_sp(dt, 0.0, target_v, target_a, 0.0);
  auto speed_profile = speed_builder->Build(start_sp, target_sp, dt);
  double target_s = speed_profile->GetSpeedPointAtTime(dt).s;
  double target_v_f = speed_profile->GetSpeedPointAtTime(dt).v;
  double add_dis = 0.0;
  if (target_v_f > 0.0) {
    add_dis = -0.5 * (target_v_f * target_v_f) / target_a;
  }
  double brake_distance = target_s + add_dis;
  double brake_buffer = 1.0;
  tl_output_debug.emplace_back(
      absl::StrCat("yellow-brake_distance:", brake_distance));
  tl_output_debug.emplace_back(
      absl::StrCat("yellow-head_to_stopline:", head_to_stopline));
  if (brake_distance < (head_to_stopline + brake_buffer)) {
    tl_output_debug.emplace_back(" can brake before stopline(buffer 1.0)");
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    return true;
  }

  constexpr double kTotalYellowTime = 2.25;
  double keep_time =
      kTotalYellowTime - (plan_start_time - has_green_to_yellow_time);
  keep_time = std::max(keep_time, 0.0);
  double keep_distance = plan_start_point.v() * keep_time +
                         0.5 * plan_start_point.a() * keep_time * keep_time;
  if (keep_distance < dis_to_stopline) {
    tl_output_debug.emplace_back("yellow light brake 2.25");
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    return true;
  }
  return false;
}

bool ExtendStopLines(const PlannerSemanticMapManager& psmm,
                     const DrivePassage& passage,
                     const uint64_t first_virtual_lane_id,
                     const uint64_t focus_lane_id,
                     std::vector<ConstraintProto::StopLineProto>& tl_stop_lines,
                     double stop_s, double adapt_tl_standoff,
                     bool need_precede) {
  // if (!psmm.map_ptr()->GetLaneById(first_virtual_lane_id)) return false;
  const auto& first_virtual_lane =
      psmm.map_ptr()->GetLaneById(first_virtual_lane_id);
  if (!passage.lane_seq_info()) return false;
  const auto& laneseq = passage.lane_seq_info()->lane_seq;
  if (!laneseq) return false;
  const LanePtr final_common_lane =
      laneseq->GetPreLaneOnLaneSequence(first_virtual_lane);
  if (!final_common_lane) return false;
  const auto& first_focus_lane =
      need_precede ? final_common_lane : first_virtual_lane;
  const auto& final_section =
      psmm.map_ptr()->GetSectionById(first_focus_lane->section_id());
  if (!final_section) return false;

  for (const auto& lane : final_section->lanes()) {
    if (lane == first_focus_lane->id()) continue;
    const auto& lane_ptr = psmm.map_ptr()->GetLaneById(lane);
    if (!lane_ptr || lane_ptr->points().empty()) continue;
    const Vec2d center = lane_ptr->points().back();
    ConstraintProto::StopLineProto stop_line;
    const auto heading = passage.QueryTangentAngleAtS(stop_s);
    const auto& s = passage.QueryFrenetLonOffsetAt(center);
    if (!s.ok() || stop_s - s.value().accum_s > 15.0) {
      continue;
    }
    if (!heading.ok()) {
      continue;
    }
    const Vec2d unit = Vec2d::FastUnitFromAngle(*heading);
    // const double halfwidth =
    //     0.5 * lane_ptr->GetWidthAtPoint(center.x(), center.y());  // m.
    HalfPlane fence(center - 1.5 * unit.Perp(), center + 1.5 * unit.Perp());
    fence.ToProto(stop_line.mutable_half_plane());
    stop_line.set_s(s.value().accum_s);
    stop_line.set_standoff(0.7);
    stop_line.set_time(0.0);
    stop_line.set_id(
        absl::StrFormat("extended_traffic_light_%d", focus_lane_id));
    stop_line.set_is_extended(true);
    stop_line.set_is_traffic_light(true);
    stop_line.mutable_source()->mutable_traffic_light()->set_lane_id(lane);
    stop_line.mutable_source()->mutable_traffic_light()->set_id(focus_lane_id);
    tl_stop_lines.push_back(std::move(stop_line));
    std::vector<Vec2d> stop_line_points;
    auto left_point = center - 1.5 * unit.Perp();
    stop_line_points.push_back(left_point);
    auto right_point = center + 1.5 * unit.Perp();
    stop_line_points.push_back(right_point);
    Log2DDS::LogLineV2(
        absl::StrCat("extended-traffic-stop-line-", focus_lane_id),
        Log2DDS::kYellow, {}, stop_line_points);
  }
  return true;
}

FsdTrafficLightDeciderInfo GetFsdTrafficLightDeciderInfo(
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const Behavior_FunctionId& map_func_id,
    const ApolloTrajectoryPointProto& plan_start_point) {
  FsdTrafficLightDeciderInfo info;
  double distance = 0.0;
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_info = traffic_light_status_map.find(seg.lane_id);
    if (lane_info == traffic_light_status_map.end()) {
      break;
    }
    const auto& traffic_light_status = lane_info->second;
    if (traffic_light_status.junction_id.has_value() &&
        info.first_virtual_lane == mapping::kInvalidElementId &&
        traffic_light_status.light_status !=
            ad_byd::planning::LightStatus::NONE_LIGHT) {
      info.first_virtual_lane = seg.lane_id;
      info.first_virtual_lane_light = traffic_light_status.light_status;
      info.light_countdown = traffic_light_status.light_countdown;
      info.stopline_angle_flag = traffic_light_status.stopline_angle_flag;
      info.light_turn_type = traffic_light_status.light_turn_type;
      info.first_virtual_lane_is_leftwait =
          traffic_light_status.is_left_wait_lane;
      info.dist_to_stopline = distance;
    }
    if (traffic_light_status.junction_id.has_value() &&
        !traffic_light_status.is_left_wait_lane &&
        traffic_light_status.light_status !=
            ad_byd::planning::LightStatus::NONE_LIGHT) {
      info.focus_lane = seg.lane_id;
      info.focus_lane_light = traffic_light_status.light_status;
      info.light_countdown = traffic_light_status.light_countdown;
      info.stopline_angle_flag = traffic_light_status.stopline_angle_flag;
      info.light_turn_type = traffic_light_status.light_turn_type;
      info.dist_to_leftwait_stopline = distance;
      break;
    }
    if (traffic_light_status.junction_id.has_value() &&
        traffic_light_status.is_left_wait_lane &&
        traffic_light_status.light_status !=
            ad_byd::planning::LightStatus::NONE_LIGHT) {
      info.left_wait_lane = seg.lane_id;
      info.left_wait_lane_light = traffic_light_status.light_status;
      info.light_countdown = traffic_light_status.light_countdown;
      info.stopline_angle_flag = traffic_light_status.stopline_angle_flag;
      info.light_turn_type = traffic_light_status.light_turn_type;
      info.focus_lane = seg.lane_id;
      info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
      info.dist_to_leftwait_stopline = distance;
    }
    distance += (seg.end_s - seg.start_s);
  }

  const auto& start_lane_info =
      traffic_light_status_map.find(lane_path_from_start.lane_ids().front());
  if (start_lane_info != traffic_light_status_map.end()) {
    info.ego_is_in_leftwait = start_lane_info->second.is_left_wait_lane;
  }

  std::vector<std::string> tl_input_debug;
  tl_input_debug.reserve(7);
  tl_input_debug.emplace_back(
      absl::StrCat("dist_to_stopline:", info.dist_to_stopline));
  tl_input_debug.emplace_back(absl::StrCat("dist_to_leftwait_stopline:",
                                           info.dist_to_leftwait_stopline));
  tl_input_debug.emplace_back(
      absl::StrCat("ego_is_in_leftwait:", info.ego_is_in_leftwait));
  tl_input_debug.emplace_back(
      absl::StrCat("virtual_lane:", info.first_virtual_lane, "*",
                   info.first_virtual_lane_light));
  tl_input_debug.emplace_back(
      absl::StrCat("focus_lane:", info.focus_lane, "*", info.focus_lane_light));
  tl_input_debug.emplace_back(absl::StrCat(
      "left_wait_lane:", info.left_wait_lane, "*", info.left_wait_lane_light));
  tl_input_debug.emplace_back(
      absl::StrCat("light_countdown:", info.light_countdown));
  Log2DDS::LogDataV2("fsd-traffic", tl_input_debug);
  return info;
}

FsdTrafficLightDeciderInfo GetFsdTrafficLightDeciderInfoForLCC(
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const Behavior_FunctionId& map_func_id,
    const ApolloTrajectoryPointProto& plan_start_point) {
  FsdTrafficLightDeciderInfo info;
  const auto& stop_line_map = psmm.map_ptr()->stop_line_map();
  if (stop_line_map.empty()) {
    return info;
  }
  const auto& lane_ptr = psmm.map_ptr()->lanes().front();
  if (!lane_ptr) {
    return info;
  }

  const auto& stop_line_info = stop_line_map.begin()->second;
  const double raw_dist =
      ComputeDisToStopLine(passage, stop_line_info, plan_start_point);

  info.first_virtual_lane = lane_ptr->id();
  info.first_virtual_lane_light = stop_line_info->light_type();
  info.focus_lane = lane_ptr->id();
  info.focus_lane_light = stop_line_info->light_type();
  info.dist_to_stopline = raw_dist;

  std::vector<std::string> tl_input_debug;
  tl_input_debug.emplace_back(
      absl::StrCat("dist_to_stopline:", info.dist_to_stopline));
  // tl_input_debug.emplace_back(absl::StrCat("focus_lane:", info.focus_lane,
  // "*",
  //                                          info.focus_lane_light));
  tl_input_debug.emplace_back(
      absl::StrCat("traffic light: ", info.focus_lane_light));
  Log2DDS::LogDataV2("fsd-traffic", tl_input_debug);
  return info;
}

FsdTrafficLightDeciderInfo GetFsdTrafficLightDeciderInfoForMapless(
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const Behavior_FunctionId& map_func_id,
    const ApolloTrajectoryPointProto& plan_start_point, bool& need_precede) {
  FsdTrafficLightDeciderInfo info;
  const auto& stop_line_map = psmm.map_ptr()->stop_line_map();
  for (const auto& seg : lane_path_from_start) {
    const auto& lane_ptr = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if (!lane_ptr) {
      break;
    }
    if (lane_ptr->overlap_stop_lines().empty()) {
      continue;
    }
    // TODO(qiao)
    // if (lane_ptr->overlap_stop_lines().front().substr(0, 3) == "vr_") {
    //   continue;
    // }
    const auto& stop_line_found =
        stop_line_map.find(lane_ptr->overlap_stop_lines().front());
    if (stop_line_found == stop_line_map.end()) {
      break;
    }
    const auto& stop_line_info = stop_line_found->second;
    if (stop_line_info->virtual_type() == 0) {
      continue;
    }
    const double raw_dist =
        ComputeDisToStopLine(passage, stop_line_info, plan_start_point);
    if (!lane_ptr->overlap_stop_lines().empty() &&
        info.first_virtual_lane == mapping::kInvalidElementId) {
      info.first_virtual_lane = lane_ptr->id();
      info.first_virtual_lane_light = stop_line_info->light_type();
      info.first_virtual_lane_is_leftwait = (stop_line_info->sub_type() == 4);
      info.dist_to_stopline = raw_dist;
    }
    if (!lane_ptr->overlap_stop_lines().empty() &&
        stop_line_info->sub_type() != 4) {
      info.focus_lane = lane_ptr->id();
      info.focus_lane_light = stop_line_info->light_type();
      info.dist_to_leftwait_stopline = raw_dist;
      break;
    }
    if (!lane_ptr->overlap_stop_lines().empty() &&
        stop_line_info->sub_type() == 4) {
      info.left_wait_lane = lane_ptr->id();
      info.left_wait_lane_light = stop_line_info->light_type();
      info.focus_lane = lane_ptr->id();
      info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
      info.dist_to_leftwait_stopline = raw_dist;
    }
  }
  if (info.dist_to_stopline == std::numeric_limits<double>::max()) {
    double distance = 0.0;
    for (const auto& seg : lane_path_from_start) {
      const auto& lane_info = traffic_light_status_map.find(seg.lane_id);
      if (lane_info == traffic_light_status_map.end()) {
        break;
      }
      const auto& traffic_light_status = lane_info->second;
      if (traffic_light_status.junction_id.has_value() &&
          info.first_virtual_lane == mapping::kInvalidElementId) {
        info.first_virtual_lane = seg.lane_id;
        info.first_virtual_lane_light = traffic_light_status.light_status;
        info.first_virtual_lane_is_leftwait =
            traffic_light_status.is_left_wait_lane;
        info.dist_to_stopline = distance;
        need_precede = true;
      }
      if (traffic_light_status.junction_id.has_value() &&
          !traffic_light_status.is_left_wait_lane) {
        info.focus_lane = seg.lane_id;
        info.focus_lane_light = traffic_light_status.light_status;
        info.dist_to_leftwait_stopline = distance;
        break;
      }
      if (traffic_light_status.junction_id.has_value() &&
          traffic_light_status.is_left_wait_lane) {
        info.left_wait_lane = seg.lane_id;
        info.left_wait_lane_light = traffic_light_status.light_status;
        info.focus_lane = seg.lane_id;
        info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
        info.dist_to_leftwait_stopline = distance;
      }
      distance += (seg.end_s - seg.start_s);
    }
    if (info.dist_to_stopline == std::numeric_limits<double>::max()) {
      const auto& route_info = psmm.map_ptr()->route()->GetRouteInfo();
      const auto& sections_info = route_info.sections;
      double stop_line_distance_oh = 0.0;
      bool has_start = false;
      for (const auto& section_info : sections_info) {
        if (!has_start) {
          if (section_info.id == route_info.navi_start.section_id) {
            has_start = true;
          } else {
            continue;
          }
        }
        const auto& lane_id =
            static_cast<mapping::ElementId>(section_info.lane_ids.at(0));
        const auto& lane_info_oh = traffic_light_status_map.find(lane_id);
        if (lane_info_oh == traffic_light_status_map.end()) {
          break;
        }
        const auto& traffic_light_status_oh = lane_info_oh->second;
        if (traffic_light_status_oh.junction_id.has_value() &&
            info.first_virtual_lane == mapping::kInvalidElementId) {
          info.first_virtual_lane = lane_id;
          info.first_virtual_lane_light = traffic_light_status_oh.light_status;
          info.first_virtual_lane_is_leftwait =
              traffic_light_status_oh.is_left_wait_lane;
          info.dist_to_stopline = stop_line_distance_oh;
          need_precede = true;
        }
        if (traffic_light_status_oh.junction_id.has_value() &&
            !traffic_light_status_oh.is_left_wait_lane) {
          info.focus_lane = lane_id;
          info.focus_lane_light = traffic_light_status_oh.light_status;
          info.dist_to_leftwait_stopline = stop_line_distance_oh;
          break;
        }
        if (traffic_light_status_oh.junction_id.has_value() &&
            traffic_light_status_oh.is_left_wait_lane) {
          info.left_wait_lane = lane_id;
          info.left_wait_lane_light = traffic_light_status_oh.light_status;
          info.focus_lane = lane_id;
          info.focus_lane_light = LightStatus::UNKNOWN_LIGHT;
        }
        if (section_info.id == route_info.navi_start.section_id) {
          stop_line_distance_oh +=
              section_info.length - route_info.navi_start.s_offset;
        } else {
          stop_line_distance_oh += section_info.length;
        }
      }
    }
  }

  const auto& start_lane_info =
      traffic_light_status_map.find(lane_path_from_start.lane_ids().front());
  if (start_lane_info != traffic_light_status_map.end()) {
    info.ego_is_in_leftwait = start_lane_info->second.is_left_wait_lane;
    if (info.ego_is_in_leftwait) {
      info.dist_to_stopline = 0.0;
    }
  }

  std::vector<std::string> tl_input_debug;
  tl_input_debug.emplace_back(
      absl::StrCat("dist_to_stopline:", info.dist_to_stopline));
  tl_input_debug.emplace_back(absl::StrCat("dist_to_leftwait_stopline:",
                                           info.dist_to_leftwait_stopline));
  tl_input_debug.emplace_back(
      absl::StrCat("ego_is_in_leftwait:", info.ego_is_in_leftwait));
  tl_input_debug.emplace_back(
      absl::StrCat("virtual_lane:", info.first_virtual_lane, "*",
                   info.first_virtual_lane_light));
  tl_input_debug.emplace_back(
      absl::StrCat("focus_lane:", info.focus_lane, "*", info.focus_lane_light));
  tl_input_debug.emplace_back(absl::StrCat(
      "left_wait_lane:", info.left_wait_lane, "*", info.left_wait_lane_light));
  Log2DDS::LogDataV2("fsd-traffic", tl_input_debug);
  return info;
}

bool IfNeedBlockLightReset(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const StopLineReason& last_stop_reason, double dist_to_stopline,
    const st::VehicleGeometryParamsProto& vehicle_geo_params,
    const uint64_t virtual_lane,
    const ApolloTrajectoryPointProto& plan_start_point) {
  // passage.lane_path().lane_seq()->lanes().front()
  //  Decision Check
  if (last_stop_reason != StopLineReason::REASON_LIGHT_RED &&
      last_stop_reason != StopLineReason::REASON_LIGHT_YELLOW) {
    return false;
  }

  if (!passage.lane_seq_info()) return false;
  const auto& laneseq = passage.lane_seq_info()->lane_seq;
  if (!laneseq) return false;

  // const auto& vr_lane = psmm.map_ptr()->GetLaneById(virtual_lane);
  // const LanePtr pre_virtual_lane =
  // laneseq->GetPreLaneOnLaneSequence(vr_lane); CompositeTurnType turn_type =
  //     psmm.map_ptr()->CheckCompositeLane(pre_virtual_lane);
  // if (turn_type != CompositeTurnType::NORMAL_TURN) {
  //   return false;
  // }

  for (const auto& obj : st_traj_mgr.object_trajectories_map()) {
    if (obj.second.empty()) {
      continue;
    }
    const auto& obstacle = obj.second.front();
    if (obstacle->object_type() != OT_VEHICLE &&
        obstacle->object_type() != OT_LARGE_VEHICLE) {
      continue;
    }
    if (obstacle->is_stationary()) {
      continue;
    }

    constexpr double kBlockHeadingThreshold = M_PI / 6.0;
    const double theta_diff = std::fabs(NormalizeAngle(
        obstacle->pose().theta() - plan_start_point.path_point().theta()));
    if (theta_diff > kBlockHeadingThreshold) {
      continue;
    }

    const absl::StatusOr<FrenetBox> frenet_box_or =
        passage.QueryFrenetBoxAtContour(obstacle->contour());
    if (!frenet_box_or.ok()) {
      continue;
    }
    const double ego_front_to_center =
        vehicle_geo_params.front_edge_to_center();
    const double ego_back_to_center = vehicle_geo_params.back_edge_to_center();
    const double ego_half_width = vehicle_geo_params.width() * 0.5;
    double ds = 0.0;
    double dl = 0.0;
    if (frenet_box_or->s_min > ego_front_to_center) {
      ds = frenet_box_or->s_min - ego_front_to_center;
    } else if (frenet_box_or->s_max < (-1.0 * ego_front_to_center)) {
      ds = frenet_box_or->s_max + ego_back_to_center;
    } else {
      ds = 0.0;
    }
    if (frenet_box_or->l_min > ego_half_width) {
      dl = frenet_box_or->l_min - ego_half_width;
    } else if (frenet_box_or->l_max < (-1.0 * ego_half_width)) {
      dl = frenet_box_or->l_max + ego_half_width;
    } else {
      dl = 0.0;
    }

    if (ds > kEpsilon && ds < 20.0 && ds < dist_to_stopline &&
        std::fabs(dl) < kEpsilon) {
      return true;
    }
  }
  return false;
}

void LKATrafficLightPreBrake(
    FsdTrafficLightDeciderStateProto tld_state, double dist_to_stopline,
    bool& need_prebrake_for_lka,
    const ApolloTrajectoryPointProto& plan_start_point) {
  if (dist_to_stopline < kEpsilon || dist_to_stopline > 160.0) {
    return;
  }

  if (tld_state.green_to_yellow() &&
      tld_state.tl_stop_interface() == StopLineInterface::STOP_LINE_NONE) {
    return;
  }

  if (tld_state.pass_junction()) {
    return;
  }

  if (plan_start_point.v() > Kph2Mps(70.0)) {
    need_prebrake_for_lka = true;
  }
}

bool CurrentJunctionTrafficLightDecider(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDeciderStateProto& raw_tld_state, bool enable_tl_ok_btn,
    bool override_passable, const DecisionConstraintConfigProto& config,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    TrafficLightDeciderOutput& output, const Behavior_FunctionId& map_func_id,
    bool& need_prebrake_for_lka, const absl::Time plan_start_time) {
  // ---------------------------------------------------------------------
  // 0 Prepare the required information and entry criteria
  // ---------------------------------------------------------------------
  FsdTrafficLightDeciderInfo fsd_tl_info;
  bool need_precede = false;
  if (map_func_id == Behavior_FunctionId::Behavior_FunctionId_MAPLESS_NOA) {
    fsd_tl_info = GetFsdTrafficLightDeciderInfoForMapless(
        passage, lane_path_from_start, traffic_light_status_map, psmm,
        map_func_id, plan_start_point, need_precede);
  } else if (map_func_id == Behavior_FunctionId::Behavior_FunctionId_LKA) {
    fsd_tl_info = GetFsdTrafficLightDeciderInfoForLCC(
        passage, lane_path_from_start, traffic_light_status_map, psmm,
        map_func_id, plan_start_point);
  } else {
    fsd_tl_info = GetFsdTrafficLightDeciderInfo(passage, lane_path_from_start,
                                                traffic_light_status_map, psmm,
                                                map_func_id, plan_start_point);
  }
  if (fsd_tl_info.first_virtual_lane == mapping::kInvalidElementId ||
      fsd_tl_info.focus_lane == mapping::kInvalidElementId) {
    std::vector<ConstraintProto::StopLineProto> tl_stop_lines;
    std::vector<ConstraintProto::SpeedProfileProto> tl_speed_profiles;
    TrafficLightDeciderStateProto state;
    TrafficLightIndicationInfoProto ind_info;
    output = TrafficLightDeciderOutput{
        .stop_lines = std::move(tl_stop_lines),
        .speed_profiles = std::move(tl_speed_profiles),
        .traffic_light_decider_state = std::move(state),
        .tl_ind_info = std::move(ind_info)};
    return false;
  }
  double front_edge_to_center_dis =
      vehicle_geometry_params.front_edge_to_center();
  bool extern_button_ok = enable_tl_ok_btn;
  bool extern_override_passable = override_passable;
  double start_time = absl::ToUnixMillis(plan_start_time) / 1000.0;
  double start_v = plan_start_point.v();
  double start_a = plan_start_point.a();

  // ---------------------------------------------------------------------
  // 1 Reset TrafficLightDeciderState cause of passing the intersection
  // ---------------------------------------------------------------------
  FsdTrafficLightDeciderStateProto tld_state = raw_tld_state.fsd_tld_state();
  if (map_func_id == Behavior_FunctionId::Behavior_FunctionId_LKA) {
    LKATrafficLightPreBrake(tld_state, fsd_tl_info.dist_to_stopline,
                            need_prebrake_for_lka, plan_start_point);
  }

  tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_NONE);
  if (tld_state.dist_to_junction() < -5.0 ||
      (fsd_tl_info.dist_to_stopline - tld_state.dist_to_junction()) > 12.0 ||
      (fsd_tl_info.dist_to_stopline - tld_state.dist_to_junction()) < -35.0) {
    tld_state.set_last_light_status(LightStatus::NONE_LIGHT);
    tld_state.set_valid_light_status(LightStatus::NONE_LIGHT);
    tld_state.set_light_stop_reason(StopLineReason::REASON_NONE);
    tld_state.set_light_counter(0);
    tld_state.set_green_to_yellow(false);
    tld_state.set_green_to_yellow_time(0.0);
    tld_state.set_set_leftwait(false);
    tld_state.set_pass_junction(false);
    tld_state.set_normal_override_passable(false);
    tld_state.set_change_to_unknown_time(0.0);
    tld_state.set_unknown_check(false);
    tld_state.set_fail_detect_enter_leftwait(false);
    tld_state.clear_multi_junc_tl_info();
    tld_state.set_override_passable(false);
    tld_state.set_override_count(0);
    tld_state.set_block_light_count(0);
    tld_state.set_first_yellow_time(0.0);
    tld_state.set_yellow_keep_time(0.0);
  }

  // if (map_func_id == Behavior_FunctionId::Behavior_FunctionId_LKA ||
  //     map_func_id == Behavior_FunctionId::Behavior_FunctionId_MAPLESS_NOA) {
  //   if (tld_state.dist_to_junction() > 80.0 &&
  //       (fsd_tl_info.dist_to_stopline - tld_state.dist_to_junction() > 0.0))
  //       {
  //     double dis_diff =
  //         fsd_tl_info.dist_to_stopline - tld_state.dist_to_junction();
  //     fsd_tl_info.dist_to_stopline =
  //         tld_state.dist_to_junction() + 0.0 * dis_diff;
  //   }
  // }

  // --------------------------
  // 2 Handling special lights
  // --------------------------
  LightStatus current_front_light = fsd_tl_info.focus_lane_light;
  TurnType current_front_turn_type = fsd_tl_info.light_turn_type;
  // block fail
  bool is_block_fail = false;
  if (current_front_light == LightStatus::BLOCK_FAIL) {
    is_block_fail = true;
    current_front_light = LightStatus::UNKNOWN_LIGHT;
    int block_count_temp = tld_state.block_light_count();
    tld_state.set_block_light_count(block_count_temp + 1);
    if (tld_state.block_light_count() > 3 &&
        IfNeedBlockLightReset(
            psmm, passage, st_traj_mgr,
            StopLineReason(tld_state.light_stop_reason()),
            fsd_tl_info.dist_to_stopline, vehicle_geometry_params,
            fsd_tl_info.first_virtual_lane, plan_start_point)) {
      current_front_light = LightStatus::GREEN_LIGHT;
    }
  } else {
    tld_state.set_block_light_count(0);
  }

  // detect fail
  bool is_fail_detection = false;
  if (current_front_light == LightStatus::FAIL_DETECTION) {
    is_fail_detection = true;
    if (tld_state.pass_junction()) {
      current_front_light = LightStatus::GREEN_LIGHT;
    } else {
      current_front_light = LightStatus::RED_LIGHT;
    }
  }
  if (!extern_override_passable) {
    tld_state.set_override_count(0);
  } else {
    int count_temp = tld_state.override_count();
    tld_state.set_override_count(count_temp + 1);
  }

  if (current_front_light == LightStatus::GREEN_LIGHT ||
      current_front_light == LightStatus::YELLOW_LIGHT ||
      current_front_light == LightStatus::RED_LIGHT) {
    tld_state.set_normal_override_passable(false);
  }

  LightStatus valid_front_light = current_front_light;
  uint32_t light_count = fsd_tl_info.light_countdown;
  TurnType vaild_front_light_type = current_front_turn_type;

  TlColor ind_current_light = TL_COLOR_NOLIGHT;
  switch (valid_front_light) {
    case LightStatus::GREEN_LIGHT:
    case LightStatus::GREEN_BLINKING:
      ind_current_light = TL_COLOR_GREEN;
      break;
    case LightStatus::YELLOW_LIGHT:
    case LightStatus::YELLOW_BLINKING:
      ind_current_light = TL_COLOR_YELLOW;
      break;
    case LightStatus::RED_LIGHT:
      ind_current_light = TL_COLOR_RED;
      break;
    case LightStatus::BLOCK_FAIL:
      ind_current_light = TL_COLOR_BLOCK;
      break;
    case LightStatus::UNKNOWN_LIGHT:
    case LightStatus::FAIL_DETECTION:
    case LightStatus::Blurring_Mode:
      ind_current_light = TL_COLOR_UNKNOWN;
      break;
    case LightStatus::NONE_LIGHT:
      ind_current_light = TL_COLOR_NOLIGHT;
      break;
  }

  TrafficLightDirection ind_turn_type = UNMARKED;
  switch (vaild_front_light_type) {
    case TurnType::NO_TURN:
      ind_turn_type = STRAIGHT;
      break;
    case TurnType::LEFT_TURN:
      ind_turn_type = LEFT;
      break;
    case TurnType::RIGHT_TURN:
      ind_turn_type = RIGHT;
      break;
    case TurnType::U_TURN:
      ind_turn_type = UTURN;
      break;
  }

  IndStatus ind_status = TL_NO_MESSAGE;

  // ----------------------------------------------------
  // 3 Handling GREEN_LIGHT and NONE_LIGHT
  // ----------------------------------------------------
  if (((valid_front_light == LightStatus::GREEN_LIGHT &&
        tld_state.light_stop_reason() != StopLineReason::REASON_GREENBLINK &&
        light_count > 3) ||
       valid_front_light == LightStatus::NONE_LIGHT)) {
    tld_state.set_last_light_status(current_front_light);
    tld_state.set_valid_light_status(valid_front_light);
    tld_state.set_light_stop_reason(StopLineReason::REASON_NONE);
    tld_state.set_dist_to_junction(fsd_tl_info.dist_to_stopline);
    tld_state.set_green_to_yellow(false);
    tld_state.set_green_to_yellow_time(0.0);
    tld_state.set_set_leftwait(false);
    tld_state.set_change_to_unknown_time(0.0);
    tld_state.set_unknown_check(false);
    tld_state.set_fail_detect_enter_leftwait(false);
    tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_NONE);
    tld_state.set_first_yellow_time(0.0);
    tld_state.set_yellow_keep_time(0.0);
    // decision->set_stopline_interface(StopLineInterface::STOP_LINE_NONE);
    if ((fsd_tl_info.dist_to_stopline < kEpsilon &&
         !fsd_tl_info.ego_is_in_leftwait) ||
        is_fail_detection) {
      tld_state.set_pass_junction(true);
    } else {
      tld_state.set_pass_junction(false);
    }
    std::vector<ConstraintProto::StopLineProto> tl_stop_lines;
    std::vector<ConstraintProto::SpeedProfileProto> tl_speed_profiles;
    TrafficLightDeciderStateProto state;
    TrafficLightIndicationInfoProto ind_info;
    auto mutable_fsd_tld_state = state.mutable_fsd_tld_state();
    *mutable_fsd_tld_state = tld_state;
    output = TrafficLightDeciderOutput{
        .stop_lines = std::move(tl_stop_lines),
        .speed_profiles = std::move(tl_speed_profiles),
        .traffic_light_decider_state = std::move(state),
        .tl_ind_info = std::move(ind_info)};
    return true;
  }
  // indication green light pass
  if (valid_front_light == LightStatus::GREEN_LIGHT &&
      tld_state.light_stop_reason() == StopLineReason::REASON_NONE &&
      fsd_tl_info.dist_to_stopline < kTrafficLightindicationDistThrs) {
    ind_status = TL_GREEN_LIGHT_PASS;
  }

  // ----------------------------------------------------
  // 4 Handling GREENBLINKING,YELLOW_LIGHT, RED_LIGHT, UNKNOWN and Blurring_Mode
  // ----------------------------------------------------
  bool set_stopline = false;
  StopLineReason stop_reason = StopLineReason::REASON_NONE;
  double stop_s = 0.0;
  constexpr int32_t kLightCountThrGreen = 3;
  constexpr int32_t kLightCountThrGreenMin = 0;
  constexpr int32_t kLightCountThrYellowMax = 6.0;
  if (fsd_tl_info.dist_to_stopline > kEpsilon) {  // before junction
    // green blinking
    if ((valid_front_light == LightStatus::GREEN_BLINKING ||
         (valid_front_light == LightStatus::GREEN_LIGHT &&
          light_count <= kLightCountThrGreen &&
          light_count > kLightCountThrGreenMin)) &&
        IfNeedGreenBlinkLightBrake(
            front_edge_to_center_dis, fsd_tl_info.dist_to_stopline,
            plan_start_point, StopLineReason(tld_state.light_stop_reason()),
            light_count)) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_GREENBLINK;
      ind_status = TL_GREEN_BLINK_STOP;  // indication green blink light stop
    }
    // green to yellow
    bool green_to_yellow_side =
        (tld_state.valid_light_status() == LightStatus::GREEN_LIGHT ||
         tld_state.valid_light_status() == LightStatus::GREEN_BLINKING) &&
        (valid_front_light == LightStatus::YELLOW_LIGHT ||
         valid_front_light == LightStatus::YELLOW_BLINKING ||
         valid_front_light == LightStatus::UNKNOWN_LIGHT);
    tld_state.set_green_to_yellow(tld_state.green_to_yellow() ||
                                  green_to_yellow_side);
    if (green_to_yellow_side) {
      tld_state.set_green_to_yellow_time(start_time);
    }
    // YELLOW_LIGHT && YELLOWBLINKING
    if ((valid_front_light == LightStatus::YELLOW_LIGHT ||
         valid_front_light == LightStatus::YELLOW_BLINKING) &&
        IfNeedYellowLightBrake(front_edge_to_center_dis,
                               fsd_tl_info.dist_to_stopline, plan_start_point,
                               StopLineReason(tld_state.light_stop_reason()),
                               tld_state.green_to_yellow(),
                               tld_state.green_to_yellow_time(), start_time) &&
        tld_state.yellow_keep_time() < kLightCountThrYellowMax) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_YELLOW;
      ind_status = TL_YELLOW_LIGHT_STOP;
    }
    // RED_LIGHT
    if (valid_front_light == LightStatus::RED_LIGHT) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_RED;
      ind_status = TL_RAD_LIGHT_STOP;
    }
    // UNKNOWN
    if (valid_front_light == LightStatus::UNKNOWN_LIGHT &&
        (tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_RED ||
         tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_YELLOW ||
         tld_state.light_stop_reason() == StopLineReason::REASON_GREENBLINK) &&
        !tld_state.normal_override_passable()) {
      set_stopline = true;
      stop_reason = StopLineReason(tld_state.light_stop_reason());
      ind_status = TL_PAY_ATTENTION_TO_TRAFFICLIGHT;
    }
    // Blurring_Mode
    if (valid_front_light == LightStatus::Blurring_Mode) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_Blurring_Mode;
      ind_status = TL_PAY_ATTENTION_TO_TRAFFICLIGHT;
    }
    // continuous yellow light
    if (stop_reason == StopLineReason::REASON_LIGHT_YELLOW) {
      if (tld_state.light_stop_reason() !=
          StopLineReason::REASON_LIGHT_YELLOW || is_block_fail) {
        tld_state.set_first_yellow_time(start_time);
      }
      double yellow_keep_time = start_time - tld_state.first_yellow_time();
      if (yellow_keep_time > kLightCountThrYellowMax) {
        set_stopline = false;
        stop_reason = StopLineReason::REASON_NONE;
      }
      tld_state.set_yellow_keep_time(yellow_keep_time);
      std::vector<std::string> tl_output_debug;
      tl_output_debug.reserve(3);
      tl_output_debug.push_back(absl::StrCat("plan_start_time:", start_time));
      tl_output_debug.push_back(
          absl::StrCat("first_yellow_time:", tld_state.first_yellow_time()));
      tl_output_debug.push_back(
          absl::StrCat("yellow_keep_time:", yellow_keep_time));
      Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
    }
    if (set_stopline) {
      stop_s = passage.lane_path_start_s() + fsd_tl_info.dist_to_stopline -
               config.normal_offset();
    }
  } else {  // in junction
    // tld_state.set_green_to_yellow(false);
    // tld_state.set_green_to_yellow_time(0.0);
    // RED_LIGHT
    if (valid_front_light == LightStatus::RED_LIGHT &&
        (tld_state.light_stop_reason() != StopLineReason::REASON_NONE ||
         fsd_tl_info.ego_is_in_leftwait)) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_RED;
      ind_status = TL_RAD_LIGHT_STOP;
    }
    // YELLOW_LIGHT
    if (valid_front_light == LightStatus::YELLOW_LIGHT &&
        start_v < Kph2Mps(10.0) &&
        tld_state.light_stop_reason() != StopLineReason::REASON_NONE) {
      set_stopline = true;
      stop_reason = StopLineReason::REASON_LIGHT_YELLOW;
      ind_status = TL_YELLOW_LIGHT_STOP;
    }
    // UNKNOWN
    if (valid_front_light == LightStatus::UNKNOWN_LIGHT &&
        (tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_RED ||
         tld_state.light_stop_reason() ==
             StopLineReason::REASON_LIGHT_YELLOW)) {
      set_stopline = true;
      stop_reason = StopLineReason(tld_state.light_stop_reason());
      ind_status = TL_PAY_ATTENTION_TO_TRAFFICLIGHT;
    }
    //
    if (tld_state.pass_junction() || tld_state.override_passable()) {
      set_stopline = false;
      stop_reason = StopLineReason::REASON_NONE;
    }
    if (!set_stopline &&
        (!fsd_tl_info.ego_is_in_leftwait ||
         tld_state.light_stop_reason() == StopLineReason::REASON_NONE)) {
      tld_state.set_pass_junction(true);
    }
    if (set_stopline) {
      stop_s = passage.lane_path_start_s() - config.normal_offset();
    }
  }

  // ----------------------------------------------------
  // 5 Handling left wait
  // ----------------------------------------------------
  tld_state.set_fail_detect_enter_leftwait(
      is_fail_detection && fsd_tl_info.dist_to_stopline > kEpsilon &&
      fsd_tl_info.first_virtual_lane_is_leftwait &&
      fsd_tl_info.first_virtual_lane_light == LightStatus::FAIL_DETECTION &&
      (extern_button_ok || tld_state.fail_detect_enter_leftwait()));
  if (tld_state.fail_detect_enter_leftwait()) {
    tld_state.set_set_leftwait(true);

    std::vector<std::string> tl_output_debug;
    tl_output_debug.reserve(1);
    tl_output_debug.emplace_back("ok-button: enter leftwait");
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
  }
  if (set_stopline &&
      LeftWaitCheck(fsd_tl_info.left_wait_lane, fsd_tl_info.focus_lane,
                    fsd_tl_info.left_wait_lane_light,
                    fsd_tl_info.focus_lane_light, tld_state.set_leftwait(),
                    fsd_tl_info.ego_is_in_leftwait)) {
    stop_s = passage.lane_path_start_s() +
             fsd_tl_info.dist_to_leftwait_stopline - config.normal_offset() -
             kExtraLeftWaitStopBuffer;
    tld_state.set_set_leftwait(true);
  } else {
    tld_state.set_set_leftwait(false);
  }

  // ----------------------------------------------------
  // 6 Interface
  // ----------------------------------------------------
  // unknown ok button info
  if (tld_state.valid_light_status() != LightStatus::UNKNOWN_LIGHT &&
      valid_front_light == LightStatus::UNKNOWN_LIGHT) {
    tld_state.set_change_to_unknown_time(start_time);
  } else if (valid_front_light != LightStatus::UNKNOWN_LIGHT) {
    tld_state.set_change_to_unknown_time(0.0);
  }
  if (!tld_state.unknown_check() && set_stopline &&
      fsd_tl_info.dist_to_stopline < kEpsilon && start_v < 0.5 &&
      valid_front_light == LightStatus::UNKNOWN_LIGHT &&
      (tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_RED ||
       tld_state.light_stop_reason() == StopLineReason::REASON_LIGHT_YELLOW)) {
    double unknown_time = start_time - tld_state.change_to_unknown_time();
    double time_buffer = 15.0;
    if (is_block_fail) {
      time_buffer = 10.0;
    } else if (fsd_tl_info.ego_is_in_leftwait) {
      time_buffer = 5.0;
    }
    if (unknown_time > time_buffer &&
        tld_state.change_to_unknown_time() > kEpsilon) {
      tld_state.set_unknown_check(true);
    }
  }

  if (!fsd_tl_info.ego_is_in_leftwait &&
      fsd_tl_info.dist_to_stopline < kEpsilon &&
      tld_state.override_count() > 3) {
    tld_state.set_override_passable(true);
  }

  if ((fsd_tl_info.dist_to_stopline > kEpsilon &&
       fsd_tl_info.dist_to_stopline < 30.0 && tld_state.override_count() > 4)) {
    tld_state.set_normal_override_passable(true);
  }

  // interface
  if (tld_state.unknown_check() ||
      (is_fail_detection &&
       (fsd_tl_info.dist_to_stopline - config.normal_offset()) <
           std::fmax(start_v * 6.0, 10.0))) {
    if (tld_state.unknown_check()) {
      tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_UNKNOWN);
      // decision->set_stopline_interface(StopLineInterface::STOP_LINE_UNKNOWN);
    } else if (is_fail_detection) {
      tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_FAIL_DETECT);
      // decision->set_stopline_interface(
      //     StopLineInterface::STOP_LINE_FAIL_DETECT);
    }
    if (extern_button_ok) {
      tld_state.set_pass_junction(true);
    }

    if (is_fail_detection && fsd_tl_info.dist_to_stopline > kEpsilon &&
        fsd_tl_info.first_virtual_lane_is_leftwait &&
        fsd_tl_info.first_virtual_lane_light != LightStatus::FAIL_DETECTION) {
      tld_state.set_pass_junction(false);
      tld_state.set_tl_stop_interface(stop_reason);
      // decision->set_stopline_interface((int)stop_reason);
    }
    if (tld_state.fail_detect_enter_leftwait()) {
      tld_state.set_pass_junction(false);
      tld_state.set_tl_stop_interface(stop_reason);
      // decision->set_stopline_interface((int)stop_reason);
    }
    if (tld_state.pass_junction()) {
      tld_state.set_tl_stop_interface(StopLineInterface::STOP_LINE_NONE);
      // decision->set_stopline_interface(StopLineInterface::STOP_LINE_NONE);
    }
  } else {
    tld_state.set_tl_stop_interface(stop_reason);
    // decision->set_stopline_interface((int)stop_reason);
  }

  // output
  std::vector<ConstraintProto::StopLineProto> tl_stop_lines;
  std::vector<ConstraintProto::SpeedProfileProto> tl_speed_profiles;
  if (set_stopline) {
    const auto& curbs = passage.QueryCurbPointAtS(stop_s);
    if (!curbs.ok()) {
      TrafficLightDeciderStateProto state;
      TrafficLightIndicationInfoProto ind_info;
      output = TrafficLightDeciderOutput{
          .stop_lines = std::move(tl_stop_lines),
          .speed_profiles = std::move(tl_speed_profiles),
          .traffic_light_decider_state = std::move(state),
          .tl_ind_info = std::move(ind_info)};
      return false;
    }
    double adapt_tl_standoff = config.tl_standoff();
    if (start_v > Kph2Mps(config.tl_standoff_adapt_v())) {
      double dist_to_stop = fsd_tl_info.dist_to_stopline -
                            config.normal_offset() - front_edge_to_center_dis;
      double ttc = dist_to_stop / std::max(1e-6, start_v);
      adapt_tl_standoff =
          PiecewiseLinearFunctionFromProto(config.tl_standoff_ttc_plf())(ttc);
    }

    uint32_t stop_angle_flag = fsd_tl_info.stopline_angle_flag;
    double kTrafficOutofFOVStandOff = 1.5;  // m.
    switch (stop_angle_flag) {
      case 1:
        kTrafficOutofFOVStandOff = 1.5;  // m.
        break;
      case 2:
        kTrafficOutofFOVStandOff = 3.5;  // m.
        break;
      case 3:
        kTrafficOutofFOVStandOff = 3.5;  // m.
        break;
      default:
        break;
    }

    ConstraintProto::StopLineProto stop_line;
    stop_line.set_s(stop_s);
    stop_line.set_standoff(std::clamp(
        std::max(kTrafficLightStandoff, kTrafficOutofFOVStandOff),
        kTrafficLightOutFovStandoffMin, kTrafficLightOutFovStandoffMax));
    stop_line.set_time(0.0);
    stop_line.set_id(
        absl::StrFormat("traffic_light_%d", fsd_tl_info.focus_lane));
    stop_line.mutable_source()->mutable_traffic_light()->set_lane_id(
        fsd_tl_info.focus_lane);
    stop_line.mutable_source()->mutable_traffic_light()->set_id(
        fsd_tl_info.focus_lane);
    stop_line.set_is_traffic_light(true);
    HalfPlane halfplane(curbs->first, curbs->second);
    halfplane.ToProto(stop_line.mutable_half_plane());

    tl_stop_lines.push_back(std::move(stop_line));
    if (map_func_id == Behavior_FunctionId_LKA ||
        map_func_id == Behavior_FunctionId_MAPLESS_NOA) {
      ExtendStopLines(psmm, passage, fsd_tl_info.first_virtual_lane,
                      fsd_tl_info.focus_lane, tl_stop_lines, stop_s,
                      adapt_tl_standoff, need_precede);
    }

    std::vector<std::string> tl_output_debug;
    tl_output_debug.emplace_back(absl::StrCat("fsd-traffic-stop-s:", stop_s));
    Log2DDS::LogDataV2("fsd-traffic", tl_output_debug);
  }

  TrafficLightIndicationInfoProto ind_info;
  ind_info.set_traffic_light_id(
      absl::StrFormat("traffic_light_%d", fsd_tl_info.focus_lane));
  ind_info.set_ind_color(ind_current_light);
  ind_info.set_ind_color_turn_type(ind_turn_type);
  ind_info.set_ind_status(ind_status);

  tld_state.set_last_light_status(current_front_light);
  tld_state.set_valid_light_status(valid_front_light);
  tld_state.set_light_stop_reason(stop_reason);
  tld_state.set_dist_to_junction(fsd_tl_info.dist_to_stopline);
  TrafficLightDeciderStateProto state;
  auto mutable_fsd_tld_state = state.mutable_fsd_tld_state();
  *mutable_fsd_tld_state = tld_state;
  output =
      TrafficLightDeciderOutput{.stop_lines = std::move(tl_stop_lines),
                                .speed_profiles = std::move(tl_speed_profiles),
                                .traffic_light_decider_state = std::move(state),
                                .tl_ind_info = std::move(ind_info)};
  return true;
}

absl::StatusOr<TrafficLightDeciderOutput> BuildTrafficLightConstraints(
    const st::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const TrafficLightStatusMap& traffic_light_status_map,
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDeciderStateProto& tld_state, bool enable_tl_ok_btn,
    bool override_passable, const DecisionConstraintConfigProto& config,
    const Behavior_FunctionId& map_func_id,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    bool traffic_light_fun_enable, bool& need_prebrake_for_lka,
    const absl::Time plan_start_time) {
  TrafficLightDeciderOutput output;
  if (map_func_id == Behavior_FunctionId_LKA && !traffic_light_fun_enable) {
    return output;
  }
  if (!CurrentJunctionTrafficLightDecider(
          vehicle_geometry_params, plan_start_point, passage,
          lane_path_from_start, traffic_light_status_map, psmm, tld_state,
          enable_tl_ok_btn, override_passable, config, st_traj_mgr, output,
          map_func_id, need_prebrake_for_lka, plan_start_time)) {
    return output;
  }

  if (output.traffic_light_decider_state.fsd_tld_state().light_stop_reason() !=
      StopLineReason::REASON_NONE) {
    return output;
  }

  return output;
}
}  // namespace planning
}  // namespace st



#include "plan_common/maps/map.h"

#include <unordered_set>

#include "plan_common/async/async_util.h"
#include "plan_common/async/future.h"
#include "plan_common/gflags.h"
#include "plan_common/log.h"
#include "plan_common/log_data.h"
#include "plan_common/math/double.h"
// #include "plan_common/util/thread_pool.h"
#include <typeindex>

#include "plan_common/async/parallel_for.h"
#include "plan_common/log.h"

namespace ad_byd {
namespace planning {

using KDPoint2D = KDPoint<double, 2, KDValue>;
using Double = math::Double;

Map::Map(const MapInfo &map_info) : thread_pool_(0, "Map") {
  ConvertMapInfo(map_info);
  UpdateLane();
  if (FLAGS_planner_enable_use_exp_traj) {
    UpdateExpTrajectory();
  }
  UpdateJunction();
  UpdateSection();
  UpdateBoundarySection();
  InitAABoxKDTree2d();
}

LaneConstPtr Map::GetLaneById(const uint64_t id) const {
  auto it = lane_map_.find(id);
  return it != lane_map_.end() ? it->second : nullptr;
}

ClearAreaConstPtr Map::GetClearAreaById(const uint64_t id) const {
  auto it = clear_area_map_.find(id);
  return it != clear_area_map_.end() ? it->second : nullptr;
}

StopLineConstPtr Map::GetStopLineById(const uint64_t id) const {
  auto it = stop_line_map_.find(id);
  return it != stop_line_map_.end() ? it->second : nullptr;
}
SpeedBumpConstPtr Map::GetSpeedBumpById(const uint64_t id) const {
  auto it = speed_bump_map_.find(id);
  return it != speed_bump_map_.end() ? it->second : nullptr;
}
LaneConstPtr Map::GetNearestLane(const Point2d &pt, const double &dis) const {
  if (!IsValid()) {
    return nullptr;
  }
  std::map<double, LaneConstPtr> lane_candidate_map;
  for (const auto &it : lane_map_) {
    if (!it.second->center_line().IsValid()) continue;
    auto dis_temp = it.second->center_line().GetDistance(pt);
    if (dis_temp < dis) {
      lane_candidate_map[dis_temp] = it.second;
    }
  }
  LaneConstPtr res = nullptr;
  if (!lane_candidate_map.empty()) {
    res = lane_candidate_map.begin()->second;
  }
  return res;
}

LaneConstPtr Map::GetNearestLane(
    const std::vector<LaneConstPtr> &candidate_lanes, const Point2d &pt,
    const double &heading) const {
  if (!IsValid() || candidate_lanes.empty()) {
    return nullptr;
  }
  int32_t min_idx = 0;
  double min_value = std::numeric_limits<double>::max();
  for (int32_t i = 0; i < static_cast<int32_t>(candidate_lanes.size()); ++i) {
    SLPoint sl_point(0.0, 0.0);
    if (!candidate_lanes.at(i)->GetSLWithoutLimit(pt, &sl_point)) {
      // LINFO("fail to project for lane %s",
      // candidate_lanes.at(i)->id().c_str());
      continue;
    }
    double lane_heading = 0.0;
    if (!candidate_lanes.at(i)->GetHeadingFromS(sl_point.s, &lane_heading)) {
      // LINFO("fail to get heading for lane %s 2 s : %.2f length %.2f ",
      // candidate_lanes.at(i)->id().c_str(), sl_point.s,
      //       candidate_lanes.at(i)->curve_length());
      continue;
    }
    double heading_diff = std::abs(math::AngleDiff(lane_heading, heading));
    // LINFO("Lane %s heading diff %.3f lane heading %.2f/%.2f",
    // candidate_lanes.at(i)->id().c_str(), heading_diff,
    //       lane_heading, heading);
    if (heading_diff < min_value) {
      min_value = heading_diff;
      min_idx = i;
    }
  }
  return candidate_lanes.at(min_idx);
}

LaneConstPtr Map::GetNearestLane(const Point2d &pt, const double &heading,
                                 const double &dis, bool is_navi,
                                 bool is_virtual, double angle_range) const {
  //
  if (!IsValid()) {
    return nullptr;
  }
  std::map<double, LaneConstPtr> lane_candidate_map;
  std::unordered_map<LaneConstPtr, double> angle_diff;
  for (const auto &it : lane_map_) {
    if (!it.second->center_line().IsValid()) continue;
    if (is_navi && !it.second->is_navigation()) continue;
    if (is_virtual && it.second->type() != LANE_VIRTUAL_JUNCTION) continue;
    double s_temp;
    int index_temp1, index_temp2;
    ad_byd::planning::math::Vec2d nearestpoint;
    auto dis_temp =
        it.second->center_line().GetDistance(pt, &nearestpoint, &s_temp);
    index_temp1 = std::min(it.second->center_line().GetIndexByS(s_temp),
                           (int)(it.second->center_line().points().size() - 2));
    index_temp2 = it.second->center_line().GetIndexByS(s_temp + 5.0);
    index_temp2 = index_temp2 == index_temp1 ? index_temp1 + 1 : index_temp2;

    double theta_temp = ad_byd::planning::math::NormalizeAngle(
        (it.second->center_line().points().at(index_temp2) -
         it.second->center_line().points().at(index_temp1))
            .Angle());
    if (dis_temp < dis &&
        fabs(ad_byd::planning::math::NormalizeAngle(theta_temp - heading)) <
            (it.second->turn_type() == U_TURN ? 2 * M_PI / 3.0 : angle_range)) {
      lane_candidate_map[dis_temp] = it.second;
      angle_diff[it.second] =
          fabs(ad_byd::planning::math::NormalizeAngle(theta_temp - heading));
    }
  }
  LaneConstPtr res = nullptr;
  if (!lane_candidate_map.empty() && !angle_diff.empty()) {
    auto it_first = lane_candidate_map.begin();
    auto it_second = std::next(it_first);
    // check if lane_type is special
    auto IsSpecialLane = [](const LaneConstPtr &lane) {
      const auto &type = lane->type();
      return type == LANE_UNKNOWN || type == LANE_EMERGENCY ||
             type == LANE_HARBOR_STOP;
    };
    // only has one candidate
    if (lane_candidate_map.size() == 1) {
      res = it_first->second;
    }
    // if has multiple candidates and the distance difference is small,
    // choose the one with smaller angle diff
    else if ((it_second->first - it_first->first) < 0.5) {
      if (IsSpecialLane(it_first->second) &&
          !IsSpecialLane(it_second->second)) {
        res = it_second->second;
      } else if (!IsSpecialLane(it_first->second) &&
                 IsSpecialLane(it_second->second)) {
        res = it_first->second;
      } else {
        res = (angle_diff[it_first->second] > angle_diff[it_second->second])
                  ? it_second->second
                  : it_first->second;
      }
    } else {
      res = it_first->second;
    }
  }
  return res;
}

LaneConstPtr Map::GetLeftLane(const LaneConstPtr &lane) const {
  if (!IsValid() || !lane) {
    return nullptr;
  }
  auto left_it = lane_map_.find(lane->left_lane_id());
  if (left_it != lane_map_.end()) {
    return left_it->second;
  }
  return nullptr;
}

LaneConstPtr Map::GetRightLane(const LaneConstPtr &lane) const {
  if (!IsValid() || !lane) {
    return nullptr;
  }
  auto right_it = lane_map_.find(lane->right_lane_id());
  if (right_it != lane_map_.end()) {
    return right_it->second;
  }
  return nullptr;
}

void Map::GetPrecedeLanes(const LaneConstPtr &lane,
                          std::vector<LaneConstPtr> *const lanes,
                          bool only_navi) const {
  const double min_diff_dist = 2.0;
  if (!IsValid() || !lane) return;
  for (const auto &id : lane->pre_lane_ids()) {
    auto it = lane_map_.find(id);
    if (it != lane_map_.end()) {
      // check connected - check 1.0m diff
      bool check =
          fabs(it->second->topo_length() - it->second->curve_length()) > 1.0 ||
          fabs(lane->topo_length() - lane->curve_length()) > 1.0;
      if (lane->center_line().IsValid() &&
          it->second->center_line().IsValid() && !check &&
          (lane->center_line().begin_point().DistanceTo(
               it->second->center_line().end_point()) >= min_diff_dist)) {
        // LOG_WARN << "[map] precede connected lane diff > 1.0m";
        continue;
      }
      if (only_navi && !lane->is_navigation()) {
        continue;
      }
      if (only_navi && !it->second->is_navigation()) {
        continue;
      }
      lanes->emplace_back(it->second);
    } else {
      // LOG_ERROR << "next lane is invalid!";
    }
  }
}

std::vector<LaneConstPtr> Map::GetPrecedeLanes(const LaneConstPtr &lane) const {
  std::vector<LaneConstPtr> lanes;
  if (!lane) return lanes;
  lanes.reserve(lane->pre_lane_ids().size());
  for (const auto &id : lane->pre_lane_ids()) {
    auto it = lane_map_.find(id);
    if (it != lane_map_.end() && it->second) {
      lanes.emplace_back(it->second);
    }
  }
  return lanes;
}

std::vector<LaneConstPtr> Map::GetValidNextLanes(
    const LaneConstPtr &lane) const {
  std::vector<LaneConstPtr> lanes;

  if (!IsValid() || !lane) return lanes;
  for (const auto &id : lane->next_lane_ids()) {
    auto it = lane_map_.find(id);
    if (it != lane_map_.end())
    // TODO(qiao)
    // && id.find("nextvr") == id.npos)
    {
      lanes.emplace_back(it->second);
    }
  }
  return lanes;
}

std::vector<LaneConstPtr> Map::GetNextLanes(const LaneConstPtr &lane) const {
  std::vector<LaneConstPtr> lanes;
  const double min_diff_dist = 2.0;
  if (!IsValid() || !lane) return lanes;
  for (const auto &id : lane->next_lane_ids()) {
    auto it = lane_map_.find(id);
    // check connected - check 1.0m diff
    if (it != lane_map_.end()) {
      bool check =
          fabs(it->second->topo_length() - it->second->curve_length()) > 1.0 ||
          fabs(lane->topo_length() - lane->curve_length()) > 1.0;
      if (lane->center_line().IsValid() &&
          it->second->center_line().IsValid() && !check &&
          (lane->center_line().end_point().DistanceTo(
               it->second->center_line().begin_point()) >= min_diff_dist)) {
        LOG_ERROR << "[map] next connected lane diff > 2.0m";
        continue;
      }
      lanes.emplace_back(it->second);
    } else {
      // LOG_ERROR << "next lane is invalid!";
    }
  }
  return lanes;
}

std::vector<std::vector<LaneConstPtr>> Map::GetSortedValidNextSectionsLanes(
    const LaneConstPtr &pre_lane) const {
  // 1. get all next lanes.
  std::vector<LaneConstPtr> all_next_lanes = GetNextLanes(pre_lane);

  // 2. put lanes into their own sections map.
  std::unordered_map<uint64_t, std::vector<LaneConstPtr>> sections;
  for (const auto &lane : all_next_lanes) {
    sections[lane->section_id()].emplace_back(lane);
  }

  // 3. sort lanes in every sections, from left to right.
  auto sort_lanes = [](std::vector<LaneConstPtr> &lanes) {
    std::sort(lanes.begin(), lanes.end(),
              [&](const LaneConstPtr &lane1, const LaneConstPtr &lane2) {
                return lane1 != nullptr && lane2 != nullptr &&
                       lane1->lane_ind_in_section() <
                           lane2->lane_ind_in_section();
              });
  };
  std::vector<std::vector<LaneConstPtr>> sorted_sections;
  for (auto &section : sections) {
    sort_lanes(section.second);
    sorted_sections.emplace_back(section.second);
  }

  // 4. sort sections from left to right.
  auto sort_sections = [](std::vector<std::vector<LaneConstPtr>> &sections) {
    std::sort(sections.begin(), sections.end(),
              [&](const std::vector<LaneConstPtr> &section1,
                  const std::vector<LaneConstPtr> &section2) {
                if (section1.empty() || section2.empty()) {
                  if (!section1.empty()) return true;
                  return false;
                }
                LaneConstPtr lane1 = section1.front();
                LaneConstPtr lane2 = section2.front();
                if (!lane1 || !lane2) {
                  if (lane1) return true;
                  return false;
                }

                if (lane1->points().size() < 2 || lane2->points().size() < 2) {
                  if (lane2->points().size() < 2) return true;
                  return false;
                }

                Vec2d v1 = lane1->center_line().end_point() -
                           lane1->center_line().begin_point();
                Vec2d v2 = lane2->center_line().end_point() -
                           lane2->center_line().begin_point();

                return v1.CrossProd(v2) < 0.0;
              });
  };
  sort_sections(sorted_sections);
  // std::stringstream ss;
  // ss << "[debug] sorted_sections:";
  // for (auto &section : sorted_sections) {
  //   for (const auto &lane : section) {
  //     ss << lane->id() << ", ";
  //   }
  // }
  // LOG(INFO) << ss.str();
  return sorted_sections;
}

LaneConstPtr Map::GetLeftmostValidNextLanes(
    const LaneConstPtr &pre_lane, const bool &navi_is_priority) const {
  std::vector<std::vector<LaneConstPtr>> sorted_sections =
      GetSortedValidNextSectionsLanes(pre_lane);
  LaneConstPtr leftmostLane = nullptr;
  LaneConstPtr leftmostLaneNavi = nullptr;
  for (auto &section : sorted_sections) {
    if (section.empty()) continue;
    if (!section.front()) continue;
    // the first valid lane is the rightmost lane.
    if (!leftmostLane) leftmostLane = section.front();
    if (navi_is_priority) {
      if (section.front()->is_navigation()) {
        leftmostLaneNavi = section.front();
        break;
      }
    } else {
      break;
    }
  }
  // std::stringstream ss;
  // if (leftmostLaneNavi)
  //   ss << "[debug] leftmostLaneNavi:" << leftmostLaneNavi->id();
  // else
  //   ss << "[debug] leftmostLaneNavi:null";
  // if (leftmostLane)
  //   ss << "leftmostLane:" << leftmostLane->id();
  // else
  //   ss << "leftmostLane:null";
  // LOG(INFO) << ss.str();
  return (leftmostLaneNavi && navi_is_priority) ? leftmostLaneNavi
                                                : leftmostLane;
}

LaneConstPtr Map::GetRightmostValidNextLanes(
    const LaneConstPtr &pre_lane, const bool &navi_is_priority) const {
  std::vector<std::vector<LaneConstPtr>> sorted_sections =
      GetSortedValidNextSectionsLanes(pre_lane);
  LaneConstPtr rightmostLane = nullptr;
  LaneConstPtr rightmostLaneNavi = nullptr;
  for (auto section = sorted_sections.rbegin();
       section != sorted_sections.rend(); ++section) {
    if (section->empty()) continue;
    if (!section->back()) continue;
    rightmostLane = section->back();
    if (navi_is_priority) {
      if (section->back()->is_navigation()) {
        rightmostLaneNavi = section->back();
        break;
      }
    } else {
      break;
    }
  }
  // std::stringstream ss;
  // if (rightmostLaneNavi)
  //   ss << "[debug] rightmostLaneNavi:" << rightmostLaneNavi->id();
  // else
  //   ss << "[debug] rightmostLaneNavi:null";
  // if (rightmostLane)
  //   ss << ", rightmostLane:" << rightmostLane->id();
  // else
  //   ss << ", rightmostLane:null";
  // LOG(INFO) << ss.str();
  return (rightmostLaneNavi && navi_is_priority) ? rightmostLaneNavi
                                                 : rightmostLane;
}

LaneSequencePtr Map::GetLaneSequence(const LaneConstPtr &lane,
                                     const bool &navi_is_priority) const {
  LOG_INFO << "Map::GetLaneSequence()";
  auto cur_lane = GetSameLane(lane);
  if (!IsValid() || !cur_lane) {
    return nullptr;
  }
  std::vector<LaneConstPtr> lanes;
  lanes.emplace_back(cur_lane);
  while (cur_lane) {
    cur_lane = GetOptimalNextLane(cur_lane, navi_is_priority);
    if (!cur_lane) {
      LOG_WARN << "[GetLaneSequence] next lane is null";
      break;
    }
    bool flag = false;
    for (const auto &exist_lane : lanes) {
      if (exist_lane->id() == cur_lane->id()) {
        flag = true;
      }
    }
    if (flag) {
      break;
    }
    lanes.emplace_back(cur_lane);
  }
  return std::make_shared<LaneSequence>(lanes);
}

LaneSequencePtr Map::GetSameLaneSequenceV2(
    const LaneSequencePtr &lane_sequence, const double &x, const double &y,
    const double &heading, std::vector<std::string> &debug,
    std::vector<Point2d> &debug_match_point) const {
  LOG_INFO << "Map::GetSameLaneSequenceV2()";
  if (!IsValid() || !lane_sequence || !lane_sequence->IsValid()) {
    return nullptr;
  }
  LaneConstPtr nearest_lane = lane_sequence->GetNearestLane(math::Vec2d{x, y});
  if (!nearest_lane) {
    return nullptr;
  }
  // Remove back lane faraway
  std::vector<LaneConstPtr> last_valid_lanes;
  bool add = false;
  for (const auto &lane : lane_sequence->lanes()) {
    if (!lane->center_line().IsValid()) break;
    if (!add) {
      auto d = lane->center_line().GetDistance(x, y);
      if (d < 30.0) {
        add = true;
      }
    }
    if (add) {
      last_valid_lanes.emplace_back(lane);
    }
  }
  if (last_valid_lanes.empty()) {
    LOG_ERROR << "GetSameLaneSequenceV2(): last lanes is empty!";
    return nullptr;
  }
  LaneSequencePtr last_valid_laneseq =
      std::make_shared<ad_byd::planning::LaneSequence>(last_valid_lanes);
  debug.emplace_back(absl::StrCat("last first id:",
                                  last_valid_laneseq->lanes().front()->id()));

  // Sample match points
  const double sample_interval = 1.0;
  const double near_range_min = 20.0;
  const double near_range_max = 50.0;
  const int near_interval = 5;
  const int far_interval = 10;

  std::vector<Point2d> sample_points;
  double ego_s_offset = 0.0;
  last_valid_laneseq->GetProjectionDistance({x, y}, &ego_s_offset);
  double sample_s_min = std::fmax(ego_s_offset - near_range_min, 0.0);
  last_valid_laneseq->SamplePoints(sample_s_min, &sample_points,
                                   sample_interval);
  std::vector<Point2d> supple_sample_pts;
  for (const auto &lane : last_valid_laneseq->lanes()) {
    if (!lane || !lane->IsValid() ||
        lane->curve_length() > near_interval * sample_interval + 1.0) {
      continue;
    }
    const double sample_s = lane->curve_length() * 0.5;
    supple_sample_pts.emplace_back(lane->center_line().GetPointAtS(sample_s));
    supple_sample_pts.emplace_back(
        lane->center_line().GetPointAtS(sample_s + 1.0));
  }
  debug.emplace_back(absl::StrCat("match range s min:", sample_s_min));

  // Match lanes
  std::map<uint64_t, int> match_lane_ids;
  double accumulate_s = sample_s_min;
  for (int i = 0; !sample_points.empty() && i < sample_points.size() - 1;) {
    auto point = sample_points[i];
    auto point_1 = sample_points[i + 1];
    auto theta = atan2(point_1.y() - point.y(), point_1.x() - point.x());
    LaneConstPtr lane =
        GetNearestLane(Point2d(point.x(), point.y()), theta, 2.0, true);
    if (lane) {
      if (match_lane_ids.count(lane->id()) == 0) match_lane_ids[lane->id()] = 1;
      if (match_lane_ids.count(lane->id()) != 0) match_lane_ids[lane->id()]++;
    }
    if (accumulate_s < ego_s_offset + near_range_max) {
      i += near_interval;
      accumulate_s += near_interval * sample_interval;
    } else {
      i += far_interval;
      accumulate_s += far_interval * sample_interval;
    }
    debug_match_point.emplace_back(point);
  }
  for (int i = 0;
       !supple_sample_pts.empty() && i < supple_sample_pts.size() - 1; i += 2) {
    auto point = supple_sample_pts[i];
    auto point_1 = supple_sample_pts[i + 1];
    auto theta = atan2(point_1.y() - point.y(), point_1.x() - point.x());
    LaneConstPtr lane =
        GetNearestLane(Point2d(point.x(), point.y()), theta, 2.0, true);
    if (lane) {
      if (match_lane_ids.count(lane->id()) == 0) match_lane_ids[lane->id()] = 1;
      if (match_lane_ids.count(lane->id()) != 0) match_lane_ids[lane->id()]++;
    }
    debug_match_point.emplace_back(point);
  }
  if (match_lane_ids.empty()) {
    LOG_ERROR << "GetSameLaneSequenceV2(): match lanes is empty!";
    return nullptr;
  }
  std::string de = "match_lane_ids:";
  for (const auto &it : match_lane_ids) {
    de += absl::StrCat("(", it.first, "-", it.second, ")");
  }
  debug.emplace_back(de);

  // Choose best
  std::vector<uint64_t> best_ids;
  std::map<uint64_t, int> overlapping_lanes;
  double project_s = 0.0, project_l = 0.0;
  for (const auto &match_lane : match_lane_ids) {
    LaneConstPtr lane = GetLaneById(match_lane.first);
    if (!lane ||
        !lane->center_line().GetProjection({x, y}, &project_s, &project_l))
      continue;
    // Check if the ego_position overlaps with the lane longitudinally
    if (project_s < 0.0 || project_s > lane->curve_length()) {
      continue;
    } else {
      overlapping_lanes[lane->id()] = match_lane.second;
    }
  }
  // If have lanes overlapping with ego_position
  if (!overlapping_lanes.empty()) {
    best_ids.emplace_back(overlapping_lanes.begin()->first);
    int max_cnt = overlapping_lanes.begin()->second;
    if (overlapping_lanes.size() == 1) {
      best_ids.clear();
      best_ids.emplace_back(overlapping_lanes.begin()->first);
    } else {
      // If there are multiple overlapping lanes, compare match_count
      for (const auto &overlapping_lane : overlapping_lanes) {
        if (overlapping_lane.second > max_cnt) {
          best_ids.clear();
          best_ids.emplace_back(overlapping_lane.first);
          max_cnt = overlapping_lane.second;
        } else if (overlapping_lane.second == max_cnt &&
                   std::find(best_ids.begin(), best_ids.end(),
                             overlapping_lane.first) == best_ids.end()) {
          best_ids.emplace_back(overlapping_lane.first);
        }
      }
    }
  } else {
    best_ids.emplace_back(match_lane_ids.begin()->first);
    int max_match_cnt = match_lane_ids.begin()->second;
    for (const auto &match_lane : match_lane_ids) {
      if (match_lane.second > max_match_cnt) {
        best_ids.clear();
        best_ids.emplace_back(match_lane.first);
        max_match_cnt = match_lane.second;
      } else if (match_lane.second == max_match_cnt &&
                 std::find(best_ids.begin(), best_ids.end(),
                           match_lane.first) == best_ids.end()) {
        best_ids.emplace_back(match_lane.first);
      }
    }
  }
  std::vector<LaneConstPtr> best_lanes;
  for (const auto &lane_id : best_ids) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (!lane) continue;
    best_lanes.emplace_back(lane);
  }
  LaneConstPtr best_lane = GetNearestLane(best_lanes, {x, y}, heading);
  if (!best_lane) {
    LOG_ERROR << "GetSameLaneSequenceV2(): best lane is null!";
    return nullptr;
  }
  de = "best_ids:";
  for (const auto &it : best_ids) {
    de += absl::StrCat("(", it, ")");
  }
  debug.emplace_back(de);

  // General match laneseq
  std::vector<LaneConstPtr> valid_lanes;
  valid_lanes.emplace_back(best_lane);
  // Backward
  while (valid_lanes.front()) {
    int max_match_cnt = 0;
    uint64_t best_id = 0;
    for (const auto &lane_id : valid_lanes.front()->pre_lane_ids()) {
      if (match_lane_ids.count(lane_id) != 0 &&
          match_lane_ids[lane_id] > max_match_cnt) {
        max_match_cnt = match_lane_ids[lane_id];
        best_id = lane_id;
      }
    }
    bool exists = std::any_of(
        valid_lanes.begin(), valid_lanes.end(),
        [best_id](const LaneConstPtr &ptr) { return ptr->id() == best_id; });
    if (exists) {
      LOG_INFO << " Already exists in valid_lanes, breaking loop.";
      break;
    }
    LaneConstPtr lane = GetLaneById(best_id);
    if (!lane || best_id == valid_lanes.front()->id()) break;
    valid_lanes.insert(valid_lanes.begin(), lane);
  }
  // Forward
  // while (valid_lanes.back()) {
  //   int max_match_cnt = 0;
  //   uint64_t best_id = 0;
  //   for (const auto &lane_id : valid_lanes.back()->next_lane_ids()) {
  //     if (match_lane_ids.count(lane_id) != 0 &&
  //         match_lane_ids[lane_id] > max_match_cnt) {
  //       max_match_cnt = match_lane_ids[lane_id];
  //       best_id = lane_id;
  //     }
  //   }
  //   LaneConstPtr lane = GetLaneById(best_id);
  //   if (!lane || best_id == valid_lanes.back()->id()) break;
  //   valid_lanes.emplace_back(lane);
  // }
  de = "valid_lanes:";
  for (const auto &it : valid_lanes) {
    de += absl::StrCat("(", it->id(), ")");
  }
  debug.emplace_back(de);

  return std::make_shared<LaneSequence>(valid_lanes);
}

void Map::GenerateAllLaneSequences(const Vec2d &start_point,
                                   const double start_point_v,
                                   const LaneConstPtr &start_lane,
                                   const st::Behavior_FunctionId function_id,
                                   LaneSequencePtr &cur_lane_sequence,
                                   LaneSequencePtr &left_lane_sequence,
                                   LaneSequencePtr &right_lane_sequence) {
  const auto &last_lanes = cur_lane_sequence;
  if (!last_lanes) {
    LOG_ERROR
        << "cannot generate lane sequence: cur_lane sequence donot exist!!!";
    return;
  }
  std::ostringstream info_;
  info_ << "start_point_x:" << start_point.x()
        << "start_point_y:" << start_point.y();
  info_ << "lanes:";
  for (const auto &lane : last_lanes->lanes()) {
    info_ << "," << lane->id();
  }
  Log2DDS::LogDataV0("raw_seqs", info_.str());
  const auto &nearest_lane = start_lane;
  if (!nearest_lane) {
    LOG_ERROR << "cannot generate lane sequence: nearest_lane donot exist!!!";
    return;
  }
  // const auto &map = planning_context.map();
  // if (!map) {
  //   LOG_ERROR << "cannot generate lane sequence: map donot exist!!!";
  //   return;
  // }
  auto lanes = last_lanes->lanes();
  bool find_nearest_lane = false;
  bool find_far_lane = false;
  LaneConstPtr parent_lane = nullptr;
  // double split_point_min_range = std::fmax(start_point_v * 2.0, 7.0);
  double split_point_min_range = 2.0;
  double dis_to_split = 0.0;

  for (auto it = lanes.begin(); it != lanes.end(); it++) {
    if (!find_nearest_lane && (*it)->id() == nearest_lane->id()) {
      find_nearest_lane = true;
    }
    if (find_nearest_lane) {
      if (!find_far_lane && (*it)->center_line().end_point().DistanceTo(
                                start_point) > split_point_min_range) {
        find_far_lane = true;
      }
    }
    if (find_far_lane && (*it)->id() != nearest_lane->id() &&
        GetNextLanes(parent_lane).size() > 1) {
      if (parent_lane->center_line().end_point().DistanceTo(start_point) >
          split_point_min_range) {
        lanes.erase(it, lanes.end());
        // decision->set_cur_lane_seqs_freezed(false);
      } else {
        lanes.erase(it + 1, lanes.end());
        // decision->set_cur_lane_seqs_freezed(true);
      }
      break;
    }
    parent_lane = *it;
    if (find_nearest_lane) {
      if (dis_to_split < Constants::ZERO) {
        dis_to_split +=
            nearest_lane->topo_length() - route()->navi_start().s_offset;
      } else if (parent_lane) {
        dis_to_split += parent_lane->topo_length();
      }
    }
  }
  if (!find_nearest_lane) {
    lanes.clear();
    lanes.emplace_back(nearest_lane);
  }
  if (dis_to_split < Constants::ZERO) {
    dis_to_split = DBL_MAX;
  }
  // Log2DDS::LogDataV0("lane_change_debug",
  //                    absl::StrCat("cur_lane_seqs_freezed:",
  //                                 decision->cur_lane_seqs_freezed()));
  // extand future lanes
  auto last_lane = lanes.back();  // last_lane == 分叉点前的那个lane
  if (!last_lane) {  // lanes为空 将last_lane设为本车所在lane
    last_lane = last_lanes->GetNearestLane(start_point);
    lanes.clear();
    lanes.emplace_back(last_lane);
  }
  LaneConstPtr next_lane = nullptr;
  if (is_on_highway_ || function_id == st::Behavior_FunctionId_HW_NOA) {
    next_lane = GetOptimalNextLane(lanes.back(), true);
  } else if (function_id == st::Behavior_FunctionId_MAPLESS_NOA) {
    next_lane = GetOptimalNextLaneMaplessNoa(lanes.back(), true);
  } else if (function_id == st::Behavior_FunctionId_LKA_PLUS) {
    next_lane = GetSmoothNextLane(lanes.back()->id());
  } else {
    next_lane = GetContinueNextLane(lanes.back()->id());
  }
  int iter_time = 0;
  std::unordered_set<uint64_t> check_lanes;
  while (next_lane) {
    ++iter_time;
    if (next_lane->id() == 0 || next_lane->id() == lanes.back()->id() ||
        check_lanes.count(next_lane->id()) > 0 || iter_time > 10000) {
      break;
    }
    check_lanes.insert(next_lane->id());
    lanes.emplace_back(next_lane);
    if (is_on_highway_ || function_id == st::Behavior_FunctionId_HW_NOA) {
      next_lane = GetOptimalNextLane(lanes.back(), true);
    } else if (function_id == st::Behavior_FunctionId_MAPLESS_NOA) {
      next_lane = GetOptimalNextLaneMaplessNoa(lanes.back(), true);
    } else if (function_id == st::Behavior_FunctionId_LKA_PLUS) {
      next_lane = GetSmoothNextLane(lanes.back()->id());
    } else {
      next_lane = GetContinueNextLane(lanes.back()->id());
    }
  };
  //
  int nearest_lane_idx = -1;
  for (auto it = lanes.begin(); it != lanes.end(); it++) {
    if ((*it)->id() == nearest_lane->id()) {
      nearest_lane_idx = it - lanes.begin();
      break;
    }
  }
  cur_lane_sequence = std::make_shared<LaneSequence>(lanes);
  // decision->set_split_safety_select(false);
  const auto &left_lane = GetLaneById(nearest_lane->left_lane_id());
  const auto &right_lane = GetLaneById(nearest_lane->right_lane_id());
  // left_lane_sequence = GetLeftLaneSequence(left_lane, last_lanes);
  // right_lane_sequence = GetRightLaneSequence(right_lane, last_lanes);
  if (right_lane && right_lane->center_line().IsValid() &&
      right_lane->type() != LANE_NON_MOTOR &&
      right_lane->type() != LANE_UNKNOWN &&
      right_lane->type() != LANE_EMERGENCY) {
    std::vector<LaneConstPtr> lanes;
    lanes.emplace_back(right_lane);
    LaneConstPtr next_lane = nullptr;
    next_lane = GetOptimalNextLaneByNeighborLane(
        right_lane, cur_lane_sequence, nearest_lane_idx, lanes.size(), false);
    if (!next_lane) next_lane = GetOptimalNextLane(right_lane, true);
    int count = 0;
    while (next_lane) {
      if (next_lane->id() == 0 ||
          (!lanes.empty() && next_lane->id() == lanes.back()->id()) ||
          count > 2000) {
        break;
      }
      lanes.emplace_back(next_lane);
      next_lane = GetOptimalNextLaneByNeighborLane(
          lanes.back(), cur_lane_sequence, nearest_lane_idx, lanes.size(),
          false);
      if (!next_lane) next_lane = GetOptimalNextLane(lanes.back(), true);
      ++count;
    };
    right_lane_sequence = std::make_shared<LaneSequence>(lanes);
  }
  if (left_lane && left_lane->center_line().IsValid() &&
      left_lane->type() != LANE_NON_MOTOR &&
      left_lane->type() != LANE_UNKNOWN &&
      left_lane->type() != LANE_EMERGENCY) {
    std::vector<LaneConstPtr> lanes;
    lanes.emplace_back(left_lane);
    LaneConstPtr next_lane = nullptr;
    next_lane = GetOptimalNextLaneByNeighborLane(
        left_lane, cur_lane_sequence, nearest_lane_idx, lanes.size(), true);
    if (!next_lane) next_lane = GetOptimalNextLane(left_lane, true);
    int count = 0;
    while (next_lane) {
      if (next_lane->id() == 0 ||
          (!lanes.empty() && next_lane->id() == lanes.back()->id()) ||
          count > 2000) {
        break;
      }
      lanes.emplace_back(next_lane);
      next_lane = GetOptimalNextLaneByNeighborLane(
          lanes.back(), cur_lane_sequence, nearest_lane_idx, lanes.size(),
          true);
      if (!next_lane) next_lane = GetOptimalNextLane(lanes.back(), true);
      ++count;
    };
    left_lane_sequence = std::make_shared<LaneSequence>(lanes);
  }

  if (left_lane_sequence && !left_lane_sequence->lanes().empty() &&
      left_lane_sequence->lanes().front()) {
    GetContinuePreLaneseqs(start_point, left_lane_sequence);
  }
  if (right_lane_sequence && !right_lane_sequence->lanes().empty() &&
      right_lane_sequence->lanes().front()) {
    GetContinuePreLaneseqs(start_point, right_lane_sequence);
  }
  if (cur_lane_sequence && !cur_lane_sequence->lanes().empty() &&
      cur_lane_sequence->lanes().front()) {
    GetContinuePreLaneseqs(start_point, cur_lane_sequence);
  }
  // if (cur_lane_sequence && planning_context.function_mode() ==
  // Function_HWNOA) {
  //   // donnot use distoPOI because this method will take dis_to_split ==
  //   DBL_MAX CheckIfParallelogramSelect(start_point, planning_context,
  //   decision,
  //                              cur_lane_sequence, dis_to_split, parent_lane);
  //   // must sent to back
  //   // CheckIfSplitSafetySelect(start_point, planning_context, decision,
  //   //                          cur_lane_sequence, parent_lane,
  //   //                          left_lane_sequence, right_lane_sequence);
  // }

  // int pri_relation = 100;
  // pri_relation = std::fabs(GetPriorityLaneRelation(nearest_lane));

  // if (planning_context.function_mode() == Function_HWNOA && pri_relation ==
  // 1) {
  //   CheckIfNeedEmergencyLaneChange(start_point, planning_context, decision,
  //                                  cur_lane_sequence, left_lane_sequence,
  //                                  right_lane_sequence);
  // }

  // left_lane_sequence = map->GetLaneSequence(left_lane, true);
  // right_lane_sequence = map->GetLaneSequence(right_lane, true);
  // check left_laneseq merge
  // left_lane and left_lane_sequence should not be nullptr
  std::vector<uint64_t> lane_seqs_ids{};
  if (cur_lane_sequence) {
    for (auto l : cur_lane_sequence->lanes()) {
      if (!l->center_line().points().empty()) {
        lane_seqs_ids.push_back(l->id());
      }
    }
  }
  if (left_lane_sequence) {
    for (auto ls : left_lane_sequence->lanes()) {
      if (!ls->center_line().points().empty()) {
        lane_seqs_ids.push_back(ls->id());
      }
    }
  }
  if (right_lane_sequence) {
    for (auto ls : right_lane_sequence->lanes()) {
      if (!ls->center_line().points().empty()) {
        lane_seqs_ids.push_back(ls->id());
      }
    }
  }
  std::string str_lane_seqs_ids;
  for (const auto id : lane_seqs_ids) {
    str_lane_seqs_ids += absl::StrCat(id);
  }

  Log2DDS::LogDataV0("lane_seqs", str_lane_seqs_ids);
}

LaneSequencePtr Map::GetLeftLaneSequence(const LaneConstPtr &left_lane,
                                         const LaneSequencePtr &lane_sequence,
                                         const bool is_city) {
  if (!left_lane || !left_lane->center_line().IsValid() || !lane_sequence ||
      lane_sequence->lanes().empty()) {
    return nullptr;
  }
  if (left_lane->type() == LANE_NON_MOTOR ||
      left_lane->type() == LANE_UNKNOWN ||
      left_lane->type() == LANE_EMERGENCY ||
      left_lane->type() == LANE_HARBOR_STOP ||
      left_lane->type() == LANE_DIVERSION) {
    return nullptr;
  }

  std::vector<LaneConstPtr> left_lanes;
  const auto &left_start_id = left_lane->id();
  if (lane_map_.count(left_start_id) > 0) {
    left_lanes.emplace_back(lane_map_[left_start_id]);
  } else {
    return nullptr;
  }

  bool find_left_lane = false;
  for (int i = 0; i < lane_sequence->lanes().size(); ++i) {
    const auto &left_lane_id = lane_sequence->lanes()[i]->left_lane_id();
    if (!find_left_lane) {
      if (left_lanes.back()->id() == left_lane_id) {
        find_left_lane = true;
      }
      continue;
    }

    if (left_lanes.back()->IsNext(left_lane_id) &&
        lane_map_.count(left_lane_id) > 0) {
      auto next_left_lane = lane_map_[left_lane_id];
      if (!next_left_lane || !next_left_lane->is_navigation()) break;
      left_lanes.emplace_back(next_left_lane);
    } else if (left_lanes.back()->IsNext(lane_sequence->lanes()[i]->id())) {
      if (is_city) return nullptr;

      left_lanes.insert(left_lanes.end(), lane_sequence->lanes().begin() + i,
                        lane_sequence->lanes().end());
      break;
    } else {
      break;
    }
  }
  if (!find_left_lane) {
    return nullptr;
  }
  LaneConstPtr last_left_lane = left_lanes.back();
  left_lanes.pop_back();
  auto appended_sequence = GetLaneSequence(last_left_lane, true);
  if (appended_sequence) {
    left_lanes.insert(left_lanes.end(), appended_sequence->lanes().begin(),
                      appended_sequence->lanes().end());
  }
  return std::make_shared<LaneSequence>(left_lanes);
}

LaneSequencePtr Map::GetRightLaneSequence(
    const LaneConstPtr &right_lane, const LaneSequencePtr &lane_sequence,
    const bool is_city,
    const ad_byd::planning::EIEChoiceType &eie_choice_type) {
  if (!right_lane || !right_lane->center_line().IsValid() || !lane_sequence ||
      lane_sequence->lanes().empty()) {
    return nullptr;
  }
  if (right_lane->type() == LANE_NON_MOTOR ||
      right_lane->type() == LANE_UNKNOWN ||
      (right_lane->type() == LANE_EMERGENCY &&
       eie_choice_type !=
        ad_byd::planning::EIEChoiceType::CHOICE_RIGHT_LANE) ||
      right_lane->type() == LANE_HARBOR_STOP ||
      right_lane->type() == LANE_DIVERSION) {
    return nullptr;
  }
  std::vector<LaneConstPtr> right_lanes;
  const auto &right_start_id = right_lane->id();
  if (lane_map_.count(right_start_id) > 0) {
    right_lanes.emplace_back(lane_map_[right_start_id]);
  } else {
    return nullptr;
  }

  bool find_right_lane = false;
  for (int i = 0; i < lane_sequence->lanes().size(); ++i) {
    const auto &right_lane_id = lane_sequence->lanes()[i]->right_lane_id();
    if (!find_right_lane) {
      if (right_lanes.back()->id() == right_lane_id) {
        find_right_lane = true;
      }
      continue;
    }

    if (right_lanes.back()->IsNext(right_lane_id) &&
        lane_map_.count(right_lane_id) > 0) {
      auto next_right_lane = lane_map_[right_lane_id];
      if (!next_right_lane || !next_right_lane->is_navigation()) break;
      right_lanes.emplace_back(next_right_lane);
    } else if (right_lanes.back()->IsNext(lane_sequence->lanes()[i]->id())) {
      if (is_city) return nullptr;

      right_lanes.insert(right_lanes.end(), lane_sequence->lanes().begin() + i,
                         lane_sequence->lanes().end());
      break;
    } else {
      break;
    }
  }
  if (!find_right_lane) {
    return nullptr;
  }
  LaneConstPtr last_right_lane = right_lanes.back();
  right_lanes.pop_back();
  auto appended_sequence = GetLaneSequence(last_right_lane, true);
  if (appended_sequence) {
    right_lanes.insert(right_lanes.end(), appended_sequence->lanes().begin(),
                       appended_sequence->lanes().end());
  }
  return std::make_shared<LaneSequence>(right_lanes);
}

LaneConstPtr Map::GetSmoothNextLane(const uint64_t lane_id,
                                    const bool navi_is_priority) {
  LaneConstPtr lane = GetLaneById(lane_id);
  if (!lane || lane->next_lane_ids().empty()) {
    return nullptr;
  }
  // Get valid next lanes
  std::vector<LaneConstPtr> next_lanes;
  for (const auto &next_lane_id : lane->next_lane_ids()) {
    const auto &next_lane = GetLaneById(next_lane_id);
    if (!next_lane) {
      continue;
    }
    if (navi_is_priority && !next_lane->is_navigation()) {
      continue;
    }
    if (next_lane->type() == LANE_UNKNOWN ||
        next_lane->type() == LANE_EMERGENCY ||
        next_lane->type() == LANE_NON_MOTOR) {
      continue;
    }
    next_lanes.emplace_back(next_lane);
  }
  if (next_lanes.empty()) {
    return nullptr;
  }
  if (next_lanes.size() == 1) {
    return next_lanes.front();
  }
  // Compute next_lane's cost
  std::map<double, LaneConstPtr> cost_map;
  for (const auto &next_lane : next_lanes) {
    double cost = 0.0;
    LaneConstPtr supply_lane = GetContinueNextLane(next_lane->id());
    if (next_lane->turn_type() != TurnType::NO_TURN ||
        (supply_lane && supply_lane->type() == LaneType::LANE_ROUND_ABOUT)) {
      cost = ComputeTurnCost(lane, next_lane);
    } else {
      cost = ComputeSmoothCost(lane, next_lane, navi_is_priority);
    }
    if (!next_lane->is_navigation()) {
      cost += 2.0;
    }
    cost_map[cost] = next_lane;
  }
  return cost_map.empty() ? nullptr : cost_map.begin()->second;
}

double Map::ComputeSmoothCost(const LaneConstPtr &first_lane,
                              const LaneConstPtr &second_lane,
                              const bool navi_is_priority) {
  if (!first_lane || !second_lane) {
    return 1.0;
  }
  //
  if (!first_lane->center_line().IsValid() ||
      !second_lane->center_line().IsValid()) {
    return second_lane->split_topology() == TOPOLOGY_SPLIT_NONE ? 0.0 : 1.0;
  }
  LaneConstPtr third_lane = GetContinueNextLane(second_lane->id());
  if (!third_lane) {
    // 若候选分叉的后继不存在，但是其他候选分叉的后继存在，则该候选车道平滑cost为1.0
    for (const auto &lane_id : first_lane->next_lane_ids()) {
      LaneConstPtr lane = GetLaneById(lane_id);
      if (!lane) {
        continue;
      }
      if (navi_is_priority && !lane->is_navigation()) {
        continue;
      }
      if (lane->type() == LANE_UNKNOWN || lane->type() == LANE_EMERGENCY ||
          lane->type() == LANE_NON_MOTOR) {
        continue;
      }
      for (const auto &next_lane_id : lane->next_lane_ids()) {
        if (GetLaneById(next_lane_id)) {
          return 1.0;
        }
      }
    }
  }

  // 1 compute vector
  Vec2d first_vec = first_lane->center_line().end_point() -
                    first_lane->center_line().GetPointAtS(std::fmax(
                        first_lane->center_line().length() - 5.0, 0.0));

  Vec2d second_vec = second_lane->center_line().end_point() -
                     second_lane->center_line().begin_point();
  Vec2d third_vec;
  math::LineCurve2d third_center_line;
  // get center_line from third_lane, require min_length and max_length
  std::vector<Vec2d> points;
  double length = 0.0;
  LaneConstPtr lane = third_lane;
  while (lane && lane->center_line().IsValid()) {
    length += lane->curve_length();
    points.insert(points.end(), lane->center_line().points().begin(),
                  lane->center_line().points().end());
    if (length > 20.0) {
      break;
    }
    lane = GetContinueNextLane(lane->id());
  }
  if (length > 10.0) {
    third_center_line.InitializePoints(points);
  }

  // different second and third vec if split lane within or outside the junction
  if (second_lane->junction_id() == 0 && third_center_line.IsValid()) {
    third_vec = third_center_line.GetPointAtS(
                    std::fmin(third_center_line.length(), 15.0)) -
                third_center_line.points().front();
  } else if (!(second_lane->junction_id() == 0) &&
             third_center_line.IsValid()) {
    Vec2d modify_point = third_center_line.GetPointAtS(5.0);
    third_vec = third_center_line.GetPointAtS(
                    std::fmin(third_center_line.length(), 15.0)) -
                modify_point;
    second_vec = modify_point - second_lane->center_line().begin_point();
  }

  // 2 compute road angle
  double road_trans = 0.0;
  if (third_center_line.IsValid()) {
    road_trans = (first_vec.CrossProd(third_vec)) /
                 (first_vec.Length() * third_vec.Length());
  } else {
    road_trans = (first_vec.CrossProd(second_vec)) /
                 (first_vec.Length() * second_vec.Length());
  }

  // 3 compute smooth
  // Straight lanes with larger turning angles, compute cost by index
  if (std::fabs(road_trans) > std::sin(20.0 * Constants::DEG2RAD) &&
      third_center_line.IsValid()) {
    size_t start_size = 0;
    size_t start_index = GetSameDirLanePosition(
        first_lane, second_lane->turn_type(), &start_size);
    // SectionInfo target_section;
    // route()->GetSectionByIdFromRoute(third_lane->section_id(),
    // target_section);
    SectionConstPtr target_section_ptr =
        GetSectionById(third_lane->section_id());
    size_t target_size = GetNormalLaneSize(target_section_ptr);
    size_t target_index =
        GetNormalLanePosition(third_lane->id(), target_section_ptr);

    double d_index = road_trans > 0.0 ? std::fabs(start_index - target_index)
                                      : std::fabs((start_size - start_index) -
                                                  (target_size - target_index));
    if (start_size == 0 && target_size == 0) {
      return 1.0;
    }
    return d_index / std::max(start_size, target_size);
  } else {
    double cost = first_vec.InnerProd(second_vec) /
                  (first_vec.Length() * second_vec.Length());
    if (third_center_line.IsValid()) {
      cost += second_vec.InnerProd(third_vec) /
              (second_vec.Length() * third_vec.Length());
      cost /= 2.0;
    }
    return 1.0 - cost;
  }
}

double Map::ComputeTurnCost(const LaneConstPtr &first_lane,
                            const LaneConstPtr &second_lane) {
  if (!first_lane || !second_lane) {
    return 1.0;
  }
  // 1 get index of enter lane and quit lane
  const LaneConstPtr third_lane = GetContinueNextLane(second_lane->id());
  const LaneConstPtr target_lane = third_lane ? third_lane : second_lane;
  // SectionInfo target_section;
  // route()->GetSectionByIdFromRoute(target_lane->section_id(),
  // target_section);
  SectionConstPtr target_section_ptr =
      GetSectionById(target_lane->section_id());
  size_t size_e = GetNormalLaneSize(target_section_ptr);
  size_t index_e = GetNormalLanePosition(target_lane->id(), target_section_ptr);
  size_t size_s = 0;
  size_t index_s =
      GetSameDirLanePosition(first_lane, second_lane->turn_type(), &size_s);
  int max_size = std::max(size_s, size_e);
  if (max_size == 0) {
    return 1.0;
  }
  // 2 compute cost by index
  double d_index = max_size;
  if (third_lane && third_lane->type() == LaneType::LANE_ROUND_ABOUT) {
    // round about cost: right first enter lane into right first lane,
    // other enter lane into right second lane
    if ((size_s - index_s) == 0) {
      d_index = (size_e - index_e);
    } else {
      d_index = std::fabs((size_e - index_e) - 1);
    }
  } else {
    // turn cost
    if (second_lane->turn_type() == TurnType::LEFT_TURN ||
        second_lane->turn_type() == TurnType::U_TURN) {
      d_index = third_lane ? std::fabs(index_s - index_e) : index_e;
    } else if (second_lane->turn_type() == TurnType::RIGHT_TURN) {
      d_index = third_lane ? std::fabs((size_s - index_s) - (size_e - index_e))
                           : (size_e - index_e);
    }
    // curvature cost
    if (GetAverageKappa(second_lane) > 1.0 / 6.0) {
      d_index += 1.5;
    }
  }
  double cost = d_index / max_size;
  return std::fmin(cost, 1.0);
}

double Map::GetAverageKappa(const LaneConstPtr &lane) {
  if (!lane || !lane->center_line().IsValid()) {
    return 1.0;
  }
  Path path(lane->center_line().points());
  std::vector<PathPoint> points;
  path.SamplePoints(0.0, path.length(), 1.0, &points);
  double sum_kappa = 0.0;
  double max_kappa = 0.0;
  for (const auto &point : points) {
    sum_kappa += std::fabs(point.kappa);
    if (std::fabs(point.kappa) > max_kappa) {
      max_kappa = std::fabs(point.kappa);
    }
  }
  double avg_kappa = 1.0;
  if (!points.empty()) {
    avg_kappa = sum_kappa / points.size();
  }
  return avg_kappa;
}

LaneConstPtr Map::GetContinueNextLane(const uint64_t lane_id) {
  LaneConstPtr cur_lane = GetLaneById(lane_id);
  if (!cur_lane) {
    return nullptr;
  }
  // Tip: Cant confirm which lane in junction is continue: maybe the lane in
  // junction turn left/right, or Oblique insertion (xie cha)
  for (const auto &lane_id : cur_lane->next_lane_ids()) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (lane && lane->is_navigation() &&
        lane->split_topology() == TOPOLOGY_SPLIT_NONE) {
      return lane;
    }
  }
  for (const auto &lane_id : cur_lane->next_lane_ids()) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (lane && lane->turn_type() == NO_TURN &&
        lane->split_topology() == TOPOLOGY_SPLIT_NONE) {
      return lane;
    } else if (lane && lane_id == cur_lane->next_lane_ids().back()) {
      return lane;
    }
  }
  return nullptr;
}

size_t Map::GetNormalLanePosition(const uint64_t lane_id,
                                  const SectionConstPtr &section) {
  size_t lane_position = 0;
  if (!section) return lane_position;
  for (const auto &section_lane_id : section->lanes()) {
    const auto &lane = GetLaneById(section_lane_id);
    if (!lane) {
      continue;
    }
    if (lane &&
        (lane->type() == LANE_UNKNOWN || lane->type() == LANE_EMERGENCY ||
         lane->type() == LANE_NON_MOTOR)) {
      continue;
    }
    lane_position++;
    if (lane_id == section_lane_id) {
      return lane_position;
    }
  }
  return 0;
}

size_t Map::GetNormalLaneSize(const SectionConstPtr &section) {
  size_t lane_size = 0;
  if (!section) return lane_size;
  for (const auto &section_lane_id : section->lanes()) {
    const auto &lane = GetLaneById(section_lane_id);
    if (!lane) {
      continue;
    }
    if (lane &&
        (lane->type() == LANE_UNKNOWN || lane->type() == LANE_EMERGENCY ||
         lane->type() == LANE_NON_MOTOR)) {
      continue;
    }
    lane_size++;
  }
  return lane_size;
}
size_t Map::GetSameDirLanePosition(const LaneConstPtr &target_lane,
                                   const TurnType &turn_type, size_t *size) {
  if (!target_lane) {
    return 0;
  }
  size_t lane_position = 0;
  size_t lane_size = 0;
  // SectionInfo section;
  // route()->GetSectionByIdFromRoute(target_lane->section_id(), section);
  SectionConstPtr section_ptr = GetSectionById(target_lane->section_id());
  if (!section_ptr) return 0;
  for (const auto &section_lane_id : section_ptr->lanes()) {
    const auto &lane = GetLaneById(section_lane_id);
    if (!lane) {
      continue;
    }
    if (lane &&
        (lane->type() == LANE_UNKNOWN || lane->type() == LANE_EMERGENCY ||
         lane->type() == LANE_NON_MOTOR)) {
      continue;
    }
    for (const auto &lane_id : lane->next_lane_ids()) {
      LaneConstPtr next_lane = GetLaneById(lane_id);
      if (next_lane && next_lane->turn_type() == turn_type) {
        lane_size++;
        if (target_lane->id() == section_lane_id) {
          lane_position = lane_size;
        }
        break;
      }
    }
  }
  if (size) {
    *size = lane_size;
  }
  return lane_position;
}

LaneConstPtr Map::GetContinuePreLane(const uint64_t lane_id,
                                     const bool is_build_left_seq) {
  LaneConstPtr cur_lane = GetLaneById(lane_id);
  if (!cur_lane || !cur_lane->center_line().IsValid()) {
    return nullptr;
  }
  for (const auto &lane_id : cur_lane->pre_lane_ids()) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (lane && lane->center_line().IsValid() &&
        (is_build_left_seq || lane->is_navigation()) &&
        lane->split_topology() == TOPOLOGY_SPLIT_NONE &&
        lane->merge_topology() == TOPOLOGY_MERGE_NONE &&
        lane->type() == cur_lane->type()) {
      return lane;
    }
  }
  for (const auto &lane_id : cur_lane->pre_lane_ids()) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (lane && lane->center_line().IsValid() &&
        (is_build_left_seq || lane->is_navigation()) &&
        lane->split_topology() == TOPOLOGY_SPLIT_NONE &&
        lane->merge_topology() == TOPOLOGY_MERGE_NONE) {
      return lane;
    }
  }
  for (const auto &lane_id : cur_lane->pre_lane_ids()) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (lane && lane->center_line().IsValid() &&
        (is_build_left_seq || lane->is_navigation()) &&
        lane->turn_type() == NO_TURN) {
      return lane;
    }
  }
  for (const auto &lane_id : cur_lane->pre_lane_ids()) {
    LaneConstPtr lane = GetLaneById(lane_id);
    if (lane && lane->center_line().IsValid() &&
        (is_build_left_seq || lane->is_navigation())) {
      return lane;
    }
  }
  return nullptr;
}

void Map::GetContinuePreLaneseqs(const Vec2d &start_point,
                                 LaneSequencePtr &seqs,
                                 const bool is_build_left_seq) {
  double cur_dis = 0.0;
  double RearMaxDistance = 30.0;
  int MaxCount = 100;
  LaneConstPtr nearest_lane = nullptr;
  // enlarge seqs
  if (seqs && !seqs->lanes().empty() && seqs->lanes().front()) {
    nearest_lane = seqs->lanes().front();
  }
  if (nearest_lane) {
    if (nearest_lane->center_line().IsValid()) {
      double l = 0.0;
      nearest_lane->center_line().GetProjection(
          {start_point.x(), start_point.y()}, &cur_dis, &l);
    } else {
      cur_dis = route()->navi_start().s_offset;
    }
  } else {
    return;
  }

  // 1. remove repeated section; 2. get remain sections' set
  std::unordered_set<uint64_t> section_set = seqs->RemoveRepeatedSections();

  int count = 0;
  while (cur_dis < RearMaxDistance) {
    const auto cur_lane = seqs->lanes().front();
    if (!cur_lane || !cur_lane->center_line().IsValid()) {
      break;
    }
    LaneConstPtr pre_lane = nullptr;
    pre_lane = GetContinuePreLane(cur_lane->id(), is_build_left_seq);
    if (!pre_lane || !pre_lane->center_line().IsValid()) {
      break;
    }

    // don't keep repeat section
    if (section_set.count(pre_lane->section_id()) > 0) {
      LOG_INFO << "Meet the repeated section[" << pre_lane->section_id()
               << "] when find pre lane, quit.";
      break;
    }
    section_set.emplace(pre_lane->section_id());

    cur_dis += pre_lane->center_line().length();
    ++count;
    if (count > MaxCount) {
      break;
    }
    seqs->mutable_lanes().insert(seqs->mutable_lanes().begin(), pre_lane);
  }
}

LaneSequencePtr Map::GetSameLaneSequence(const LaneSequencePtr &lane_sequence,
                                         const double &x, const double &y) {
  LOG_INFO << "Map::GetSameLaneSequence()";
  std::vector<LaneConstPtr> lanes;
  if (!IsValid() || !lane_sequence || !lane_sequence->IsValid()) {
    return nullptr;
  }
  LaneConstPtr nearest_lane = lane_sequence->GetNearestLane(math::Vec2d{x, y});
  if (!nearest_lane) {
    return nullptr;
  }
  bool has_find_nearest = false;
  bool add = false;
  for (const auto &lane : lane_sequence->lanes()) {
    if (lane->id() == nearest_lane->id()) {
      has_find_nearest = true;
    }
    if (!lane->center_line().IsValid()) break;
    if (!add) {
      auto d = lane->center_line().GetDistance(x, y);
      if (d < 60.0) {
        add = true;
      }
    }
    if (add) {
      auto cur = GetSameLane(lane);
      if (cur && (cur->center_line().IsValid() || has_find_nearest)) {
        lanes.emplace_back(cur);
        const auto next_lanes = GetNextLanes(cur);
        if (next_lanes.empty() && !has_find_nearest) lanes.clear();
        if (next_lanes.empty() && has_find_nearest) break;
      } else {
        if (has_find_nearest) {
          break;
        } else {
          lanes.clear();
        }
      }
    }
  }
  if (lanes.empty()) {
    LOG_ERROR << "GetSameLaneSequence():lanes is empty!";
    return nullptr;
  }
  return std::make_shared<LaneSequence>(lanes);
}

LaneConstPtr Map::GetOptimalNextLane(const LaneConstPtr &lane,
                                     const bool &navi_is_priority,
                                     const uint64_t split_lane_id) const {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  std::vector<LaneConstPtr> next_candidates;
  bool next_is_navi = false;
  for (const auto &next_lane : next_lanes) {
    if (navi_is_priority && next_lane->is_navigation() &&
        next_lane->id() != split_lane_id) {
      next_candidates.emplace_back(next_lane);
      next_is_navi = true;
    }
  }
  if (next_candidates.empty()) {
    next_candidates.assign(next_lanes.begin(), next_lanes.end());
  }
  // only one candidate
  if (next_candidates.size() == 1) {
    return next_candidates.front();
  } else {
    std::map<double, LaneConstPtr> next_map;
    for (const auto &next_lane : next_candidates) {
      double cost = 0.0;
      if (next_is_navi) {
        cost += 5.0 * std::abs(GetPriorityLaneRelation(next_lane));
        // merge cost
        auto it = std::find(split_merge_shortdis_ids_.begin(),
                            split_merge_shortdis_ids_.end(), next_lane->id());
        if (it != split_merge_shortdis_ids_.end()) cost += 20.0;
      }
      if (next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_LEFT ||
          next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
        cost += 10.0;
      }
      if (lane->type() != next_lane->type()) {
        cost += 5.0;
      }
      // encourage drive more navi_distance
      if (next_is_navi) {
        cost -= std::min(800.0, next_lane->navi_distance());
      }

      next_map[cost] = next_lane;
    }
    return next_map.begin()->second;
  }
}

LaneConstPtr Map::GetCityOptimalNextLane(const LaneConstPtr &lane,
                                         const bool &navi_is_priority) const {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  std::vector<LaneConstPtr> next_candidates;
  bool next_is_navi = false;
  for (const auto &next_lane : next_lanes) {
    if (navi_is_priority && next_lane->is_navigation()) {
      next_candidates.emplace_back(next_lane);
      next_is_navi = true;
    }
  }
  if (next_candidates.empty()) {
    next_candidates.assign(next_lanes.begin(), next_lanes.end());
  }
  // only one candidate
  if (next_candidates.size() == 1) {
    return next_candidates.front();
  } else {
    std::map<double, LaneConstPtr> next_map;
    for (const auto &next_lane : next_candidates) {
      double cost = 0.0;
      if (next_is_navi) {
        cost += 5.0 * std::abs(GetPriorityLaneRelation(next_lane));
      }
      if (lane->type() != next_lane->type()) {
        cost += 5.0;
      }
      // encourage drive more navi_distance
      if (next_is_navi) {
        cost -= std::min(800.0, next_lane->navi_distance());
      }

      next_map[cost] = next_lane;
    }
    return next_map.begin()->second;
  }
}

LaneConstPtr Map::GetOptimalNextLaneByNeighborLane(
    const LaneConstPtr &lane, const LaneSequencePtr &cur_lane_seq,
    int nearest_lane_idx, int neighbor_lane_size, bool is_left) const {
  if (!lane || !cur_lane_seq) return nullptr;

  int cur_lane_seq_idx = neighbor_lane_size + nearest_lane_idx;
  if (nearest_lane_idx < 0 || cur_lane_seq_idx < 0 ||
      cur_lane_seq_idx >= cur_lane_seq->lanes().size())
    return nullptr;

  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) return nullptr;

  for (const auto &next_lane : next_lanes) {
    if ((is_left && next_lane->right_lane_id() ==
                        cur_lane_seq->lanes().at(cur_lane_seq_idx)->id()) ||
        (!is_left && next_lane->left_lane_id() ==
                         cur_lane_seq->lanes().at(cur_lane_seq_idx)->id())) {
      return next_lane;
    }
  }
  return nullptr;
}

LaneConstPtr Map::GetOptimalNextLaneMaplessNoa(
    const LaneConstPtr &lane, const bool &navi_is_priority) const {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  std::vector<LaneConstPtr> next_candidates;
  bool next_is_navi = false;
  for (const auto &next_lane : next_lanes) {
    if (navi_is_priority && next_lane->is_navigation()) {
      next_candidates.emplace_back(next_lane);
      next_is_navi = true;
    }
  }
  if (next_candidates.empty()) {
    next_candidates.assign(next_lanes.begin(), next_lanes.end());
  }
  std::map<double, LaneConstPtr> next_map;
  for (const auto &next_lane : next_candidates) {
    double cost = 0.0;
    if (next_is_navi) {
      cost += 5.0 * std::abs(GetPriorityLaneRelation(next_lane));
    }
    if (next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_LEFT ||
        next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
      cost += 10.0;
    }
    if (lane->type() != next_lane->type()) {
      cost += 5.0;
    }
    // encourage drive more navi_distance
    if (next_is_navi) {
      cost -= std::min(1250.0, next_lane->navi_distance());
    }
    next_map[cost] = next_lane;
  }
  return next_map.begin()->second;
}

LaneConstPtr Map::GetOptimalSmoothNextLane(const LaneConstPtr &lane,
                                           const bool &navi_is_priority,
                                           std::string &debug) const {
  if (!lane) {
    return nullptr;
  }
  const auto &next_lanes = GetNextLanes(lane);
  if (next_lanes.empty()) {
    return nullptr;
  }
  std::vector<LaneConstPtr> next_candidates;
  bool next_is_navi = false;
  for (const auto &next_lane : next_lanes) {
    if (navi_is_priority && next_lane->is_navigation()) {
      next_candidates.emplace_back(next_lane);
      next_is_navi = true;
    }
  }
  if (next_candidates.empty()) {
    next_candidates.assign(next_lanes.begin(), next_lanes.end());
  }
  std::map<double, LaneConstPtr> next_map;
  for (const auto &next_lane : next_candidates) {
    if (!next_lane) continue;
    debug += absl::StrCat("(", next_lane->id(), ":");
    double cost = 0.0;
    if (next_is_navi) {
      debug += absl::StrCat("navi=", next_lane->navi_section_cnt(), ",");
      cost -= 100.0 * next_lane->navi_section_cnt();
    }
    if (next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_LEFT ||
        next_lane->split_topology() == SplitTopology::TOPOLOGY_SPLIT_RIGHT) {
      debug += "topo=10.0,";
      cost += 10.0;
    }

    const double ref_start_s = std::fmax(lane->curve_length() - 15.0, 0.0);
    math::Vec2d ref_vec = lane->center_line().end_point() -
                          lane->center_line().GetPointAtS(ref_start_s);
    // use 4 pts, s = 15.0, 12.0, 9.0, 6.0
    std::vector<std::pair<double, double>> distances;
    double dis_avg = 0.0;
    for (int i = 0; i < 4; i++) {
      double next_end_s = std::fmin(next_lane->curve_length(), 15.0 - 3.0 * i);
      math::Vec2d next_vec = next_lane->center_line().GetPointAtS(next_end_s) -
                             next_lane->center_line().begin_point();
      next_vec = math::Vec2d::CreateUnitVec2d(next_vec.Angle()) * 10.0;
      double distance = std::fabs(ref_vec.CrossProd(next_vec) /
                                  std::fmax(ref_vec.Length(), Constants::ZERO));
      distances.emplace_back(distance, 0.0);
      dis_avg += distance;
    }
    for (auto &i : distances) {
      i.second = std::fabs(i.first - dis_avg * 0.25);
    }
    std::sort(distances.begin(), distances.end(),
              [](const auto &lhs, const auto &rhs) {
                return lhs.second < rhs.second;
              });
    double distance = (dis_avg - distances.back().first) / 3.0;
    if (next_lane->center_line().IsValid()) {
      cost += distance;
    } else {
      cost += 10.0;
    }
    debug += absl::StrCat("distances=", distance, "-rm-",
                          distances.back().first, ")");
    next_map[cost] = next_lane;
  }
  if (next_map.empty()) return nullptr;
  return next_map.begin()->second;
}

LaneConstPtr Map::GetSameLane(const LaneConstPtr &lane) const {
  if (lane == nullptr) {
    return nullptr;
  }
  auto it = lane_map_.find(lane->id());
  if (it != lane_map_.end()) {
    return it->second;
  }
  if (lane->center_line().points().empty()) {
    return nullptr;
  }
  auto point =
      lane->center_line().GetPointAtS(lane->center_line().length() * 0.25);
  auto point_1 = lane->center_line().GetPointAtS(
      lane->center_line().length() * 0.25 + 1.0);
  auto theta = atan2(point_1.y() - point.y(), point_1.x() - point.x());
  return GetNearestLane(Point2d(point.x(), point.y()), theta, 1.0);
}

bool Map::IsValid() const { return !lane_map_.empty(); }

void Map::GetPolygonSRange(const LaneConstPtr &lane,
                           const math::Polygon2d &polygon, double *s_min,
                           double *s_max) const {
  if (!lane || !lane->center_line().IsValid() || !s_min || !s_max) return;
  const auto &pts = lane->center_line().points();
  std::size_t idx = 1u;
  *s_min = 0.0;
  *s_max = 0.0;
  if (polygon.IsPointIn(pts.front()) && polygon.IsPointIn(pts.back())) {
    *s_max = lane->curve_length();
    return;
  }
  Point2d first_pt, second_pt;
  for (; idx < pts.size(); idx++) {
    *s_min += pts[idx - 1].DistanceTo(pts[idx]);
    *s_max += pts[idx - 1].DistanceTo(pts[idx]);
    math::LineSegment2d segment(pts[idx - 1], pts[idx]);
    if (!polygon.IsPointIn(pts[idx]) &&
        polygon.GetOverlap(segment, &first_pt, &second_pt)) {
      *s_min -= first_pt.DistanceTo(pts[idx]);
      *s_max -= second_pt.DistanceTo(pts[idx]);
      return;
    } else if (!polygon.IsPointIn(pts[idx - 1]) &&
               polygon.IsPointIn(pts[idx]) &&
               polygon.GetOverlap(segment, &first_pt, &second_pt)) {
      *s_min -= first_pt.DistanceTo(pts[idx]);
      break;
    }
  }
  if (polygon.IsPointIn(pts.back())) {
    *s_max = lane->curve_length();
    return;
  }
  idx++;
  for (; idx < pts.size(); idx++) {
    *s_max += pts[idx - 1].DistanceTo(pts[idx]);
    math::LineSegment2d segment(pts[idx - 1], pts[idx]);
    if (polygon.IsPointIn(pts[idx - 1]) && !polygon.IsPointIn(pts[idx]) &&
        polygon.GetOverlap(segment, &first_pt, &second_pt)) {
      *s_max -= second_pt.DistanceTo(pts[idx]);
      break;
    }
  }
}

void Map::SetLaneInfoBySection(SectionInfo section, double section_length,
                               double next_section_length) {
  for (const auto &lane_id : section.lane_ids) {
    LanePtr lane;
    if (lane_map_.find(lane_id) != lane_map_.end()) {
      lane = lane_map_[lane_id];
    }
    if (!lane) continue;
    //  use max lane length as section length
    section_length = std::max(section_length, lane->topo_length());
  }
  for (const auto &lane_id : section.lane_ids) {
    LanePtr lane;
    if (lane_map_.find(lane_id) != lane_map_.end()) {
      lane = lane_map_[lane_id];
    }
    if (!lane) continue;
    // navi distance
    double navi_distance = 0.0;
    int navi_section_cnt = 1;
    for (const auto &next_id : lane->next_lane_ids()) {
      auto next_lane = GetLaneById(next_id);
      if (next_lane && next_lane->is_navigation()) {
        navi_distance = std::max(
            navi_distance, next_section_length + next_lane->navi_distance());
        navi_section_cnt =
            std::max(navi_section_cnt, 1 + next_lane->navi_section_cnt());
      }
    }
    // set navi info
    lane->SetIsNavigation(true, navi_distance, navi_section_cnt);
    lane->SetSectionId(section.id);
    // LOG(ERROR) << " navi_distance: " << navi_distance << " navi_section_cnt:
    // "
    // << navi_section_cnt << " belong to lane: " << lane->id();
    // set lane length
    // if (!lane->is_virtual_navigation())
    lane->SetTrueLength(section_length);
  }
}

void Map::ConvertMapInfo(const MapInfo &map_info) {
  // basic info
  is_on_highway_ = map_info.is_on_highway;
  type_ = map_info.type;
  sub_type_ = map_info.sub_type;
  // convert boundary
  v2_info_ = map_info.v2_info;
  seq_ = map_info.seq;
  timestamp_ = map_info.timestamp;
  for (const auto &boundary_info : map_info.all_lane_boundaries_vec) {
    auto boundary_ptr = std::make_shared<LaneBoundary>(boundary_info);
    lane_boundary_map_[boundary_info.id] = boundary_ptr;
  }
  // construct road boundaries
  for (auto &road_boundary : map_info.all_road_boundaries_vec) {
    road_boundary_map_[road_boundary.id] =
        std::make_shared<RoadBoundary>(road_boundary);
  }
  // convert lane
  for (const auto &lane_info : map_info.all_lanes_vec) {
    LaneBoundariesPtr left_boundaries_ptr =
        BuildLaneBoundaries(lane_info.left_lane_boundary_ids);
    LaneBoundariesPtr right_boundaries_ptr =
        BuildLaneBoundaries(lane_info.right_lane_boundary_ids);
    RoadBoundariesConstPtr left_road_boundaries_ptr =
        BuildRoadBoundaries(lane_info.left_road_boundary_ids);
    RoadBoundariesConstPtr right_road_boundaries_ptr =
        BuildRoadBoundaries(lane_info.right_road_boundary_ids);
    for (const auto &left_bound_id : lane_info.left_lane_boundary_ids) {
      if (lane_boundary_map_.find(left_bound_id) != lane_boundary_map_.end()) {
        lane_boundary_map_.at(left_bound_id)->PushRightLane(lane_info.id);
      }
    }
    for (const auto &right_bound_id : lane_info.right_lane_boundary_ids) {
      if (lane_boundary_map_.find(right_bound_id) != lane_boundary_map_.end()) {
        lane_boundary_map_.at(right_bound_id)->PushLeftLane(lane_info.id);
      }
    }

    lane_map_[lane_info.id] = std::make_shared<Lane>(
        lane_info, left_boundaries_ptr, right_boundaries_ptr,
        left_road_boundaries_ptr, right_road_boundaries_ptr);
  }
  // construct clear areas
  for (const auto &clear_area : map_info.all_clear_areas_vec) {
    clear_area_map_[clear_area.id] = std::make_shared<ClearArea>(clear_area);
  }
  // construct route
  route_ = std::make_shared<Route>(map_info.route, this);
  // update navigation info
  double next_section_length = 0.0;
  std::unordered_set<int> visited_extend_section_idx;
  for (auto section = route_->sections().rbegin();
       section != route_->sections().rend(); section++) {
    double section_length = section->length;
    SetLaneInfoBySection(*section, section_length, next_section_length);
    if (section->is_neighbor_extend && !section->extend_sections_index.empty()) {
      double next_section_length_tmp = next_section_length;
      for (auto extend_section_idx : section->extend_sections_index) {
        if (extend_section_idx >= 0 &&
            extend_section_idx < route_->extend_sections_vec().size() &&
            visited_extend_section_idx.find(extend_section_idx) == 
              visited_extend_section_idx.end()) {
          visited_extend_section_idx.insert(extend_section_idx);
          auto &extend_sections = route_->extend_sections_vec()
                                      .at(extend_section_idx)
                                      .extend_sections;
          for (auto extend_section = extend_sections.rbegin();
              extend_section != extend_sections.rend(); extend_section++) {
            double extend_section_length = extend_section->length;
            SetLaneInfoBySection(*extend_section, extend_section_length,
                                next_section_length_tmp);
            next_section_length_tmp = extend_section_length;
          }
        }
      }
    }
    // update next_section_length
    next_section_length = section_length;
  }
  // junction
  for (const auto &junction_info : map_info.all_junctions_vec) {
    if (junction_info.id == 0 || junction_info.points.size() < 3u) {
      continue;
    }
    junction_map_[junction_info.id] = std::make_shared<Junction>(junction_info);
  }
  // cross walk
  for (const auto &cross_walk_info : map_info.all_cross_walks_vec) {
    crosswalk_map_[cross_walk_info.id] =
        std::make_shared<Crosswalk>(cross_walk_info);
  }
  // stop line
  for (const auto &stop_line_info : map_info.all_stop_lines_vec) {
    stop_line_map_[stop_line_info.id] =
        std::make_shared<StopLine>(stop_line_info);
  }
  // speed_bump
  for (const auto &speed_bump_info : map_info.all_speed_bumps_vec) {
    speed_bump_map_[speed_bump_info.id] =
        std::make_shared<SpeedBump>(speed_bump_info);
  }

  // experience trajectory
  for (const auto &exp_traj : map_info.all_exp_trajectories_vec) {
    exp_trajectory_map_[exp_traj.id] =
        std::make_shared<ExpTrajectory>(exp_traj);
  }
}

double Map::GetLaneAnglediff(const LaneConstPtr &lane_split_front,
                             const LaneConstPtr &lane_split_back) {
  constexpr double kQueryDistAfterSplitPoint = 20.0;   // m.
  constexpr double kQueryDistbeforeSplitPoint = 25.0;  // m.
  double nearest_split_angle = 0.0;

  if (lane_split_front != nullptr && lane_split_back != nullptr &&
      lane_split_front->curve_length() > 0.0 &&
      lane_split_back->curve_length() > 0.0) {
    const double query_fraction = std::min(
        1.0, kQueryDistAfterSplitPoint / lane_split_back->curve_length());
    const Vec2d split_back_point =
        lane_split_back->LerpPointFromFraction(query_fraction);
    const Vec2d split_point = lane_split_front->LerpPointFromFraction(1.0);
    const double split_front_query_fraction =
        std::max(0.0, 1.0 - kQueryDistbeforeSplitPoint /
                                lane_split_front->curve_length());
    const Vec2d split_front_point =
        lane_split_front->LerpPointFromFraction(split_front_query_fraction);
    nearest_split_angle = NormalizeAngle2D(split_point - split_front_point,
                                           split_back_point - split_point);
  }
  return nearest_split_angle;
}

bool Map::CheckLaneIsJunction(const LanePtr &lane_ptr) {
  if (!lane_ptr) return false;
  if ((lane_ptr->junction_id() != 0 &&
       (lane_ptr->type() == LaneType::LANE_VIRTUAL_JUNCTION ||
        lane_ptr->type() == LaneType::LANE_VIRTUAL_CONNECTED_LANE ||
        lane_ptr->type() == LaneType::LANE_U_TURN_LANE)) ||
      lane_ptr->turn_type() == TurnType::U_TURN) {
    return true;
  } else {
    return false;
  }
}

void Map::ModifySplitTopology(const LanePtr &current_lane_info_ptr,
                              const std::vector<uint64_t> &next_lane_ids) {
  auto pre_lane_kappa = GetAverageKappa(current_lane_info_ptr);
  double kappa_cost_weight =
      st::Lerp(0.0, 0.0, 0.95, 0.025, pre_lane_kappa, true);
  double angle_cost_weight = 1.0 - kappa_cost_weight;

  auto split_none_lane_id = next_lane_ids.front();
  double min_cost = INT64_MAX;
  SplitTopology candidate_split_topo = TOPOLOGY_SPLIT_NONE;

  for (auto next_lane_id : next_lane_ids) {
    if (lane_map_.find(next_lane_id) != lane_map_.end()) {
      const auto &next_lane_info_ptr = lane_map_.at(next_lane_id);

      auto angle_diff =
          GetLaneAnglediff(current_lane_info_ptr, next_lane_info_ptr);
      auto next_lane_kappa = GetAverageKappa(next_lane_info_ptr);
      auto kappa_diff = next_lane_kappa - pre_lane_kappa;

      double split_cost =
          angle_cost_weight * fabs(angle_diff) +
          kappa_cost_weight * fabs(kappa_diff);  // consider kappa difference
                                                 // and angle difference

      if (min_cost > split_cost) {
        min_cost = split_cost;
        split_none_lane_id = next_lane_id;
      }
    }
  }

  for (auto next_lane_id : next_lane_ids) {
    if (lane_map_.find(next_lane_id) != lane_map_.end() &&
        lane_map_.find(split_none_lane_id) != lane_map_.end()) {
      const auto &split_none_lane_info_ptr = lane_map_.at(split_none_lane_id);
      auto &next_lane_info_ptr = lane_map_.at(next_lane_id);
      if (split_none_lane_id != next_lane_id &&
          split_none_lane_info_ptr->section_id() ==
              next_lane_info_ptr->section_id()) {
        if (next_lane_info_ptr->lane_ind_in_section() >
            split_none_lane_info_ptr->lane_ind_in_section()) {
          candidate_split_topo = TOPOLOGY_SPLIT_RIGHT;
        } else {
          candidate_split_topo = TOPOLOGY_SPLIT_LEFT;
        }
        next_lane_info_ptr->SetSplitTopology(candidate_split_topo);
        next_lane_info_ptr->SetSplitTopoModifyTrue();
      } else {
        continue;
      }
    }
  }
}

void Map::ModifyMergeTopology(const LanePtr &current_lane_info_ptr,
                              const std::vector<uint64_t> &pre_lane_ids) {
  auto merge_none_lane_id = pre_lane_ids.front();
  double min_cost = INT64_MAX;
  MergeTopology candidate_merge_topo = TOPOLOGY_MERGE_NONE;

  for (auto pre_lane_id : pre_lane_ids) {
    if (lane_map_.find(pre_lane_id) != lane_map_.end()) {
      const auto &pre_lane_info_ptr = lane_map_.at(pre_lane_id);

      auto merge_angle_diff =
          GetLaneAnglediff(pre_lane_info_ptr, current_lane_info_ptr);
      double merge_angle_cost_weight = 1.0;
      double merge_cost = merge_angle_cost_weight * fabs(merge_angle_diff);

      if (min_cost > merge_cost) {
        min_cost = merge_cost;
        merge_none_lane_id = pre_lane_id;
      }
    }
  }

  for (auto pre_lane_id : pre_lane_ids) {
    if (lane_map_.find(pre_lane_id) != lane_map_.end() &&
        lane_map_.find(merge_none_lane_id) != lane_map_.end()) {
      const auto &merge_none_lane_info_ptr = lane_map_.at(merge_none_lane_id);
      auto &pre_lane_info_ptr = lane_map_.at(pre_lane_id);
      if (merge_none_lane_id != pre_lane_id &&
          merge_none_lane_info_ptr->section_id() ==
              pre_lane_info_ptr->section_id()) {
        if (pre_lane_info_ptr->lane_ind_in_section() >
            merge_none_lane_info_ptr->lane_ind_in_section()) {
          candidate_merge_topo = TOPOLOGY_MERGE_LEFT;
        } else {
          candidate_merge_topo = TOPOLOGY_MERGE_RIGHT;
        }
        pre_lane_info_ptr->SetMergeTopology(candidate_merge_topo);
        pre_lane_info_ptr->SetMergeTopoModifyTrue();
      } else {
        continue;
      }
    }
  }
}

void Map::UpdateLaneSplitMergeTopology() {
  for (auto &lane : lane_map_) {
    const auto &current_lane_info_ptr = lane.second;
    const auto &pre_lane_ids = current_lane_info_ptr->pre_lane_ids();
    const auto &next_lane_ids = current_lane_info_ptr->next_lane_ids();

    // 1. check if any split_topo missing
    if (next_lane_ids.size() > 1) {
      bool all_next_lane_no_split{true};
      for (auto next_lane_id : next_lane_ids) {
        auto next_lane_it = lane_map_.find(next_lane_id);

        bool is_junction = (next_lane_it != lane_map_.end() &&
                            CheckLaneIsJunction(next_lane_it->second));

        bool is_split_topo =
            (next_lane_it != lane_map_.end() &&
             (next_lane_it->second->split_topology() == TOPOLOGY_SPLIT_LEFT ||
              next_lane_it->second->split_topology() == TOPOLOGY_SPLIT_RIGHT));

        if (is_split_topo || is_junction) {
          all_next_lane_no_split = false;
          break;
        }
      }

      if (all_next_lane_no_split) {
        ModifySplitTopology(current_lane_info_ptr, next_lane_ids);
      }
    }

    // 2. check if any merge_topo missing
    if (type_ == HD_MAP && pre_lane_ids.size() > 1) {
      bool all_pre_lane_no_merge{true};
      for (auto pre_lane_id : pre_lane_ids) {
        auto pre_lane_it = lane_map_.find(pre_lane_id);

        bool is_junction = (pre_lane_it != lane_map_.end() &&
                            CheckLaneIsJunction(pre_lane_it->second));

        bool is_merge_topo =
            (pre_lane_it != lane_map_.end() &&
             (pre_lane_it->second->merge_topology() == TOPOLOGY_MERGE_LEFT ||
              pre_lane_it->second->merge_topology() == TOPOLOGY_MERGE_RIGHT ||
              pre_lane_it->second->merge_topology() == TOPOLOGY_TO_BE_MERGED));

        bool is_split_topo =
            (pre_lane_it != lane_map_.end() &&
             (pre_lane_it->second->split_topology() == TOPOLOGY_SPLIT_LEFT ||
              pre_lane_it->second->split_topology() == TOPOLOGY_SPLIT_RIGHT));

        if (is_merge_topo || is_split_topo || is_junction) {
          all_pre_lane_no_merge = false;
          break;
        }
      }
      if (all_pre_lane_no_merge) {
        ModifyMergeTopology(current_lane_info_ptr, pre_lane_ids);
      }
    }

    // 3. change merge topology
    if (type_ == HD_MAP && is_on_highway_ && pre_lane_ids.size() == 2) {
      auto pre_lane_front = lane_map_.find(pre_lane_ids.front());
      auto pre_lane_back = lane_map_.find(pre_lane_ids.back());
      if (pre_lane_front != lane_map_.end() &&
          pre_lane_back != lane_map_.end() && pre_lane_front->second &&
          pre_lane_back->second && pre_lane_front->second->junction_id() == 0 &&
          pre_lane_back->second->junction_id() == 0 &&
          pre_lane_front->second->section_id() ==
              pre_lane_back->second->section_id()) {
        LanePtr pre_left_lane_ptr = nullptr, pre_right_lane_ptr = nullptr;
        if (pre_lane_front->second->left_lane_id() ==
            pre_lane_back->second->id()) {
          pre_left_lane_ptr = pre_lane_back->second;
          pre_right_lane_ptr = pre_lane_front->second;
        } else if (pre_lane_front->second->right_lane_id() ==
                   pre_lane_back->second->id()) {
          pre_left_lane_ptr = pre_lane_front->second;
          pre_right_lane_ptr = pre_lane_back->second;
        }
        if (pre_left_lane_ptr && pre_right_lane_ptr)
          ChangeLaneMergeTopo(pre_left_lane_ptr, pre_right_lane_ptr);
      }
    }
  }
}

void Map::ChangeLaneMergeTopo(LanePtr &left_lane, LanePtr &right_lane) {
  if (!left_lane || !right_lane) return;
  if (left_lane->type() != right_lane->type()) return;
  LaneConstPtr left_neighbor_lane = nullptr, right_neighbor_lane = nullptr;
  if (left_lane->left_lane_id() != 0) {
    auto left_neighbor_iter = lane_map_.find(left_lane->left_lane_id());
    if (left_neighbor_iter != lane_map_.end())
      left_neighbor_lane = left_neighbor_iter->second;
  }
  if (right_lane->right_lane_id() != 0) {
    auto right_neighbor_iter = lane_map_.find(right_lane->right_lane_id());
    if (right_neighbor_iter != lane_map_.end())
      right_neighbor_lane = right_neighbor_iter->second;
  }
  auto IsValidlLane = [](const LaneConstPtr &lane) {
    if (!lane) return false;
    const auto &type = lane->type();
    return type != LANE_UNKNOWN && type != LANE_EMERGENCY &&
           type != LANE_NON_MOTOR && type != LANE_DIVERSION;
  };
  if (IsValidlLane(left_neighbor_lane)) {
    if (!IsValidlLane(right_neighbor_lane) ||
        ((left_neighbor_lane->navi_distance() >
          right_neighbor_lane->navi_distance()) &&
         (left_neighbor_lane->type() == LANE_NORMAL ||
          right_neighbor_lane->type() != LANE_NORMAL))) {
      if (left_lane->merge_topology() != TOPOLOGY_MERGE_NONE) {
        left_lane->SetMergeTopology(TOPOLOGY_MERGE_NONE);
        left_lane->SetMergeTopoModifyTrue();
      }
      if (right_lane->merge_topology() != TOPOLOGY_MERGE_LEFT) {
        right_lane->SetMergeTopology(TOPOLOGY_MERGE_LEFT);
        right_lane->SetMergeTopoModifyTrue();
      }
    }
  }
  if (IsValidlLane(right_neighbor_lane)) {
    if (!IsValidlLane(left_neighbor_lane) ||
        ((right_neighbor_lane->navi_distance() >
          left_neighbor_lane->navi_distance()) &&
         (left_neighbor_lane->type() != LANE_NORMAL ||
          right_neighbor_lane->type() == LANE_NORMAL))) {
      if (left_lane->merge_topology() != TOPOLOGY_MERGE_RIGHT) {
        left_lane->SetMergeTopology(TOPOLOGY_MERGE_RIGHT);
        left_lane->SetMergeTopoModifyTrue();
      }
      if (right_lane->merge_topology() != TOPOLOGY_MERGE_NONE) {
        right_lane->SetMergeTopology(TOPOLOGY_MERGE_NONE);
        right_lane->SetMergeTopoModifyTrue();
      }
    }
  }
}

void Map::UpdateExpTrajectory() {
  exp_traj_to_lane_map_.clear();

  for (const auto &[traj_id, traj_ptr] : exp_trajectory_map_) {
    std::unordered_set<uint64_t> candidate_lane_ids;

    auto start_lane_it = lane_map_.find(traj_ptr->start_lane_id());
    if (start_lane_it != lane_map_.end()) {
      const auto &next_lane_ids = start_lane_it->second->next_lane_ids();
      candidate_lane_ids.insert(next_lane_ids.begin(), next_lane_ids.end());
    }

    auto end_lane_it = lane_map_.find(traj_ptr->end_lane_id());
    if (end_lane_it != lane_map_.end()) {
      const auto &prev_lane_ids = end_lane_it->second->pre_lane_ids();
      for (const auto &prev_id : prev_lane_ids) {
        if (candidate_lane_ids.count(prev_id) > 0) {
          exp_traj_to_lane_map_[traj_id] = prev_id;
          break;
        }
      }
    }
  }

  for (auto &[exp_traj_id, lane_id] : exp_traj_to_lane_map_) {
    if (lane_map_.find(lane_id) != lane_map_.end()) {
      auto &lane_ptr = lane_map_.at(lane_id);
      lane_ptr->UpdateCenterLinePoints(
          exp_trajectory_map_.at(exp_traj_id)->points());
      lane_ptr->SetExpTrajFlagTrue();
    }
  }
}

void Map::UpdateLane() {
  // Updata previous lane && insert overlap lane to junction
  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    for (const auto &next_id : lane_ptr->next_lane_ids()) {
      if (lane_map_.find(next_id) == lane_map_.end()) {
        continue;
      }
      auto &next_lane_ptr = lane_map_.at(next_id);
      next_lane_ptr->AddPreviousLane(lane_ptr->id());
    }
    if (junction_map_.find(lane_ptr->junction_id()) != junction_map_.end()) {
      auto &junction_ptr = junction_map_.at(lane_ptr->junction_id());
      junction_ptr->PushOverlapLane(lane_ptr);
    }
  }
  // traffic_status
  for (const auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    TrafficLightStatus traffic_light_status;
    traffic_light_status.lane_id = id_lane.first;
    if (lane_ptr->junction_id()) {
      traffic_light_status.junction_id = lane_ptr->junction_id();
    }
    traffic_light_status.light_status =
        static_cast<LightStatus>(lane_ptr->light_status());
    traffic_light_status.light_countdown = lane_ptr->light_countdown();
    traffic_light_status.traffic_set_reason =
        static_cast<TrafficSetReason>(lane_ptr->traffic_set_reason());
    traffic_light_status.pre_has_light = lane_ptr->pre_has_light();
    traffic_light_status.stopline_angle_flag = lane_ptr->stopline_angle_flag();
    traffic_light_status.light_turn_type = lane_ptr->turn_type();
    traffic_light_status.stop_line = lane_ptr->stop_line(),
    traffic_light_status.is_left_wait_lane =
        lane_ptr->type() == LANE_LEFT_WAIT ? true : false;
    traffic_light_status_map_[traffic_light_status.lane_id] =
        traffic_light_status;
  }
  // storage const lane ptr && storage valid lane
  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    // set valid predecessor lane
    std::vector<uint64_t> valid_pre_lanes;
    for (const auto &lane_id : lane_ptr->pre_lane_ids()) {
      if (lane_map_.find(lane_id) != lane_map_.end() &&
          lane_map_.at(lane_id)->IsValid()) {
        valid_pre_lanes.emplace_back(lane_id);
      }
    }
    lane_ptr->SetValidPredLaneIds(std::move(valid_pre_lanes));
    // set valid successor lane
    std::vector<uint64_t> valid_next_lanes;
    std::unordered_set<TurnType> next_turn_types;
    for (const auto &lane_id : lane_ptr->next_lane_ids()) {
      if (lane_map_.find(lane_id) != lane_map_.end() &&
          lane_map_.at(lane_id)->IsValid()) {
        valid_next_lanes.emplace_back(lane_id);
        next_turn_types.insert(lane_map_.at(lane_id)->turn_type());
      }
    }
    lane_ptr->SetValidNextLaneIds(std::move(valid_next_lanes));

    // emplace const ptr
    lanes_.emplace_back(lane_ptr);
  }

  // sort next lane id form left to right
  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    const math::LineCurve2d &center_line = lane_ptr->center_line();
    math::Vec2d process_lane_vector =
        center_line.end_point() - center_line.begin_point();
    // set valid successor lane
    std::vector<uint64_t> sorted_next_lanes;
    std::unordered_map<uint64_t, double> angle_of_next_lanes;

    for (const auto &lane_id : lane_ptr->next_lane_ids()) {
      if (lane_map_.find(lane_id) != lane_map_.end() &&
          lane_map_.at(lane_id)->IsValid()) {
        const auto &next_lane_ptr = lane_map_.at(lane_id);
        const math::LineCurve2d &next_center_line =
            next_lane_ptr->center_line();
        math::Vec2d next_lane_vector =
            next_center_line.end_point() - next_center_line.begin_point();
        double epsilon = 1e-10;
        double denominator =
            process_lane_vector.Length() * next_lane_vector.Length();
        if (denominator < epsilon) {
          continue;
        }
        double inner_prod = process_lane_vector.InnerProd(next_lane_vector);
        double cos_angle = inner_prod / denominator;
        double angle_rad = std::acos(cos_angle);
        if (process_lane_vector.CrossProd(next_lane_vector) < 0) {
          angle_rad = -angle_rad;
        }
        angle_of_next_lanes.insert(std::make_pair(lane_id, angle_rad));
        sorted_next_lanes.emplace_back(lane_id);
      }
    }
    std::sort(sorted_next_lanes.begin(), sorted_next_lanes.end(),
              [&](const uint64_t poly_id_1, const uint64_t poly_id_2) {
                return angle_of_next_lanes.at(poly_id_1) >
                       angle_of_next_lanes.at(poly_id_2);
              });
    lane_ptr->SetSortedNextLaneIds(std::move(sorted_next_lanes));
  }

  // set lane index in lane section
  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    std::size_t lane_ind_in_section = 1u;
    auto cur_lane_ptr = lane_ptr;
    std::vector<uint64_t> processed_lane_ids;
    while (!(cur_lane_ptr->left_lane_id() == 0) &&
           std::find(processed_lane_ids.begin(), processed_lane_ids.end(),
                     cur_lane_ptr->id()) == processed_lane_ids.end()) {
      processed_lane_ids.emplace_back(cur_lane_ptr->id());

      auto left_lane_it = lane_map_.find(cur_lane_ptr->left_lane_id());
      if (left_lane_it == lane_map_.end()) {
        break;
      }
      lane_ind_in_section++;
      cur_lane_ptr = left_lane_it->second;
    }
    lane_ptr->SetLaneIndInSection(lane_ind_in_section);
  }

  UpdateLaneSplitMergeTopology();

  // set lane interaction
  for (auto &id_lane : lane_map_) {
    auto &lane_ptr = id_lane.second;
    if (lane_ptr->center_line().IsValid()) {
      int pre_lane_ids_size = lane_ptr->pre_lane_ids().size();
      if (pre_lane_ids_size >= 2) {
        for (int i = 0; i < pre_lane_ids_size; i++) {
          auto pre_lane_iter = lane_map_.find(lane_ptr->pre_lane_ids().at(i));
          if (pre_lane_iter == lane_map_.end()) continue;
          if (pre_lane_iter->second->lane_info().merge_topology ==
                  TOPOLOGY_MERGE_LEFT ||
              pre_lane_iter->second->lane_info().merge_topology ==
                  TOPOLOGY_MERGE_RIGHT) {
            LaneInteraction interaction;
            interaction.geometric_configuration =
                LaneInteraction::GeometricConfiguration::MERGE;
            interaction.reaction_rule =
                LaneInteraction::ReactionRule::YIELD_MERGE;
            interaction.other_lane_id = i == 0
                                            ? lane_ptr->pre_lane_ids().back()
                                            : lane_ptr->pre_lane_ids().front();
            pre_lane_iter->second->AddInteractions(interaction);
          }
        }
      }
    }
  }
}

void Map::UpdateJunction() {
  st::MapParallelFor(
      junction_map_.begin(), junction_map_.end(),
      [&](std::pair<uint64_t, JunctionPtr> id_junction) {
        this->SetEntryLanesForJunction(id_junction.second);
        this->SetExitLanesForJunction(id_junction.second);
        this->SetCrosswalkForJunction(id_junction.second);
        const auto &junction_ptr = id_junction.second;
        LINFO("junction %d entry/exit lane %lu/%lu crosswalks %lu",
              junction_ptr->id(), junction_ptr->entry_lanes().size(),
              junction_ptr->exit_lanes().size(),
              junction_ptr->crosswalks().size());
      });
  for (auto &id_junction : junction_map_) {
    junctions_.emplace_back(id_junction.second);
  }
}

void Map::ConstructSection(SectionInfo &section) {
  SectionPtr section_ptr = std::make_shared<Section>();
  section_ptr->set_id(section.id);
  section_ptr->set_topo_length(section.length);
  std::unordered_set<uint64_t> next_sections_set;
  std::unordered_set<uint64_t> pre_sections_set;
  for (const auto &lane_id : section.lane_ids) {
    const auto &lane_it = lane_map_.find(lane_id);
    if (lane_it == lane_map_.end()) {
      continue;
    }
    auto lane = lane_it->second;
    section_ptr->add_lanes(lane->id());
    section_ptr->set_curve_length(
        std::max(section_ptr->curve_length(), lane->curve_length()));
    section_ptr->set_topo_length(
        std::max(section_ptr->topo_length(), lane->topo_length()));
    section_ptr->set_speed_limit(
        std::max(section_ptr->speed_limit(), lane->speed_limit()));
    for (const auto &next_lane : lane->next_lane_ids()) {
      if (lane_map_.find(next_lane) != lane_map_.end() &&
          lane_map_[next_lane] &&
          !next_sections_set.count(lane_map_[next_lane]->section_id())) {
        section_ptr->add_outgoing_sections(lane_map_[next_lane]->section_id());
        next_sections_set.insert(lane_map_[next_lane]->section_id());
      }
    }
    for (const auto &pre_lane : lane->pre_lane_ids()) {
      if (lane_map_.find(pre_lane) != lane_map_.end() && lane_map_[pre_lane] &&
          !pre_sections_set.count(lane_map_[pre_lane]->section_id())) {
        section_ptr->add_incoming_sections(lane_map_[pre_lane]->section_id());
        pre_sections_set.insert(lane_map_[pre_lane]->section_id());
      }
    }
  }
  std::sort(section_ptr->mutable_lanes()->begin(),
            section_ptr->mutable_lanes()->end(),
            [&](const uint64_t id1, const uint64_t id2) {
              const auto &lane1 = lane_map_.find(id1);
              const auto &lane2 = lane_map_.find(id2);
              return lane1 != lane_map_.end() && lane2 != lane_map_.end() &&
                     lane1->second->lane_ind_in_section() <
                         lane2->second->lane_ind_in_section();
            });
  if (section.road_class == MapRoadClass::RC_EXPRESSWAY) {
    section_ptr->set_road_class(Section::RoadClass::HIGHWAY);
  } else if (section.road_class == MapRoadClass::RC_URBAN_EXPRESSWAY) {
    section_ptr->set_road_class(Section::RoadClass::CITY_EXPRESS);
  } else {
    section_ptr->set_road_class(Section::RoadClass::NORMAL);
  }
  section_ptr->set_average_limit(section_ptr->speed_limit());
  section_ptr->set_none_odd_type((Section::NoneOddType)section.none_odd_type);
  section_map_[section_ptr->id()] = section_ptr;
  route_section_seq_.emplace_back(section_ptr->id());
  // LOG(ERROR) << " section_ptr->id(): " << section_ptr->id();
  // sort section lane ids
  section.lane_ids.clear();
  for (const auto &sec_lane_id : section_ptr->lanes()) {
    section.lane_ids.emplace_back(sec_lane_id);
  }
  // set direction of outgoing section
  const auto outgoing_sections = section_ptr->outgoing_sections();
  if (outgoing_sections.size() > 1) {
    // set the successor section direction of the first lane to LEFT
    bool outgoing_left_set = false;
    for (const auto &lane_id : section_ptr->lanes()) {
      const auto lane_ptr = GetLaneById(lane_id);
      if (!lane_ptr) continue;
      for (const auto &next_lane_id : lane_ptr->next_lane_ids()) {
        const auto next_lane_ptr = GetLaneById(next_lane_id);
        if (next_lane_ptr) {
          section_ptr->set_outgoing_direction(next_lane_ptr->section_id(),
                                              Section::SectionDirection::LEFT);
          outgoing_left_set = true;
          break;
        }
      }
      if (outgoing_left_set) break;
    }
    // set the successor section direction of the last lane to RIGHT
    bool outgoing_right_set = false;
    for (auto it = section_ptr->lanes().rbegin();
         it != section_ptr->lanes().rend(); ++it) {
      const auto lane_id = *it;
      const auto lane_ptr = GetLaneById(lane_id);
      if (!lane_ptr) continue;
      for (const auto &next_lane_id : lane_ptr->next_lane_ids()) {
        const auto next_lane_ptr = GetLaneById(next_lane_id);
        if (next_lane_ptr) {
          section_ptr->set_outgoing_direction(next_lane_ptr->section_id(),
                                              Section::SectionDirection::RIGHT);
          outgoing_right_set = true;
          break;
        }
      }
      if (outgoing_right_set) break;
    }
  } else {
    // set direction to NONE if number of successor sections is less than 2
    for (const auto &outgoing_section : outgoing_sections) {
      section_ptr->set_outgoing_direction(outgoing_section,
                                          Section::SectionDirection::NONE);
    }
  }
}

void Map::UpdateSection() {
  route_section_seq_.clear();
  route_section_seq_.reserve(route_->mutable_route_info()->sections.size());
  for (auto &section : route_->mutable_route_info()->sections) {
    if (section.is_neighbor_extend) {
      for (auto idx : section.extend_sections_index) {
        for (auto &extend_section : route_->mutable_route_info()
                                        ->extend_sections_vec.at(idx)
                                        .extend_sections) {
          ConstructSection(extend_section);
        }
      }
    }
    ConstructSection(section);
  }
}

void Map::UpdateBoundarySection() {
  for (const auto &lane : lanes_) {
    if (lane->left_boundary()) {
      for (const auto &lane_boundary :
           lane->left_boundary()->lane_boundaries()) {
        if (lane_boundary->section_id() == 0 &&
            lane_boundary_map_.find(lane_boundary->id()) !=
                lane_boundary_map_.end()) {
          auto mutable_lane_boundary =
              lane_boundary_map_.at(lane_boundary->id());
          mutable_lane_boundary->set_section_id(lane->section_id());
        }
      }
    }

    if (lane->right_boundary()) {
      for (const auto &lane_boundary :
           lane->right_boundary()->lane_boundaries()) {
        if (lane_boundary->section_id() == 0 &&
            lane_boundary_map_.find(lane_boundary->id()) !=
                lane_boundary_map_.end()) {
          auto mutable_lane_boundary =
              lane_boundary_map_.at(lane_boundary->id());
          mutable_lane_boundary->set_section_id(lane->section_id());
        }
      }
    }
    if (lane->left_road_boundary()) {
      for (const auto &lane_boundary :
           lane->left_road_boundary()->road_boundaries()) {
        if (lane_boundary->section_id() == 0 &&
            lane_boundary_map_.find(lane_boundary->id()) !=
                lane_boundary_map_.end()) {
          auto mutable_lane_boundary =
              road_boundary_map_.at(lane_boundary->id());
          mutable_lane_boundary->set_section_id(lane->section_id());
        }
      }
    }
    if (lane->right_road_boundary()) {
      for (const auto &lane_boundary :
           lane->right_road_boundary()->road_boundaries()) {
        if (lane_boundary->section_id() == 0 &&
            lane_boundary_map_.find(lane_boundary->id()) !=
                lane_boundary_map_.end()) {
          auto mutable_lane_boundary =
              road_boundary_map_.at(lane_boundary->id());
          mutable_lane_boundary->set_section_id(lane->section_id());
        }
      }
    }
  }
}

void Map::SetCrosswalkForJunction(JunctionPtr &junction_ptr) {
  std::unordered_set<CrosswalkConstPtr> crosswalks;
  std::unordered_set<uint64_t> crosswalk_ids;
  for (const auto &overlap_lane_ptr : junction_ptr->overlap_lanes()) {
    for (const auto &crosswalk_id : overlap_lane_ptr->crosswalks()) {
      if (crosswalk_ids.find(crosswalk_id) == crosswalk_ids.end() &&
          crosswalk_map_.find(crosswalk_id) != crosswalk_map_.end() &&
          crosswalk_map_.at(crosswalk_id) != nullptr) {
        crosswalk_ids.insert(crosswalk_id);
        crosswalks.insert(crosswalk_map_.at(crosswalk_id));
      }
    }
  }
  junction_ptr->SetCrosswalks(std::move(crosswalks));
}

void Map::SetEntryLanesForJunction(JunctionPtr &junction_ptr) {
  std::vector<LaneConstPtr> entry_lanes;
  std::unordered_set<uint64_t> entry_lane_ids;
  for (const auto &overlap_lane_ptr : junction_ptr->overlap_lanes()) {
    for (const auto &lane_id : overlap_lane_ptr->pre_lane_ids()) {
      auto lane_ptr = GetLaneById(lane_id);
      if (lane_ptr == nullptr || lane_ptr->IsVirtual() ||
          entry_lane_ids.find(lane_id) != entry_lane_ids.end()) {
        continue;
      }
      // LDEBUG("lane id %s predecessor %s nullptr %d virtual %d find %d",
      //        overlap_lane_ptr->id().c_str(), lane_id.c_str(),
      //        lane_ptr == nullptr, lane_ptr->IsVirtual(),
      //        entry_lane_ids.find(lane_id) != entry_lane_ids.end());
      entry_lane_ids.insert(lane_id);
      entry_lanes.emplace_back(lane_ptr);
    }
  }
  junction_ptr->SetEntryLanes(entry_lanes);
}

void Map::SetExitLanesForJunction(JunctionPtr &junction_ptr) {
  std::vector<LaneConstPtr> exit_lanes;
  std::unordered_set<uint64_t> exit_lane_ids;
  for (const auto &overlap_lane_ptr : junction_ptr->overlap_lanes()) {
    for (const auto &lane_id : overlap_lane_ptr->next_lane_ids()) {
      auto lane_ptr = GetLaneById(lane_id);
      if (lane_ptr == nullptr || lane_ptr->IsVirtual() ||
          exit_lane_ids.find(lane_id) != exit_lane_ids.end()) {
        continue;
      }
      // LDEBUG("lane id %s successor %s nullptr %d virtual %d find %d",
      //        overlap_lane_ptr->id().c_str(), lane_id.c_str(),
      //        lane_ptr == nullptr, lane_ptr->IsVirtual(),
      //        exit_lane_ids.find(lane_id) != exit_lane_ids.end());
      exit_lane_ids.insert(lane_id);
      exit_lanes.emplace_back(lane_ptr);
    }
  }
  std::unordered_map<uint64_t, LaneConstPtr> neighbors_lane_map;
  for (const auto &lane_id : exit_lane_ids) {
    auto lane_ptr = GetLaneById(lane_id);
    if (lane_ptr == nullptr) {
      continue;
    }
    // check left lane is exit lane
    {
      const auto &left_lane_id = lane_ptr->left_lane_id();
      auto left_lane_ptr = GetLaneById(left_lane_id);
      if (left_lane_ptr != nullptr && !left_lane_ptr->IsVirtual() &&
          exit_lane_ids.find(left_lane_id) == exit_lane_ids.end() &&
          neighbors_lane_map.find(left_lane_id) == neighbors_lane_map.end()) {
        neighbors_lane_map.insert(std::make_pair(left_lane_id, left_lane_ptr));
      }
    }
    // check right exit lane
    {
      const auto &right_lane_id = lane_ptr->right_lane_id();
      auto right_lane_ptr = GetLaneById(right_lane_id);
      if (right_lane_ptr != nullptr && !right_lane_ptr->IsVirtual() &&
          exit_lane_ids.find(right_lane_id) == exit_lane_ids.end() &&
          neighbors_lane_map.find(right_lane_id) == neighbors_lane_map.end()) {
        neighbors_lane_map.insert(
            std::make_pair(right_lane_id, right_lane_ptr));
      }
    }
  }
  for (const auto &neight_lane : neighbors_lane_map) {
    exit_lanes.emplace_back(neight_lane.second);
  }
  junction_ptr->SetExitLanes(exit_lanes);
}

void Map::GetLanes(const Point2d &query_pt, const double &heading,
                   const double &dist, const double &heading_limit,
                   std::vector<LaneConstPtr> *const lanes) const {
  if (lane_map_.empty()) {
    LINFO("Map::GetLanes: no lane in map");
    return;
  }
  lanes->clear();
  double search_radius =
      std::max(FLAGS_ad_byd_planning_map_minimum_boundary_search_radius, dist);

  const auto &boundaries_segment = FindLaneBoundarySegmentsInRadius(
      query_pt.x(), query_pt.y(), search_radius);
  if (boundaries_segment.empty()) {
    return;
  }

  std::unordered_set<uint64_t> candidata_lane_ids;
  for (const auto &boundary_segment : boundaries_segment) {
    const auto &boundary_id = boundary_segment.element_id;
    int32_t pt_idx = boundary_segment.segment_id.value();
    if (lane_boundary_map_.find(boundary_id) == lane_boundary_map_.end() ||
        !lane_boundary_map_.at(boundary_id)->IsValid()) {
      continue;
    }
    const auto boundary_ptr = lane_boundary_map_.at(boundary_id);
    if (pt_idx < 0 ||
        static_cast<int32_t>(boundary_ptr->curve_points().size()) <= pt_idx) {
      continue;
    }

    // check lane by boundary
    std::unordered_set<uint64_t> boundary_lanes;
    double l =
        GetMinLDistByPointIndex(query_pt, boundary_ptr->curve_points(), pt_idx);
    const auto &cmp = Double::Compare(l, 0.0);
    if (cmp == Double::CompareType::EQUAL) {
      // point on boundary check left & right boundary lane
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else if (cmp == Double::CompareType::GREATER) {
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else {
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
    }
    for (const auto &lane_id : boundary_lanes) {
      if (lane_map_.find(lane_id) == lane_map_.end() ||
          !lane_map_.at(lane_id)->IsValid() ||
          (std::fabs(l) > dist && !lane_map_.at(lane_id)->IsOnLane(query_pt))) {
        continue;
      }
      // check heading
      SLPoint sl_pt;
      double lane_heading = 0.0;
      const auto &lane_ptr = lane_map_.at(lane_id);
      lane_ptr->GetSLWithoutLimit(query_pt, &sl_pt);
      lane_ptr->GetHeadingFromS(sl_pt.s, &lane_heading);
      if (std::fabs(math::AngleDiff(lane_heading, heading)) > heading_limit) {
        continue;
      }
      candidata_lane_ids.insert(lane_id);
    }
  }
  for (const auto &lane_id : candidata_lane_ids) {
    lanes->emplace_back(lane_map_.at(lane_id));
  }
}

void Map::GetLanes(const Point2d &query_pt, const double &heading,
                   const double &dist,
                   std::vector<LaneConstPtr> *const lanes) const {
  GetLanes(query_pt, heading, dist,
           FLAGS_ad_byd_planning_map_search_heading_limit, lanes);
}

void Map::GetLanes(const Point2d &query_pt, const double &dist,
                   std::vector<LaneConstPtr> *const lanes) const {
  if (lane_map_.empty()) {
    LINFO("Map::GetLanes: no lane in map");
    return;
  }
  lanes->clear();
  double search_radius =
      std::max(FLAGS_ad_byd_planning_map_minimum_boundary_search_radius, dist);

  const auto &boundaries_segment = FindLaneBoundarySegmentsInRadius(
      query_pt.x(), query_pt.y(), search_radius);
  if (boundaries_segment.empty()) {
    return;
  }

  std::unordered_set<uint64_t> candidata_lane_ids;
  for (const auto &boundary_segment : boundaries_segment) {
    const auto &boundary_id = boundary_segment.element_id;
    int32_t pt_idx = boundary_segment.segment_id.value();
    if (lane_boundary_map_.find(boundary_id) == lane_boundary_map_.end() ||
        !lane_boundary_map_.at(boundary_id)->IsValid()) {
      continue;
    }
    const auto boundary_ptr = lane_boundary_map_.at(boundary_id);
    if (pt_idx < 0 ||
        static_cast<int32_t>(boundary_ptr->curve_points().size()) <= pt_idx) {
      continue;
    }

    // check lane by boundary
    std::unordered_set<uint64_t> boundary_lanes;
    double l =
        GetMinLDistByPointIndex(query_pt, boundary_ptr->curve_points(), pt_idx);
    const auto &cmp = Double::Compare(l, 0.0);
    if (cmp == Double::CompareType::EQUAL) {
      // point on boundary check left & right boundary lane
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else if (cmp == Double::CompareType::GREATER) {
      boundary_lanes.insert(boundary_ptr->right_lanes().begin(),
                            boundary_ptr->right_lanes().end());
    } else {
      boundary_lanes.insert(boundary_ptr->left_lanes().begin(),
                            boundary_ptr->left_lanes().end());
    }
    for (const auto &lane_id : boundary_lanes) {
      if (lane_map_.find(lane_id) == lane_map_.end() ||
          !lane_map_.at(lane_id)->IsValid()) {
        continue;
      }
      const auto &lane_ptr = lane_map_.at(lane_id);
      SLPoint sl_point;
      lane_ptr->GetSLWithoutLimit(query_pt, &sl_point);
      double dist_to_lane = 0.0;
      if (sl_point.s < 0.0) {
        dist_to_lane =
            lane_ptr->center_line().points().front().DistanceTo(query_pt);
      } else if (sl_point.s > lane_ptr->curve_length()) {
        dist_to_lane =
            lane_ptr->center_line().points().back().DistanceTo(query_pt);
      } else {
        double lw = 0.0;
        double rw = 0.0;
        lane_ptr->GetWidthFromS(sl_point.s, &lw, &rw);
        dist_to_lane = std::max(sl_point.l - lw, rw - sl_point.l);
      }
      if (math::Double::Compare(dist_to_lane, dist) !=
          math::Double::CompareType::GREATER) {
        candidata_lane_ids.insert(lane_id);
      }
    }
  }
  for (const auto &lane_id : candidata_lane_ids) {
    lanes->emplace_back(lane_map_.at(lane_id));
  }
}

void Map::OnLanes(const Point2d &query_pt, const double &heading,
                  const double &heading_limit,
                  std::vector<LaneConstPtr> *const lanes) const {
  std::vector<LaneConstPtr> candidate_lanes;
  GetLanes(query_pt, heading, 0.0, heading_limit, &candidate_lanes);
  FilterPredecessorLanes(query_pt, candidate_lanes, lanes);
}

void Map::OnLanes(const Point2d &query_pt, const double &heading,
                  std::vector<LaneConstPtr> *const lanes) const {
  std::vector<LaneConstPtr> candidate_lanes;
  GetLanes(query_pt, heading, 0.0, &candidate_lanes);
  FilterPredecessorLanes(query_pt, candidate_lanes, lanes);
}

void Map::OnLanes(const Point2d &query_pt,
                  std::vector<LaneConstPtr> *const lanes) const {
  std::vector<LaneConstPtr> candidate_lanes;
  GetLanes(query_pt, 0.0, &candidate_lanes);
  FilterPredecessorLanes(query_pt, candidate_lanes, lanes);
}

double Map::GetMinLDistByPointIndex(const Point2d &query_pt,
                                    const std::vector<Point2d> &points,
                                    const int32_t &pt_idx) const {
  double l = 0.0;
  if (pt_idx == 0) {
    Point2d bound_vec = points.at(pt_idx + 1) - points.at(pt_idx);
    Point2d query_vec = query_pt - points.at(pt_idx);
    l = bound_vec.CrossProd(query_vec) /
        std::max(FLAGS_ad_byd_planning_zero_threshold, bound_vec.Length());
  } else if (pt_idx + 1 == static_cast<int32_t>(points.size())) {
    Point2d bound_vec = points.at(pt_idx) - points.at(pt_idx - 1);
    Point2d query_vec = query_pt - points.at(pt_idx - 1);
    l = bound_vec.CrossProd(query_vec) /
        std::max(FLAGS_ad_byd_planning_zero_threshold, bound_vec.Length());
  } else {
    Point2d prev_vec = points.at(pt_idx) - points.at(pt_idx - 1);
    Point2d next_vec = points.at(pt_idx + 1) - points.at(pt_idx);
    Point2d query_prev_vec = query_pt - points.at(pt_idx - 1);
    Point2d query_next_vec = query_pt - points.at(pt_idx);
    double prev_l =
        prev_vec.CrossProd(query_prev_vec) /
        std::max(FLAGS_ad_byd_planning_zero_threshold, prev_vec.Length());
    double next_l =
        next_vec.CrossProd(query_next_vec) /
        std::max(FLAGS_ad_byd_planning_zero_threshold, next_vec.Length());
    l = std::fabs(prev_l) > std::fabs(next_l) ? prev_l : next_l;
  }
  return l;
}

LaneBoundariesPtr Map::BuildLaneBoundaries(
    const std::vector<uint64_t> &lane_boundary_ids) {
  std::vector<LaneBoundaryConstPtr> boundaries;
  for (const auto &boundary_id : lane_boundary_ids) {
    if (lane_boundary_map_.find(boundary_id) != lane_boundary_map_.end() &&
        lane_boundary_map_.at(boundary_id)->IsValid()) {
      boundaries.emplace_back(lane_boundary_map_.at(boundary_id));
    }
  }
  return std::make_shared<LaneBoundaries>(boundaries);
}

RoadBoundariesPtr Map::BuildRoadBoundaries(
    const std::vector<uint64_t> &road_boundary_ids) {
  std::vector<RoadBoundaryConstPtr> boundaries;
  for (const auto &boundary_id : road_boundary_ids) {
    if (road_boundary_map_.find(boundary_id) != road_boundary_map_.end() &&
        road_boundary_map_.at(boundary_id)->IsValid()) {
      boundaries.emplace_back(road_boundary_map_.at(boundary_id));
    }
  }
  return std::make_shared<RoadBoundaries>(boundaries);
}

void Map::FilterPredecessorLanes(
    const Point2d &query_pt, const std::vector<LaneConstPtr> &candidate_lanes,
    std::vector<LaneConstPtr> *const lanes) const {
  std::vector<uint64_t> candidate_lane_ids;
  std::vector<uint64_t> prev_lane_ids;
  for (const auto &lane_ptr : candidate_lanes) {
    if (lane_ptr->IsOnLane(query_pt)) {
      candidate_lane_ids.emplace_back(lane_ptr->id());
      prev_lane_ids.insert(prev_lane_ids.end(),
                           lane_ptr->pre_lane_ids().begin(),
                           lane_ptr->pre_lane_ids().end());
    }
  }
  std::sort(candidate_lane_ids.begin(), candidate_lane_ids.end());
  std::sort(prev_lane_ids.begin(), prev_lane_ids.end());

  std::vector<uint64_t> result_lanes;
  std::set_difference(candidate_lane_ids.begin(), candidate_lane_ids.end(),
                      prev_lane_ids.begin(), prev_lane_ids.end(),
                      std::back_inserter(result_lanes));
  lanes->clear();
  for (const auto &lane_id : result_lanes) {
    lanes->emplace_back(lane_map_.at(lane_id));
  }
}

JunctionConstPtr Map::GetJunctionById(const uint64_t id) const {
  if (junction_map_.find(id) == junction_map_.end()) return nullptr;
  return junction_map_.at(id);
}

void Map::GetJunctions(const Point2d &pt, const double &dist,
                       std::vector<JunctionConstPtr> *const junctions) const {
  junctions->clear();
  for (const auto &id_junction : junction_map_) {
    const auto &junction_ptr = id_junction.second;
    if (Double::Compare(junction_ptr->DistanceTo(pt), dist) !=
        Double::CompareType::GREATER) {
      junctions->emplace_back(junction_ptr);
    }
  }
}

CrosswalkConstPtr Map::GetCrosswalkById(const uint64_t id) const {
  if (crosswalk_map_.find(id) == crosswalk_map_.end()) return nullptr;
  return crosswalk_map_.at(id);
}

RoadBoundaryConstPtr Map::GetRoadBoundaryById(const uint64_t id) const {
  if (road_boundary_map_.find(id) == road_boundary_map_.end()) return nullptr;
  if (road_boundary_map_.at(id)->type().boundary_type ==
      ad_byd::planning::BoundaryType::VIRTUAL)
    return nullptr;
  return road_boundary_map_.at(id);
}

LaneBoundaryConstPtr Map::GetLaneBoundaryById(const uint64_t id) const {
  if (lane_boundary_map_.find(id) == lane_boundary_map_.end()) return nullptr;
  return lane_boundary_map_.at(id);
}
SectionConstPtr Map::GetSectionById(const uint64_t id) const {
  if (section_map_.find(id) == section_map_.end()) return nullptr;
  return section_map_.at(id);
}
SectionPtr Map::GetMutableSectionById(const uint64_t id) {
  if (section_map_.find(id) == section_map_.end()) return nullptr;
  return section_map_.at(id);
}

std::vector<uint64_t> Map::FindNearRoadBoundaryIds(
    const std::string &obs_id, const Vec2d &position, const double heading,
    double range, bool check_angle, double heading_thresh,
    int *left_or_right) const {
  std::vector<uint64_t> near_ids;
  const auto &boundaries_segment =
      FindLaneBoundarySegmentsInRadius(position.x(), position.y(), range);
  if (boundaries_segment.empty()) {
    return near_ids;
  }
  LDEBUG("query pos(%f, %f) range %f search %lu points", position.x(),
         position.y(), range, boundaries_segment.size());
  for (const auto &pt : boundaries_segment) {
    const auto &boundary_id = pt.element_id;
    if (lane_boundary_map_.find(boundary_id) == lane_boundary_map_.end()) {
      continue;
    }
    LDEBUG("Obstacle [%s] find near lane boundary %lu", obs_id.c_str(),
           boundary_id);
    const auto boundary_ptr = lane_boundary_map_.at(boundary_id);
    std::vector<uint64_t> lane_ids;
    lane_ids.insert(lane_ids.end(), boundary_ptr->left_lanes().begin(),
                    boundary_ptr->left_lanes().end());
    lane_ids.insert(lane_ids.end(), boundary_ptr->right_lanes().begin(),
                    boundary_ptr->right_lanes().end());
    for (const auto &lane_id : lane_ids) {
      auto lane_ptr = GetLaneById(lane_id);
      if (lane_ptr == nullptr) {
        continue;
      }
      LDEBUG("Obstacle [%s] find near lane %lu", obs_id.c_str(), lane_id);
      if (lane_ptr->left_road_boundary() != nullptr) {
        const auto &cur_boundarys =
            lane_ptr->left_road_boundary()->road_boundaries();
        for (const auto &cur_boundary : cur_boundarys)
          near_ids.push_back(cur_boundary->id());
      }
      if (lane_ptr->right_road_boundary() != nullptr) {
        const auto &cur_boundarys =
            lane_ptr->right_road_boundary()->road_boundaries();
        for (const auto &cur_boundary : cur_boundarys)
          near_ids.push_back(cur_boundary->id());
      }
    }
  }
  return near_ids;
}
namespace {

const absl::flat_hash_map<std::type_index, ad_byd::planning::FeatureType>
    kGeoObjectTypeToFeatureType = {
        {typeid(ad_byd::planning::LaneBoundaryPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_LANEBOUNDARY},
        {typeid(ad_byd::planning::RoadBoundaryPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_ROADBOUNDARY},
        {typeid(ad_byd::planning::LanePtr),
         ad_byd::planning::FeatureType::FEATURETYPE_LANE},
        {typeid(ad_byd::planning::ClearAreaPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_CLEARAREA},
        {typeid(ad_byd::planning::JunctionPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_JUNCTION},
        {typeid(ad_byd::planning::CrosswalkPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_CROSSWALK},
        {typeid(ad_byd::planning::StopLinePtr),
         ad_byd::planning::FeatureType::FEATURETYPE_STOPLINE},
        {typeid(ad_byd::planning::SpeedBumpPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_SPEEDBUMP},
        {typeid(ad_byd::planning::LaneBoundaryConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_LANEBOUNDARY},
        {typeid(ad_byd::planning::RoadBoundaryConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_ROADBOUNDARY},
        {typeid(ad_byd::planning::LaneConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_LANE},
        {typeid(ad_byd::planning::ClearAreaConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_CLEARAREA},
        {typeid(ad_byd::planning::JunctionConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_JUNCTION},
        {typeid(ad_byd::planning::CrosswalkConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_CROSSWALK},
        {typeid(ad_byd::planning::StopLineConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_STOPLINE},
        {typeid(ad_byd::planning::SpeedBumpConstPtr),
         ad_byd::planning::FeatureType::FEATURETYPE_SPEEDBUMP}};

template <typename Segment, typename Object, typename Tree>
const Segment *FindNearestSegment(const Tree &feature_aabox_tree,
                                  const Vec2d &geo_point) {
  const auto it =
      feature_aabox_tree.find(kGeoObjectTypeToFeatureType.at(typeid(Object)));
  if (it == feature_aabox_tree.end() ||
      it->second.Get()->aabox_tree == nullptr) {
    return nullptr;
  }

  const auto point = geo_point;
  return it->second.Get()->aabox_tree->GetNearestObject(point);
}

template <typename Segment, typename Object, typename Tree>
std::vector<const Segment *> FindSegmentsInRadius(
    const Tree &feature_aabox_tree, const Vec2d &geo_point, double radius) {
  const auto it =
      feature_aabox_tree.find(kGeoObjectTypeToFeatureType.at(typeid(Object)));
  if (it == feature_aabox_tree.end() ||
      it->second.Get()->aabox_tree == nullptr) {
    return {};
  }

  const auto point = geo_point;
  return it->second.Get()->aabox_tree->GetObjects(point, radius);
}

template <typename Segment, typename Object, typename Tree>
Object FindNearestObject(
    const Tree &feature_aabox_tree, const Vec2d &geo_point,
    const std::function<Object(st::mapping::ElementId)> &find_object) {
  const auto *object =
      FindNearestSegment<Segment, Object, Tree>(feature_aabox_tree, geo_point);
  if (object == nullptr) {
    return nullptr;
  }

  return find_object(object->GetElementId());
}

template <typename ObjectSegment, typename Object, typename Tree>
absl::StatusOr<st::mapping::v2::Segment> FindNearestObjectSegment(
    const Tree &feature_aabox_tree, const Vec2d &geo_point) {
  const auto *object_segment = FindNearestSegment<ObjectSegment, Object, Tree>(
      feature_aabox_tree, geo_point);
  if (object_segment == nullptr) {
    return absl::NotFoundError("Failed to find nearest segment.");
  }
  return st::mapping::v2::Segment{.element_id = object_segment->GetElementId(),
                                  .segment_id = object_segment->GetSegmentId()};
}

template <typename Segment, typename Object, typename Tree>
std::vector<Object> FindObjectsInRadius(
    const Tree &feature_aabox_tree, const Vec2d &geo_point, double radius,
    const std::function<Object(st::mapping::ElementId)> &find_object) {
  absl::flat_hash_set<st::mapping::ElementId> object_ids_in_radius;
  for (const auto &object : FindSegmentsInRadius<Segment, Object, Tree>(
           feature_aabox_tree, geo_point, radius)) {
    object_ids_in_radius.insert(object->GetElementId());
  }

  std::vector<Object> objects_in_radius;
  for (const auto &object_id : object_ids_in_radius) {
    if (const auto obj = find_object(object_id); obj != nullptr) {
      objects_in_radius.push_back(obj);
    }
  }
  return objects_in_radius;
}

template <typename ObjectSegment, typename Object, typename Tree>
std::vector<st::mapping::v2::Segment> FindObjectSegmentsInRadius(
    const Tree &feature_aabox_tree, const Vec2d &geo_point, double radius) {
  const auto segments = FindSegmentsInRadius<ObjectSegment, Object, Tree>(
      feature_aabox_tree, geo_point, radius);
  std::vector<st::mapping::v2::Segment> segments_in_radius;
  segments_in_radius.reserve(segments.size());
  for (const auto &seg : segments) {
    segments_in_radius.push_back(
        {.element_id = seg->GetElementId(), .segment_id = seg->GetSegmentId()});
  }

  return segments_in_radius;
}

template <typename Container, typename Node, typename Tree>
void BuildAABoxNodes(
    const Container &elements,
    const std::function<bool(const typename Container::value_type &)>
        &filter_element,
    std::vector<Node> *nodes, Tree *tree) {
  if (elements.size() == 0) {
    return;
  }
  nodes->reserve(elements.size() * 3);
  for (const auto &element : elements) {
    if (filter_element(element) || !element.second) {
      continue;
    }
    const auto &segment_points = element.second->points();
    st::mapping::SegmentId segment_id(0);
    if (segment_points.empty()) {
      continue;
    }
    for (auto first = segment_points.begin(), second = std::next(first);
         second != segment_points.end(); ++first, ++second, ++segment_id) {
      nodes->emplace_back(element.second->id(), segment_id, *first, *second);
    }
  }
  nodes->shrink_to_fit();
  *tree = std::make_unique<st::AABoxKDTree2d<Node>>(
      *nodes, st::AABoxKDTreeParams{.max_leaf_size = 4});
}

void FindSegmentsByMinDistanceToPoint(
    const Vec2d &smooth_point,
    const std::vector<st::Segment2d> &smooth_segments, double *min_dist_square,
    std::vector<st::Segment2d> *segments_with_min_dist) {
  bool is_inserted = false;
  for (const auto &smooth_segment : smooth_segments) {
    const auto length_square = smooth_segment.length_sqr();
    if (const auto max_dist = *min_dist_square + length_square +
                              2 * std::max(*min_dist_square, length_square);
        smooth_point.DistanceSquareTo(smooth_segment.start()) > max_dist) {
      continue;
    }
    const auto dist_square = smooth_segment.DistanceSquareTo(smooth_point);
    if (std::fabs(dist_square - *min_dist_square) < 1E-6) {
      if (!is_inserted) {
        segments_with_min_dist->push_back(smooth_segment);
        is_inserted = true;
      }
    } else if (dist_square < *min_dist_square) {
      segments_with_min_dist->clear();
      segments_with_min_dist->push_back(smooth_segment);
      is_inserted = true;
      *min_dist_square = dist_square;
    }
  }
}

double ComputeDistanceToSegments(const Vec2d &point,
                                 const std::vector<st::Segment2d> &segments) {
  CHECK_GT(segments.size(), 0);

  const double dist_to_curb = segments[0].DistanceTo(point);

  if (segments.size() == 2) {
    const auto reverse = segments[1].end() == segments[0].start();
    const auto &seg_0 = reverse ? segments[1] : segments[0];
    const auto &seg_1 = reverse ? segments[0] : segments[1];

    if (seg_0.DistanceTo(seg_1) < 1E-3 &&
        std::fabs(seg_0.unit_direction().CrossProd(seg_1.unit_direction())) <
            1E-2 &&
        seg_0.unit_direction().Dot(seg_1.unit_direction()) < 0) {
      return dist_to_curb;
    }

    if (seg_0.end() == seg_1.start()) {
      const auto counterclockwise = seg_0.ProductOntoUnit(seg_1.end()) >= 0.0;
      const auto sign = counterclockwise
                            ? (seg_0.ProductOntoUnit(point) >= 0.0 &&
                               seg_1.ProductOntoUnit(point) >= 0.0)
                            : (seg_0.ProductOntoUnit(point) > 0.0 ||
                               seg_1.ProductOntoUnit(point) > 0.0);
      return sign ? -dist_to_curb : dist_to_curb;
    }

    return seg_0.ProductOntoUnit(point) >= 0.0 ? -dist_to_curb : dist_to_curb;
  }

  return segments[0].ProductOntoUnit(point) >= 0.0 ? -dist_to_curb
                                                   : dist_to_curb;
}

}  // namespace

void Map::InitAABoxKDTree2d() {
  // _ARG1("Map::InitAABoxKDTree2d", "level",
  //                    absl::StrFormat("%d", level_id));

  feature_aabox_tree_.reserve(kGeoObjectTypeToFeatureType.size() / 2);

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_LANE] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            lane_map(),
            [](const std::pair<uint64_t, ad_byd::planning::LanePtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_LANEBOUNDARY] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // (
        //  "Map::InitAABoxKDTree2d::LANEBOUNDARY");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            lane_boundary_map(),
            [](const std::pair<uint64_t, ad_byd::planning::LaneBoundaryPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_ROADBOUNDARY] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // (
        //  "Map::InitAABoxKDTree2d::LANEBOUNDARY");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            road_boundary_map(),
            [](const std::pair<uint64_t, ad_byd::planning::RoadBoundaryPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_JUNCTION] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // (
        //  "Map::InitAABoxKDTree2d::INTERSECTION");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            junction_map(),
            [](const std::pair<uint64_t, ad_byd::planning::JunctionPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_CROSSWALK] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // ("Map::InitAABoxKDTree2d::CROSSWALK");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            crosswalk_map(),
            [](const std::pair<uint64_t, ad_byd::planning::CrosswalkPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_SPEEDBUMP] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // ("Map::InitAABoxKDTree2d::SPEEDBUMP");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            speed_bump_map(),
            [](const std::pair<uint64_t, ad_byd::planning::SpeedBumpPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_STOPLINE] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // ("Map::InitAABoxKDTree2d::STOPSIGN");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            stop_line_map(),
            [](const std::pair<uint64_t, ad_byd::planning::StopLinePtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });

  feature_aabox_tree_[ad_byd::planning::FeatureType::FEATURETYPE_CLEARAREA] =
      st::ScheduleFuture(&thread_pool_, [&]() {
        // (
        //  "Map::InitAABoxKDTree2d::PARKING_AREA");
        std::shared_ptr<AABoxTree> tree = std::make_shared<AABoxTree>();
        BuildAABoxNodes(
            clear_area_map(),
            [](const std::pair<uint64_t, ad_byd::planning::ClearAreaPtr> &) {
              return false;
            },
            &(tree->aabox_nodes), &(tree->aabox_tree));
        return tree;
      });
}

absl::StatusOr<st::mapping::v2::Segment> Map::FindNearestLaneSegment(
    double lon, double lat) const {
  return FindNearestObjectSegment<AABoxNode, ad_byd::planning::LanePtr>(
      feature_aabox_tree_, {lon, lat});
}

absl::StatusOr<st::mapping::v2::Segment> Map::FindNearestRoadBoundarySegment(
    double lon, double lat) const {
  return FindNearestObjectSegment<AABoxNode, ad_byd::planning::RoadBoundaryPtr>(
      feature_aabox_tree_, {lon, lat});
  ;
}

absl::StatusOr<st::mapping::v2::Segment> Map::FindNearestLaneBoundarySegment(
    double lon, double lat) const {
  return FindNearestObjectSegment<AABoxNode, ad_byd::planning::LaneBoundaryPtr>(
      feature_aabox_tree_, {lon, lat});
}

ad_byd::planning::LaneConstPtr Map::FindNearestLane(double lon,
                                                    double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::LaneConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetLaneById, this,
                std::placeholders::_1));
}

ad_byd::planning::LaneBoundaryConstPtr Map::FindNearestLaneBoundary(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::LaneBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetLaneBoundaryById, this,
                std::placeholders::_1));
}

ad_byd::planning::RoadBoundaryConstPtr Map::FindNearestRoadBoundary(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::RoadBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetRoadBoundaryById, this,
                std::placeholders::_1));
}
ad_byd::planning::StopLineConstPtr Map::FindNearestStopLine(double lon,
                                                            double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::StopLineConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetStopLineById, this,
                std::placeholders::_1));
}
ad_byd::planning::JunctionConstPtr Map::FindNearestJunction(double lon,
                                                            double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::JunctionConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetJunctionById, this,
                std::placeholders::_1));
}
ad_byd::planning::CrosswalkConstPtr Map::FindNearestCrosswalk(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::CrosswalkConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetCrosswalkById, this,
                std::placeholders::_1));
}
ad_byd::planning::SpeedBumpConstPtr Map::FindNearestSpeedBump(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::SpeedBumpConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetSpeedBumpById, this,
                std::placeholders::_1));
}
ad_byd::planning::ClearAreaConstPtr Map::FindNearestClearArea(
    double lon, double lat) const {
  return FindNearestObject<AABoxNode, ad_byd::planning::ClearAreaConstPtr>(
      feature_aabox_tree_, {lon, lat},
      std::bind(&ad_byd::planning::Map::GetClearAreaById, this,
                std::placeholders::_1));
}

std::vector<st::mapping::v2::Segment> Map::FindLaneSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_byd::planning::LanePtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}

std::vector<st::mapping::v2::Segment> Map::FindLaneBoundarySegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode,
                                    ad_byd::planning::LaneBoundaryPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<st::mapping::v2::Segment> Map::FindRoadBoundarySegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode,
                                    ad_byd::planning::RoadBoundaryPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<st::mapping::v2::Segment> Map::FindStopLineSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_byd::planning::StopLinePtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<st::mapping::v2::Segment> Map::FindJunctionSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_byd::planning::JunctionPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<st::mapping::v2::Segment> Map::FindCrosswalkSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_byd::planning::CrosswalkPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<st::mapping::v2::Segment> Map::FindSpeedBumpSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_byd::planning::SpeedBumpPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}
std::vector<st::mapping::v2::Segment> Map::FindClearAreaSegmentsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectSegmentsInRadius<AABoxNode, ad_byd::planning::ClearAreaPtr>(
      feature_aabox_tree_, {lon, lat}, radius);
}

std::vector<ad_byd::planning::LaneConstPtr> Map::FindLanesInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::LaneConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetLaneById, this,
                std::placeholders::_1));
}

std::vector<ad_byd::planning::LaneBoundaryConstPtr>
Map::FindLaneBoundariesInRadius(double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::LaneBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetLaneBoundaryById, this,
                std::placeholders::_1));
}
std::vector<ad_byd::planning::RoadBoundaryConstPtr>
Map::FindRoadBoundariesInRadius(double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::RoadBoundaryConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetRoadBoundaryById, this,
                std::placeholders::_1));
}
std::vector<ad_byd::planning::StopLineConstPtr> Map::FindStopLinesInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::StopLineConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetStopLineById, this,
                std::placeholders::_1));
}
std::vector<ad_byd::planning::JunctionConstPtr> Map::FindJunctionsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::JunctionConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetJunctionById, this,
                std::placeholders::_1));
}
std::vector<ad_byd::planning::CrosswalkConstPtr> Map::FindCrosswalksInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::CrosswalkConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetCrosswalkById, this,
                std::placeholders::_1));
}
std::vector<ad_byd::planning::SpeedBumpConstPtr> Map::FindSpeedBumpsInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::SpeedBumpConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetSpeedBumpById, this,
                std::placeholders::_1));
}
std::vector<ad_byd::planning::ClearAreaConstPtr> Map::FindClearAreasInRadius(
    double lon, double lat, double radius) const {
  return FindObjectsInRadius<AABoxNode, ad_byd::planning::ClearAreaConstPtr>(
      feature_aabox_tree_, {lon, lat}, radius,
      std::bind(&ad_byd::planning::Map::GetClearAreaById, this,
                std::placeholders::_1));
}

void Map::FindAllLaneSequence(
    std::vector<LaneConstPtr> &lanes,
    std::vector<std::vector<LaneConstPtr>> &lane_sequences) const {
  if (lanes.empty()) {
    return;
  }
  auto next_lanes = GetNextLanes(lanes.back());
  if (next_lanes.empty()) {
    lane_sequences.emplace_back(lanes);
  } else {
    for (auto &next : next_lanes) {
      bool flag = false;
      for (auto &exist_lane : lanes) {
        if (next->id() == exist_lane->id() ||
            (!(next->section_id() == 0) &&
             next->section_id() == exist_lane->section_id())) {
          flag = true;
        }
      }
      if (flag) {
        // TODO(qiao)
        // || next->id().find("nextvr") != next->id().npos) {
        lane_sequences.emplace_back(lanes);
        continue;
      }
      lanes.emplace_back(next);
      FindAllLaneSequence(lanes, lane_sequences);
      lanes.pop_back();
    }
  }
}

void Map::FindAllLaneSequence(
    std::vector<LaneConstPtr> &lanes, const double &length_limit,
    std::vector<std::vector<LaneConstPtr>> &lane_sequences) const {
  if (lanes.empty()) {
    return;
  }
  auto next_lanes_ = GetNextLanes(lanes.back());
  std::vector<LaneConstPtr> next_lanes;
  for (const auto &lane : next_lanes_) {
    if (lane->is_navigation()) {
      next_lanes.emplace_back(lane);
    }
  }
  if (next_lanes.empty()) {
    next_lanes = next_lanes_;
  }
  if (next_lanes.empty()) {
    lane_sequences.emplace_back(lanes);
  } else {
    for (auto &next : next_lanes) {
      double total_length = Constants::ZERO;
      bool flag = false;
      total_length += next->topo_length();
      if (total_length >= length_limit) {
        flag = true;
      }
      if (flag) {
        lanes.emplace_back(next);
        lane_sequences.emplace_back(lanes);
        lanes.pop_back();
        continue;
      }
      lanes.emplace_back(next);
      FindAllLaneSequence(lanes, length_limit - total_length, lane_sequences);
      lanes.pop_back();
    }
  }
}

void Map::FindAllLaneSequence(
    std::vector<LaneConstPtr> &lanes, const double &length_limit,
    std::vector<std::vector<LaneConstPtr>> &lane_sequences,
    const bool &arrive_length_limit) const {
  if (lanes.empty()) {
    return;
  }
  std::vector<LaneConstPtr> all_next_lanes;
  if (arrive_length_limit) {
    auto lane = GetCityOptimalNextLane(lanes.back(), true);
    if (lane != nullptr) all_next_lanes.emplace_back(lane);
  } else {
    all_next_lanes = GetNextLanes(lanes.back());
  }
  std::vector<LaneConstPtr> next_lanes;
  for (const auto &lane : all_next_lanes) {
    if (lane->is_navigation()) {
      next_lanes.emplace_back(lane);
    }
  }
  if (next_lanes.empty()) {
    next_lanes = all_next_lanes;
  }
  if (next_lanes.empty()) {
    lane_sequences.emplace_back(lanes);
  } else {
    for (auto &next : next_lanes) {
      bool circle_flag = false;
      for (auto &exist_lane : lanes) {
        if (next->id() == exist_lane->id() ||
            (!(next->section_id() == 0) &&
             next->section_id() == exist_lane->section_id())) {
          circle_flag = true;
          break;
        }
      }
      if (circle_flag) {
        lane_sequences.emplace_back(lanes);
        continue;
      }
      double total_length = Constants::ZERO;
      bool flag = false;
      total_length += next->topo_length();
      if (total_length >= length_limit) {
        flag = true;
      }
      lanes.emplace_back(next);
      FindAllLaneSequence(lanes, length_limit - total_length, lane_sequences,
                          flag);
      lanes.pop_back();
    }
  }
}

void Map::FindAllLaneSequenceBfs(
    std::vector<LaneConstPtr> &lanes, const double &length_limit,
    std::vector<std::vector<LaneConstPtr>> &lane_sequences) const {
  if (lanes.empty()) {
    return;
  }
  const int64_t kTotalSize = 20;
  std::queue<std::pair<std::vector<LaneConstPtr>, double>> q;
  q.push({lanes, length_limit});

  while (!q.empty()) {
    auto [current_lanes, remaining_length] = q.front();
    q.pop();

    const auto &last_lane = current_lanes.back();
    std::vector<LaneConstPtr> all_next_lanes;
    bool flag = remaining_length <= Constants::ZERO;
    if (flag || q.size() > kTotalSize || !last_lane->is_navigation()) {
      auto lane = GetCityOptimalNextLane(last_lane, true);
      if (lane != nullptr) all_next_lanes.emplace_back(lane);
    } else {
      all_next_lanes = GetNextLanes(last_lane);
    }
    std::vector<LaneConstPtr> next_lanes;
    for (const auto &lane : all_next_lanes) {
      if (lane->is_navigation()) {
        next_lanes.emplace_back(lane);
      }
    }
    if (next_lanes.empty()) {
      next_lanes = all_next_lanes;
    }

    if (next_lanes.empty()) {
      lane_sequences.emplace_back(current_lanes);
    } else {
      for (const auto &next : next_lanes) {
        bool circle_flag = false;
        for (const auto &exist_lane : current_lanes) {
          if (next->id() == exist_lane->id() ||
              (!(next->section_id() == 0) &&
               next->section_id() == exist_lane->section_id())) {
            circle_flag = true;
            break;
          }
        }
        if (circle_flag) {
          lane_sequences.emplace_back(current_lanes);
          continue;
        }

        double total_length = remaining_length - next->topo_length();
        std::vector<LaneConstPtr> new_path = current_lanes;
        new_path.emplace_back(next);
        q.push({new_path, total_length});
      }
    }
  }
}

void Map::GetAllLaneSequences(
    const LaneConstPtr &lane,
    std::vector<std::vector<LaneConstPtr>> &sequences) const {
  if (!lane) return;
  std::vector<LaneConstPtr> lane_ptr_vec;
  std::vector<std::vector<LaneConstPtr>> candidate_sequences;
  lane_ptr_vec.emplace_back(lane);
  FindAllLaneSequence(lane_ptr_vec, candidate_sequences);
  for (auto &candidate_sequence : candidate_sequences) {
    sequences.emplace_back(std::move(candidate_sequence));
  }
}

void Map::GetAllLaneSequences(
    const LaneConstPtr &lane, const double &length,
    std::vector<std::vector<LaneConstPtr>> &sequences) const {
  if (!lane) return;
  std::vector<LaneConstPtr> lanes;
  std::vector<std::vector<LaneConstPtr>> candidate_sequences;
  lanes.emplace_back(lane);
  FindAllLaneSequence(lanes, length, candidate_sequences);
  for (auto &candidate_sequence : candidate_sequences) {
    sequences.emplace_back(std::move(candidate_sequence));
  }
}

void Map::GetAllLaneSequences(const LaneConstPtr &lane, const double &length,
                              std::vector<std::vector<LaneConstPtr>> &sequences,
                              const bool &arrive_length_limit) const {
  if (!lane) return;
  std::vector<LaneConstPtr> lanes;
  std::vector<std::vector<LaneConstPtr>> candidate_sequences;
  lanes.emplace_back(lane);
  // FindAllLaneSequence(lanes, length, candidate_sequences,
  // arrive_length_limit);
  FindAllLaneSequenceBfs(lanes, length, candidate_sequences);
  for (auto &candidate_sequence : candidate_sequences) {
    sequences.emplace_back(std::move(candidate_sequence));
  }
}

CompositeTurnType Map::CheckCompositeLane(const LanePtr &lane) const {
  if (!lane || lane->next_lane_ids().size() < 2) {
    return CompositeTurnType::NORMAL_TURN;
  }
  bool left_turn_check = false;
  bool right_turn_check = false;
  bool no_turn_check = false;
  for (const auto &lane_id : lane->next_lane_ids()) {
    const auto &lane = GetLaneById(lane_id);
    if (!lane || lane->junction_id() == 0) {
      continue;
    }
    switch (lane->turn_type()) {
      case LEFT_TURN:
      case U_TURN:
        left_turn_check = true;
        break;
      case RIGHT_TURN:
        right_turn_check = true;
        break;
      case NO_TURN:
        no_turn_check = true;
        break;
      default:
        break;
    }
  }
  if (left_turn_check && no_turn_check && right_turn_check) {
    return CompositeTurnType::LEFT_STRAIGHT_RIGHT;
  }
  if (left_turn_check && no_turn_check) {
    return CompositeTurnType::LEFT_STRAIGHT;
  }
  if (no_turn_check && right_turn_check) {
    return CompositeTurnType::STRAIGHT_RIGHT;
  }
  if (left_turn_check && right_turn_check) {
    return CompositeTurnType::LEFT_RIGHT;
  }
  return CompositeTurnType::NORMAL_TURN;
}

bool Map::CheckIfValidCommonLane(const LaneConstPtr &check_lane) const {
  if (!check_lane) {
    return false;
  }
  if (check_lane->type() != LANE_VIRTUAL_COMMON) {
    return false;
  }
  if (check_lane->turn_type() != TurnType::NO_TURN) {
    return false;
  }
  if (check_lane->pre_lane_ids().empty()) {
    return false;
  }
  LaneConstPtr pre_lane = GetLaneById(check_lane->pre_lane_ids().front());
  if (!pre_lane) {
    return false;
  }
  for (const auto &suc_lane_id : pre_lane->next_lane_ids()) {
    LaneConstPtr suc_lane = GetLaneById(suc_lane_id);
    if (suc_lane && suc_lane->type() == LANE_VIRTUAL_JUNCTION) {
      return false;
    }
  }
  return true;
}

void Map::UpdateNaviPriorityLanes(const LaneConstPtr &ego_lane) {
  if (!ego_lane || !route_) {
    // LOG_ERROR << "[Map] cannot update optimal lanes";
    return;
  }
  // update section_id_before_toll
  auto route_info = route_->mutable_route_info();
  double dist_to_toll = v2_info().dist_to_toll;
  double cur_lane_dist_to_toll = 0.0;
  uint64_t ego_section_id = route_info->navi_start.section_id;
  double ego_s_offset = route_info->navi_start.s_offset;
  bool loc_in_ego_section = false;
  int section_id_before_toll = route_section_seq_.size() - 1;
  for (int i = 0; i < route_section_seq_.size(); i++) {
    const auto &section_id = route_section_seq_.at(i);
    auto section_ptr = GetSectionById(section_id);
    if (!section_ptr || section_ptr->lanes().empty()) {
      continue;
    }
    if (ego_section_id == section_id) {
      loc_in_ego_section = true;
      cur_lane_dist_to_toll -= ego_s_offset;
    }
    if (!loc_in_ego_section) {
      continue;
    }
    cur_lane_dist_to_toll += section_ptr->topo_length();
    if (cur_lane_dist_to_toll > dist_to_toll) {
      section_id_before_toll = i;
      break;
    }
  }

  // generate optimal lane collection
  std::vector<std::vector<LANE>> lane_sequences_candidates;
  bool find_last_lane_info = false;
  std::set<uint64_t> empty_pre_lanes;
  for (int sec_idx = section_id_before_toll; sec_idx >= 0; sec_idx--) {
    auto section_ptr = GetSectionById(route_section_seq_.at(sec_idx));
    for (const auto &lane_id : section_ptr->lanes()) {
      const auto &final_lane = GetLaneById(lane_id);
      if (!final_lane) continue;
      find_last_lane_info = true;
      std::vector<LANE> lane_sequence;
      lane_sequence.emplace_back(LANE(final_lane));
      FindReverseLaneSequence(lane_sequence, lane_sequences_candidates,
                              ego_lane->section_id(), empty_pre_lanes);
    }
    if (find_last_lane_info) break;
  }
  if (lane_sequences_candidates.empty()) {
    std::vector<std::string> route_debug_strings;
    route_debug_strings.emplace_back("Cannot find optimal lanes!");
    std::ostringstream route_stream;
    for (const auto &id : empty_pre_lanes) {
      route_stream << id << ", ";
    }
    route_stream << "has no pre_lanes";
    route_debug_strings.push_back(route_stream.str());
    Log2DDS::LogDataV2("route_debug", route_debug_strings);
    return;
  }
  // SectionInfo section_info;
  // GetSectionByIdFromRoute(ego_lane->section_id(), section_info);
  // const auto it_ego = std::find(section_info.lane_ids.begin(),
  //                               section_info.lane_ids.end(), ego_lane->id());
  // for (auto &lanes_candidate : lane_sequences_candidates) {
  //   const auto it_last =
  //       std::find(section_info.lane_ids.begin(), section_info.lane_ids.end(),
  //                 lanes_candidate.back().lane->id());
  //   lanes_candidate.back().lane_change =
  //       std::abs(std::distance(it_ego, it_last));
  // }
  // Get section from map
  SectionConstPtr section_ptr = GetSectionById(ego_lane->section_id());
  if (section_ptr) {
    const auto it_ego = std::find(section_ptr->lanes().begin(),
                                  section_ptr->lanes().end(), ego_lane->id());
    for (auto &lanes_candidate : lane_sequences_candidates) {
      const auto it_last =
          std::find(section_ptr->lanes().begin(), section_ptr->lanes().end(),
                    lanes_candidate.back().lane->id());
      lanes_candidate.back().lane_change =
          std::abs(std::distance(it_ego, it_last));
    }
  }

  // update cost
  std::map<double, std::vector<LANE>> cost_map;
  std::vector<std::string> route_debug_strings;
  split_merge_shortdis_ids_.clear();
  for (const auto &lanes_candidate : lane_sequences_candidates) {
    double cost = CalculateSequenceCost(lanes_candidate);
    cost_map[cost] = lanes_candidate;

    // std::ostringstream stream;
    // stream << "Route candidate:";
    // for (const auto &lane : lanes_candidate) {
    //   stream << lane.lane->id() << " , ";
    // }
    // stream << "..cost:" << cost;
    // route_debug_strings.push_back(stream.str());
    // LOG_ERROR << stream.str();
  }
  // update optimal lane on section
  for (const auto &lane : cost_map.begin()->second) {
    UpdateNaviPriorityLaneId(lane.lane);
  }
  std::ostringstream stream;
  stream << "Route best:";
  for (const auto &lane : cost_map.begin()->second) {
    stream << lane.lane->id() << " , ";
  }
  route_debug_strings.push_back(stream.str());
  Log2DDS::LogDataV2("navi-priority-cost", route_debug_strings);
  // LOG_ERROR << "*********Route: " << stream.str();
}
void Map::FindReverseLaneSequence(
    std::vector<LANE> &lane_sequence,
    std::vector<std::vector<LANE>> &lane_sequences,
    const uint64_t stop_section_id, std::set<uint64_t> &empty_pre_lanes) {
  if (lane_sequence.empty() || !lane_sequence.back().lane->is_navigation()) {
    return;
  }
  auto &current = lane_sequence.back();
  if (current.lane->section_id() == stop_section_id) {
    lane_sequences.emplace_back(lane_sequence);
  } else {
    std::vector<LaneConstPtr> pre_lanes;
    current.lane_change = GetPreNaviLanes(current.lane, pre_lanes);
    constexpr size_t kMaxPreLanesNum = 100;
    if (pre_lanes.empty() && empty_pre_lanes.size() < kMaxPreLanesNum) {
      empty_pre_lanes.insert(current.lane->id());
    }
    for (auto &pre : pre_lanes) {
      auto equal_to = [&](const LANE &lane) {
        return lane.lane->id() == pre->id();
      };
      auto it =
          std::find_if(lane_sequence.begin(), lane_sequence.end(), equal_to);
      if (it != lane_sequence.end()) {
        return;
      }
      // occupied check
      // if (pre && pre->is_occupied()) {
      //   LaneConstPtr raw_pre = pre;
      //   bool pre_check = false;
      //   pre = map_->GetLaneById(pre->left_lane_id());
      //   while (pre && pre->is_occupied()) {
      //     pre = map_->GetLaneById(pre->left_lane_id());
      //   }
      //   if (pre && !pre->is_occupied()) {
      //     pre_check = true;
      //   } else {
      //     pre = raw_pre;
      //   }
      // }

      lane_sequence.emplace_back(LANE(pre));
      FindReverseLaneSequence(lane_sequence, lane_sequences, stop_section_id,
                              empty_pre_lanes);
      lane_sequence.pop_back();
    }
  }
}
size_t Map::GetPreNaviLanes(const LaneConstPtr &current_lane,
                            std::vector<LaneConstPtr> &pre_lanes) {
  // version: Get section from map
  pre_lanes.clear();
  if (!current_lane) return 0;
  SectionConstPtr section_ptr = GetSectionById(current_lane->section_id());
  if (!section_ptr) return 0;
  auto it = std::find(section_ptr->lanes().begin(), section_ptr->lanes().end(),
                      current_lane->id());
  if (it == section_ptr->lanes().end()) {
    return 0;
  }
  auto idx = std::distance(section_ptr->lanes().begin(), it);
  for (auto i = idx; i >= 0; --i) {
    GetPrecedeLanes(GetLaneById(section_ptr->lanes().at(i)), &pre_lanes, true);
    if (!pre_lanes.empty()) {
      return idx - i;
    }
  }
  for (auto i = idx + 1; i < static_cast<int>(section_ptr->lanes().size());
       ++i) {
    GetPrecedeLanes(GetLaneById(section_ptr->lanes().at(i)), &pre_lanes, true);
    if (!pre_lanes.empty()) {
      return i - idx;
    }
  }
  pre_lanes.clear();
  return 0;
}

double Map::CalculateSequenceCost(const std::vector<LANE> &lanes) const {
  double total_cost = 0.0;
  if (lanes.empty()) {
    return total_cost;
  }
  const double lane_change_weight = 10.0;
  const double topology_weight = 1.0;
  const double topology_hw_merge_weight = 1.0;
  const double topology_hw_split_weight = 2.0;
  constexpr const double SThreshold_split2mergedist = 1000.0;
  constexpr const double SThreshold_find_split2merge = 1500.0;
  double lane_accum_s = 0.0;
  bool first_split_isfound = false;
  bool laneseq_condition_met = true;
  std::string first_split_laneid;
  std::vector<std::pair<uint64_t, double>> splite_merge_infos_seq;
  for (auto it = lanes.rbegin(); it != lanes.rend(); ++it) {
    const LANE &lane = *it;
    // std::cout<<lane.lane->id()<< ",";
    // const auto &section_ptr =
    // map_->GetSectionByIdFromRoute(lane.lane->section_id()); bool
    // in_highway_scene =
    //     (section_ptr &&
    //      (section_ptr->road_class() == Section::RoadClass::CITY_EXPRESS ||
    //       section_ptr->road_class() == Section::RoadClass::HIGHWAY)) ||
    //     (lane.lane->type() == LaneType::LANE_RAMP);
    bool in_highway_scene = is_on_highway();
    // caculate lane_accum_s from next lane
    // split point found
    auto next_it = std::next(it);
    if (next_it != lanes.rend()) {
      lane_accum_s += next_it->lane->topo_length();
      const auto &target_id = next_it->lane->id();
      auto nextisexsit = std::find(lane.lane->next_lane_ids().begin(),
                                   lane.lane->next_lane_ids().end(), target_id);
      if (nextisexsit == lane.lane->next_lane_ids().end()) {
        laneseq_condition_met = false;
      }
    }
    if (next_it != lanes.rend() && first_split_isfound &&
        laneseq_condition_met) {
      if (!splite_merge_infos_seq.empty()) {
        for (auto &mergedis : splite_merge_infos_seq) {
          mergedis.second += next_it->lane->topo_length();
        }
      }
    }
    if (lane.lane->next_lane_ids().size() >= 2 &&
        lane_accum_s < SThreshold_find_split2merge && laneseq_condition_met) {
      first_split_isfound = true;
      if (next_it != lanes.rend()) {
        splite_merge_infos_seq.emplace_back(next_it->lane->id(),
                                            next_it->lane->topo_length());
      }
    }
    // topology cost
    if (lane.lane->split_topology() == TOPOLOGY_SPLIT_LEFT ||
        lane.lane->split_topology() == TOPOLOGY_SPLIT_RIGHT ||
        lane.lane->split_topology() == TOPOLOGY_SPLIT_UNKNOWN) {
      if (in_highway_scene) {
        total_cost += topology_hw_split_weight * 1.0;
      } else {
        total_cost += topology_weight * 1.0;
      }
    }
    if (lane.lane->merge_topology() == TOPOLOGY_MERGE_LEFT ||
        lane.lane->merge_topology() == TOPOLOGY_MERGE_RIGHT ||
        lane.lane->merge_topology() == TOPOLOGY_MERGE_UNKNOWN) {
      if (in_highway_scene) {
        total_cost += topology_hw_merge_weight * 1.0;
      } else {
        total_cost += topology_weight * 1.0;
      }
    }
    // split to merge dis < 1000m, memeory the spilited lane id
    if (lane.lane->merge_topology() == TOPOLOGY_MERGE_LEFT ||
        lane.lane->merge_topology() == TOPOLOGY_MERGE_RIGHT) {
      if (first_split_isfound && in_highway_scene &&
          !splite_merge_infos_seq.empty() && laneseq_condition_met) {
        for (auto splite_merge_infos : splite_merge_infos_seq) {
          if (splite_merge_infos.second < SThreshold_split2mergedist) {
            split_merge_shortdis_ids_.insert(splite_merge_infos.first);
          }
        }
      }
      first_split_isfound = false;
      splite_merge_infos_seq.clear();
    }
    // if (in_highway_scene && lane.lane->pre_lane_ids().size() == 1) {
    //   LaneConstPtr pre_lane_ptr =
    //       map_->GetLaneById(lane.lane->pre_lane_ids().front());
    //   std::string other_lane_id = "";
    //   if (pre_lane_ptr && pre_lane_ptr->next_lane_ids().size() == 2) {
    //     other_lane_id = lane.lane->id() ==
    //     pre_lane_ptr->next_lane_ids().front()
    //                         ? pre_lane_ptr->next_lane_ids().back()
    //                         : pre_lane_ptr->next_lane_ids().front();
    //     if (!other_lane_id.empty()) {
    //       if ((pre_lane_ptr->split_topology() == TOPOLOGY_SPLIT_LEFT &&
    //            lane.lane->right_lane_id() == other_lane_id) ||
    //           (pre_lane_ptr->split_topology() == TOPOLOGY_SPLIT_RIGHT &&
    //            lane.lane->left_lane_id() == other_lane_id)) {
    //         total_cost += topology_hw_split_weight * 1.0;
    //       }
    //     }
    //   }
    // }
    // lane change cost
    total_cost += lane_change_weight * double(lane.lane_change);
    // split to merge dis > 1000m, no care, reset
    if (lane_accum_s > SThreshold_find_split2merge) {
      first_split_isfound = false;
      splite_merge_infos_seq.clear();
    }
    // merge, reset
    if (lane.lane->pre_lane_ids().size() > 1) {
      first_split_isfound = false;
      splite_merge_infos_seq.clear();
    }
  }
  return total_cost;
}

void Map::UpdateNaviPriorityLaneId(const LaneConstPtr &lane) {
  // version: get section from map
  if (!lane) return;
  auto section_ptr = GetMutableSectionById(lane->section_id());
  if (!section_ptr) return;
  section_ptr->set_navi_priority_lane_id(lane->id());
  // LOG(ERROR) << " lane->id() " << lane->id() << " section id: " <<
  // lane->section_id();
}
LaneConstPtr Map::GetNaviPriorityLane(const uint64_t section_id) const {
  // get section;
  SectionConstPtr section_ptr = GetSectionById(section_id);
  if (!section_ptr) return nullptr;
  // get lane;
  uint16_t priority_lane_id = section_ptr->get_navi_priority_lane_id();
  return GetLaneById(priority_lane_id);
}

int Map::GetPriorityLaneRelation(const LaneConstPtr &lane) const {
  // version: Get section from map
  int d = 0;
  if (!lane) return d;
  SectionConstPtr section_ptr = GetSectionById(lane->section_id());
  if (!section_ptr) return d;
  uint64_t navi_priority_lane_id = section_ptr->get_navi_priority_lane_id();
  LaneConstPtr next_lane = lane;
  int count = 50;
  while (navi_priority_lane_id == 0 && next_lane->is_navigation()) {
    count--;
    if (next_lane->next_lane_ids().empty() || count < 0) {
      return d;
    }
    next_lane = GetLaneById(next_lane->next_lane_ids().at(0));
    if (!next_lane) {
      return d;
    }
    SectionConstPtr section_ptr_tmp = GetSectionById(next_lane->section_id());
    if (!section_ptr_tmp) {
      return d;
    }
    navi_priority_lane_id = section_ptr_tmp->get_navi_priority_lane_id();
    section_ptr = section_ptr_tmp;
  }

  // TODO: if this is a extend section ...
  auto it0 = std::find(section_ptr->lanes().begin(), section_ptr->lanes().end(),
                       navi_priority_lane_id);
  // LOG(ERROR) << "navi_priority_lane_id: " << navi_priority_lane_id <<
  // " belong to section: " << section_ptr->id() << " lane id: " << lane->id()
  // << " next lane id: " << next_lane->id();
  auto it1 = std::find(section_ptr->lanes().begin(), section_ptr->lanes().end(),
                       next_lane->id());
  if (it0 != section_ptr->lanes().end() && it1 != section_ptr->lanes().end()) {
    d = static_cast<int>(std::distance(it0, it1));
  }
  return d;
}
}  // namespace planning
}  // namespace ad_byd

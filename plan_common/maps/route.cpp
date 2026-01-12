
#include "plan_common/maps/route.h"

#include <limits>

#include "plan_common/log_data.h"
#include "plan_common/maps/map.h"

namespace ad_byd {
namespace planning {

Route::Route(const RouteInfo &route_info, const Map *map) : map_(map) {
  route_info_ = route_info;
  for (auto &path_extend : route_info_.extend_sections_vec) {
    int64_t from_section_idx = path_extend.from_section_idx;
    uint64_t to_section_idx = path_extend.to_section_idx;
    // LOG(ERROR) << " from: " << from_section_idx << " to: " << to_section_idx;
    bool is_find_start = false;
    for (auto &extend_section : path_extend.extend_sections) {
      // LOG(ERROR) << "before extend_section id: " << extend_section.id;
      if (extend_section.id == route_info_.navi_start.section_id) {
        is_find_start = true;
      }
    }
    if (is_find_start) {
      std::vector<SectionInfo> exchange_sections_tmp(
          route_info_.sections.begin() + from_section_idx + 1,
          route_info_.sections.begin() + to_section_idx);
      route_info_.sections.erase(
          route_info_.sections.begin() + from_section_idx + 1,
          route_info_.sections.begin() + to_section_idx);
      route_info_.sections.insert(
          route_info_.sections.begin() + from_section_idx + 1,
          path_extend.extend_sections.begin(),
          path_extend.extend_sections.end());
      path_extend.extend_sections = exchange_sections_tmp;

      break;
    }
  }
  // for (auto path_extend : route_info_.extend_sections_vec) {
  //   for (auto extend_section : path_extend.extend_sections) {
  //     LOG(ERROR) << "after extend_section id: " << extend_section.id;
  //   }
  // }
  // for (auto section : route_info_.sections) {
  //   LOG(ERROR) << "after section id: " << section.id;
  // }
}

bool Route::CanDriveToRouteEnd(const LaneSequencePtr &lane_seq,
                               const LaneConstPtr &start_lane) const {
  if (route_info_.sections.empty()) {
    return false;
  }
  if (!lane_seq || lane_seq->lanes().empty() || !start_lane) {
    return false;
  }
  const auto &end_section = route_info_.sections.back();
  LaneConstPtr end_lane = nullptr;
  bool find_start = false;
  for (const auto &lane : lane_seq->lanes()) {
    if (lane->id() == start_lane->id()) {
      find_start = true;
    }
    if (find_start) {
      if (!lane->is_navigation()) {
        break;
      }
      end_lane = lane;
    }
  }
  if (end_lane == nullptr) {
    return false;
  }

  for (const auto &lane_id : end_section.lane_ids) {
    if (lane_id == end_lane->id()) {
      return true;
    }
  }
  return false;
}

double Route::GetDistanceToX(const XRoadType &x_road_type, const double &x,
                             const double &y) const {
  double dist_to_x = std::numeric_limits<double>::max();
  if (!map_ || !map_->IsValid()) return dist_to_x;
  const auto &nearest_lane = map_->GetNearestLane(
      Point2d(x, y), 0.0, Constants::MIN_HALF_LANE_WIDTH * 2.5);
  if (!nearest_lane || route_info_.sections.empty() ||
      nearest_lane->section_id() == 0) {
    // LOG_WARN << "[route] cannot get nearest lane or empty!!!";
    return dist_to_x;
  }
  double start_lane_s = 0.0;
  math::Vec2d p(x, y);
  double s, l;
  if (nearest_lane->center_line().GetProjection(p, &s, &l)) {
    start_lane_s += s;
    start_lane_s = nearest_lane->center_line().length() - start_lane_s;
    dist_to_x = 0.0;
  } else {
    return dist_to_x;
  }
  auto equal_to = [&](const SectionInfo &sec) {
    return sec.id == nearest_lane->section_id();
  };
  auto it = std::find_if(route_info_.sections.begin(),
                         route_info_.sections.end(), equal_to);
  while (it != route_info_.sections.end()) {
    // TODO : need to add x type
    if (x_road_type == 1 && it->none_odd_type >= 1) {
      break;
    }
    dist_to_x += it->length;
    it++;
  }
  if (it == route_info_.sections.end())
    dist_to_x = std::numeric_limits<double>::max();
  return dist_to_x;
}

double Route::GetDistanceToNaviEnd() const {
  double dis_to_navi_end = DBL_MAX;
  if (route_info_.navi_end.section_id == 0 ||
      route_info_.navi_start.section_id == 0) {
    return dis_to_navi_end;
  }
  if (!map_ || !map_->IsValid()) return dis_to_navi_end;
  bool find_navi_start = false;
  bool find_navi_end = false;
  for (std::size_t index = 0; index < route_info_.sections.size(); index++) {
    if (route_info_.sections[index].id == 0) break;
    if (route_info_.sections[index].id == route_info_.navi_start.section_id) {
      dis_to_navi_end = 0.0;
      find_navi_start = true;
    }
    if (route_info_.sections[index].id == route_info_.navi_end.section_id) {
      find_navi_end = true;
      break;
    }
    if (find_navi_start) {
      dis_to_navi_end += route_info_.sections[index].length;
    }
  }

  return find_navi_end ? dis_to_navi_end - route_info_.navi_start.s_offset +
                             route_info_.navi_end.s_offset
                       : DBL_MAX;
}

SerialJunctionType Route::NextTwoJunctionOnNavi(
    double &dist_between_two_junction, TurnType &first_junction_turn_type,
    uint64_t &section_id) {
  if (route_info_.sections.empty() || route_info_.navi_start.section_id == 0) {
    return SerialJunctionType::NO_NAVI;
  }
  bool find_start_section = false;
  bool find_first_junction = false;
  bool find_second_junction = false;
  TurnType first_junction = NO_TURN;
  TurnType second_junction = NO_TURN;
  dist_between_two_junction = std::numeric_limits<double>::max();

  for (const auto &section : route_info_.sections) {
    if (section.lane_ids.empty()) {
      LOG_ERROR << "Route::NextTwoJunctionOnNavi->section has no lane!";
      continue;
    }
    if (section.id == route_info_.navi_start.section_id) {
      find_start_section = true;
    }
    if (!find_start_section) {
      continue;
    }

    // Get valid lane in the section
    uint64_t valid_lane_id = 0;
    for (const auto &lane_id : section.lane_ids) {
      LaneConstPtr lane = map_->GetLaneById(lane_id);
      if (lane) {
        valid_lane_id = lane_id;
        break;
      }
    }
    LaneConstPtr lane = map_->GetLaneById(valid_lane_id);
    if (!lane) break;

    if (find_first_junction && !find_second_junction) {
      find_second_junction = !(lane->junction_id() == 0) &&
                             !map_->CheckIfValidCommonLane(lane) &&
                             lane->type() != LANE_LEFT_WAIT;
      if (find_second_junction) {
        second_junction = lane->turn_type();
        break;
      }
    }
    if (find_first_junction) {
      dist_between_two_junction += section.length;
    }
    if (!find_first_junction) {
      find_first_junction = !(lane->junction_id() == 0) &&
                            !map_->CheckIfValidCommonLane(lane) &&
                            lane->type() != LANE_LEFT_WAIT;
      if (find_first_junction) {
        dist_between_two_junction = 0.0;
        first_junction = lane->turn_type();
        first_junction_turn_type = first_junction;
        section_id = section.id;
      }
    }
  }

  if (first_junction == NO_TURN && second_junction == NO_TURN)
    return SerialJunctionType::STRAIGHT_STRAIGHT;

  if (first_junction == NO_TURN && second_junction == LEFT_TURN)
    return SerialJunctionType::STRAIGHT_LEFT;

  if (first_junction == NO_TURN && second_junction == RIGHT_TURN)
    return SerialJunctionType::STRAIGHT_RIGHT;

  if (first_junction == LEFT_TURN && second_junction == NO_TURN)
    return SerialJunctionType::LEFT_STRAIGHT;

  if (first_junction == LEFT_TURN && second_junction == LEFT_TURN)
    return SerialJunctionType::LEFT_LEFT;

  if (first_junction == LEFT_TURN && second_junction == RIGHT_TURN)
    return SerialJunctionType::LEFT_RIGHT;

  if (first_junction == RIGHT_TURN && second_junction == NO_TURN)
    return SerialJunctionType::RIGHT_STRAIGHT;

  if (first_junction == RIGHT_TURN && second_junction == LEFT_TURN)
    return SerialJunctionType::RIGHT_LEFT;

  if (first_junction == RIGHT_TURN && second_junction == RIGHT_TURN)
    return SerialJunctionType::RIGHT_RIGHT;
  return SerialJunctionType::NO_NAVI;
}

double Route::GetDistToJunctionEndOnNavi() const {
  double dist_to_exit_junction = 0.0;
  if (route_info_.sections.empty() || route_info_.navi_start.section_id == 0) {
    return std::numeric_limits<double>::max();
  }
  bool find_start_section = false;
  bool find_first_junction = false;
  for (const auto &section : route_info_.sections) {
    if (section.lane_ids.empty()) {
      continue;
    }
    if (section.id == route_info_.navi_start.section_id) {
      find_start_section = true;
    }
    if (!find_start_section) {
      continue;
    }

    uint64_t valid_lane_id = 0;
    for (const auto &lane_id : section.lane_ids) {
      LaneConstPtr lane = map_->GetLaneById(lane_id);
      if (lane && lane->is_navigation()) {
        valid_lane_id = lane_id;
        break;
      }
    }
    LaneConstPtr lane = map_->GetLaneById(valid_lane_id);
    if (!lane) break;
    if (!find_first_junction) {
      dist_to_exit_junction += section.length;
      find_first_junction = !(lane->junction_id() == 0) &&
                            !map_->CheckIfValidCommonLane(lane) &&
                            lane->type() != LANE_LEFT_WAIT;
      if (find_first_junction) break;
    }
  }
  if (!find_first_junction) return std::numeric_limits<double>::max();
  return dist_to_exit_junction - route_info_.navi_start.s_offset;
}

}  // namespace planning
}  // namespace ad_byd

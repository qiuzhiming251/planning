

#include "plan_common/gflags.h"
#include "plan_common/log.h"
#include "plan_common/ref_line/ref_routing_lane_center.h"
#include "plan_common/math/double.h"

namespace ad_byd {
namespace planning {

RefRoutingLaneCenter::RefRoutingLaneCenter(const Vec2d& pos, MapConstPtr map,
                                           RoutePtr route) {
  Init(pos, map, route);
}

bool RefRoutingLaneCenter::GetProjection(const Vec2d& point, double* const s,
                                         double* const l) const {
  if (center_line_.points().size() < 2) {
    return false;
  }
  center_line_.GetProjection(point, s, l);
  return true;
}

void RefRoutingLaneCenter::Init(const Vec2d& pos, MapConstPtr map,
                                RoutePtr route) {
  if (map == nullptr) {
    LERROR("map is null, fail to build ref routing lane center");
    return;
  }

  LaneConstPtr ego_current_lane = nullptr;
  ego_current_lane =
      map->GetNearestLane(pos, FLAGS_ad_byd_current_lane_search_dist);
  if (ego_current_lane == nullptr) {
    LERROR("adc not on lane, fail to build ref routing lane center");
    return;
  }

  std::unordered_set<uint64_t> all_routing_lane_set;
  if (route != nullptr) {
    for (const auto& section : route->GetRouteInfo().sections) {
      for (const auto& lane_id : section.lane_ids) {
        if (map->GetLaneById(lane_id) != nullptr) {
          all_routing_lane_set.insert(lane_id);
        }
      }
    }
  }

  std::vector<LaneConstPtr> route_lanes{ego_current_lane};
  std::unordered_set<uint64_t> iter_lanes{ego_current_lane->id()};
  bool is_find_loop = false;
  auto iter_lane = ego_current_lane;
  double iter_heading = 0.0;
  bool iter_heading_ok = ego_current_lane->GetHeadingFromS(
      ego_current_lane->topo_length() - 0.1, &iter_heading);
  while ((!iter_lane->valid_next_lane_ids().empty()) && (!is_find_loop)) {
    double min_angle_diff = 1000.0;
    LaneConstPtr min_angle_diff_lane = nullptr;

    for (auto suc_lane_id : iter_lane->valid_next_lane_ids()) {
      auto suc_lane = map->GetLaneById(suc_lane_id);
      if (suc_lane == nullptr) {
        continue;
      }
      if (iter_lanes.find(suc_lane_id) != iter_lanes.end()) {
        is_find_loop = true;
        break;
      } else {
        iter_lanes.insert(suc_lane_id);
      }
      double suc_heading = 0.0;
      bool suc_heading_ok = suc_lane->GetHeadingFromS(0.1, &suc_heading);
      double angle_diff = 0.0;
      if (iter_heading_ok && suc_heading_ok) {
        angle_diff = std::abs(math::AngleDiff(suc_heading, iter_heading));
      } else {
        angle_diff = 0.0;
      }
      if (angle_diff < min_angle_diff) {
        min_angle_diff = angle_diff;
        min_angle_diff_lane = suc_lane;
      }
    }

    if (min_angle_diff_lane != nullptr) {
      iter_lane = min_angle_diff_lane;
      iter_heading_ok = iter_lane->GetHeadingFromS(
          iter_lane->topo_length() - 0.1, &iter_heading);
      if (all_routing_lane_set.find(iter_lane->id()) !=
          all_routing_lane_set.end()) {
        route_lanes.push_back(min_angle_diff_lane);
      }
    }
  }
  double acc_s = 0.0;
  // acc_length_.push_back(acc_s);
  std::vector<Vec2d> routing_lane_curve;
  for (const auto& lane_ptr : route_lanes) {
    const auto& points = lane_ptr->center_line().points();
    double lane_length = lane_ptr->center_line().IsValid()
                             ? lane_ptr->center_line().GetAccuLength().back()
                             : 0;
    acc_s = acc_s + lane_length;
    acc_length_.push_back(acc_s);
    routing_lane_curve.insert(routing_lane_curve.end(), points.begin(),
                              points.end());
  }
  if (!route_lanes.empty()) {
    double begin_heading = 0.0, end_heading = 0.0;
    route_lanes.front()->GetHeadingFromS(0.0, &begin_heading);
    route_lanes.back()->GetHeadingFromS(route_lanes.back()->curve_length(),
                                        &end_heading);
    is_straight_flag_ =
        std::fabs(math::AngleDiff(end_heading, begin_heading)) < m_pi_3_;
  }
  center_line_.InitializePoints(routing_lane_curve);
  ego_position_ = pos;
  center_line_.GetProjection(ego_position_, &adc_s_, &adc_l_);
  route_lanes_ = route_lanes;
}

void RefRoutingLaneCenter::Clear() {
  center_line_.Clear();
  adc_l_ = 0.0;
  adc_s_ = 0.0;
}

double RefRoutingLaneCenter::GetHeadingFromS(const double& s) const {
  double heading = 0;
  center_line_.GetHeadingFromS(s, &heading);
  return heading;
}

bool RefRoutingLaneCenter::XYFromS(const double& s, Vec2d* const pos) const {
  if (center_line_.points().size() < 2) {
    return false;
  }
  center_line_.GetPoint(s, 0, pos);
  return true;
}

bool RefRoutingLaneCenter::GetLanePtrFromS(const double& s,
                                           LaneConstPtr& lane_ptr) const {
  if (route_lanes_.empty()) {
    return false;
  }
  if (s < 0) {
    lane_ptr = route_lanes_.front();
  } else if (s > acc_length_.back()) {
    lane_ptr = route_lanes_.back();
  } else {
    int left = 0;
    int right = static_cast<int>(acc_length_.size());
    while (left < right) {
      int mid = left + (right - left) / 2;
      if (acc_length_[mid] < s) {
        left = mid + 1;
      } else {
        right = mid;
      }
    }
    lane_ptr = route_lanes_[left];
  }
  return true;
}
bool RefRoutingLaneCenter::IsStraight() const { return is_straight_flag_; }
const std::vector<LaneConstPtr>& RefRoutingLaneCenter::route_lane() const {
  return route_lanes_;
}
}  // namespace planning
}  // namespace ad_byd
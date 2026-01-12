
#ifndef AD_BYD_PLANNING_MAP_LANE_H
#define AD_BYD_PLANNING_MAP_LANE_H

#include <memory>
#include <unordered_set>

#include "plan_common/maps/crosswalk.h"
#include "plan_common/maps/lane_boundaries.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/road_boundaries.h"
#include "plan_common/maps/road_boundary.h"
#include "plan_common/math/curve_limits.h"
#include "plan_common/math/line_curve2d.h"
#include "plan_common/type_def.h"

#include <cereal/access.hpp>

namespace ad_byd {
namespace planning {
enum ValidationRule {
  LANE_TO_CURB_DIST = 1,
  LANE_MAX_LENGTH = 2,
  TL_BIND = 3,
  LANE_TOPOLOGICAL_CONNECTION = 4,
  LANE_CURVATURE = 5,
  LANE_DIRECTION = 6,
  CURB_CONNECTION = 7,
  LANE_INTERSECT_IN_INTERSECTION = 8,
  SECTION_VALIDITY = 9,
  TL_HEADING = 10,
  REPEATING_FEATURE = 11
};

struct ValidationRecord {
  ValidationRule rule;
  std::string note;
};

struct LaneInteraction {
  uint64_t other_lane_id = 0;

  enum GeometricConfiguration { CROSS = 1, MERGE = 2 };
  GeometricConfiguration geometric_configuration;

  enum ReactionRule {
    // We have stop sign, the other lane does not.
    STOP = 1,
    // Both lane have stop signs.
    BOTH_STOP = 2,
    // We have lower precedence than the other lane.
    YIELD = 3,
    // We have higher precedence than the other lane.
    PROCEED = 4,
    // When the traffic light is red, we should yield to the other lane.
    YIELD_ON_RED = 6,
    // When the traffic light is green circle, we should yield to the other
    // lane.
    YIELD_ON_GREEN_CIRCLE = 7,
    // Both lanes have the same precedence.
    FFA = 5,  // FFA stands for Free For All
    // The other lane is merging lane, we should yield.
    YIELD_MERGE = 8,
    // We are merging lane, the other lane should yield.
    PROCEED_MERGE = 9
  };
  ReactionRule reaction_rule;
  std::string intersection_id;
  double this_lane_fraction;
  double other_lane_fraction;
  bool keep_it;
  int32_t belonging_levels;
  ValidationRecord skip_records;
};

class Lane {
 public:
  struct SampledWidth {
    double s = 0.0;
    double dist = 0.0;
  };
  Lane(const LaneInfo& lane_info, LaneBoundariesConstPtr left_boundary,
       LaneBoundariesConstPtr right_boundary,
       RoadBoundariesConstPtr left_road_boundary,
       RoadBoundariesConstPtr right_road_boundary);
  ~Lane() = default;

  void SetValidPredLaneIds(const std::vector<uint64_t>& lane_ids) {
    valid_pre_lane_ids_ = lane_ids;
  }
  void SetValidNextLaneIds(const std::vector<uint64_t>& lane_ids) {
    valid_next_lane_ids_ = lane_ids;
  }
  void SetNextTurnTypes(const std::unordered_set<TurnType>& turn_types) {
    next_turn_types_ = turn_types;
  }
  void SetSortedNextLaneIds(const std::vector<uint64_t>& lane_ids) {
    sorted_next_lane_ids_ = lane_ids;
  }
  void SetLaneIndInSection(std::size_t lane_ind_in_section) {
    lane_ind_in_section_ = lane_ind_in_section;
  }

  void SetSplitTopology(SplitTopology split_topology) {
    lane_info_.split_topology = split_topology;
  }
  void SetMergeTopology(MergeTopology merge_topology) {
    lane_info_.merge_topology = merge_topology;
  }

  const uint64_t id() const { return lane_info_.id; }
  const uint64_t section_id() const { return lane_info_.section_id; }
  const uint64_t junction_id() const { return lane_info_.junction_id; }
  LaneType type() const { return lane_info_.type; }
  bool is_navigation() const { return lane_info_.is_navigation; }
  bool is_virtual_navigation() const {
    return lane_info_.is_virtual_navigation;
  }
  bool stop_line() const { return lane_info_.stop_line; }
  double speed_limit() const { return lane_info_.speed_limit; }
  SplitTopology split_topology() const { return lane_info_.split_topology; }
  MergeTopology merge_topology() const { return lane_info_.merge_topology; }
  // const std::vector<double>& points_mse() const {
  //   return lane_info_.points_mse;
  // }
  /// @brief get total length, center line may be shorter than total length
  /// because some points at start or end may be cut off
  double topo_length() const { return lane_info_.length; }
  double curve_length() const { return center_line_.length(); }
  NoneOddType none_odd_type() const { return lane_info_.none_odd_type; }
  TurnType turn_type() const { return lane_info_.turn_type; }
  bool is_exp_traj() const { return lane_info_.is_exp_traj; }
  bool is_split_topo_modify() const { return lane_info_.is_split_topo_modify_; }
  bool is_merge_topo_modify() const { return lane_info_.is_merge_topo_modify_; }
  LightStatus light_status() const { return lane_info_.light_status; }
  TrafficSetReason traffic_set_reason() const {
    return lane_info_.traffic_set_reason;
  }
  bool pre_has_light() const {
    return lane_info_.pre_has_light;
  }
  const uint64_t left_lane_id() const { return lane_info_.left_lane_id; }
  const uint64_t right_lane_id() const { return lane_info_.right_lane_id; }
  const std::vector<uint64_t>& pre_lane_ids() const { return prev_lane_ids_; }
  const std::vector<uint64_t>& next_lane_ids() const {
    return lane_info_.next_lane_ids;
  }
  const std::vector<uint64_t>& valid_pre_lane_ids() const {
    return valid_pre_lane_ids_;
  }
  const std::vector<uint64_t>& valid_next_lane_ids() const {
    return valid_next_lane_ids_;
  }
  const std::unordered_set<TurnType>& next_turn_types() const {
    return next_turn_types_;
  }
  const LaneInfo& lane_info() const { return lane_info_; }
  const LaneBoundariesConstPtr& left_boundary() const { return left_boundary_; }
  const LaneBoundariesConstPtr& right_boundary() const {
    return right_boundary_;
  }
  const RoadBoundariesConstPtr& left_road_boundary() const {
    return left_road_boundary_;
  }
  const RoadBoundariesConstPtr& right_road_boundary() const {
    return right_road_boundary_;
  }
  const std::vector<uint64_t>& sorted_next_lane_ids() const {
    return sorted_next_lane_ids_;
  };
  bool IsValid() const;
  bool IsVirtual() const { return lane_info_.is_virtual; }
  bool IsBoundaryVirtual() const;
  const std::vector<uint64_t>& crosswalks() const {
    return lane_info_.cross_walks;
  }
  std::size_t lane_ind_in_section() const { return lane_ind_in_section_; }
  int32_t lane_operation_type() const { return lane_info_.lane_operation_type; }
  int32_t arrow_type() const { return lane_info_.arrow_type; }
  int32_t light_countdown() const { return lane_info_.light_countdown; }
  uint32_t stopline_angle_flag() const {
    return lane_info_.stopline_angle_flag;
  }
  bool is_acc_adj_lane() const { return lane_info_.is_acc_adj_lane; }
  double merge_start_dis() const { return lane_info_.merge_start_dis; }
  double merge_end_dis() const { return lane_info_.merge_end_dis; }
  bool GetCustomSpeedLimit(double* custom_speed_limit) const;
  /// @brief get boundary_type from s
  /// @param is_left left or right
  /// @param s lane center line accumulate_s
  /// @param boundary_type : boundary_type s is start from input s
  /// @return void
  void forward_boundary_type(
      bool is_left, double s,
      std::vector<LaneBoundaryType>& boundary_type) const;

  /// @brief get boundary_type from point
  /// @param is_left left or right
  /// @param point
  /// @param boundary_type : boundary_type s is start from input point
  /// @return void
  void forward_boundary_type(
      bool is_left, const Point2d& point,
      std::vector<LaneBoundaryType>& boundary_type) const;

  const math::LineCurve2d& center_line() const { return center_line_; }
  const std::vector<Point2d>& points() const { return center_line_.points(); }
  void UpdateCenterLinePoints(const std::vector<Point2d>& points);

  /// @brief overlap map elements
  const std::vector<uint64_t>& overlap_cross_walks() const {
    return lane_info_.cross_walks;
  }
  const std::vector<uint64_t>& overlap_speed_bumps() const {
    return lane_info_.speed_bumps;
  }
  const std::vector<uint64_t>& overlap_parking_spaces() const {
    return lane_info_.parking_spaces;
  }
  const std::vector<uint64_t>& overlap_clear_areas() const {
    return lane_info_.clear_areas;
  }
  const std::vector<uint64_t>& overlap_stop_lines() const {
    return lane_info_.traffic_stop_lines;
  }

  const uint32_t& passable_env_state() const {
    return lane_info_.restricted_info.passable_env_state;
  }

  /// @brief check if point is locate on lane
  /// @param point
  /// @return true - false
  bool IsOnLane(const math::Vec2d& point) const;

  /// @brief get left and right width at s_offset
  /// @param s_offset
  /// @return left&  right width
  void GetWidthAtAccumS(double s, double* left_w, double* right_w) const;

  /// @brief get total width at point{x, y}
  /// @param x
  /// @param y
  /// @return total width
  double GetWidthAtPoint(double x, double y) const;

  /// @brief get total width at s_offset
  /// @param s_offset
  /// @return total width
  double GetWidthAtAccumS(double s) const;

  /// @brief if input lane is precede
  /// @param lane_id
  /// @return t/f
  bool IsPrecede(const uint64_t lane_id) const;

  /// @brief if input lane is next
  /// @param lane_id
  /// @return t/f
  bool IsNext(const uint64_t lane_id) const;

  bool GetSLWithLimit(const math::Vec2d& query_point,
                      SLPoint* const sl_point) const;
  bool GetSLWithoutLimit(const math::Vec2d& query_point,
                         SLPoint* const sl_point) const;
  bool GetXYWithoutLimit(const SLPoint& sl_point,
                         math::Vec2d* const xy_point) const;
  bool GetHeadingFromS(double query_s, double* heading) const;
  bool GetWidthFromS(double query_s, double* lw, double* rw) const;

  double navi_distance() const { return navi_distance_; }
  int navi_section_cnt() const { return navi_section_cnt_; }

  bool endpoint_toll() const { return endpoint_toll_; };
  /// @brief set lane navigation info
  /// @param is_navi
  /// @return void
  void SetIsNavigation(bool is_navi, double navi_distance,
                       int navi_section_cnt) {
    lane_info_.is_navigation = is_navi;
    navi_distance_ = is_navi ? navi_distance : 0.0;
    navi_section_cnt_ = is_navi ? navi_section_cnt : 0.0;
  };
  void SetIsNavigation(bool is_navi) { lane_info_.is_navigation = is_navi; };
  void SetSectionId(const uint64_t id) {
    if (id > 0) {
      lane_info_.section_id = id;
    }
  };
  void SetExpTrajFlagTrue() { lane_info_.is_exp_traj = true; }
  void SetSplitTopoModifyTrue() { lane_info_.is_split_topo_modify_ = true; }
  void SetMergeTopoModifyTrue() { lane_info_.is_merge_topo_modify_ = true; }
  void SetTrueLength(double length) { lane_info_.length = length; };
  void AddPreviousLane(const uint64_t id) { prev_lane_ids_.emplace_back(id); }

  double LaneFraction(int segment, double segment_fraction) const;

  std::pair<int, double> SegmentFraction(double lane_fraction) const;

  bool GetTangent(double fraction, math::Vec2d* tangent) const;

  math::Vec2d GetTangent(double fraction) const;

  math::Vec2d LerpPointFromFraction(double fraction) const;

  // default funciton for st planning
  std::vector<std::string> startpoint_associated_traffic_lights() const {
    return {};
  }
  const std::vector<LaneInteraction>& interactions() const {
    return interactions_;
  }
  void AddInteractions(const LaneInteraction& interact) {
    interactions_.emplace_back(interact);
  }

 private:
  double CalculateWidth(const math::LineSegment2d& base, const Point2d& p,
                        int lr) const;
  void GenerateSampleWidth(const LaneBoundariesConstPtr& boundary,
                           std::list<SampledWidth>* const sample_width);
  double GetDistanceFromSample(
      double query_s,
      const std::list<SampledWidth>* const sampled_widths) const;

 private:
  std::list<SampledWidth> sampled_left_width_;
  std::list<SampledWidth> sampled_right_width_;

  LaneInfo lane_info_;
  std::vector<uint64_t> prev_lane_ids_;
  std::vector<uint64_t> valid_pre_lane_ids_;
  std::vector<uint64_t> valid_next_lane_ids_;
  std::vector<uint64_t> sorted_next_lane_ids_;
  LaneBoundariesConstPtr left_boundary_ = nullptr;
  LaneBoundariesConstPtr right_boundary_ = nullptr;
  RoadBoundariesConstPtr left_road_boundary_ = nullptr;
  RoadBoundariesConstPtr right_road_boundary_ = nullptr;
  math::LineCurve2d center_line_;
  double navi_distance_ = 0.0;
  int navi_section_cnt_ = 0;
  std::unordered_set<TurnType> next_turn_types_;
  std::size_t lane_ind_in_section_ = 0u;
  std::vector<LaneInteraction> interactions_;

  // TODO: merge_map, false only, to delete
  bool endpoint_toll_ = false;

  template <class Archive>
  friend void serialize(Archive& ar, Lane& lane);
};

using LanePtr = std::shared_ptr<Lane>;
using LaneConstPtr = std::shared_ptr<const Lane>;

}  // namespace planning
}  // namespace ad_byd

#endif  // PILOT_PLANNING_MAP_LANE_H_

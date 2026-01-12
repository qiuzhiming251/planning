

#ifndef AD_BYD_PLANNING_MAP_LANE_SEQUENCE_H
#define AD_BYD_PLANNING_MAP_LANE_SEQUENCE_H
#include <limits>
#include <vector>

#include <cereal/access.hpp>
#include "plan_common/maps/lane.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/section.h"

namespace ad_byd {
namespace planning {
enum SplitTasksState { None = 0, Left_Task = 1, Right_Task = 2 };
enum SequenceDirection { Cur = 0, Left = 1, Right = 2, Extend = 3 };
class LaneSequence {
 public:
  explicit LaneSequence(const std::vector<LaneConstPtr> &lanes_ptr_vec);
  ~LaneSequence() = default;
  const std::vector<LaneConstPtr> &lanes() const { return lanes_; }
  std::vector<LaneConstPtr> &mutable_lanes() { return lanes_; }

  /// @brief get the first lane in sequence
  /// @return
  LaneConstPtr front_lane() const;

  /// @brief check if has more than one valid lane
  /// @return
  bool IsValid();

  /// @return lane sequence length
  double GetTrueLength() const;
  double GetPointsLength() const;

  /// @brief sample points from start_s to end with interval
  /// @param start_s the begin position from lane start
  /// @param interval
  /// @return points
  bool SamplePoints(double start_s, std::vector<Point2d> *const points,
                    double interval = 3.0) const;

  /// @brief add new lane into lane_sequence back
  /// @param lane input lane_ptr
  /// @return new-constructed lane_sequence
  bool AddLane(const LaneConstPtr &lane);

  /// @brief get point on center line at s
  /// @param s the length from first lane_ptr start
  /// @return x-y
  math::Vec2d GetPointAtS(double s) const;

  /// @brief get dist_to_start_lane_end and start_index
  void GetStartDistAndIndex(const ad_byd::planning::NaviPosition &navi_start,
                            const LaneConstPtr &nearest_lane, double s_offset,
                            double &dist_to_start_lane_end, int &start_index);

  /// @brief get the min projection distance from the center line by natural
  /// extension
  /// @param x coordinate
  /// @param y coordinate
  /// @return min abs projection distance by natural extension
  double GetProjectionDistance(const double &x, const double &y) const;

  /// @brief get the min projection distance from the center line by natural
  /// extension
  /// @param point
  /// @return min abs projection distance by natural extension
  double GetProjectionDistance(const math::Vec2d &point) const;

  /// @brief get nearest lane
  /// @param point
  /// @param dis output min distance
  /// @return
  LaneConstPtr GetNearestLane(const math::Vec2d &point,
                              double *dis = nullptr) const;
  LaneConstPtr GetNearestLane(const Point2d &pt, const double &heading) const;
  /// @brief get the min projection distance from the center line by natural
  /// extension
  /// @param point
  /// @param s_offset the length from start of lane
  /// @return min abs projection distance by natural extension
  double GetProjectionDistance(const math::Vec2d &point, double *s_offset,
                               double *l_offset = nullptr) const;

  /// @brief get the min distance from input coordinate to event
  /// @param poi_type poi type
  /// @param x input point x
  /// @param y input point y
  /// @return min distance
  double GetDistanceToPOI(const PoiType &poi_type, const double &x,
                          const double &y) const;

  /// @brief get the min distance from input coordinate to event
  /// @param poi_type poi type
  /// @param x input point x
  /// @param y input point y
  /// @param poi_lane output, the min distance even lane
  /// @return min distance
  double GetDistanceToPOI(
      const PoiType &poi_type, const double &x, const double &y,
      LaneConstPtr &poi_lane,
      const ad_byd::planning::NaviPosition *navi_start = nullptr) const;

  double GetDistanceToPOI(const PoiType &poi_type, const double &x,
                          const double &y, LaneConstPtr &poi_lane,
                          double dist_to_start_lane_end, int start_index) const;

  /// @brief get forward boundary type vector from lane sequence start s
  /// @param is_left left or right
  /// @param point input point
  /// @param vector<BoundaryType>
  /// @param boundary_type, boundary_type s is start from input s
  /// @return void
  void GetBoundaryTypeFromS(const bool &is_left, const double &s,
                            std::vector<LaneBoundaryType> &boundary_type) const;

  /// @brief get forward boundary type vector from input point
  /// @param is_left left or right
  /// @param point input point
  /// @param vector<BoundaryType>
  /// @param boundary_type, boundary_type s is start from point
  void GetBoundaryTypeFromPoint(
      const bool &is_left, const math::Vec2d &point,
      std::vector<LaneBoundaryType> &boundary_type) const;
  void GetRoadSegments(const bool is_left,
                       std::vector<math::LineSegment2d> *road_segs) const;

  /// @brief get the min distance from input coordinate to junction, default
  /// light_status, turn_type on poi_lane, if ego in junction, return 0.0
  /// @param navi_start
  /// @param poi_lane output, the virtual lane with junction id
  /// @return min distance
  double GetDistanceToJunction(LaneConstPtr &poi_lane,
                               const ad_byd::planning::NaviPosition &navi_start,
                               bool end_check = false,
                               bool special_check = false) const;

  /// @brief get distance between two junctions, return 0.0 if the
  /// distance exceeds 100 meters
  /// @param begin_junction_id
  /// @return
  double GetDistanceBetweenJunction(const uint64_t begin_junction_id,
                                    LaneConstPtr &next_virtual_lane) const;
  double GetDistanceToJunction(const double &x, const double &y,
                               LaneConstPtr &poi_lane) const;

  double GetDistanceToJunction(double dist_to_start_lane_end, int start_index,
                               LaneConstPtr &poi_lane) const;

  double GetDistanceExitJunction(
      const ad_byd::planning::NaviPosition &navi_start) const;
  /// @brief judge if can pass the next junction
  /// @param navi_start
  /// @param dist_to_junction
  /// @param next_junction  inside junction, if need judge next junction
  /// @return
  bool CanPassJunction(const ad_byd::planning::NaviPosition &navi_start,
                       double *dist_to_junction = nullptr,
                       bool next_junction = false);
  bool IsTurnLeft(const double &x, const double &y);
  bool IsTurnRight(const double &x, const double &y);

  bool IsOnLaneSequence(const LaneConstPtr &lane) const;
  const LanePtr GetPreLaneOnLaneSequence(const LaneConstPtr &lane) const;

  double GetDistanceToTargetLaneType(
      LaneConstPtr &poi_lane, const std::unordered_set<LaneType> &lane_type_set,
      const ad_byd::planning::NaviPosition &navi_start) const;
  double GetDistanceToTargetLaneType(
      LaneConstPtr &poi_lane, const std::unordered_set<LaneType> &lane_type_set,
      double dist_to_start_lane_end, int start_index) const;
  std::vector<std::pair<double, double>> GetAllDistanceToTargetLaneType(
      const std::unordered_set<LaneType> &lane_type_set,
      double dist_to_start_lane_end, int start_index,
      std::vector<uint32_t>& bus_lane_passable_mark);
  double GetDistanceToCustomSpeedLimit(
      const ad_byd::planning::NaviPosition &navi_start,
      double *speed_limit) const;
  std::optional<LaneConstPtr> FindPreLaneOnLaneseq(const Lane &lane);
  std::unordered_set<uint64_t> RemoveRepeatedSections();

  void SetSplitTaskState(SplitTasksState split_task_state) {
    split_task_state_ = split_task_state;
  }
  SplitTasksState GetSplitTasksState() const { return split_task_state_; }

  void SetSequenceDirection(SequenceDirection sequence_direction) {
    sequence_direction_ = sequence_direction;
  }
  SequenceDirection GetSequenceDirection() const { return sequence_direction_; }

  void SetSplitLcNum(int split_lc_num) { split_lc_num_ = split_lc_num; }
  int GetSplitLcNum() const { return split_lc_num_; }

  void SetSplitLcNumIsMax(int split_lc_num_is_max) {
    split_lc_num_is_max_ = true;
  }
  bool GetSplitLcNumIsMax() const { return split_lc_num_is_max_; }

  void SetSplitLaneId(int split_lane_id) { split_lane_id_ = split_lane_id; }
  int GetSplitLaneId() const { return split_lane_id_; }

  void SetHasStraightJunction(bool has_straight_junc) {
    set_has_straight_junction_ = has_straight_junc;
  }
  bool HasStraightJunction() const { return set_has_straight_junction_; }
  void SetSucceedStraightJunctionLcNum(int lc_num) {
    succeed_straight_lc_num_ = lc_num;
  }
  int GetSucceedStraightJunctionLcNum() const {
    return succeed_straight_lc_num_;
  }
  void SetBaseLane(LaneConstPtr lane) { baseLane_ = lane; }
  LaneConstPtr GetBaseLane() const { return baseLane_; }

 private:
  std::vector<LaneConstPtr> lanes_;

  template <class Archive>
  friend void serialize(Archive &ar, LaneSequence &lane_sequence);
  SplitTasksState split_task_state_ = SplitTasksState::None;
  SequenceDirection sequence_direction_ = SequenceDirection::Cur;
  bool split_lc_num_is_max_ = false;
  int split_lane_id_ = 0;
  int split_lc_num_ = 0;
  bool set_has_straight_junction_ = false;
  int succeed_straight_lc_num_ = 0;
  LaneConstPtr baseLane_ = nullptr;
};

class NavigableLaneSequence : public LaneSequence {
 public:
  explicit NavigableLaneSequence(
      const std::vector<LaneConstPtr> &lanes_ptr_vec);
  ~NavigableLaneSequence() = default;
  void SetNaviDistance(const float navi_dis) { navi_distance_ = navi_dis; }
  void SetNearestObstacleVel(const float nearest_obstacle_vel) {
    nearest_obstacle_vel_ = nearest_obstacle_vel;
  }
  void SetNearestObstacleDistance(const float nearest_obstacle_distance) {
    nearest_obstacle_distance_ = nearest_obstacle_distance;
  }
  void SetDistanceToEgoCar(const float distance_to_ego_car) {
    distance_to_ego_car_ = distance_to_ego_car;
  };
  void SetSequenceIdx(const float sequence_idx) {
    sequence_idx_ = sequence_idx;
  }
  void SetIsNaviSequence(const bool is_navi_sequence) {
    is_navi_sequence_ = is_navi_sequence;
  }
  void SetProbability(const float probability) { probability_ = probability; }
  void SetLcReason(const LcReason lc_reason) { lc_reason_ = lc_reason; }
  void SetDebugContent(const std::string &debug_content) {
    debug_content_ += debug_content;
  }
  void SetProjectionPoint(const Point2d &projection_point) {
    projection_point_ = projection_point;
  }
  void SetIsOccupyLaneSeq(bool is_occupy_lane_seq) {
    is_occupy_lane_seq_ = is_occupy_lane_seq;
  }
  void SetIsOptimalLaneSeq(bool is_optimal_lane_seq) {
    is_optimal_lane_seq_ = is_optimal_lane_seq;
  }
  void SetCompleteSequences(
      std::vector<std::vector<LaneConstPtr>> &complete_sequences) {
    complete_sequences_ = complete_sequences;
  }
  void SetNearestObstacle(const ObstacleForLane &obstacle) {
    nearest_obstacle_ = obstacle;
  }
  void SetRightRoundaboutLanesNum(
      const std::int32_t right_roundabout_lanes_num) {
    right_roundabout_lanes_num_ = right_roundabout_lanes_num;
  }
  void SetRightLanesNum(const std::int32_t right_lanes_num) {
    right_lanes_num_ = right_lanes_num;
  }
  void SetScoreInStraightJunction(const float score_in_straight_junction) {
    score_in_straight_junction_ = score_in_straight_junction;
  }
  void AddObstacleInSequence(
      const std::pair<std::string, double> &obstacle_in_sequence) {
    obstacles_in_sequence_.emplace_back(obstacle_in_sequence);
  }
  const float GetNaviDistance() const { return navi_distance_; }
  const float GetSequenceIdx() const { return sequence_idx_; }
  const float GetNearestObstacleVel() const { return nearest_obstacle_vel_; }
  const float GetNearestObstacleDistance() const {
    return nearest_obstacle_distance_;
  }
  const float GetDistanceToEgoCar() const { return distance_to_ego_car_; }
  const float GetProbability() const { return probability_; }
  const LcReason &GetLcReason() const { return lc_reason_; }
  bool IsOverLap(const std::vector<LaneConstPtr> &other_lane_sequence);
  bool IsTwoSplitLane(const std::vector<LaneConstPtr> &other_lane_sequence);
  const bool GetIsNaviSequence() const { return is_navi_sequence_; }
  const bool GetIsOccupyLaneSeq() const { return is_occupy_lane_seq_; }
  const bool GetIsOptimalLaneSeq() const { return is_optimal_lane_seq_; }
  const std::int32_t GetRightRoundaboutLanesNum() const {
    return right_roundabout_lanes_num_;
  }
  const std::int32_t GetRightLanesNum() const { return right_lanes_num_; }
  const std::vector<std::pair<std::string, double>> &GetObstaclesInSequence()
      const {
    return obstacles_in_sequence_;
  }
  const std::string &GetDebugContent() const { return debug_content_; }
  const Point2d &GetProjectionPoint() const { return projection_point_; }
  const std::vector<std::vector<LaneConstPtr>> &GetCompleteSequences() const {
    return complete_sequences_;
  }
  const ObstacleForLane &GetNearestObstacle() const {
    return nearest_obstacle_;
  }
  const float GetScoreInStraightJunction() const {
    return score_in_straight_junction_;
  }
  uint64_t FindSamePrevLane(
      const std::vector<LaneConstPtr> &other_lane_sequence);

 private:
  bool is_occupy_lane_seq_ = false;
  bool is_optimal_lane_seq_ = false;
  float navi_distance_ = 1000.0;
  float sequence_idx_ = 0.0;
  float nearest_obstacle_vel_ = 0.0;
  float nearest_obstacle_distance_ = 150;
  float distance_to_ego_car_ = 0.0;
  float score_in_straight_junction_ = 0.0;
  bool is_navi_sequence_ = false;
  std::int32_t right_roundabout_lanes_num_ = -1;
  std::int32_t right_lanes_num_ = -1;
  std::string debug_content_;
  Point2d projection_point_;
  std::vector<std::vector<LaneConstPtr>> complete_sequences_;
  std::vector<std::pair<std::string, double>> obstacles_in_sequence_;
  float probability_ = 0.0;
  LcReason lc_reason_ = LcReason::LC_REASON_NONE;
  ObstacleForLane nearest_obstacle_;
};

typedef std::shared_ptr<NavigableLaneSequence> NavigableLaneSequencePtr;
typedef std::shared_ptr<LaneSequence> LaneSequencePtr;

using CandidateSequencesPtr =
    std::shared_ptr<std::vector<NavigableLaneSequencePtr>>;
using CandidateSequencesConstPtr =
    std::shared_ptr<const std::vector<NavigableLaneSequencePtr>>;

enum BehaviorCommand {
  Command_Invalid = 0,
  Command_LaneChangeLeft = 1,
  Command_LaneChangeRight = 2,
  Command_LaneChangeCancel = 3
};
enum LaneChangeDir { Lc_Straight = 0, Lc_Left = 1, Lc_Right = 2 };

struct LaneSeqInfo {
  // Secition
  SectionConstPtr cur_section_ptr = nullptr;
  int cur_section_lane_num = 0;
  int cur_lane_position = -1;  // 1:most left, 2:right beside the most left

  // LANE Supplement
  LaneSequencePtr lane_seq = nullptr;
  LaneConstPtr nearest_lane = nullptr;
  LaneConstPtr navi_end_lane = nullptr;
  LaneConstPtr junction_lane = nullptr;
  double dist_to_navi_end = std::numeric_limits<double>::infinity();
  bool lane_seq_connect_navi_end = false;
  bool is_current = false;
  BehaviorCommand lc_dir = Command_Invalid;
  // BoundaryLine
  double dist_to_left_solid_line = std::numeric_limits<double>::infinity();
  double dist_to_right_solid_line = std::numeric_limits<double>::infinity();
  // Merge
  LaneConstPtr merge_lane = nullptr;
  BehaviorCommand merge_command = Command_Invalid;
  // NAVI Supplement
  double dist_to_junction = std::numeric_limits<double>::infinity();
  double dist_to_junction_id = std::numeric_limits<double>::infinity();
  double dist_to_virtual_lane = std::numeric_limits<double>::infinity();
  BehaviorCommand navi_lc_command = Command_Invalid;
  int lc_num = 0;
  bool next_junction_turn = false;

  // OVERTAKE supplment
  BehaviorCommand overtake_lc_command = Command_Invalid;
  // Special lane type
  double dist_to_bus_lane = std::numeric_limits<double>::infinity();
  std::vector<std::pair<double, double>> dist_to_bus_lane_vec;
  std::vector<uint32_t> bus_lane_passable_mark;
  // PnP
  int pnp_top1_lc_reason = 0;
  // RuleBasedReason
  LcReason lc_reason = LC_REASON_NONE;

  // Merge && Split
  double dist_to_merge = std::numeric_limits<double>::infinity();
  std::vector<double> dist_to_merge_vec;
  double dist_to_split = std::numeric_limits<double>::infinity();
  std::pair<double, double> dist_to_nearest_split = {
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::max()};
  double nearest_split_angle = 0.0;
  SplitTasksState split_task_state = SplitTasksState::None;
  // Merged zone
  std::pair<double, double> dist_to_merged_zone = {
      std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::max()};

  // V2 Information
  double dist_to_junction_v2 = std::numeric_limits<double>::infinity();
  double dist_to_navi_end_v2 = std::numeric_limits<double>::infinity();
  ad_byd::planning::V2TurnInfo::V2DetailTurnType nearest_turn_type_v2 =
      ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE;
  ad_byd::planning::V2TurnInfo::V2DetailTurnType last_turn_type_v2 =
      ad_byd::planning::V2TurnInfo::V2DetailTurnType::NONE;

  // section fork && merge
  double dist_to_section_fork = std::numeric_limits<double>::infinity();
  double dist_to_section_merge = std::numeric_limits<double>::infinity();

  // ld lite
  bool lite_map_match_success = false;
  int lc_num_lite = 0;
  double dist_to_navi_end_lite = std::numeric_limits<double>::infinity();
  int lc_num_env = 0;
  double dist_to_navi_end_env = std::numeric_limits<double>::infinity();
  int lc_num_suc_junc = 0;
  std::string DebugString() const {
    return absl::StrFormat(
        " lane_seq_info: lc_num: %d, dist_to_navi_end: %.2f, connect_navi_end: "
        "%d, dist_to_junction: %.2f, dist_to_virtual_lane: %.2f, "
        "dist_to_section_fork: %.2f, dist_to_split: %.2f, dist_to_merge: %.2f, "
        "nearest_split_angle: %.6f , dist_to_bus_lane: %.2f, split_task_state: "
        "%d, "
        "lite_match_success: %d, lc_num_lite: %d, dist_to_navi_end_lite: %.2f, "
        "lc_num_env: %d, dist_to_navi_end_env: %.2f, lc_num_suc_junc=%d",
        lc_num, dist_to_navi_end, lane_seq_connect_navi_end, dist_to_junction,
        dist_to_virtual_lane, dist_to_section_fork, dist_to_split,
        dist_to_merge, nearest_split_angle, dist_to_bus_lane, split_task_state,
        lite_map_match_success, lc_num_lite, dist_to_navi_end_lite, lc_num_env,
        dist_to_navi_end_env, lc_num_suc_junc);
  }
};
typedef std::shared_ptr<LaneSeqInfo> LaneSeqInfoPtr;
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_LANE_SEQUENCE_H

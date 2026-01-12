

#ifndef AD_BYD_PLANNING_MAP_MAP_H
#define AD_BYD_PLANNING_MAP_MAP_H
#include <unordered_map>
#include <cereal/cereal.hpp>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "plan_common/async/future.h"
#include "plan_common/async/thread_pool.h"
#include "plan_common/maps/clear_aera.h"
#include "plan_common/maps/crosswalk.h"
#include "plan_common/maps/junction.h"
#include "plan_common/maps/lane.h"
#include "plan_common/maps/lane_boundaries.h"
#include "plan_common/maps/lane_boundary.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/road_boundary.h"
#include "plan_common/maps/route.h"
#include "plan_common/maps/section.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/maps/speed_bump.h"
#include "plan_common/maps/stop_line.h"
#include "plan_common/maps/exp_trajectory.h"
#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/aabox_kdtree2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/behavior.pb.h"
#include "plan_common/path/path.h"
#include "plan_common/util/kdtree.h"
namespace ad_byd {
namespace planning {

using Vec2d = math::Vec2d;

struct KDValue {
  KDValue(const std::string &id, int idx) : boundary_id(id), point_idx(idx) {}
  KDValue() {}
  std::string boundary_id;
  int point_idx = -1;
};

class Map {
 public:
  explicit Map(const MapInfo &map_info);

  /// @brief get the map mode, RT or HD
  /// @return
  const MapType &type() const { return type_; }
  const MapSubType &sub_type() const { return sub_type_; }

  const bool is_on_highway() const { return is_on_highway_; }

  const std::vector<V2RoadClass> &GetRoadClass() const {
    return v2_info_.road_class;
  }

  /// @brief get utm_to_cur_frame transform(left multiplication matrix).
  /// @return transform info
  const TransformInfo &utm_to_cur_frame() const { return utm_to_cur_frame_; }

  /// @brief get route info
  /// @return route
  RoutePtr route() const { return route_; }

  /// @brief get clear_area by id
  /// @param id the input clear_area id
  /// @return clear_area
  ClearAreaConstPtr GetClearAreaById(const uint64_t id) const;

  /// @brief get lane by id
  /// @param id the input lane id
  /// @return lane
  LaneConstPtr GetLaneById(const uint64_t id) const;

  StopLineConstPtr GetStopLineById(const uint64_t id) const;

  SpeedBumpConstPtr GetSpeedBumpById(const uint64_t id) const;

  void GetLanes(const Point2d &query_pt, const double &heading,
                const double &dist, const double &heading_limit,
                std::vector<LaneConstPtr> *const lanes) const;
  void GetLanes(const Point2d &query_pt, const double &heading,
                const double &dist,
                std::vector<LaneConstPtr> *const lanes) const;
  void GetLanes(const Point2d &query_pt, const double &dist,
                std::vector<LaneConstPtr> *const lanes) const;
  void OnLanes(const Point2d &query_pt, const double &heading,
               const double &heading_limit,
               std::vector<LaneConstPtr> *const lanes) const;
  void OnLanes(const Point2d &query_pt, const double &heading,
               std::vector<LaneConstPtr> *const lanes) const;
  void OnLanes(const Point2d &query_pt,
               std::vector<LaneConstPtr> *const lanes) const;

  /// @brief get nearest lane by coordinate
  /// @param x position coordinate x
  /// @param y position coordinate y
  /// @param heading ego heading
  /// @param dis the distance error
  /// @return the nearest lane in dis, null if not exist
  LaneConstPtr GetNearestLane(const Point2d &pt, const double &heading,
                              const double &dis, bool is_navi = false,
                              bool is_virtual = false,
                              double angle_range = M_PI / 6.0) const;

  /// @brief get nearest lane by least heading diff
  /// @param pt
  /// @param heading
  /// @param heading_limit
  /// @return
  LaneConstPtr GetNearestLane(const std::vector<LaneConstPtr> &candidate_lanes,
                              const Point2d &pt, const double &heading) const;

  /// @brief get nearestlanewihtheading
  /// @param x position coordinate x
  /// @param y position coordinate y
  /// @param dis the distance error
  /// @return the nearest lane in dis, null if not exist
  LaneConstPtr GetNearestLane(const Point2d &pt, const double &dis) const;

  /// @brief get the left lane of input lane
  /// @param lane the current lane
  /// @return the left lane, null if not exist
  LaneConstPtr GetLeftLane(const LaneConstPtr &lane) const;

  /// get the right lane of input lane
  /// @param lane the current lane
  /// @return the right lane,null if not exist
  LaneConstPtr GetRightLane(const LaneConstPtr &lane) const;

  void GetPrecedeLanes(const LaneConstPtr &lane,
                       std::vector<LaneConstPtr> *const lanes,
                       bool only_navi = false) const;
  std::vector<LaneConstPtr> GetPrecedeLanes(const LaneConstPtr &lane) const;
  /// check map is valid
  /// @return true if has current and neighbor lanes, false if not
  bool IsValid() const;

  const std::vector<LaneConstPtr> &lanes() const { return lanes_; }
  const std::unordered_map<uint64_t, LanePtr> &lane_map() const {
    return lane_map_;
  }

  JunctionConstPtr GetJunctionById(const uint64_t id) const;

  void GetJunctions(const Point2d &pt, const double &dist,
                    std::vector<JunctionConstPtr> *const junctions) const;

  const std::vector<JunctionConstPtr> &junctions() const { return junctions_; }

  const std::unordered_map<uint64_t, JunctionPtr> &junction_map() const {
    return junction_map_;
  }

  CrosswalkConstPtr GetCrosswalkById(const uint64_t id) const;
  const std::unordered_map<uint64_t, CrosswalkPtr> &crosswalk_map() const {
    return crosswalk_map_;
  }

  RoadBoundaryConstPtr GetRoadBoundaryById(const uint64_t id) const;
  LaneBoundaryConstPtr GetLaneBoundaryById(const uint64_t id) const;
  SectionConstPtr GetSectionById(const uint64_t id) const;
  SectionPtr GetMutableSectionById(const uint64_t id);
  const std::unordered_map<uint64_t, RoadBoundaryPtr> &road_boundary_map()
      const {
    return road_boundary_map_;
  }
  const std::unordered_map<uint64_t, LaneBoundaryPtr> &lane_boundary_map()
      const {
    return lane_boundary_map_;
  };
  const std::unordered_map<uint64_t, ClearAreaPtr> &clear_area_map() const {
    return clear_area_map_;
  }
  const std::unordered_map<uint64_t, StopLinePtr> &stop_line_map() const {
    return stop_line_map_;
  }
  const std::unordered_map<uint64_t, SpeedBumpPtr> &speed_bump_map() const {
    return speed_bump_map_;
  };
  const TrafficLightStatusMap &traffic_light_status_map() const {
    return traffic_light_status_map_;
  }
  const EHPV2Info &v2_info() const { return v2_info_; }
  double timestamp() const { return timestamp_; };
  int64_t seq() const { return seq_; };
  std::vector<uint64_t> FindNearRoadBoundaryIds(const std::string &obs_id,
                                                const Vec2d &position,
                                                const double heading,
                                                double range, bool check_angle,
                                                double heading_thresh,
                                                int *left_or_right) const;

  /// @brief get the next lanes of input lane according topology
  /// @param lane
  /// @return all next connected lanes
  std::vector<LaneConstPtr> GetNextLanes(const LaneConstPtr &lane) const;
  std::vector<LaneConstPtr> GetValidNextLanes(const LaneConstPtr &lane) const;
  std::vector<std::vector<LaneConstPtr>> GetSortedValidNextSectionsLanes(
      const LaneConstPtr &lane) const;
  LaneConstPtr GetLeftmostValidNextLanes(const LaneConstPtr &lane,
                                         const bool &navi_is_priority) const;
  LaneConstPtr GetRightmostValidNextLanes(const LaneConstPtr &lane,
                                          const bool &navi_is_priority) const;
  LaneConstPtr GetOptimalNextLane(const LaneConstPtr &lane,
                                  const bool &navi_is_priority,
                                  const uint64_t split_lane_id = 0) const;
  LaneConstPtr GetCityOptimalNextLane(const LaneConstPtr &lane,
                                      const bool &navi_is_priority) const;
  LaneConstPtr GetOptimalNextLaneByNeighborLane(
      const LaneConstPtr &lane, const LaneSequencePtr &cur_lane_seq,
      int nearest_lane_idx, int neighbor_lane_size, bool is_left) const;
  LaneConstPtr GetOptimalNextLaneMaplessNoa(const LaneConstPtr &lane,
                                            const bool &navi_is_priority) const;
  LaneConstPtr GetOptimalSmoothNextLane(const LaneConstPtr &lane,
                                        const bool &navi_is_priority,
                                        std::string &debug) const;
  LaneConstPtr GetContinueNextLane(const uint64_t lane_id);
  LaneConstPtr GetSmoothNextLane(const uint64_t lane_id,
                                 const bool navi_is_priority = true);
  /// get the most straight pre lane of input lane
  /// @param lane the current lane
  /// @return the pre lane,null if not exist
  LaneConstPtr GetContinuePreLane(const uint64_t lane_id,
                                  const bool is_build_left_seq = false);
  void GetContinuePreLaneseqs(const Vec2d &start_point, LaneSequencePtr &seqs,
                              const bool is_build_left_seq = false);
  /// @brief get lane sequence start with input lane
  /// @param lane the start lane
  /// @return the lane sequence
  /// @param navi_is_priority select the navigation lane first
  /// @return the sequence
  LaneSequencePtr GetLaneSequence(const LaneConstPtr &lane,
                                  const bool &navi_is_priority) const;
  LaneSequencePtr GetLeftLaneSequence(const LaneConstPtr &left_lane,
                                      const LaneSequencePtr &lane_sequence,
                                      const bool is_city = false);
  LaneSequencePtr GetRightLaneSequence(
      const LaneConstPtr &right_lane, const LaneSequencePtr &lane_sequence,
      const bool is_city = false,
      const ad_byd::planning::EIEChoiceType &eie_choice_type =
          ad_byd::planning::EIEChoiceType::CHOICE_NONE);
  /// @brief find the most consistent lane sequence with input
  /// @param lane_sequence the recorded lane sequence in last cycle
  /// @param x the current vehicle position
  /// @param y the current vehicle position
  /// @return the same lane sequence in current map beyond the (x,y)
  LaneSequencePtr GetSameLaneSequence(const LaneSequencePtr &lane_sequence,
                                      const double &x, const double &y);
  LaneSequencePtr GetSameLaneSequenceV2(
      const LaneSequencePtr &lane_sequence, const double &x, const double &y,
      const double &heading, std::vector<std::string> &debug,
      std::vector<Point2d> &debug_match_point) const;
  void GetAllLaneSequences(
      const LaneConstPtr &lane,
      std::vector<std::vector<LaneConstPtr>> &all_lane_sequences) const;
  void GetAllLaneSequences(
      const LaneConstPtr &lane, const double &length,
      std::vector<std::vector<LaneConstPtr>> &sequences) const;
  void GetAllLaneSequences(const LaneConstPtr &lane, const double &length,
                           std::vector<std::vector<LaneConstPtr>> &sequences,
                           const bool &arrive_length_limit) const;
  void GenerateAllLaneSequences(const Vec2d &start_point,
                                const double start_point_v,
                                const LaneConstPtr &start_lane,
                                const st::Behavior_FunctionId function_id,
                                LaneSequencePtr &cur_lane_sequence,
                                LaneSequencePtr &left_lane_sequence,
                                LaneSequencePtr &right_lane_sequence);

  double ComputeSmoothCost(const LaneConstPtr &first_lane,
                           const LaneConstPtr &second_lane,
                           const bool navi_is_priority);
  double ComputeTurnCost(const LaneConstPtr &first_lane,
                         const LaneConstPtr &second_lane);
  double GetAverageKappa(const LaneConstPtr &lane);

  size_t GetNormalLanePosition(const uint64_t lane_id,
                               const SectionConstPtr &section);
  size_t GetNormalLaneSize(const SectionConstPtr &section);
  size_t GetSameDirLanePosition(const LaneConstPtr &target_lane,
                                const TurnType &turn_type, size_t *size);
  CompositeTurnType CheckCompositeLane(const LanePtr &lane) const;

  absl::StatusOr<st::mapping::v2::Segment> FindNearestLaneSegment(
      double lon, double lat) const;
  absl::StatusOr<st::mapping::v2::Segment> FindNearestRoadBoundarySegment(
      double lon, double lat) const;
  absl::StatusOr<st::mapping::v2::Segment> FindNearestLaneBoundarySegment(
      double lon, double lat) const;
  ad_byd::planning::LaneConstPtr FindNearestLane(double lon, double lat) const;
  ad_byd::planning::LaneBoundaryConstPtr FindNearestLaneBoundary(
      double lon, double lat) const;
  ad_byd::planning::RoadBoundaryConstPtr FindNearestRoadBoundary(
      double lon, double lat) const;
  ad_byd::planning::StopLineConstPtr FindNearestStopLine(double lon,
                                                         double lat) const;
  ad_byd::planning::JunctionConstPtr FindNearestJunction(double lon,
                                                         double lat) const;
  ad_byd::planning::CrosswalkConstPtr FindNearestCrosswalk(double lon,
                                                           double lat) const;
  ad_byd::planning::SpeedBumpConstPtr FindNearestSpeedBump(double lon,
                                                           double lat) const;
  ad_byd::planning::ClearAreaConstPtr FindNearestClearArea(double lon,
                                                           double lat) const;
  std::vector<st::mapping::v2::Segment> FindLaneSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindLaneBoundarySegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindRoadBoundarySegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindStopLineSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindJunctionSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindCrosswalkSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindSpeedBumpSegmentsInRadius(
      double lon, double lat, double radius) const;
  std::vector<st::mapping::v2::Segment> FindClearAreaSegmentsInRadius(
      double lon, double lat, double radius) const;

  std::vector<ad_byd::planning::LaneConstPtr> FindLanesInRadius(
      double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::LaneBoundaryConstPtr>
  FindLaneBoundariesInRadius(double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::RoadBoundaryConstPtr>
  FindRoadBoundariesInRadius(double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::StopLineConstPtr> FindStopLinesInRadius(
      double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::JunctionConstPtr> FindJunctionsInRadius(
      double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::CrosswalkConstPtr> FindCrosswalksInRadius(
      double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::SpeedBumpConstPtr> FindSpeedBumpsInRadius(
      double lon, double lat, double radius) const;
  std::vector<ad_byd::planning::ClearAreaConstPtr> FindClearAreasInRadius(
      double lon, double lat, double radius) const;
  void FindAllLaneSequence(
      std::vector<LaneConstPtr> &lanes,
      std::vector<std::vector<LaneConstPtr>> &lane_sequences) const;
  bool CheckIfValidCommonLane(const LaneConstPtr &check_lane) const;

  // void SetRouteSectionSeq(const std::vector<uint64_t> &section_seq) {
  //   route_section_seq_ = section_seq;
  // }
  // void AddRouteSectionSeq(const uint64_t section_id) {
  //   route_section_seq_.emplace_back(section_id);
  // }
  const std::vector<uint64_t> &GetRouteSectionSeq() const {
    return route_section_seq_;
  }
  // void SetExtendSectionSeq(const std::vector<uint64_t> &section_seq) {
  //   extend_section_seq_ = section_seq;
  // }
  // void AddExtendSectionSeq(const uint64_t section_id) {
  //   extend_section_seq_.emplace_back(section_id);
  // }
  const std::vector<uint64_t> &GetExtendSectionSeq() const {
    return extend_section_seq_;
  }
  void UpdateNaviPriorityLanes(const LaneConstPtr &ego_lane);
  LaneConstPtr GetNaviPriorityLane(const uint64_t section_id) const;
  int GetPriorityLaneRelation(const LaneConstPtr &lane) const;
  std::vector<uint64_t> GetNaviRouteIds() const { return route_section_seq_; };

 private:
  LaneConstPtr GetSameLane(const LaneConstPtr &lane) const;
  void GetPolygonSRange(const LaneConstPtr &lane,
                        const math::Polygon2d &polygon, double *s_min,
                        double *s_max) const;
  double GetLaneAnglediff(const LaneConstPtr &lane_split_front,
                          const LaneConstPtr &lane_split_back);
  void SetLaneInfoBySection(SectionInfo section, double section_length,
                            double next_section_length);
  void ConvertMapInfo(const MapInfo &map_info);
  void UpdateLane();
  void UpdateJunction();
  void ConstructSection(SectionInfo &section);
  void UpdateSection();
  void UpdateBoundarySection();
  void UpdateExpTrajectory();
  void UpdateLaneSplitMergeTopology();
  void ModifySplitTopology(const LanePtr &current_lane_info_ptr,
                           const std::vector<uint64_t> &next_lane_ids);
  void ModifyMergeTopology(const LanePtr &current_lane_info_ptr,
                           const std::vector<uint64_t> &pre_lane_ids);
  bool CheckLaneIsJunction(const LanePtr &lane_ptr);
  void ChangeLaneMergeTopo(LanePtr &left_lane, LanePtr &right_lane);
  void SetEntryLanesForJunction(JunctionPtr &junction_ptr);
  void SetExitLanesForJunction(JunctionPtr &junction_ptr);
  void SetCrosswalkForJunction(JunctionPtr &junction_ptr);

  inline double GetMinLDistByPointIndex(const Point2d &query_pt,
                                        const std::vector<Point2d> &points,
                                        const int32_t &pt_idx) const;
  LaneBoundariesPtr BuildLaneBoundaries(
      const std::vector<uint64_t> &lane_boundary_ids);
  RoadBoundariesPtr BuildRoadBoundaries(
      const std::vector<uint64_t> &road_boundary_ids);
  void FilterPredecessorLanes(const Point2d &query_pt,
                              const std::vector<LaneConstPtr> &candidate_lanes,
                              std::vector<LaneConstPtr> *const lanes) const;

  void FindAllLaneSequence(
      std::vector<LaneConstPtr> &lanes, const double &length_limit,
      std::vector<std::vector<LaneConstPtr>> &lane_sequences) const;
  void FindAllLaneSequence(
      std::vector<LaneConstPtr> &lanes, const double &length_limit,
      std::vector<std::vector<LaneConstPtr>> &lane_sequences,
      const bool &arrive_length_limit) const;
  void FindAllLaneSequenceBfs(
      std::vector<LaneConstPtr> &lanes, const double &length_limit,
      std::vector<std::vector<LaneConstPtr>> &lane_sequences) const;

 private:
  struct LANE {
    explicit LANE(LaneConstPtr _lane) { lane = std::move(_lane); }
    LaneConstPtr lane;
    size_t lane_change = 0;
  };

  double CalculateSequenceCost(const std::vector<LANE> &lanes) const;

  void UpdateNaviPriorityLaneId(const LaneConstPtr &lane);

  void FindReverseLaneSequence(std::vector<LANE> &lanes,
                               std::vector<std::vector<LANE>> &lane_sequences,
                               const uint64_t stop_section_id,
                               std::set<uint64_t> &empty_pre_lanes);
  size_t GetPreNaviLanes(const LaneConstPtr &current_lane,
                         std::vector<LaneConstPtr> &pre_lanes);
  const std::set<uint64_t> &GetSplitMergeShortDisIds() const {
    return split_merge_shortdis_ids_;
  }

 private:
  MapType type_ = PERCEPTION_MAP;
  MapSubType sub_type_;
  bool is_on_highway_ = false;
  TransformInfo utm_to_cur_frame_;
  std::unordered_map<uint64_t, LanePtr> lane_map_;
  std::vector<LaneConstPtr> lanes_;
  std::unordered_map<uint64_t, LaneBoundaryPtr> lane_boundary_map_;
  std::unordered_map<uint64_t, RoadBoundaryPtr> road_boundary_map_;
  std::unordered_map<uint64_t, ClearAreaPtr> clear_area_map_;
  std::unordered_map<uint64_t, JunctionPtr> junction_map_;
  std::unordered_map<uint64_t, CrosswalkPtr> crosswalk_map_;
  std::unordered_map<uint64_t, StopLinePtr> stop_line_map_;
  std::unordered_map<uint64_t, SpeedBumpPtr> speed_bump_map_;
  std::unordered_map<uint64_t, ExpTrajectoryPtr> exp_trajectory_map_;
  std::unordered_map<uint64_t, uint64_t> exp_traj_to_lane_map_;
  std::vector<JunctionConstPtr> junctions_;
  TrafficLightStatusMap traffic_light_status_map_;
  double timestamp_ = 0.0;
  int64_t seq_ = 0;
  RoutePtr route_;
  std::unordered_map<uint64_t, SectionPtr> section_map_;
  std::vector<uint64_t> route_section_seq_;
  std::vector<uint64_t> extend_section_seq_;
  mutable std::set<uint64_t> split_merge_shortdis_ids_;
  EHPV2Info v2_info_;

  std::unordered_map<uint64_t, std::vector<uint64_t>> boundary_id_lane_id_map_;

  struct SmoothPoint {
    double x;
    double y;
  };

  struct AABoxNode {
    AABoxNode(st::mapping::ElementId element_id,
              st::mapping::SegmentId segment_id, const Vec2d &start,
              const Vec2d &end)
        : element_id(element_id),
          segment_id(segment_id),
          start{start.x(), start.y()},
          end{end.x(), end.y()} {}

    st::AABox2d ComputeAABox() const {
      return st::AABox2d({start.x, start.y}, {end.x, end.y});
    }
    double DistanceSquareTo(const Vec2d &point) const {
      return st::Segment2d({start.x, start.y}, {end.x, end.y})
          .DistanceSquareTo(point);
    }

    st::mapping::ElementId GetElementId() const { return element_id; }
    st::mapping::SegmentId GetSegmentId() const { return segment_id; }

   private:
    st::mapping::ElementId element_id;
    st::mapping::SegmentId segment_id;
    SmoothPoint start;
    SmoothPoint end;
  };
  struct AABoxTree {
    std::vector<AABoxNode> aabox_nodes;
    std::unique_ptr<st::AABoxKDTree2d<AABoxNode>> aabox_tree;
  };
  absl::flat_hash_map<ad_byd::planning::FeatureType,
                      st::Future<std::shared_ptr<AABoxTree>>>
      feature_aabox_tree_;

  st::ThreadPool thread_pool_;
  void InitAABoxKDTree2d();

  template <typename Archive>
  friend void serialize(Archive &ar, Map &map);
};

using MapPtr = std::shared_ptr<Map>;
using MapConstPtr = std::shared_ptr<const Map>;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_MAP_H

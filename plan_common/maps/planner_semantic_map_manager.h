

#ifndef AD_BYD_PLANNING_PLAN_PLANNER_SEMANTIC_MAP_MANAGER_H
#define AD_BYD_PLANNING_PLAN_PLANNER_SEMANTIC_MAP_MANAGER_H

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <cereal/cereal.hpp>

#include "absl/status/status.h"
#include "plan_common/async/thread_pool.h"
#include "map_def.h"
#include "plan_common/maps/ld_lite_map.h"
#include "plan_common/maps/map.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/vec.h"
#include "semantic_map_defs.h"
#include "plan_common/log_data.h"
namespace st::planning {

constexpr double kDefaultMaxSpeedLimit = 50;  // m/s

struct PlannerSemanticMapModification {
  std::map<mapping::ElementId, double> lane_speed_limit_map;
  double max_speed_limit = kDefaultMaxSpeedLimit;

  bool IsEmpty() const {
    return max_speed_limit >= kDefaultMaxSpeedLimit &&
           lane_speed_limit_map.empty();
  }
};

struct ImpassableBoundaryInfo {
  Segment2d segment;
  std::string id;
  std::optional<double> height;
  ad_byd::planning::RoadBoundaryType type;
};

class PlannerSemanticMapManager {
 public:
  // NOTE: We use a `shared_ptr` for `semantic_map_manager` because
  // this class may run in async mode. It has to own the underlying semantic
  // map when running in another thread with a different life cycle.
  explicit PlannerSemanticMapManager(
      ad_byd::planning::MapPtr map_ptr,
      ad_byd::planning::LdLiteMapPtr lite_map_ptr);

  explicit PlannerSemanticMapManager(ad_byd::planning::MapPtr map_ptr,
                                     PlannerSemanticMapModification modifier);

  // NOTE: smm v2 = elements + spatial_index + no_coordinate_converter,
  // use semantic map manager to find elements by id, use
  // semantic_map_multilevel_spatial_index to find elements by spatial index.
  const ad_byd::planning::MapPtr& map_ptr() const { return map_ptr_; }
  const ad_byd::planning::LdLiteMapPtr& lite_map_ptr() const {
    return lite_map_ptr_;
  }

  const bool IsOnHighway() const { return map_ptr_->is_on_highway(); }

  /**
   * @brief Get road class of current road from v2
   * @return ad_byd::planning::V2RoadClass::V2RoadClassType
   */
  const ad_byd::planning::V2RoadClass::V2RoadClassType GetRoadClass() const {
    return map_ptr_->GetRoadClass().empty()
               ? ad_byd::planning::V2RoadClass::V2RoadClassType::UNKNOWN_ROAD
               : map_ptr_->GetRoadClass()[0].type;
  }

  const PlannerSemanticMapModification& GetSemanticMapModifier() const {
    return modifier_;
  }

  // Should called before GetXXX
  void SetSemanticMapModifier(PlannerSemanticMapModification modifier) {
    modifier_ = std::move(modifier);
  }

  // Returned value unit is m/s.
  double QueryLaneSpeedLimitById(mapping::ElementId lane_id) const;

  // Spatial search
  /************************* lane *****************************/
  // TODO: Change function naming following the style guide.
  std::vector<ad_byd::planning::LaneConstPtr> GetLanesInRadius(
      const Vec2d& smooth_coord, double radius) const;

  ad_byd::planning::LaneConstPtr GetNearestLaneWithHeading(
      const Vec2d& smooth_coord, double theta, double radius,
      double max_heading_diff) const;

  std::vector<Segment2d> GetImpassableBoundaries(const Vec2d& smooth_coord,
                                                 double radius) const;
  std::vector<ImpassableBoundaryInfo> GetImpassableBoundariesInfo(
      const Vec2d& smooth_coord, double radius) const;

  bool GetLaneProjection(const Vec2d& smooth_coord, mapping::ElementId lane_id,
                         double* const fraction = nullptr,
                         Vec2d* const point = nullptr,
                         double* const min_dist = nullptr,
                         Segment2d* const segment = nullptr) const;

  ad_byd::planning::LaneConstPtr GetNearestLane(
      const Vec2d& smooth_coord) const;

  enum class Side {
    kLEFT = 0,
    kRIGHT = 1,
  };

  std::optional<double> ComputeLaneWidth(const Vec2d& smooth_coord,
                                         mapping::ElementId lane_id,
                                         Side side) const;

  std::optional<double> GetLeftLaneWidth(const Vec2d& smooth_coord,
                                         mapping::ElementId lane_id) const;

  std::optional<double> GetRightLaneWidth(const Vec2d& smooth_coord,
                                          mapping::ElementId lane_id) const;

  /********************* lane boundary ***********************/
  std::vector<ad_byd::planning::LaneBoundaryConstPtr> GetLaneBoundaries(
      const Vec2d& smooth_coord, double radius) const;
  std::vector<ad_byd::planning::RoadBoundaryConstPtr> GetRoadBoundaries(
      const Vec2d& smooth_coord, double radius) const;

  /********************** clear area ************************/
  std::vector<ad_byd::planning::ClearAreaConstPtr> GetClearAreas(
      const Vec2d& smooth_coord, double radius) const;

  /************************ crosswalk ***********************/
  std::vector<ad_byd::planning::CrosswalkConstPtr> GetCrosswalks(
      const Vec2d& smooth_coord, double radius) const;

  /************************ intersection ********************/
  ad_byd::planning::JunctionConstPtr GetNearestJunction(
      const Vec2d& smooth_coord) const;

  // Query elements.
  // TODO: Adapt to smm v2.
  ad_byd::planning::LaneConstPtr FindLaneByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::LaneConstPtr FindCurveLaneByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::LaneBoundaryConstPtr FindLaneBoundaryByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::RoadBoundaryConstPtr FindRoadBoundaryByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::JunctionConstPtr FindJunctionByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::StopLineConstPtr FindStopLineByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::CrosswalkConstPtr FindCrosswalkByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::SpeedBumpConstPtr FindSpeedBumpByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::ClearAreaConstPtr FindClearAreaByIdOrNull(
      mapping::ElementId id) const;
  ad_byd::planning::SectionConstPtr FindSectionByIdOrNull(
      mapping::ElementId id) const;
  std::vector<ad_byd::planning::CrosswalkConstPtr> GetCrosswalksInRadius(
      const Vec2d& smooth_coord, double radius) const;
  double GetpointsmseById(mapping::ElementId id, double sample_s) const;
  void SectionLanesFilter(
      const Vec2d& ego_pos, const double ego_theta,
      std::vector<std::pair<uint64_t, double>>& section_lanes) const;
  std::vector<uint64_t> GetValidLanesInRange(const Vec2d& ego_pos,
                                             const double look_forward_s) const;

 private:
  std::vector<uint64_t> GetValidLanesInRangeHdMap(
      const Vec2d& ego_pos, double look_forward_s,
      std::vector<std::string>& debug) const;
  std::vector<uint64_t> GetValidLanesInRangeBevMap(
      const Vec2d& ego_pos, double look_forward_s,
      std::vector<std::string>& debug) const;
  void SectionLanesFilterByGap(
      const double s,
      std::vector<std::pair<ad_byd::planning::LaneConstPtr, double>>&
          valid_lanes_with_l) const;

 private:
  ad_byd::planning::MapPtr map_ptr_;
  ad_byd::planning::LdLiteMapPtr lite_map_ptr_;
  // Modifier contains speed limits, unit is m/s.
  PlannerSemanticMapModification modifier_;

  template <typename Archive>
  friend void serialize(Archive& ar, PlannerSemanticMapManager& psmm);
};
}  // namespace st::planning

#endif  // AD_BYD_PLANNING_PLAN_PLANNER_SEMANTIC_MAP_MANAGER_H



#ifndef ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_
#define ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <cereal/cereal.hpp>

#include "plan_common/log.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/container/strong_vector.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/frenet_frame.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/geometry/polygon2d.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/block.pb.h"

namespace st::planning {

DECLARE_STRONG_VECTOR(Station);

using LaneType = ad_byd::planning::LaneType;
using TurnType = ad_byd::planning::TurnType;
using SplitTopology = ad_byd::planning::SplitTopology;
using MergeTopology = ad_byd::planning::MergeTopology;
using BlockReason = st::planning::BlockReason;

struct StationInfo {
  bool is_in_intersection = false;
  bool is_exclusive_right_turn = false;
  bool is_in_roundabout = false;
  double speed_limit = DBL_MAX;
  LaneType lane_type = LaneType::LANE_NORMAL;
  TurnType turn_type = TurnType::NO_TURN;
  SplitTopology split_topo = SplitTopology::TOPOLOGY_SPLIT_NONE;
  MergeTopology merge_topo = MergeTopology::TOPOLOGY_MERGE_NONE;
};

struct StationCenter {
  mapping::ElementId lane_id = mapping::kInvalidElementId;
  double fraction = 0.0;
  Vec2d xy;
  Vec2d tangent;
  double accum_s;
  double speed_limit;
  // double point_mse;
  bool is_virtual;
  bool is_merging;
  bool is_splitting;
  bool is_in_intersection;
  bool has_cross_curb;
  uint64_t cross_curb_id{0};
  ad_byd::planning::TurnType turn_type;
  StationInfo station_info;

  mapping::LanePoint GetLanePoint() const {
    return mapping::LanePoint(lane_id, fraction);
  }

  Vec2d lat_point(double signed_offset) const {
    return xy + tangent.Perp() * signed_offset;
  }
  Vec2d lon_point(double signed_offset) const {
    return xy + tangent * signed_offset;
  }
  double lat_offset(const Vec2d& v) const { return tangent.CrossProd(v - xy); }
  double lon_offset(const Vec2d& v) const { return tangent.Dot(v - xy); }
};

enum StationBoundaryType {
  // UNKNOWN = 0,
  // SOLID = 1,
  // DASHED = 2,
  // SOLID_SOLID = 3,
  // DASHED_DASHED = 4,
  // SOLID_DASHED = 5,
  // DASHED_SOLID = 6,
  // SHADED_AREA = 7,
  // VIRTUAL_LANE = 8,
  // VIRTUAL_JUNCTION = 9,
  // ROAD_BOUNDARY = 10

  BROKEN_WHITE = 0,
  SOLID_WHITE = 1,
  BROKEN_YELLOW = 2,
  SOLID_YELLOW = 3,
  SOLID_DOUBLE_YELLOW = 4,
  CURB = 5,
  VIRTUAL_CURB = 6,
  BROKEN_LEFT_DOUBLE_WHITE = 7,   // Left broken right solid
  BROKEN_RIGHT_DOUBLE_WHITE = 8,  // Left solid right broken
  VIRTUAL_LANE = 9,

  UNKNOWN_TYPE = 99
};

inline std::string StationBoundaryTypeName(StationBoundaryType type) {
  // switch (type) {
  //   case StationBoundaryType::UNKNOWN:
  //     return "UNKNOWN";
  //   case StationBoundaryType::SOLID:
  //     return "SOLID";
  //   case StationBoundaryType::DASHED:
  //     return "DASHED";
  //   case StationBoundaryType::SOLID_SOLID:
  //     return "SOLID_SOLID";
  //   case StationBoundaryType::SOLID_DASHED:
  //     return "SOLID_DASHED";
  //   case StationBoundaryType::DASHED_SOLID:
  //     return "DASHED_SOLID";
  //   case StationBoundaryType::SHADED_AREA:
  //     return "SHADED_AREA";
  //   case StationBoundaryType::VIRTUAL_LANE:
  //     return "VIRTUAL_LANE";
  //   case StationBoundaryType::VIRTUAL_JUNCTION:
  //     return "VIRTUAL_JUNCTION";
  //   case StationBoundaryType::ROAD_BOUNDARY:
  //     return "ROAD_BOUNDARY";
  // }
  return "TODO";
}

struct StationBoundary {
  StationBoundaryType type;
  double lat_offset;

  bool IsSolid(double query_lat_offset) const {
    constexpr double kEpsilon = 0.5;  // m.
    switch (type) {
      case StationBoundaryType::SOLID_WHITE:
      case StationBoundaryType::SOLID_YELLOW:
      case StationBoundaryType::SOLID_DOUBLE_YELLOW:
      case StationBoundaryType::CURB:
      case StationBoundaryType::VIRTUAL_CURB:
        return true;
      case StationBoundaryType::BROKEN_LEFT_DOUBLE_WHITE:
        return query_lat_offset < lat_offset - kEpsilon;
      case StationBoundaryType::BROKEN_RIGHT_DOUBLE_WHITE:
        return query_lat_offset > lat_offset + kEpsilon;
      case StationBoundaryType::UNKNOWN_TYPE:
      case StationBoundaryType::BROKEN_WHITE:
      case StationBoundaryType::BROKEN_YELLOW:
      case StationBoundaryType::VIRTUAL_LANE:
        return false;
      default:
        throw std::runtime_error("switch case on enum unexpected");
    }
  }
};

using OptionalBoundary = std::optional<StationBoundary>;
struct BoundaryQueryResponse {
  OptionalBoundary right;
  OptionalBoundary left;
};

struct TrafficStaticObstaclesInfo {
  bool enable_stop{false};
  bool enable_slow_down{false};
  BlockReason block_reason{BlockReason::NONE};
  double stop_s{1000.0};
  bool is_construction_scene{false};
  bool construction_scene_debouncing{false};
  bool have_avliable_zone{true};
  std::string obs_id{"none"};
  bool construction_is_left{false};
  bool is_emergence_lane_scene{false};
};

class Station {
 public:
  explicit Station(StationCenter center, std::vector<StationBoundary> bounds)
      : center_(std::move(center)), boundaries_(std::move(bounds)) {}

  mapping::ElementId lane_id() const { return center_.lane_id; }
  const Vec2d& xy() const { return center_.xy; }
  const Vec2d& tangent() const { return center_.tangent; }
  double accumulated_s() const { return center_.accum_s; }
  double speed_limit() const { return center_.speed_limit; }
  bool is_virtual() const { return center_.is_virtual; }

  const StationInfo& station_info() const { return center_.station_info; }
  bool is_merging() const { return center_.is_merging; }
  bool is_splitting() const { return center_.is_splitting; }
  bool is_in_intersection() const { return center_.is_in_intersection; }
  bool has_cross_curb() const { return center_.has_cross_curb; }
  ad_byd::planning::TurnType turn_type() const { return center_.turn_type; }

  uint64_t cross_curb_id() const { return center_.cross_curb_id; }

  Vec2d lat_point(double signed_offset) const {
    return center_.lat_point(signed_offset);
  }
  Vec2d lon_point(double signed_offset) const {
    return center_.lon_point(signed_offset);
  }
  double lat_offset(const Vec2d& v) const { return center_.lat_offset(v); }
  double lon_offset(const Vec2d& v) const { return center_.lon_offset(v); }

  absl::Span<const StationBoundary> boundaries() const { return boundaries_; }

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      double signed_lat) const;
  // Including curb.
  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      double signed_lat) const;

  mapping::LanePoint GetLanePoint() const { return center_.GetLanePoint(); }

 private:
  // Stations are sampled on the target lane path.
  StationCenter center_;
  // Ordered by offset from right to left
  std::vector<StationBoundary> boundaries_;

  template <typename Archive>
  friend void serialize(Archive& ar, Station& station);
};

struct StationWaypoint {
  StationIndex station_index;
  double lon_offset;
  double accum_s;
};

class DrivePassage {
 public:
  DrivePassage() = default;
  DrivePassage(StationVector<Station> stations, mapping::LanePath lane_path,
               mapping::LanePath extend_lane_path, double lane_path_start_s,
               bool reach_destination, FrenetFrameType type,
               std::vector<int> change_index,
               ad_byd::planning::LaneSeqInfoPtr lane_seq_info = nullptr);

  DrivePassage(DrivePassage const& o)
      : stations_(o.stations_),
        last_real_station_index_(o.last_real_station_index_),
        center_seg_inv_len_(o.center_seg_inv_len_),
        lane_path_(o.lane_path_),
        extend_lane_path_(o.extend_lane_path_),
        beyond_lane_path_(o.beyond_lane_path_),
        reach_destination_(o.reach_destination_),
        lane_path_start_s_(o.lane_path_start_s_),
        segments_(o.segments_),
        type_(o.type_),
        lane_seq_info_(o.lane_seq_info_),
        change_index_(o.change_index_) {
    BuildFrenetFrame();
  }

  DrivePassage& operator=(DrivePassage const& o) {
    // //("DrivePassage::operator=");
    stations_ = o.stations_;
    last_real_station_index_ = o.last_real_station_index_;
    center_seg_inv_len_ = o.center_seg_inv_len_;
    lane_path_ = o.lane_path_;
    extend_lane_path_ = o.extend_lane_path_;
    beyond_lane_path_ = o.beyond_lane_path_;
    reach_destination_ = o.reach_destination_;
    lane_path_start_s_ = o.lane_path_start_s_;
    segments_ = o.segments_;
    type_ = o.type_;
    BuildFrenetFrame();
    return *this;
  }

  DrivePassage(DrivePassage&& o) = default;
  DrivePassage& operator=(DrivePassage&& o) = default;

  // ########## query operations ##########
  // NOTE : each pair.first represents the right side and .second the
  // left side, with all lateral offsets on the right side always smaller than 0
  absl::StatusOr<double> QuerySpeedLimitAt(const Vec2d& point) const;
  absl::StatusOr<double> QuerySpeedLimitAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAt(
      const Vec2d& point) const;
  absl::StatusOr<std::pair<double, double>> QueryCurbOffsetAtS(double s) const;

  absl::StatusOr<std::pair<double, double>> QueryNearestBoundaryLateralOffset(
      double s) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAt(
      const Vec2d& point) const;

  absl::StatusOr<std::pair<Vec2d, Vec2d>> QueryCurbPointAtS(double s) const;

  absl::StatusOr<BoundaryQueryResponse> QueryEnclosingLaneBoundariesAt(
      const Vec2d& point) const;
  BoundaryQueryResponse QueryEnclosingLaneBoundariesAtS(double s) const;

  absl::StatusOr<Vec2d> QueryLaterallyUnboundedTangentAt(
      const Vec2d& point) const;

  absl::StatusOr<Vec2d> QueryTangentAt(const Vec2d& point) const;
  absl::StatusOr<Vec2d> QueryTangentAtS(double s) const;
  absl::StatusOr<double> QueryTangentAngleAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtS(double s) const;

  absl::StatusOr<Vec2d> QueryPointXYAtSL(double s, double l) const;

  absl::StatusOr<StationWaypoint> QueryFrenetLonOffsetAt(
      const Vec2d& point) const;

  absl::StatusOr<double> QueryFrenetLatOffsetAt(const Vec2d& point) const;

  absl::StatusOr<FrenetCoordinate> QueryFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAt(const Box2d& box,
                                             bool zone_checking = true) const;

  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContour(
      const Polygon2d& contour, bool zone_checking = true) const;

  // Returns projection that is not bounded to the drive passage when object is
  // not near drive passage.
  absl::StatusOr<FrenetCoordinate> QueryLaterallyUnboundedFrenetCoordinateAt(
      const Vec2d& point) const;

  absl::StatusOr<FrenetCoordinate> QueryUnboundedFrenetCoordinateAt(
      const Vec2d& point) const;

  // Query a set of points
  absl::StatusOr<std::vector<FrenetCoordinate>> BatchQueryFrenetCoordinates(
      absl::Span<const Vec2d> points) const;

  absl::StatusOr<std::vector<std::optional<FrenetBox>>> BatchQueryFrenetBoxes(
      absl::Span<const Box2d> boxes, bool laterally_bounded) const;

  // ######## query operations end ########

  // Find the nearest point on center line.
  absl::StatusOr<Vec2d> FindNearestPointOnCenterLine(const Vec2d& point) const;

  // Find the nearest station from a point.
  StationIndex FindNearestStationIndex(const Vec2d& point) const;
  const Station& FindNearestStation(const Vec2d& point) const {
    return stations_[FindNearestStationIndex(point)];
  }
  // Find the nearest station from s.
  StationIndex FindNearestStationIndexAtS(double s) const;
  const Station& FindNearestStationAtS(double s) const {
    return stations_[FindNearestStationIndexAtS(s)];
  }

  double end_s() const { return stations_.back().accumulated_s(); }
  double front_s() const { return stations_.front().accumulated_s(); }
  double lane_path_start_s() const { return lane_path_start_s_; }
  bool beyond_lane_path() const { return beyond_lane_path_; }
  bool reach_destination() const { return reach_destination_; }

  bool ContainIntersection(const int check_step) const;
  double GetDistanceToIntersection(const Vec2d& point) const;
  double GetDistanceToNonIntersection(const Vec2d& point) const;
  const Station& GetFirstNonIntersectionStation(const Vec2d& point) const;

  bool empty() const { return stations_.size() == 0; }
  int size() const { return stations_.size(); }

  const Station& station(StationIndex index) const { return stations_[index]; }
  const StationVector<Station>& stations() const { return stations_; }
  StationIndex last_real_station_index() const {
    return last_real_station_index_;
  }
  const std::vector<int>& change_index() const { return change_index_; }

  const std::unique_ptr<FrenetFrame>& frenet_frame() const {
    return frenet_frame_;
  }

  std::unique_ptr<FrenetFrame> MoveFrenetFrame() {
    return std::move(frenet_frame_);
  }
  // Based on which the drive passage is built.
  // extend_lane_path has more segments on both sides than lane path. drive
  // passage's length is equal to extend_lane_path length. extend_lane_path is
  // also used to calculate traffic lights info on route as we want to
  // investigate the stoplines behind us.
  const mapping::LanePath& extend_lane_path() const {
    return extend_lane_path_;
  }

  // lane_path starts from plan_start_state and ends at route end or horizon
  // end. Route end stopline is calculated based on lane path.
  const mapping::LanePath& lane_path() const { return lane_path_; }

  const std::vector<Segment2d>& segments() const { return segments_; }
  const ad_byd::planning::LaneSeqInfoPtr lane_seq_info() const {
    return lane_seq_info_;
  }

  const TrafficStaticObstaclesInfo& traffic_static_obstacles_info() const {
    return traffic_static_obstacles_info_;
  }
  void SetTrafficStaticObstaclesInfo(const bool& enable_stop,
                                     const bool& enable_slow_down,
                                     const BlockReason& block_reason,
                                     const double& stop_s,
                                     const bool& have_avliable_zone,
                                     const std::string& obs_id,
                                     const bool& is_emergence_lane_scene) {
    traffic_static_obstacles_info_.enable_stop = enable_stop;
    traffic_static_obstacles_info_.enable_slow_down = enable_slow_down;
    traffic_static_obstacles_info_.block_reason = block_reason;
    traffic_static_obstacles_info_.stop_s = stop_s;
    traffic_static_obstacles_info_.have_avliable_zone = have_avliable_zone;
    traffic_static_obstacles_info_.obs_id = obs_id;
    traffic_static_obstacles_info_.is_emergence_lane_scene =
        is_emergence_lane_scene;
  }

  void ReSetIsEmergencyLaneScene() {
    traffic_static_obstacles_info_.is_emergence_lane_scene = false;
  }

  void SetTrafficIsConstruction() {
    traffic_static_obstacles_info_.is_construction_scene = true;
    traffic_static_obstacles_info_.construction_scene_debouncing = false;
  }

  void SetTrafficIsConstructionDebouncing() {
    traffic_static_obstacles_info_.is_construction_scene = true;
    traffic_static_obstacles_info_.construction_scene_debouncing = true;
  }

  void SetTrafficConstructionLocation(const bool& construction_is_left) {
    traffic_static_obstacles_info_.construction_is_left = construction_is_left;
  }

  void SetDistToNaviEndByTrafficStaticObstacles(const double& stop_s) {
    lane_seq_info_->dist_to_navi_end = stop_s;
  }

 private:
  struct ProjectionResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double accum_s;
    double signed_l;
    double lerp_factor;
  };

  absl::StatusOr<ProjectionResult> ProjectPointToStations(
      const Vec2d& point, bool allow_extrapolation) const;
  absl::StatusOr<ProjectionResult> ProjectPointToStationsWithinRadius(
      const Vec2d& point, const ProjectionResult& projection,
      const Vec2d& prev_point, double search_radius) const;
  absl::Status IsProjectionResultOnDrivePassage(
      const ProjectionResult& res) const;
  absl::StatusOr<std::optional<FrenetBox>> QueryFrenetBoxWithinRadius(
      const Box2d& box, const ProjectionResult& center_projection,
      double search_radius, bool laterally_bounded) const;
  absl::StatusOr<FrenetBox> QueryFrenetBoxAtContourPoints(
      absl::Span<const Vec2d> contour_points, bool zone_checking = true) const;

  struct BinarySeachResult {
    StationIndex station_index_1;
    StationIndex station_index_2;
    StationIndex near_station_index;
    double ds;  // s - prev_s
  };

  BinarySeachResult BinarySearchForNearStation(double s) const;

  void BuildFrenetFrame();

  // WARNING! The c'tors and assiggnment ops defined above must also be updated
  // once a new field be added!
  StationVector<Station> stations_;
  StationIndex last_real_station_index_;

  std::vector<double> center_seg_inv_len_;  // One less than stations.
  mapping::LanePath lane_path_;
  mapping::LanePath extend_lane_path_;
  bool beyond_lane_path_;
  bool reach_destination_;
  double lane_path_start_s_;
  std::vector<Segment2d> segments_;

  FrenetFrameType type_;
  ad_byd::planning::LaneSeqInfoPtr lane_seq_info_ = nullptr;
  std::vector<int> change_index_;
  std::unique_ptr<FrenetFrame> frenet_frame_;
  TrafficStaticObstaclesInfo traffic_static_obstacles_info_;
  template <typename Archive>
  friend void serialize(Archive& ar, DrivePassage& drive_passage);
};

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_H_

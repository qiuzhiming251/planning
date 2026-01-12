

#ifndef AD_BYD_PLANNING_COMMON_VECTOR_POLYLINE_H
#define AD_BYD_PLANNING_COMMON_VECTOR_POLYLINE_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "plan_common/gflags.h"
#include "plan_common/type_def.h"
#include "plan_common/maps/lane.h"
#include "plan_common/maps/map_def.h"
#include "plan_common/math/angle.h"
#include "plan_common/math/math_utils.h"
#include "plan_common/math/polygon2d.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {
using Vec2d = math::Vec2d;

struct VectorPoint {
  Vec2d start_pt;
  Vec2d end_pt;
  double length = 0.0;
  double heading = 0.0;
};

enum class PolylineType {
  UNSET = 0,
  LANE = 1,
  LANE_BOUNDARY = 2,
  ROAD_BOUNDARY = 3,
  POLYGON = 4,
  STOPLINE = 8,
  OBSTACLE = 10,
};

enum class PolygonType {
  CROSSWALK = 0,
  JUNCTION = 1,
  SPEEDBUMP = 2,
};

static const std::unordered_map<PolylineType, std::string>
    kPolylineType2NameMap = {
        {PolylineType::UNSET, "UNSET"},
        {PolylineType::OBSTACLE, "OBSTACLE"},
        {PolylineType::LANE, "LANE"},
        {PolylineType::LANE_BOUNDARY, "LANE_BOUNDARY"},
        {PolylineType::POLYGON, "POLYGON"},
        {PolylineType::STOPLINE, "STOPLINE"},
        {PolylineType::ROAD_BOUNDARY, "ROAD_BOUNDARY"},
};

class SegmentID {
 public:
  SegmentID() = default;
  SegmentID(const std::string& instance_id, const std::size_t& vec_num,
            std::size_t idx = 0);
  ~SegmentID() = default;

  void SetInstanceId(const std::string& instance_id);
  void SetIdx(const std::size_t& idx);
  void SetVecNum(const std::size_t& vec_num);

  const std::string& instance_id() const { return instance_id_; }
  const std::size_t& idx() const { return idx_; }
  const std::size_t& vec_num() const { return vec_num_; }
  std::string ToString() const;

 private:
  std::string instance_id_;
  std::size_t idx_ = 0;
  std::size_t vec_num_ = 0;
};

class PolylineID {
 public:
  PolylineID() = default;
  PolylineID(const std::vector<SegmentID>& segments_id,
             const PolylineType& type);
  ~PolylineID() = default;

  const std::vector<SegmentID>& segment_ids() const { return segment_ids_; }
  const PolylineType& type() const { return type_; }
  const std::string& ToString() const { return str_id_; }

 private:
  std::vector<SegmentID> segment_ids_;
  std::string str_id_;
  PolylineType type_ = PolylineType::UNSET;
};

class Polyline {
 public:
  Polyline() = default;
  ~Polyline() = default;

  void SetPolylineId(const PolylineID& id);
  void SetTotalLength(const double& length);
  void InsertVectorPoint(VectorPoint&& point);
  void SetVectorPoint(const std::vector<VectorPoint>& vectors);

  const PolylineID& polyline_id() const { return polyline_id_; }
  double total_length() const { return total_length_; }
  virtual std::string ToString() const { return polyline_id_.ToString(); };
  const std::vector<VectorPoint>& vectors() const { return vectors_; }

  virtual void ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& polyline_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const = 0;
  virtual double DistanceSquareTo(const Vec2d& pt) const;

 protected:
  PolylineID polyline_id_;
  double total_length_ = 0.0;
  std::vector<VectorPoint> vectors_;
};

class LanePolyline : public Polyline {
 public:
  LanePolyline() = default;
  ~LanePolyline() = default;
  void SetLaneType(const LaneType& lane_type);
  void SetLaneTurnType(const TurnType& turn_type);
  void SetNextTurnType(const std::unordered_set<TurnType>& turn_types);
  void SetLanePtr(const std::vector<LaneConstPtr>& lanes) {
    lanes_.assign(lanes.begin(), lanes.end());
  };

  const LaneType& lane_type() const { return lane_type_; }
  const TurnType& lane_turn_type() const { return lane_turn_type_; }
  const std::unordered_set<TurnType>& next_turn_types() const {
    return next_turn_types_;
  }
  virtual void ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& lane_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const;
  bool GetVectorVirtualType(const int32_t& vector_idx) const;
  LightStatus GetVectorLightStatus(const int32_t& vector_idx) const;
  double GetVectorSpeedLimit(const int32_t& vector_idx) const;
  bool GetVectorStopLineType(const int32_t& vector_idx) const;
  bool GetVectorNavigationType(const int32_t& vector_idx) const;
  MergeTopology GetVectorMergeType(const int32_t& vector_idx) const;
  SplitTopology GetVectorSplitType(const int32_t& vector_idx) const;

  const std::vector<LaneConstPtr>& lanes() const { return lanes_; }

 private:
  bool IsValid() const {
    return !lanes_.empty() &&
           lanes_.size() == polyline_id_.segment_ids().size();
  }

 private:
  LaneType lane_type_ = LaneType::LANE_UNKNOWN;
  TurnType lane_turn_type_ = TurnType::NO_TURN;
  std::unordered_set<TurnType> next_turn_types_;
  std::vector<LaneConstPtr> lanes_;
  std::size_t lane_polyline_vector_size_ =
      FLAGS_ad_byd_planning_lane_polyline_vector_num;
  std::size_t lane_vector_dim_ = FLAGS_ad_byd_planning_lane_vector_dim;
};

class LaneBoundaryPolyline : public Polyline {
 public:
  LaneBoundaryPolyline() = default;
  ~LaneBoundaryPolyline() = default;
  void SetLaneBoundaryTypes(
      const std::vector<LaneBoundaryType>& boundary_types);
  virtual void ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& lane_boundary_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const;
  LaneBoundaryType GetLaneBoundaryType(const int32_t& vector_idx) const;

 private:
  bool IsValid() const;

 private:
  std::vector<LaneBoundaryType> boundary_types_;
  std::size_t lane_boundary_polyline_vector_size_ =
      FLAGS_ad_byd_planning_road_boundary_polyline_vector_num;
  std::size_t lane_boundary_vector_dim_ =
      FLAGS_ad_byd_planning_road_boundary_vector_dim;
};

class PolygonPolyline : public Polyline {
 public:
  PolygonPolyline() = default;
  ~PolygonPolyline() = default;

  void SetPolygonType(const PolygonType& polygon_type);
  void SetPolygon(const math::Polygon2d& polygon) { polygon_ = polygon; };

  const PolygonType& polygon_type() const { return polygon_type_; }
  virtual void ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& polygon_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const;
  virtual std::string ToString() const override;
  virtual double DistanceSquareTo(const Vec2d& pt) const override;

 private:
  PolygonType polygon_type_ = PolygonType::CROSSWALK;
  math::Polygon2d polygon_;
  std::size_t polygon_polyline_vector_size_ =
      FLAGS_ad_byd_planning_polygon_polyline_vector_num;
  std::size_t polygon_polyline_vector_dim_ =
      FLAGS_ad_byd_planning_polygon_polyline_vector_dim;
};

class StopLinePolyline : public Polyline {
 public:
  StopLinePolyline() = default;
  ~StopLinePolyline() = default;

  void SetStopLinePolylineType(const StopLineType& type) { type_ = type; }
  virtual void ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& stop_line_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const;
  const StopLineType& stop_line_type() const { return type_; }

 private:
  StopLineType type_;
  std::size_t stop_line_polyline_vector_size_ =
      FLAGS_ad_byd_planning_road_boundary_polyline_vector_num;
  std::size_t stop_line_vector_dim_ =
      FLAGS_ad_byd_planning_road_boundary_vector_dim;
};

class RoadBoundaryPolyline : public Polyline {
 public:
  RoadBoundaryPolyline() = default;
  ~RoadBoundaryPolyline() = default;

  void SetRoadBoundaryType(const RoadBoundaryType& type) { type_ = type; }
  const RoadBoundaryType& road_boundary_type() const { return type_; }

  virtual void ConvertToFloats(const Vec2d& anchor_point,
                               const double& rotate_heading,
                               const int32_t& road_boundary_idx,
                               const std::size_t& start_idx,
                               std::vector<float>* const fp) const;

 private:
  RoadBoundaryType type_;
  std::size_t road_boundary_polyline_vector_size_ =
      FLAGS_ad_byd_planning_road_boundary_polyline_vector_num;
  std::size_t road_boundary_vector_dim_ =
      FLAGS_ad_byd_planning_road_boundary_vector_dim;
};

struct PolylineInfo {
  std::vector<std::string> lane_polyline_infos;
  std::vector<std::string> lane_boundary_polyline_infos;
  std::vector<std::string> road_boundary_polyline_infos;
  std::vector<std::string> crosswalk_polyline_infos;
  std::vector<std::string> junction_polyline_infos;
  std::vector<std::string> speed_bump_polyline_infos;
  std::vector<std::string> stopline_polyline_infos;
  std::vector<std::string> dynamic_obstacle_infos;
  std::vector<std::string> static_obstacle_infos;
};

using PolylineInfoPtr = std::shared_ptr<PolylineInfo>;
using PolylineInfoConstPtr = std::shared_ptr<const PolylineInfo>;

}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_VECTOR_POLYLINE_H
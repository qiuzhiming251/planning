
#include <sstream>

#include "plan_common/log.h"
#include "plan_common/maps/polyline.h"
#include "plan_common/math/angle.h"
#include "plan_common/math/double.h"

namespace ad_byd {
namespace planning {
SegmentID::SegmentID(const std::string& instance_id, const std::size_t& vec_num,
                     std::size_t idx) {
  instance_id_ = instance_id;
  vec_num_ = vec_num;
  idx_ = idx;
}

void SegmentID::SetInstanceId(const std::string& instance_id) {
  instance_id_ = instance_id;
}

void SegmentID::SetIdx(const std::size_t& idx) { idx_ = idx; }

void SegmentID::SetVecNum(const std::size_t& vec_num) { vec_num_ = vec_num; }

std::string SegmentID::ToString() const {
  std::stringstream ss;
  ss << instance_id_ << "_" << idx_;
  return ss.str();
}

PolylineID::PolylineID(const std::vector<SegmentID>& segments_id,
                       const PolylineType& type) {
  segment_ids_.assign(segments_id.begin(), segments_id.end());
  type_ = type;
  std::stringstream ss;
  if (kPolylineType2NameMap.find(type_) != kPolylineType2NameMap.end()) {
    ss << kPolylineType2NameMap.at(type_);
  }
  for (const auto& segment_id : segment_ids_) {
    ss << "&" << segment_id.ToString();
  }
  str_id_ = ss.str();
}

void Polyline::SetPolylineId(const PolylineID& id) { polyline_id_ = id; }

void Polyline::SetTotalLength(const double& length) { total_length_ = length; }

void Polyline::InsertVectorPoint(VectorPoint&& point) {
  vectors_.emplace_back(std::move(point));
}

void Polyline::SetVectorPoint(const std::vector<VectorPoint>& vectors) {
  vectors_.assign(vectors.begin(), vectors.end());
}

double Polyline::DistanceSquareTo(const Vec2d& pt) const {
  double dist = std::numeric_limits<double>::max();
  if (vectors_.empty()) return dist;
  for (const auto& vector : vectors_) {
    dist = std::min(dist, vector.end_pt.DistanceSquareTo(pt));
  }
  dist = std::min(dist, vectors_.front().start_pt.DistanceSquareTo(pt));
  return dist;
}

void LanePolyline::SetLaneType(const LaneType& lane_type) {
  lane_type_ = lane_type;
}

void LanePolyline::SetLaneTurnType(const TurnType& turn_type) {
  lane_turn_type_ = turn_type;
}

void LanePolyline::SetNextTurnType(
    const std::unordered_set<TurnType>& turn_types) {
  next_turn_types_.insert(turn_types.begin(), turn_types.end());
}
void LanePolyline::ConvertToFloats(const Vec2d& anchor_point,
                                   const double& rotate_heading,
                                   const int32_t& lane_idx,
                                   const std::size_t& start_idx,
                                   std::vector<float>* const fp) const {
  size_t vector_end_idx =
      start_idx + lane_polyline_vector_size_ * lane_vector_dim_;
  int32_t vector_end_ind = static_cast<int32_t>(
      std::min(lane_polyline_vector_size_, vectors_.size()));
  for (int32_t vector_ind = 0; vector_ind < vector_end_ind;
       vector_ind++, vector_end_idx -= lane_vector_dim_) {
    int32_t vector_idx = static_cast<int32_t>(vectors_.size() - vector_ind - 1);
    const VectorPoint& vector_point = vectors_[vector_idx];

    Vec2d start_pt = math::RotateVector2d(vector_point.start_pt - anchor_point,
                                          rotate_heading);
    Vec2d end_pt = math::RotateVector2d(vector_point.end_pt - anchor_point,
                                        rotate_heading);

    double rotated_start_x = start_pt.x();
    double rotated_start_y = start_pt.y();
    double rotated_end_x = end_pt.x();
    double rotated_end_y = end_pt.y();

    // Get other attributes
    double heading =
        math::NormalizeAngle(vector_point.heading + rotate_heading);
    double cos_vec_heading = std::cos(heading);
    double sin_vec_heading = std::sin(heading);
    bool is_navi = false;
    LightStatus light_status = LightStatus::NONE_LIGHT;
    double speed_limit = 0.0;
    bool is_end_of_stop_line = false;
    bool is_virtual_lane = false;
    std::size_t lane_ind_in_section = 1u;
    MergeTopology merge_type = TOPOLOGY_MERGE_NONE;
    SplitTopology split_type = TOPOLOGY_SPLIT_NONE;
    if (vector_ind >= 0 && vector_ind < static_cast<int32_t>(vectors_.size()) &&
        IsValid()) {
      const auto& segment_ids = polyline_id_.segment_ids();
      for (std::size_t i = 0; i < segment_ids.size(); ++i) {
        const auto& segment_id = segment_ids.at(i);
        if (vector_ind < static_cast<int32_t>(segment_id.vec_num())) {
          is_navi = lanes_.at(i)->is_navigation();
          light_status = lanes_.at(i)->light_status();
          speed_limit = lanes_.at(i)->speed_limit();
          is_end_of_stop_line = lanes_.at(i)->stop_line();
          is_virtual_lane = lanes_.at(i)->IsVirtual();
          lane_ind_in_section = lanes_.at(i)->lane_ind_in_section();
          merge_type = lanes_.at(i)->merge_topology();
          split_type = lanes_.at(i)->split_topology();
        }
      }
    }
    TurnType turn_type = lane_turn_type();
    LaneType lane_type = LanePolyline::lane_type();
    bool mask = 0;
    fp->at(vector_end_idx - lane_vector_dim_) =
        static_cast<float>(rotated_start_x);
    fp->at(vector_end_idx - lane_vector_dim_ + 1u) =
        static_cast<float>(rotated_start_y);
    fp->at(vector_end_idx - lane_vector_dim_ + 2u) =
        static_cast<float>(rotated_end_x);
    fp->at(vector_end_idx - lane_vector_dim_ + 3u) =
        static_cast<float>(rotated_end_y);
    fp->at(vector_end_idx - lane_vector_dim_ + 4u) =
        static_cast<float>(heading);
    fp->at(vector_end_idx - lane_vector_dim_ + 5u) =
        static_cast<float>(cos_vec_heading);
    fp->at(vector_end_idx - lane_vector_dim_ + 6u) =
        static_cast<float>(sin_vec_heading);
    fp->at(vector_end_idx - lane_vector_dim_ + 7u) =
        static_cast<float>(vector_idx);
    fp->at(vector_end_idx - lane_vector_dim_ + 8u) =
        static_cast<float>(is_navi);
    fp->at(vector_end_idx - lane_vector_dim_ + 9u) =
        static_cast<float>(light_status);
    fp->at(vector_end_idx - lane_vector_dim_ + 10u) =
        static_cast<float>(turn_type);
    fp->at(vector_end_idx - lane_vector_dim_ + 11u) =
        static_cast<float>(speed_limit);
    fp->at(vector_end_idx - lane_vector_dim_ + 12u) =
        static_cast<float>(is_end_of_stop_line);
    fp->at(vector_end_idx - lane_vector_dim_ + 13u) =
        static_cast<float>(lane_type);
    fp->at(vector_end_idx - lane_vector_dim_ + 14u) =
        static_cast<float>(is_virtual_lane);
    fp->at(vector_end_idx - lane_vector_dim_ + 15u) =
        static_cast<float>(lane_idx);
    fp->at(vector_end_idx - lane_vector_dim_ + 16u) =
        static_cast<float>(lane_ind_in_section);
    fp->at(vector_end_idx - lane_vector_dim_ + 17u) =
        static_cast<float>(merge_type);
    fp->at(vector_end_idx - lane_vector_dim_ + 18u) =
        static_cast<float>(split_type);
    fp->at(vector_end_idx - lane_vector_dim_ + 19u) = static_cast<float>(0);
    fp->at(vector_end_idx - lane_vector_dim_ + 20u) = static_cast<float>(0);
    fp->at(vector_end_idx - lane_vector_dim_ + 21u) = static_cast<float>(0);
    fp->at(vector_end_idx - lane_vector_dim_ + 22u) = static_cast<float>(0);
    fp->at(vector_end_idx - lane_vector_dim_ + 23u) = static_cast<float>(mask);
  }

  // for (int vec_idx = 0; vec_idx < lane_polyline_vector_size_; ++vec_idx) {
  //   std::string vector_str;
  //   for (int i = 0; i < lane_vector_dim_; ++i) {
  //     std::stringstream ss;
  //     ss << fp->at(start_idx + vec_idx * lane_vector_dim_ + i) << ", ";
  //     vector_str += ss.str();
  //   }

  //   LDEBUG("LanePolyline %s vector %d :\n %s",
  //   polyline_id().ToString().c_str(), vec_idx, vector_str.c_str());
  // }
}

bool LanePolyline::GetVectorVirtualType(const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return false;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->IsVirtual();
    }
  }

  return false;
}

LightStatus LanePolyline::GetVectorLightStatus(
    const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return LightStatus::NONE_LIGHT;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->light_status();
    }
  }
  return LightStatus::NONE_LIGHT;
}

double LanePolyline::GetVectorSpeedLimit(const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return 0.0;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->speed_limit();
    }
  }
  return 0.0;
}

bool LanePolyline::GetVectorStopLineType(const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return false;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->stop_line();
    }
  }
  return false;
}

bool LanePolyline::GetVectorNavigationType(const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return false;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->is_navigation();
    }
  }
  return false;
}

MergeTopology LanePolyline::GetVectorMergeType(
    const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return TOPOLOGY_MERGE_NONE;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->merge_topology();
    }
  }
  return TOPOLOGY_MERGE_NONE;
}

SplitTopology LanePolyline::GetVectorSplitType(
    const int32_t& vector_idx) const {
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return TOPOLOGY_SPLIT_NONE;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return lanes_.at(i)->split_topology();
    }
  }
  return TOPOLOGY_SPLIT_NONE;
}

void LaneBoundaryPolyline::SetLaneBoundaryTypes(
    const std::vector<LaneBoundaryType>& boundary_types) {
  boundary_types_.assign(boundary_types.begin(), boundary_types.end());
}

void LaneBoundaryPolyline::ConvertToFloats(const Vec2d& anchor_point,
                                           const double& rotate_heading,
                                           const int32_t& lane_boundary_idx,
                                           const std::size_t& start_idx,
                                           std::vector<float>* const fp) const {
  size_t vector_end_idx = start_idx + lane_boundary_polyline_vector_size_ *
                                          lane_boundary_vector_dim_;

  for (size_t vector_ind = 0; vector_ind < lane_boundary_polyline_vector_size_;
       vector_ind++, vector_end_idx -= lane_boundary_vector_dim_) {
    if (!IsValid()) {
      continue;
    }
    if (vector_ind < vectors_.size()) {
      int32_t vector_idx =
          static_cast<int32_t>(vectors_.size() - vector_ind - 1);
      const VectorPoint& vector_point = vectors_[vector_idx];
      Vec2d start_pt = math::RotateVector2d(
          vector_point.start_pt - anchor_point, rotate_heading);
      Vec2d end_pt = math::RotateVector2d(vector_point.end_pt - anchor_point,
                                          rotate_heading);

      double rotated_start_x = start_pt.x();
      double rotated_start_y = start_pt.y();
      double rotated_end_x = end_pt.x();
      double rotated_end_y = end_pt.y();

      // Get other attributes
      double heading =
          math::NormalizeAngle(vector_point.heading + rotate_heading);
      double cos_vec_heading = std::cos(heading);
      double sin_vec_heading = std::sin(heading);
      PolylineType super_type = polyline_id().type();
      LaneBoundaryType type = GetLaneBoundaryType(vector_idx);
      bool mask = 0;
      fp->at(vector_end_idx - lane_boundary_vector_dim_) =
          static_cast<float>(rotated_start_x);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 1u) =
          static_cast<float>(rotated_start_y);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 2u) =
          static_cast<float>(rotated_end_x);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 3u) =
          static_cast<float>(rotated_end_y);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 4u) =
          static_cast<float>(heading);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 5u) =
          static_cast<float>(cos_vec_heading);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 6u) =
          static_cast<float>(sin_vec_heading);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 7u) =
          static_cast<float>(vector_idx);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 8u) =
          static_cast<float>(super_type);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 9u) =
          static_cast<float>(type.line_type);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 10u) =
          static_cast<float>(lane_boundary_idx);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 11u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 12u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 13u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 14u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - lane_boundary_vector_dim_ + 15u) =
          static_cast<float>(mask);
    }
  }

  // for (int vec_idx = 0; vec_idx < lane_boundary_polyline_vector_size_;
  // ++vec_idx) {
  //   std::string vector_str;
  //   for (int i = 0; i < lane_boundary_vector_dim_; ++i) {
  //     std::stringstream ss;
  //     ss << fp->at(start_idx + vec_idx * lane_boundary_vector_dim_ + i) << ",
  //     "; vector_str += ss.str();
  //   }

  //   LDEBUG("LaneBoundaryPolyline %s vector %d :\n %s",
  //   polyline_id().ToString().c_str(), vec_idx, vector_str.c_str());
  // }
}

LaneBoundaryType LaneBoundaryPolyline::GetLaneBoundaryType(
    const int32_t& vector_idx) const {
  LaneBoundaryType default_type;
  if (vector_idx < 0 || vector_idx >= static_cast<int32_t>(vectors_.size()) ||
      !IsValid()) {
    return default_type;
  }
  const auto& segment_ids = polyline_id_.segment_ids();
  for (std::size_t i = 0; i < segment_ids.size(); ++i) {
    const auto& segment_id = segment_ids.at(i);
    if (vector_idx < static_cast<int32_t>(segment_id.vec_num())) {
      return boundary_types_.at(i);
    }
  }
  return default_type;
}

bool LaneBoundaryPolyline::IsValid() const {
  return !boundary_types_.empty() &&
         boundary_types_.size() == polyline_id_.segment_ids().size();
}

void PolygonPolyline::SetPolygonType(const PolygonType& polygon_type) {
  polygon_type_ = polygon_type;
}

void PolygonPolyline::ConvertToFloats(const Vec2d& anchor_point,
                                      const double& rotate_heading,
                                      const int32_t& polygon_idx,
                                      const std::size_t& start_idx,
                                      std::vector<float>* const fp) const {
  size_t vector_end_idx =
      start_idx + polygon_polyline_vector_size_ * polygon_polyline_vector_dim_;

  for (size_t vector_ind = 0; vector_ind < polygon_polyline_vector_size_;
       vector_ind++, vector_end_idx -= polygon_polyline_vector_dim_) {
    if (vector_ind < vectors_.size()) {
      int32_t vector_idx =
          static_cast<int32_t>(vectors_.size() - vector_ind - 1);
      const VectorPoint& vector_point = vectors_[vector_idx];
      Vec2d start_pt = math::RotateVector2d(
          vector_point.start_pt - anchor_point, rotate_heading);
      Vec2d end_pt = math::RotateVector2d(vector_point.end_pt - anchor_point,
                                          rotate_heading);

      double rotated_start_x = start_pt.x();
      double rotated_start_y = start_pt.y();
      double rotated_end_x = end_pt.x();
      double rotated_end_y = end_pt.y();

      PolygonType super_type = polygon_type();
      std::string idx = polyline_id().ToString();
      bool mask = 0;
      fp->at(vector_end_idx - polygon_polyline_vector_dim_) =
          static_cast<float>(rotated_start_x);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 1u) =
          static_cast<float>(rotated_start_y);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 2u) =
          static_cast<float>(rotated_end_x);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 3u) =
          static_cast<float>(rotated_end_y);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 4u) =
          static_cast<float>(super_type);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 5u) =
          static_cast<float>(polygon_idx);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 6u) =
          static_cast<float>(vector_idx);
      fp->at(vector_end_idx - polygon_polyline_vector_dim_ + 7u) =
          static_cast<float>(mask);
    }
  }
}

std::string PolygonPolyline::ToString() const {
  std::string polygon_str;
  switch (polygon_type_) {
    case PolygonType::CROSSWALK: {
      polygon_str = "CROSSWALK";
      break;
    }
    case PolygonType::JUNCTION: {
      polygon_str = "JUNCTION";
      break;
    }
    case PolygonType::SPEEDBUMP: {
      polygon_str = "SPEEDBUMP";
      break;
    }
  }
  return polygon_str + polyline_id_.ToString();
}

double PolygonPolyline::DistanceSquareTo(const Vec2d& pt) const {
  if (polygon_.is_convex()) {
    return polygon_.DistanceSquareTo(pt);
  }

  double dist = std::numeric_limits<double>::max();
  if (vectors_.empty()) return dist;
  for (const auto& vector : vectors_) {
    dist = std::min(dist, vector.end_pt.DistanceSquareTo(pt));
  }
  dist = std::min(dist, vectors_.front().start_pt.DistanceSquareTo(pt));
  return dist;
}

void StopLinePolyline::ConvertToFloats(const Vec2d& anchor_point,
                                       const double& rotate_heading,
                                       const int32_t& stop_line_idx,
                                       const std::size_t& start_idx,
                                       std::vector<float>* const fp) const {
  size_t vector_end_idx =
      start_idx + stop_line_polyline_vector_size_ * stop_line_vector_dim_;

  for (size_t vector_ind = 0; vector_ind < stop_line_polyline_vector_size_;
       vector_ind++, vector_end_idx -= stop_line_vector_dim_) {
    if (vector_ind < vectors_.size()) {
      int32_t vector_idx =
          static_cast<int32_t>(vectors_.size() - vector_ind - 1);
      const VectorPoint& vector_point = vectors_[vector_idx];
      Vec2d start_pt = math::RotateVector2d(
          vector_point.start_pt - anchor_point, rotate_heading);
      Vec2d end_pt = math::RotateVector2d(vector_point.end_pt - anchor_point,
                                          rotate_heading);

      double rotated_start_x = start_pt.x();
      double rotated_start_y = start_pt.y();
      double rotated_end_x = end_pt.x();
      double rotated_end_y = end_pt.y();

      double heading =
          math::NormalizeAngle(vector_point.heading + rotate_heading);
      double cos_vec_heading = std::cos(heading);
      double sin_vec_heading = std::sin(heading);
      PolylineType super_type = polyline_id().type();
      StopLineType type = stop_line_type();
      bool mask = 0;
      fp->at(vector_end_idx - stop_line_vector_dim_) =
          static_cast<float>(rotated_start_x);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 1u) =
          static_cast<float>(rotated_start_y);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 2u) =
          static_cast<float>(rotated_end_x);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 3u) =
          static_cast<float>(rotated_end_y);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 4u) =
          static_cast<float>(heading);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 5u) =
          static_cast<float>(cos_vec_heading);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 6u) =
          static_cast<float>(sin_vec_heading);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 7u) =
          static_cast<float>(vector_idx);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 8u) =
          static_cast<float>(super_type);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 9u) =
          static_cast<float>(type);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 10u) =
          static_cast<float>(stop_line_idx);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 11u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 12u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 13u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 14u) =
          static_cast<float>(0);
      fp->at(vector_end_idx - stop_line_vector_dim_ + 15u) =
          static_cast<float>(mask);
    }
  }
}

void RoadBoundaryPolyline::ConvertToFloats(const Vec2d& anchor_point,
                                           const double& rotate_heading,
                                           const int32_t& road_boundary_idx,
                                           const std::size_t& start_idx,
                                           std::vector<float>* const fp) const {
  size_t vector_end_idx = start_idx + road_boundary_polyline_vector_size_ *
                                          road_boundary_vector_dim_;

  for (size_t vector_ind = 0;
       vector_ind < road_boundary_polyline_vector_size_ &&
       vector_ind < vectors_.size();
       vector_ind++, vector_end_idx -= road_boundary_vector_dim_) {
    int32_t vector_idx = static_cast<int32_t>(vectors_.size() - vector_ind - 1);
    const VectorPoint& vector_point = vectors_[vector_idx];
    Vec2d start_pt = math::RotateVector2d(vector_point.start_pt - anchor_point,
                                          rotate_heading);
    Vec2d end_pt = math::RotateVector2d(vector_point.end_pt - anchor_point,
                                        rotate_heading);

    double rotated_start_x = start_pt.x();
    double rotated_start_y = start_pt.y();
    double rotated_end_x = end_pt.x();
    double rotated_end_y = end_pt.y();

    double heading =
        math::NormalizeAngle(vector_point.heading + rotate_heading);
    double cos_vec_heading = std::cos(heading);
    double sin_vec_heading = std::sin(heading);
    PolylineType super_type = polyline_id().type();
    RoadBoundaryType type = road_boundary_type();
    std::string idx = ToString();  // need to be comfirmed
    bool mask = 0;                 // need to be comfirmed
    fp->at(vector_end_idx - road_boundary_vector_dim_) =
        static_cast<float>(rotated_start_x);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 1u) =
        static_cast<float>(rotated_start_y);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 2u) =
        static_cast<float>(rotated_end_x);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 3u) =
        static_cast<float>(rotated_end_y);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 4u) =
        static_cast<float>(heading);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 5u) =
        static_cast<float>(cos_vec_heading);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 6u) =
        static_cast<float>(sin_vec_heading);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 7u) =
        static_cast<float>(vector_idx);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 8u) =
        static_cast<float>(super_type);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 9u) =
        static_cast<float>(type.boundary_type);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 10u) =
        static_cast<float>(road_boundary_idx);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 11u) =
        static_cast<float>(0);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 12u) =
        static_cast<float>(0);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 13u) =
        static_cast<float>(0);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 14u) =
        static_cast<float>(0);
    fp->at(vector_end_idx - road_boundary_vector_dim_ + 15u) =
        static_cast<float>(mask);
  }

  // for (int vec_idx = 0; vec_idx < road_boundary_polyline_vector_size_;
  // ++vec_idx) {
  //   std::string vector_str;
  //   for (int i = 0; i < road_boundary_vector_dim_; ++i) {
  //     std::stringstream ss;
  //     ss << fp->at(start_idx + vec_idx * road_boundary_vector_dim_ + i) << ",
  //     "; vector_str += ss.str();
  //   }

  //   LDEBUG("LaneBoundaryPolyline %s vector %d :\n %s",
  //   polyline_id().ToString().c_str(), vec_idx, vector_str.c_str());
  // }
}
}  // namespace planning
}  // namespace ad_byd

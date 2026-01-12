

#ifndef ONBOARD_MAPS_SEGMENT_MATCHER_H_
#define ONBOARD_MAPS_SEGMENT_MATCHER_H_

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "plan_common/log.h"
#include "plan_common/math/geometry/aabox2d.h"
#include "plan_common/math/geometry/aabox_kdtree2d.h"
#include "plan_common/math/geometry/segment2d.h"

namespace st::mapping {

struct Segment {
  Segment(double x1, double y1, double x2, double y2)
      : x1(x1), y1(y1), x2(x2), y2(y2) {}
  explicit Segment(const Segment2d& segment) { FromSegment2d(segment); }

  void FromSegment2d(const Segment2d& segment) {
    const auto& s = segment.start();
    const auto& e = segment.end();
    x1 = s.x();
    y1 = s.y();
    x2 = e.x();
    y2 = e.y();
  }
  Segment2d ToSegment2d() const { return Segment2d({x1, y1}, {x2, y2}); }

  AABox2d ToAABox2d() const { return AABox2d({x1, y1}, {x2, y2}); }
  double x1, y1, x2, y2;
};

template <typename T>
class SegmentAABox {
 public:
  SegmentAABox(const T& segments, int64_t idx)
      : segments_(segments), idx_(idx) {}

  AABox2d ComputeAABox() const {
    return std::get<Segment>(segments_[idx_]).ToAABox2d();
  }
  double DistanceSquareTo(const Vec2d& point) const {
    return std::get<Segment>(segments_[idx_])
        .ToSegment2d()
        .DistanceSquareTo(point);
  }

  int64_t index() const { return idx_; }

 private:
  const T& segments_;
  int64_t idx_;
};

template <typename T>
class SegmentMatcher {
 public:
  using Data = T;
  using SegmentData = std::vector<std::pair<Data, Segment2d>>;
  using SegmentInternalData = std::vector<std::pair<Data, Segment>>;

  explicit SegmentMatcher(const SegmentData& segment_data) {
    segment_data_.reserve(segment_data.size());
    for (const auto& [data, segment] : segment_data) {
      segment_data_.emplace_back(data, segment);
    }
    InitAABoxKdTree();
  }

  explicit SegmentMatcher(const SegmentInternalData& segment_data)
      : segment_data_(segment_data) {
    InitAABoxKdTree();
  }

  explicit SegmentMatcher(SegmentInternalData&& segment_data)
      : segment_data_(std::move(segment_data)) {
    InitAABoxKdTree();
  }

  std::vector<Segment2d> segments() const {
    std::vector<Segment2d> segments;
    segments.reserve(segment_data_.size());
    for (const auto& [_, segment] : segment_data_) {
      segments.push_back(segment.ToSegment2d());
    }
    return segments;
  }

  bool GetNearestData(double x, double y, Data* data) const {
    CHECK_NOTNULL(data);
    const auto& object = segment_tree_->GetNearestObject({x, y});
    if (object == nullptr) {
      return false;
    }

    *data = std::get<Data>(segment_data_[object->index()]);
    return true;
  }

  std::vector<Data> GetKNearestData(int k, double x, double y) const {
    const auto objects = segment_tree_->GetKNearestObjects(k, {x, y});
    if (objects.empty()) {
      return {};
    }
    std::vector<const Data> result;
    result.reserve(objects.size());
    for (const auto& object : objects) {
      result.push_back(std::get<Data>(segment_data_[object->index()]));
    }
    return result;
  }

  std::vector<Data> GetDataInRadius(double x, double y, double r) const {
    const auto& objects = segment_tree_->GetObjects({x, y}, r);
    std::vector<Data> data;
    data.reserve(objects.size());
    for (const auto& object : objects) {
      data.push_back(std::get<Data>(segment_data_[object->index()]));
    }
    return data;
  }

  bool GetNearestSegment(double x, double y, Segment2d* segment) const {
    const auto& object = segment_tree_->GetNearestObject({x, y});
    if (object == nullptr) {
      return false;
    }

    *segment = std::get<Segment>(segment_data_[object->index()]).ToSegment2d();
    return true;
  }

  std::vector<Segment2d> GetKNearestSegments(int k, double x, double y) const {
    const auto objects = segment_tree_->GetKNearestObjects(k, {x, y});
    if (objects.empty()) {
      return {};
    }

    std::vector<Segment2d> result;
    result.reserve(objects.size());
    for (const auto& object : objects) {
      result.push_back(
          std::get<Segment>(segment_data_[object->index()]).ToSegment2d());
    }
    return result;
  }

  std::vector<Segment2d> GetSegmentInRadius(double x, double y,
                                            double r) const {
    const auto& objects = segment_tree_->GetObjects({x, y}, r);
    std::vector<Segment2d> segments;
    segments.reserve(objects.size());
    for (const auto& object : objects) {
      segments.push_back(
          std::get<Segment>(segment_data_[object->index()]).ToSegment2d());
    }
    return segments;
  }

  bool GetNearestSegmentData(double x, double y, Segment2d* segment,
                             Data* data) const {
    const auto& object = segment_tree_->GetNearestObject({x, y});
    if (object == nullptr) {
      return false;
    }
    const auto idx = object->index();
    *segment = std::get<Segment>(segment_data_[idx]).ToSegment2d();
    *data = std::get<Data>(segment_data_[idx]);
    return true;
  }

  SegmentData GetSegmentDataInRadius(double x, double y, double r) const {
    const auto& objects = segment_tree_->GetObjects({x, y}, r);
    SegmentData segment_data;
    segment_data.reserve(objects.size());
    for (const auto& object : objects) {
      const auto& [data, segment] = segment_data_[object->index()];
      segment_data.emplace_back(data, segment.ToSegment2d());
    }
    return segment_data;
  }

  bool GetProjection(double x, double y, Vec2d* nearest_point = nullptr,
                     double* accumulated_s = nullptr,
                     double* min_dist = nullptr,
                     Segment2d* segment = nullptr) const {
    const Vec2d point{x, y};
    const auto& object = segment_tree_->GetNearestObject(point);
    if (object == nullptr) {
      return false;
    }

    const auto idx = object->index();
    const auto nearest_segment =
        std::get<Segment>(segment_data_[idx]).ToSegment2d();
    const double dist = nearest_segment.DistanceTo(point, nearest_point);
    if (min_dist != nullptr) {
      *min_dist = dist;
    }
    if (accumulated_s != nullptr) {
      double length = 0.0;
      for (int i = 0; i < idx; ++i) {
        length += std::get<Segment>(segment_data_[i]).ToSegment2d().length();
      }
      *accumulated_s = length + nearest_segment.ProjectOntoUnit(point);
    }
    if (segment != nullptr) {
      *segment = nearest_segment;
    }
    return true;
  }

  bool GetNearestDataWithHeading(double x, double y, double theta,
                                 double max_radius, double max_heading_diff,
                                 Data* nearest_data) const {
    const Vec2d point{x, y};
    const auto& objects = segment_tree_->GetObjects(point, max_radius);

    double min_dist = std::numeric_limits<double>::max();
    for (const auto& object : objects) {
      const auto idx = object->index();
      const auto segment = std::get<Segment>(segment_data_[idx]).ToSegment2d();
      const double heading_diff = NormalizeAngle(theta - segment.heading());
      if (std::abs(heading_diff) > max_heading_diff) {
        continue;
      }
      const auto dist = segment.DistanceSquareTo(point);
      if (dist < min_dist) {
        min_dist = dist;
        *nearest_data = std::get<Data>(segment_data_[idx]);
      }
    }

    return min_dist < std::numeric_limits<double>::max();
  }

  std::vector<Data> GetDataInRadiusWithHeading(double x, double y, double theta,
                                               double max_radius,
                                               double max_heading_diff) const {
    const auto& objects = segment_tree_->GetObjects({x, y}, max_radius);
    std::vector<Data> data_in_radius;
    for (const auto& object : objects) {
      const auto& [data, segment] = segment_data_[object->index()];
      const double heading_diff =
          NormalizeAngle(theta - segment.ToSegment2d().heading());
      if (std::abs(heading_diff) > max_heading_diff) {
        continue;
      }
      data_in_radius.push_back(data);
    }

    return data_in_radius;
  }

 private:
  void InitAABoxKdTree() {
    segment_aabox_.clear();
    segment_aabox_.reserve(segment_data_.size());
    for (size_t idx = 0; idx < segment_data_.size(); ++idx) {
      segment_aabox_.emplace_back(segment_data_, idx);
    }
    segment_tree_ =
        std::make_unique<AABoxKDTree2d<SegmentAABox<SegmentInternalData>>>(
            segment_aabox_, AABoxKDTreeParams{.max_leaf_size = 4});
  }

  SegmentInternalData segment_data_;
  std::vector<SegmentAABox<SegmentInternalData>> segment_aabox_;
  std::unique_ptr<AABoxKDTree2d<SegmentAABox<SegmentInternalData>>>
      segment_tree_;
};

}  // namespace st::mapping

#endif  // ONBOARD_MAPS_SEGMENT_MATCHER_H_

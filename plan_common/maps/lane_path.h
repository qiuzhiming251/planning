

#ifndef ST_PLANNING_MAPS_LANE_PATH
#define ST_PLANNING_MAPS_LANE_PATH

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <string>
#include <utility>
#include <vector>

#include "plan_common/log.h"

// #include "global/buffered_logger.h"
// #include "lite/check.h"
#include <cereal/access.hpp>
#include "plan_common/maps/lane_path_data.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"

namespace st::mapping {

constexpr double kEpsilon = 1e-6;

// A lane path can be defined in forward or backward direction. In forward
// direciton, the consecutive lane segment(s) are connected in the same
// direciton of the lane direction, and in the opposite direciton in backward
// direction.
class LanePath {
 public:
  LanePath() = default;
  LanePath(const ad_byd::planning::MapConstPtr& map_ptr,
           std::vector<ElementId> lane_ids, double start_fraction,
           double end_fraction, bool lane_path_in_forward_direction = true)
      : lane_path_data_(LanePathData(start_fraction, end_fraction, lane_ids,
                                     lane_path_in_forward_direction)) {
    Build(map_ptr);
  }
  LanePath(const ad_byd::planning::MapConstPtr& map_ptr, LanePoint lane_point)
      : lane_path_data_(LanePathData(lane_point)) {
    Build(map_ptr);
  }
  LanePath(const ad_byd::planning::MapConstPtr& map_ptr,
           LanePathData lane_path_data)
      : lane_path_data_(std::move(lane_path_data)) {
    Build(map_ptr);
  }

  LanePath(LanePathData lane_path_data, std::vector<double> lane_end_s,
           std::vector<double> lane_lengths)
      : lane_path_data_(std::move(lane_path_data)),
        lane_end_s_(std::move(lane_end_s)),
        lane_lengths_(std::move(lane_lengths)) {}

  const LanePathData& lane_path_data() const { return lane_path_data_; }

  const ad_byd::planning::LaneSequencePtr& lane_seq() const {
    return lane_seq_;
  }

  void set_lane_seq(const ad_byd::planning::LaneSequencePtr& lane_seq) {
    lane_seq_ = lane_seq;
  }

  int size() const { return lane_path_data_.size(); }
  int lane_ids_size() const { return lane_path_data_.size(); }

  bool IsEmpty() const { return lane_path_data_.empty(); }
  bool IsEqual(const LanePath& other) const {
    return lane_path_data_ == other.lane_path_data();
  }
  bool operator==(const LanePath& other) const { return IsEqual(other); }
  bool operator!=(const LanePath& other) const { return !IsEqual(other); }

  const std::vector<ElementId>& lane_ids() const {
    return lane_path_data_.lane_ids();
  }

  ElementId lane_id(int i) const { return lane_path_data_.lane_id(i); }
  double start_fraction() const { return lane_path_data_.start_fraction(); }
  double end_fraction() const { return lane_path_data_.end_fraction(); }
  double start_s(int i) const { return lane_end_s_[i]; }
  double end_s(int i) const { return lane_end_s_[i + 1]; }

  std::string DebugString() const;

  LanePoint front() const { return lane_path_data_.start_point(); }
  LanePoint back() const { return lane_path_data_.end_point(); }

  bool IsConnectedTo(const ad_byd::planning::MapConstPtr& map_ptr,
                     const LanePath& subsequent_path) const;
  LanePath Connect(const ad_byd::planning::MapConstPtr& map_ptr,
                   const LanePath& subsequent_path) const;

  bool IsValid(const ad_byd::planning::MapConstPtr& map_ptr) const;

  double length() const {
    CHECK(!lane_end_s_.empty());
    return lane_end_s_.back();
  }

  // Access to individual lanes.
  struct LaneSegment {
    int lane_index;
    mapping::ElementId lane_id;
    double start_fraction;
    double end_fraction;
    double start_s;
    double end_s;

    LaneSegment(int lane_index, mapping::ElementId lane_id,
                double start_fraction, double end_fraction, double start_s,
                double end_s)
        : lane_index(lane_index),
          lane_id(lane_id),
          start_fraction(start_fraction),
          end_fraction(end_fraction),
          start_s(start_s),
          end_s(end_s) {}

    double length() const { return end_s - start_s; }

    bool HasOverlap(
        const LaneSegment& other,
        std::pair<double, double>* overlap_fraction = nullptr) const;
  };

  class LaneSegmentIterator {
   public:
    explicit LaneSegmentIterator(const LanePath* lane_path, int lane_index = 0)
        : lane_path_(*CHECK_NOTNULL(lane_path)), lane_index_(lane_index) {}

    const LanePath& lane_path() const { return lane_path_; }
    int lane_index() const { return lane_index_; }

    LaneSegment operator*() const {
      return lane_path_.lane_segment(lane_index_);
    }

    LaneSegmentIterator& operator=(const LaneSegmentIterator& other) {
      DCHECK(lane_path_ == other.lane_path_);
      lane_index_ = other.lane_index_;
      return *this;
    }

    bool operator==(const LaneSegmentIterator& other) const {
      DCHECK(lane_path_ == other.lane_path_);
      return lane_index_ == other.lane_index_;
    }
    bool operator!=(const LaneSegmentIterator& other) const {
      DCHECK(lane_path_ == other.lane_path_);
      return !(*this == other);
    }
    bool operator<(const LaneSegmentIterator& other) const {
      DCHECK(lane_path_ == other.lane_path_);
      return lane_index_ < other.lane_index_;
    }

    LaneSegmentIterator operator+(int inc) const {
      const int new_lane_index = lane_index_ + inc;
      DCHECK_GE(new_lane_index, 0);
      DCHECK_LE(new_lane_index, lane_path_.size());
      return LaneSegmentIterator(&lane_path_, new_lane_index);
    }
    LaneSegmentIterator operator-(int inc) const {
      const int new_lane_index = lane_index_ - inc;
      DCHECK_GE(new_lane_index, 0);
      DCHECK_LE(new_lane_index, lane_path_.size());
      return LaneSegmentIterator(&lane_path_, new_lane_index);
    }
    int operator-(const LaneSegmentIterator& other) const {
      DCHECK(lane_path_ == other.lane_path_);
      return lane_index_ - other.lane_index_;
    }

    LaneSegmentIterator operator++() {
      DCHECK_LT(lane_index_, lane_path_.size());
      const LaneSegmentIterator old_it = *this;
      lane_index_++;
      return old_it;
    }
    const LaneSegmentIterator& operator++(int) {
      DCHECK_LT(lane_index_, lane_path_.size());
      ++lane_index_;
      return *this;
    }
    LaneSegmentIterator operator--() {
      DCHECK_GT(lane_index_, 0);
      const LaneSegmentIterator old_it = *this;
      lane_index_--;
      return old_it;
    }
    const LaneSegmentIterator& operator--(int) {
      DCHECK_GT(lane_index_, 0);
      --lane_index_;
      return *this;
    }

   private:
    const LanePath& lane_path_;
    int lane_index_;
  };

  LaneSegmentIterator begin() const { return LaneSegmentIterator(this, 0); }
  LaneSegmentIterator end() const { return LaneSegmentIterator(this, size()); }

  LaneSegmentIterator GetIteratorAt(int lane_index) const {
    return LaneSegmentIterator(this, lane_index);
  }

  LaneSegment lane_segment(int lane_index) const {
    DCHECK_GE(lane_index, 0);
    DCHECK_LT(lane_index, size());
    return {lane_index,
            lane_id(lane_index),
            lane_index == 0 ? start_fraction() : 0.0,
            lane_index + 1 == size() ? end_fraction() : 1.0,
            lane_end_s_[lane_index],
            lane_end_s_[lane_index + 1]};
  }

  // {lane index, lane fraction}.
  using LaneIndexPoint = std::pair<int, double>;

  LanePoint ArclengthToLanePoint(double s) const;
  double FirstOccurrenceOfLanePointToArclength(LanePoint point) const;
  double LastOccurrenceOfLanePointToArclength(LanePoint point) const;
  LaneIndexPoint FirstOccurrenceOfLanePointToLaneIndexPoint(
      LanePoint point) const;
  LaneIndexPoint LastOccurrenceOfLanePointToLaneIndexPoint(
      LanePoint point) const;
  int ArclengthToLaneIndex(double s) const;
  double LaneIndexToArclength(int lane_index) const;
  LaneIndexPoint ArclengthToLaneIndexPoint(double s) const;
  double LaneIndexPointToArclength(int lane_index, double lane_fraction) const;
  double LaneIndexPointToArclength(
      const LaneIndexPoint& lane_index_point) const;

  LaneSegmentIterator LaneIndexPointToLaneSegmentIterator(
      LaneIndexPoint lane_index_point) const {
    return LaneSegmentIterator(this, lane_index_point.first);
  }

  bool ContainsLanePoint(
      LanePoint point,
      LaneIndexPoint* first_encountered_lane_index_point = nullptr,
      bool forward_search = true) const;

  LanePath BeforeFirstOccurrenceOfLanePoint(LanePoint point) const;
  LanePath BeforeLastOccurrenceOfLanePoint(LanePoint point) const;
  LanePath AfterFirstOccurrenceOfLanePoint(LanePoint point) const;
  LanePath AfterLastOccurrenceOfLanePoint(LanePoint point) const;
  LanePath BeforeArclength(double s) const;
  LanePath AfterArclength(double s) const;
  LanePath BeforeLaneIndexPoint(int lane_index, double lane_fraction) const;
  LanePath AfterLaneIndexPoint(int lane_index, double lane_fraction) const;
  LanePath BeforeLaneIndexPoint(const LaneIndexPoint& lane_index_point) const;
  LanePath AfterLaneIndexPoint(const LaneIndexPoint& lane_index_point) const;

  void ToProto(LanePathProto* proto) const {
    lane_path_data_.ToLanePathProto(proto);
  }

  void FromProto(const ad_byd::planning::MapConstPtr& map_ptr,
                 const LanePathProto& proto) {
    lane_path_data_.FromLanePathProto(proto);
    Build(map_ptr);
  }

  void FillLaneLengthsByIdsIfAbsent(
      const ad_byd::planning::MapConstPtr& map_ptr);

 protected:
  void Build(const ad_byd::planning::MapConstPtr& map_ptr);
  double GetLaneLength(const ad_byd::planning::MapConstPtr& map_ptr,
                       int index) const;

 private:
  LanePathData lane_path_data_;
  std::vector<ElementId> exp_traj_string_;
  std::vector<ElementId> split_topo_modified_string_;
  std::vector<ElementId> merge_topo_modified_string_;
  std::vector<double> lane_end_s_;  // 1 longer than lane_ids_. 1st element = 0.
  std::vector<double> lane_lengths_;
  ad_byd::planning::LaneSequencePtr lane_seq_;

  template <class Archive>
  friend void serialize(Archive& ar, LanePath& lane_path);
};

}  // namespace st::mapping

#endif  // ST_PLANNING_MAPS_LANE_PATH

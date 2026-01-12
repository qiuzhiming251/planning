

#ifndef AD_BYD_PLANNING_MAP_SECTION_H
#define AD_BYD_PLANNING_MAP_SECTION_H
#include <map>
#include <unordered_map>
#include <utility>

#include "plan_common/maps/lane.h"
#include "plan_common/maps/lane_sequence.h"
#include "plan_common/maps/map_def.h"

namespace ad_byd {
namespace planning {
class Section {
 public:
  Section(){};
  ~Section(){};
  enum RoadClass { NORMAL = 1, CITY_EXPRESS = 2, HIGHWAY = 3 };
  enum NoneOddType { TYPE_NONE = 0, TYPE_TOLL = 1, TYPE_CONSTRUCTION = 2 };
  enum SectionDirection { NONE = 0, LEFT = 1, RIGHT = 2 };
  using DirectionMap = std::unordered_map<uint64_t, SectionDirection>;

  void set_id(const uint64_t id) { id_ = id; };
  void set_topo_length(double length) { length_ = length; };
  void set_curve_length(double curve_length) { curve_length_ = curve_length; };
  void set_lanes(const std::vector<uint64_t>& lanes) { lanes_ = lanes; };
  void set_outgoing_sections(const std::vector<uint64_t>& outgoing_sections) {
    outgoing_sections_ = outgoing_sections;
  };
  void set_incoming_sections(const std::vector<uint64_t>& incoming_sections) {
    incoming_sections_ = incoming_sections;
  };
  void set_speed_limit(double speed_limit) { speed_limit_ = speed_limit; };
  void set_average_limit(double average_limit) {
    average_limit_ = average_limit;
  };
  void set_road_class(RoadClass road_class) { road_class_ = road_class; };
  void set_outgoing_direction(uint64_t outgoing_section_id,
                              SectionDirection section_direction) {
    outgoing_directions_[outgoing_section_id] = section_direction;
  }

  uint64_t id() const { return id_; };
  double topo_length() const { return length_; };
  double curve_length() const { return curve_length_; };
  const std::vector<uint64_t>& lanes() const { return lanes_; };
  std::vector<uint64_t>* mutable_lanes() { return &lanes_; };
  const std::vector<uint64_t>& outgoing_sections() const {
    return outgoing_sections_;
  };
  const std::vector<uint64_t>& incoming_sections() const {
    return incoming_sections_;
  };
  double speed_limit() const { return speed_limit_; };
  double average_limit() const { return average_limit_; };
  RoadClass road_class() const { return road_class_; };
  SectionDirection get_outgoing_direction(uint64_t outgoing_section_id) const {
    auto it = outgoing_directions_.find(outgoing_section_id);
    if (it != outgoing_directions_.end()) {
      return it->second;
    }
    return SectionDirection::NONE;
  }

  void add_lanes(const uint64_t lane) { lanes_.push_back(lane); };
  void add_outgoing_sections(const uint64_t section) {
    outgoing_sections_.push_back(section);
  };
  void add_incoming_sections(const uint64_t section) {
    incoming_sections_.push_back(section);
  };
  void set_navi_priority_lane_id(const uint64_t lane_id) {
    navi_priority_lane_id_ = lane_id;
  }
  uint64_t get_navi_priority_lane_id() const { return navi_priority_lane_id_; }
  void set_none_odd_type(NoneOddType type) { none_odd_type_ = type; }
  NoneOddType get_none_odd_type() const { return none_odd_type_; }

 private:
  uint64_t id_ = 0;
  double length_ = 0.0;
  double curve_length_ = 0.0;
  std::vector<uint64_t> lanes_;

  std::vector<uint64_t> outgoing_sections_;
  std::vector<uint64_t> incoming_sections_;
  double speed_limit_ = 60 / 3.6;
  double average_limit_ = 60 / 3.6;
  RoadClass road_class_ = RoadClass::NORMAL;
  DirectionMap outgoing_directions_;
  NoneOddType none_odd_type_ = TYPE_NONE;
  uint64_t navi_priority_lane_id_ = 0;
  template <typename Archive>
  friend void serialize(Archive& ar, Section& section);
};
typedef std::shared_ptr<Section> SectionPtr;
typedef std::shared_ptr<const Section> SectionConstPtr;

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_MAP_SECTION_H



#ifndef ST_PLANNING_MAPS_LANE_PATH_UTIL
#define ST_PLANNING_MAPS_LANE_PATH_UTIL

#include <iterator>
#include <type_traits>
#include <vector>

//#include "global/logging.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "modules/cnoa_pnc/planning/proto/lane_path.pb.h"

namespace st {
namespace mapping {

template <typename LanePathType,
          std::enable_if_t<
              std::is_same_v<LanePathType, st::mapping::LanePath> ||
                  std::is_same_v<LanePathType, st::mapping::LanePathProto>,
              bool> = true>
double GetLanePathLength(const ad_byd::planning::Map& semantic_map_manager,
                         const LanePathType& lane_path) {
  const auto& lane_ids = lane_path.lane_ids();
  const auto start_fraction = lane_path.start_fraction();
  const auto end_fraction = lane_path.end_fraction();
  double len = 0.0;
  if (lane_ids.size() == 1) {
    SMM_ASSIGN_LANE_OR_RETURN(lane_info, semantic_map_manager,
                              mapping::ElementId(*lane_ids.begin()), 0.0);
    len = lane_info.curve_length() * (end_fraction - start_fraction);
  } else {
    int i = 0;
    for (const auto lane_id : lane_path.lane_ids()) {
      SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, semantic_map_manager,
                                  mapping::ElementId(lane_id));
      if (i == 0) {
        len += lane_info.curve_length() * (1.0 - start_fraction);
      } else if (i == lane_ids.size() - 1) {
        len += lane_info.curve_length() * end_fraction;
      } else {
        len += lane_info.curve_length();
      }
      ++i;
    }
  }
  return len;
}

template <typename Container>
st::mapping::LanePathProto CreateLanePathProto(
    double start_fraction, double end_fraction, const Container& c,
    bool lane_path_in_forward_direction = true) {
  st::mapping::LanePathProto lane_path_proto;
  lane_path_proto.mutable_lane_ids()->Reserve(std::size(c));
  for (const auto& lane_id : c) {
    lane_path_proto.add_lane_ids(lane_id);
  }
  lane_path_proto.set_start_fraction(start_fraction);
  lane_path_proto.set_end_fraction(end_fraction);
  lane_path_proto.set_lane_path_in_forward_direction(
      lane_path_in_forward_direction);
  return lane_path_proto;
}

}  // namespace mapping
}  // namespace st

#endif  // ST_PLANNING_MAPS_LANE_PATH_UTIL

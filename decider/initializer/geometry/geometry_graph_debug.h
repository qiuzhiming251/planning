

#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "decider/initializer/geometry/geometry_graph.h"
#include "modules/cnoa_pnc/planning/proto/initializer.pb.h"

namespace st::planning {

enum class ResampleReason {
  RESAMPLED = 1,
  NR_ZERO_REACHABLE = 2,
  NR_ALL_REACHABLE = 3,
  NR_INVALID_RANGE = 4,
  NR_LATERAL_RESOLUTION = 5,
  NOT_INITIALIZED = 6
};

struct EdgeDebugInfo {
  int start_layer_idx = 0;
  int start_node_on_layer_idx = 0;
  int end_layer_idx = 0;
  int end_node_on_layer_idx = 0;
  int start_station_idx = 0;
  int end_station_idx = 0;

  std::string Debug() const {
    std::string debug_str = "\n\tLayer Idx\tNode on Layer Idx\tStation Idx\n";
    debug_str +=
        absl::StrCat("Start\t", start_layer_idx, "\t", start_node_on_layer_idx,
                     "\t", start_station_idx, "\n");
    debug_str +=
        absl::StrCat("End\t", end_layer_idx, "\t", end_node_on_layer_idx, "\t",
                     end_station_idx, "\n");
    debug_str += "\n---------------------------------------------";
    return debug_str;
  }
};

absl::Status CheckGeometryGraphConnectivity(const GeometryGraph& graph);

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_

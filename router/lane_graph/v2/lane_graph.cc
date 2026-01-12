#include "router/lane_graph/v2/lane_graph.h"
namespace st::planning::v2 {
const LaneGraph::VertexId LaneGraph::kInvalidVertexId = ToVertId(-1, "");
const LaneGraph::VertexId LaneGraph::kTargetVertex = ToVertId(-1, "");

const LaneGraph::VertexId& InvalidVertexId(LaneGraph::VertexId) {
  return LaneGraph::kInvalidVertexId;
}

}  // namespace st::planning::v2